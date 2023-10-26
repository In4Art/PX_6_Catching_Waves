#include <Arduino.h>

#ifdef ESP8266
 #include <ESP8266WiFi.h>
#else //ESP32
 #include <WiFi.h>
#endif

#include <ModbusIP_ESP8266.h>

#include "creds.h"
#include "WifiControl.h"
#include "ModeControl.h"


#define PX_NUM 6
#define PX_REG 100 + (PX_NUM * 10) //base operational modbus register
#define PX_STATE_REG 200 + (PX_NUM * 10) // base status modbus register


//wifi setup
char ssid[] = SSID ;        // your network SSID (name)
char pass[] = PW;                    // your network password
WifiControl pxWifi(ssid, pass, PX_NUM);


#define INFLATE_VALVE D2
#define DEFLATE_VALVE D1

#define INFLATE_TIME_1 3 * 3500
#define INFLATE_TIME_2 3 * 3500
#define DEFLATE_TIME_1  4000
#define DEFLATE_TIME_2 7500

typedef enum{
    ERROR = -1,
    EMPTY,
    INFLATING,
    HALF_INFLATED,
    FULL_INFLATED,
    DEFLATING
}pxvi_state_t;

pxvi_state_t px_vi_state = FULL_INFLATED;
uint32_t px_vi_state_time = 0;
uint32_t px_vi_state_dur = 0;


enum {
  PX_ERR = -1,
  PX_OK
};



ModbusIP pxModbus;

int8_t pxState = 0;
void setState(int8_t state);

#define DEMO_SW_PIN 3
int8_t demoState = 0;
void demoCallback(uint32_t dTime, px_mode_t mode);
ModeControl pxMC(DEMO_SW_PIN, &demoCallback, 2 * DEFLATE_TIME_2, &pxWifi);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600, SERIAL_8N1, SERIAL_TX_ONLY);
  pinMode(INFLATE_VALVE, OUTPUT);
  digitalWrite(INFLATE_VALVE, LOW);
  pinMode(DEFLATE_VALVE, OUTPUT);
  digitalWrite(DEFLATE_VALVE, LOW);
  delay(2000);

  pxWifi.setTimeOut(30000);


  if(digitalRead(DEMO_SW_PIN) == HIGH){//skip 1st wifi connection attempt if demo switch is on
    Serial.println("Connecting to C&C...");
    int8_t res = pxWifi.init();
    if(res == -1){
      Serial.println("No C&C found, starting up in demo mode!");
    }
  }

  pxModbus.server(502);
  pxModbus.addHreg(PX_REG, 0);
  pxModbus.addIreg(PX_REG, 0);
  pxModbus.addHreg(PX_STATE_REG, PX_OK);

  digitalWrite(DEFLATE_VALVE, HIGH);
  delay( 2 * DEFLATE_TIME_2);
  digitalWrite(DEFLATE_VALVE, LOW);
  px_vi_state = EMPTY;
  px_vi_state_time = millis();

  pxMC.init();

  Serial.println("Setup complete");
}

void loop() {
  // put your main code here, to run repeatedly:

  pxModbus.task();

 
  pxMC.run();

  
  if(pxModbus.Hreg(PX_REG) != pxModbus.Ireg(PX_REG)){
    pxModbus.Ireg(PX_REG, pxModbus.Hreg(PX_REG));
  }

  
  if(pxState != pxModbus.Ireg(PX_REG)){
    setState((int8_t)pxModbus.Ireg(PX_REG));
  }

  
  if(px_vi_state == INFLATING || px_vi_state == DEFLATING){
    switch(px_vi_state){
      case INFLATING:
        if(pxState == 1){
            if(millis() - px_vi_state_time >= px_vi_state_dur){
            digitalWrite(INFLATE_VALVE, LOW);
            px_vi_state = HALF_INFLATED;
            }
        }else if(pxState == 2){
          if(millis() - px_vi_state_time >= px_vi_state_dur){
            digitalWrite(INFLATE_VALVE, LOW);
            px_vi_state = FULL_INFLATED;
            }

        }
        
        break;
      case DEFLATING:
        if(pxState == 1){ 
          if(millis() - px_vi_state_time >= px_vi_state_dur){
          digitalWrite(DEFLATE_VALVE, LOW);
          px_vi_state = HALF_INFLATED;}

        }else if(pxState == 0){
          if(millis() - px_vi_state_time >= px_vi_state_dur){
          digitalWrite(DEFLATE_VALVE, LOW);
          px_vi_state = EMPTY;
          }
        }
        break;
      default:
        break;
        
    }
  }

  
  //attempt to reconnect to C&C
  //do not attempt when any px_br is INFLATING
  //reConn() blocks for some time
  if(pxWifi.getStatus() != WL_CONNECTED){
    bool canReconnect = true;
    if(px_vi_state != EMPTY){
      canReconnect = false;
    }
    if(canReconnect){
      pxWifi.reConn();
    }
    
  }


}

void setState(int8_t state)
{

  if(state > 2){
    return;
  }
  if(state > pxState){
    //inflate
    digitalWrite(INFLATE_VALVE, HIGH);
    px_vi_state = INFLATING; 
    px_vi_state_time = millis();
      switch(state){
        case 1:
          px_vi_state_dur = INFLATE_TIME_1;
          break;
        case 2:
          if(pxState == 0){
            px_vi_state_dur = INFLATE_TIME_1 + INFLATE_TIME_2;
          }else{
            px_vi_state_dur = INFLATE_TIME_2;
          }
          break;
        default:
          px_vi_state_dur = 0;
          break;
    } 
  }else if(state < pxState){
    digitalWrite(INFLATE_VALVE, LOW);
    digitalWrite(DEFLATE_VALVE, HIGH);
    px_vi_state = DEFLATING;
    px_vi_state_time = millis();
    switch (state)
    {
    case 1:
      px_vi_state_dur = DEFLATE_TIME_1;
      break;
    case 0:
      if(pxState == 2){
        px_vi_state_dur = DEFLATE_TIME_2 + DEFLATE_TIME_1;
      }else{
        px_vi_state_dur = DEFLATE_TIME_2;
      }
      break;
    
    default:
      px_vi_state_dur = 0;
      break;
    }
    //deflate
  }
  pxState = state;
}

void demoCallback(uint32_t dTime, px_mode_t mode)
{
  if(mode == PX_DEMO_MODE){
    if(demoState < 4){
      demoState++;
      if(demoState < 2){
        pxModbus.Hreg(PX_REG, demoState);
      }else{
        pxModbus.Hreg(PX_REG, 4 - demoState);
      }
    }else{
      demoState = 0;
      pxModbus.Hreg(PX_REG, 0);
    }
  }else if(mode == PX_CC_MODE){
    Serial.println("to CC mode demo calback");
    demoState = 0;
    pxModbus.Hreg(PX_REG, 0);
  }

}