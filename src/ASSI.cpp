#include <Arduino.h>
#include "ASSI.h"



volatile int ASSI_status = 1;
unsigned long ASSI_YELLOW_time = 0, ASSI_BLUE_time = 0;
 

int ASSI(int ASSI_status){
  
  switch (ASSI_status)
  {
  case 1:
    digitalWrite(YELLOW_LEDS, LOW);
    digitalWrite(BLUE_LEDS, LOW);
    //digitalWrite(YELLOW_LEDS, HIGH);
    //digitalWrite(BLUE_LEDS, LOW);
    break;
  case 2:
    digitalWrite(YELLOW_LEDS, HIGH);
    digitalWrite(BLUE_LEDS, LOW);
    //digitalWrite(YELLOW_LEDS, LOW);
    //digitalWrite(BLUE_LEDS, HIGH);
    break;
  case 3:
  digitalWrite(BLUE_LEDS, LOW); 
  //digitalWrite(BLUE_LEDS, HIGH);
  if(millis() - ASSI_YELLOW_time  >= 500){
    ASSI_YELLOW_time = millis();
    digitalWrite(YELLOW_LEDS, !digitalRead(YELLOW_LEDS));
  }
    break;
    case 4:
    digitalWrite(YELLOW_LEDS, LOW);
    //digitalWrite(YELLOW_LEDS, HIGH);
    if (millis() - ASSI_BLUE_time  >= 500)
    {
      ASSI_BLUE_time = millis();
      digitalWrite(BLUE_LEDS, !digitalRead(BLUE_LEDS));
    }    
    break;
    case 5:
    digitalWrite(YELLOW_LEDS, LOW);
    digitalWrite(BLUE_LEDS, HIGH);
   // digitalWrite(YELLOW_LEDS, HIGH);
   // digitalWrite(BLUE_LEDS, LOW);
    break;

  default:
  digitalWrite(YELLOW_LEDS, LOW);
  digitalWrite(BLUE_LEDS, LOW);
  //digitalWrite(YELLOW_LEDS, HIGH);
  //digitalWrite(BLUE_LEDS, HIGH);
  //Serial.println("ASSI Status not defined");
    break;
  }
  return ASSI_status;
}



void Mission_Select(int mission){
  
  switch (mission)
  {
  case 0:
    /*digitalWrite(MS_LED1, HIGH);
    digitalWrite(MS_LED2, LOW);
    digitalWrite(MS_LED3, LOW);
    digitalWrite(MS_LED4, LOW);
    digitalWrite(MS_LED5, LOW);
    digitalWrite(MS_LED6, LOW);
    digitalWrite(MS_LED7, LOW);*/
    digitalWrite(MS_LED1, LOW);
    digitalWrite(MS_LED2, 1);
    digitalWrite(MS_LED3, 1);
    digitalWrite(MS_LED4, 1);
    digitalWrite(MS_LED5, 1);
    digitalWrite(MS_LED6, 1);
    digitalWrite(MS_LED7, 1);
    break;
  case 1:
    /*digitalWrite(MS_LED1, LOW);
    digitalWrite(MS_LED2, HIGH);
    digitalWrite(MS_LED3, LOW);
    digitalWrite(MS_LED4, LOW);
    digitalWrite(MS_LED5, LOW);
    digitalWrite(MS_LED6, LOW);
    digitalWrite(MS_LED7, LOW);*/
    digitalWrite(MS_LED1, 1);
    digitalWrite(MS_LED2, 0);
    digitalWrite(MS_LED3, 1);
    digitalWrite(MS_LED4, 1);
    digitalWrite(MS_LED5, 1);
    digitalWrite(MS_LED6, 1);
    digitalWrite(MS_LED7, 1);
    break;
  case 2:
   /* digitalWrite(MS_LED1, LOW);
    digitalWrite(MS_LED2, LOW);
    digitalWrite(MS_LED3, HIGH);
    digitalWrite(MS_LED4, LOW);
    digitalWrite(MS_LED5, LOW);
    digitalWrite(MS_LED6, LOW);
    digitalWrite(MS_LED7, LOW);*/
    digitalWrite(MS_LED1, 1);
    digitalWrite(MS_LED2, 1);
    digitalWrite(MS_LED3, 0);
    digitalWrite(MS_LED4, 1);
    digitalWrite(MS_LED5, 1);
    digitalWrite(MS_LED6, 1);
    digitalWrite(MS_LED7, 1);
    break;
  case 3:
    /*digitalWrite(MS_LED1, LOW);
    digitalWrite(MS_LED2, LOW);
    digitalWrite(MS_LED3, LOW);
    digitalWrite(MS_LED4, HIGH);
    digitalWrite(MS_LED5, LOW);
    digitalWrite(MS_LED6, LOW);
    digitalWrite(MS_LED7, LOW);*/
    digitalWrite(MS_LED1, 1);
    digitalWrite(MS_LED2, 1);
    digitalWrite(MS_LED3, 1);
    digitalWrite(MS_LED4, 0);
    digitalWrite(MS_LED5, 1);
    digitalWrite(MS_LED6, 1);
    digitalWrite(MS_LED7, 1);
    break;
  case 4:
    /*digitalWrite(MS_LED1, LOW);
    digitalWrite(MS_LED2, LOW);
    digitalWrite(MS_LED3, LOW);
    digitalWrite(MS_LED4, LOW);
    digitalWrite(MS_LED5, HIGH);
    digitalWrite(MS_LED6, LOW);
    digitalWrite(MS_LED7, LOW);*/
    digitalWrite(MS_LED1, 1);
    digitalWrite(MS_LED2, 1);
    digitalWrite(MS_LED3, 1);
    digitalWrite(MS_LED4, 1);
    digitalWrite(MS_LED5, 0);
    digitalWrite(MS_LED6, 1);
    digitalWrite(MS_LED7, 1);
    break;
  case 5:
    /*digitalWrite(MS_LED1, LOW);
    digitalWrite(MS_LED2, LOW);
    digitalWrite(MS_LED3, LOW);
    digitalWrite(MS_LED4, LOW);
    digitalWrite(MS_LED5, LOW);
    digitalWrite(MS_LED6, HIGH);
    digitalWrite(MS_LED7, LOW);*/
    digitalWrite(MS_LED1, 1);
    digitalWrite(MS_LED2, 1);
    digitalWrite(MS_LED3, 1);
    digitalWrite(MS_LED4, 1);
    digitalWrite(MS_LED5, 1);
    digitalWrite(MS_LED6, 0);
    digitalWrite(MS_LED7, 1);
    break;
  case 6:
    /*digitalWrite(MS_LED1, LOW);
    digitalWrite(MS_LED2, LOW);
    digitalWrite(MS_LED3, LOW);
    digitalWrite(MS_LED4, LOW);
    digitalWrite(MS_LED5, LOW);
    digitalWrite(MS_LED6, LOW);
    digitalWrite(MS_LED7, HIGH);*/
    digitalWrite(MS_LED1, 1);
    digitalWrite(MS_LED2, 1);
    digitalWrite(MS_LED3, 1);
    digitalWrite(MS_LED4, 1);
    digitalWrite(MS_LED5, 1);
    digitalWrite(MS_LED6, 1);
    digitalWrite(MS_LED7, 0);
    break; 
  default:
    /*digitalWrite(MS_LED1, LOW);
    digitalWrite(MS_LED2, LOW);
    digitalWrite(MS_LED3, LOW);
    digitalWrite(MS_LED4, LOW);
    digitalWrite(MS_LED5, LOW);
    digitalWrite(MS_LED6, LOW);
    digitalWrite(MS_LED7, LOW);*/
    digitalWrite(MS_LED1, 1);
    digitalWrite(MS_LED2, 1);
    digitalWrite(MS_LED3, 1);
    digitalWrite(MS_LED4, 1);
    digitalWrite(MS_LED5, 1);
    digitalWrite(MS_LED6, 1);
    digitalWrite(MS_LED7, 1);
    break;
  }

}


