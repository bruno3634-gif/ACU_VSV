#include <Arduino.h>
#include "ASSI.h"



volatile int ASSI_status = 1;
unsigned long ASSI_YELLOW_time = 0, ASSI_BLUE_time = 0;
 

#define NORMAL_LEDS 0


#if not NORMAL_LEDS
#include <Adafruit_NeoPixel.h>
#define PIN        YELLOW_LEDS // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 48 // Popular NeoPixel ring size  
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#endif
int yellow = 0;
int blue = 0;

int ASSI(int ASSI_status){
  
  switch (ASSI_status)
  {
  case 1:
  #if NORMAL_LEDS
    //digitalWrite(YELLOW_LEDS, LOW);
    //digitalWrite(BLUE_LEDS, LOW);
    digitalWrite(YELLOW_LEDS, HIGH);
    digitalWrite(BLUE_LEDS, LOW);
  #else
    for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      pixels.show();   // Send the updated pixel colors to the hardware. // Pause before next pass through loop
    }
  #endif
    break;
  case 2:
  #if NORMAL_LEDS
    //digitalWrite(YELLOW_LEDS, HIGH);
    //digitalWrite(BLUE_LEDS, LOW);
    digitalWrite(YELLOW_LEDS, LOW);
    digitalWrite(BLUE_LEDS, HIGH);
  #else
    for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
      pixels.setPixelColor(i, pixels.Color(150, 150, 0));
      pixels.show();   // Send the updated pixel colors to the hardware. // Pause before next pass through loop
    }
  #endif
    break;
  case 3:
  #if NORMAL_LEDS
  digitalWrite(BLUE_LEDS, LOW); 
  //digitalWrite(BLUE_LEDS, HIGH);
  if(millis() - ASSI_YELLOW_time  >= 500){
    ASSI_YELLOW_time = millis();
    digitalWrite(YELLOW_LEDS, !digitalRead(YELLOW_LEDS));
  }
  #else
  if(millis() - ASSI_YELLOW_time  >= 500){
    ASSI_YELLOW_time = millis();
    for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
      pixels.setPixelColor(i, pixels.Color(150 * yellow, 150 * yellow, 0));
      pixels.show();   // Send the updated pixel colors to the hardware. // Pause before next pass through loop
    }
    yellow++;
    if(yellow > 1){
      yellow = 0;
    }
  }
  #endif
    break;
    case 4:
    #if NORMAL_LEDS
    //digitalWrite(YELLOW_LEDS, LOW);
    digitalWrite(YELLOW_LEDS, HIGH);
    if (millis() - ASSI_BLUE_time  >= 500)
    {
      ASSI_BLUE_time = millis();
      digitalWrite(BLUE_LEDS, !digitalRead(BLUE_LEDS));
    }    
    #else
    if (millis() - ASSI_BLUE_time  >=
        500)
    {
      ASSI_BLUE_time = millis();
      for (int i = 0; i < NUMPIXELS; i++)
      { // For each pixel...
        pixels.setPixelColor(i, pixels.Color(0, 0, 150 * blue));
        pixels.show(); // Send the updated pixel colors to the hardware. // Pause before next pass through loop
      }
      blue++;
      if (blue > 1)
      {
        blue = 0;
      }
    }
    #endif
    break;
    case 5:
    #if NORMAL_LEDS
    //digitalWrite(YELLOW_LEDS, LOW);
    //digitalWrite(BLUE_LEDS, HIGH);
    digitalWrite(YELLOW_LEDS, HIGH);
    digitalWrite(BLUE_LEDS, LOW);
    #else
    for (int i = 0; i < NUMPIXELS; i++)
    { // For each pixel...
      pixels.setPixelColor(i, pixels.Color(0, 0, 150));
      pixels.show(); // Send the updated pixel colors to the hardware. // Pause before next pass through loop
    }
    #endif
    break;

  default:
  #if NORMAL_LEDS
  //digitalWrite(YELLOW_LEDS, LOW);
  //digitalWrite(BLUE_LEDS, LOW);
  digitalWrite(YELLOW_LEDS, HIGH);
  digitalWrite(BLUE_LEDS, HIGH);
  //Serial.println("ASSI Status not defined");
  #else
  for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    pixels.show();   // Send the updated pixel colors to the hardware. // Pause before next pass through loop
  }
  #endif
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


