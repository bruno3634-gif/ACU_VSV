#if PCB == 1


#define WDT 29
#define EBS_TANK_PRESSURE_A A13
#define EBS_TANK_PRESSURE_B A12 
#define EBS_VALLVE_A A8
#define EBS_VALLVE_B 37
#define SDC_FEEDBACK A11



#define IGN_PIN 34
#define R2D_PIN 35


// Mission Select

#define MS_BUTTON1 21
#define MS_LED1 14
#define MS_LED2 15
#define MS_LED3 16
#define MS_LED4 17  
#define MS_LED5 33
#define MS_LED6 13
#define MS_LED7 20

// ASSI

#define YELLOW_LEDS 24
#define BLUE_LEDS 23

// Solenoides

#define SOLENOID1 36
#define SOLENOID2 10


// Debug Leds

#define Debug_LED1 28
#define Debug_LED2 29
#define Debug_LED3 34
#define Debug_LED4 35
#define Debug_LED5 30
#define Debug_LED6 31

#else


#define WDT 39
#define EBS_TANK_PRESSURE_A A13
#define EBS_TANK_PRESSURE_B A12 
#define EBS_VALLVE_A A8
#define EBS_VALLVE_B 37
#define SDC_FEEDBACK A11



//#define IGN_PIN 34
#define R2D_PIN 10


// Mission Select

#define MS_BUTTON1 21
#define MS_LED1 14
#define MS_LED2 15
#define MS_LED3 16
#define MS_LED4 17  
#define MS_LED5 33
#define MS_LED6 13
#define MS_LED7 20
#define AS_SW 38
// ASSI

//#define YELLOW_LEDS 24
//#define BLUE_LEDS 23
#define YELLOW_LEDS 24
#define BLUE_LEDS 23

// Solenoides

#define SOLENOID1 36
#define SOLENOID2 10


// Debug Leds

#define HB_LED 28
#define Debug_LED2 29
#define Debug_LED3 34
#define Debug_LED4 35
#define Debug_LED5 30
#define Debug_LED6 31



#endif

#define TANK_PRESSURE_THRESHOLD 100

#define TANKS_INDEX_SIZE 6

#define ASMS 38
#define IGN_PIN 4


#define RES_ID 0x191
#define ACU_MS 0x51
#define JETSON_MS 0x61
#define VCU_IGN 0x71        // -> Envio para te dizer para abrir os contactores  1º byte -> 0 -> Ignição desligada 1 -> Ignição ligada
#define JETSON_AMS 0x502
#define IGN_FROM_VCU 0x81   // -> Envias para me dizer que os contactores estão fechados 1º byte -> 0 -> Contactores abertos 1 -> Contactores fechados
#define IGN_TO_ACU 0x512     