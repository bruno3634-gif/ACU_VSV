#include <Arduino.h>
#include "definitions.h"
#include "ASSI.h"
#include "CAN.h"
#include "Watchdog_t4.h"
#include "autonomous_temporary.h"

#define Pressure_readings_enable 1
#define SERIAL_DEBUG 0
#define RESET_TIMEOUT 3000  // ms to hold button for reset

unsigned long canPrint_Millis=0;

void peripheral_init();
void MS_INT();
void IGN_INT();
void reset_debug_leds();
void checkForResetRequest();
void sendJson();
#if Pressure_readings_enable
void Pressure_readings();
void median_pressures();
#endif

unsigned long watchdog_time = 0;
volatile uint8_t ignition_signal = 0, ignition_signal_flag = 0;
volatile int start_signal = 0;
volatile int status_ASSI = 0;
volatile uint8_t mission = 0, mission_flag = 0;
unsigned long mission_debounce = 0;
unsigned long mission_update = 0;
unsigned long HeartBit = 0;
volatile int ASMS_SIGNAL = 0;

#if Pressure_readings_enable
float EBS_TANK_PRESSURE_A_value = 0, EBS_TANK_PRESSURE_B_value = 0;
#define PRESSURE_READINGS 8
float EBS_TANK_PRESSURE_A_values[PRESSURE_READINGS], EBS_TANK_PRESSURE_B_values[PRESSURE_READINGS];
IntervalTimer PRESSURE_TIMER;

IntervalTimer CAN_TO_VCU;

int pointer = 0;
unsigned long pressure_time = 0;
#endif

unsigned long DEBUG_TIME = 0;

unsigned long wdt_time_update = 0;

unsigned long mission_ign_update;

CAN_message_t Received_CAN_MSG;
WDT_T4<WDT1> wdt_software;

unsigned long reset_button_press_time = 0;
bool reset_in_progress = false;

void wdtCallback()
{
  digitalWrite(Debug_LED6, HIGH);
}

void send_can_msg();

WDT_timings_t config;

void setup()
{
  peripheral_init();
  // At the beginning of setup
  uint8_t resetReason = 0;
  if (CrashReport) {
    resetReason = 0x10;  // Crash-induced reset
    Serial.println("System recovered from crash!");
  } else {
    resetReason = 0x20;  // Normal power-on reset
    Serial.println("System boot normally!");
  }
  Serial.println("System boot - Reset reason: " + String(resetReason));
  
  unsigned long wdt_hardware_time = 0;

  config.trigger = 1;            /* in seconds, 0->128 Warning trigger before timeout */
  config.timeout = 2;            /* in seconds, 0->128 Timeout to reset */
  config.callback = wdtCallback; // Callback function to be called on timeout

  
  
  CAN_init();
  ASSI(status_ASSI);
  /*
  while (digitalRead(IGN_PIN) == 1)
  {
    wdt_software.feed();
    Serial2.println("Waiting for ignition signal");
    delay(100);

  }
*/
  wdt_software.begin(config);  
  wdt_software.feed();                               
  // wait for res
  do
  {
    if(CrashReport)
    {
      Serial.print(CrashReport);
    }
    Serial.println("Waiting for RES");


    uint8_t mission_data[1] = {1};
      CAN_MSG_SEND(0x355, 1, mission_data);


    ASSI(status_ASSI);
    wdt_software.feed();
#if Pressure_readings_enable
    median_pressures();
#endif
/*if (wdt_hardware_time + 10 <= millis())
    {
      Serial2.println(millis()-wdt_hardware_time);
      digitalWrite(WDT, !digitalRead(WDT));
      wdt_hardware_time = millis();
      Serial2.println("WDT");
    }*/
    Received_CAN_MSG = CAN_MSG_RECEIVE();


    mission = 0;
    Mission_Select(mission);
    if (HeartBit + 500 <= millis())
    {
      digitalWrite(HB_LED, !digitalRead(HB_LED));
      HeartBit = millis();
      digitalWrite(Debug_LED2, !digitalRead(Debug_LED2));
    }
#if SERIAL_DEBUG
    if (DEBUG_TIME + 100 <= millis())
    {

      sendJson();

      DEBUG_TIME = millis();
    }
#endif

  } while (Received_CAN_MSG.id != RES_ID);

  wdt_software.feed();
  reset_debug_leds();
  wdt_software.feed();
  ASMS_SIGNAL = 1;

  attachInterrupt(digitalPinToInterrupt(MS_BUTTON1), MS_INT, FALLING);
  // attachInterrupt(digitalPinToInterrupt(IGN), IGN_INT, CHANGE);

  //  Waiting for IGNITION SIGNAL
  while (ignition_signal_flag == 0 || ignition_signal == 0)
  {
    ASSI(status_ASSI);
    wdt_software.feed();
    Received_CAN_MSG = CAN_MSG_RECEIVE();
    if (Received_CAN_MSG.id == IGN_FROM_VCU)
    {
      ignition_signal_flag = Received_CAN_MSG.buf[0];
    }
#if Pressure_readings_enable
    median_pressures();
#endif
    if (mission_update + 100 <= millis())
    {
      uint8_t mission_data[1] = {mission_flag};
      CAN_MSG_SEND(ACU_MS, 1, mission_data);
      mission_update = millis();
      digitalWrite(Debug_LED5, !digitalRead(Debug_LED5));
      uint8_t ignition_data[1] = {ignition_signal};
      CAN_MSG_SEND(VCU_IGN, 1, ignition_data);

#if SERIAL_DEBUG
      if (DEBUG_TIME + 100 <= millis())
      {
        sendJson();

        DEBUG_TIME = millis();
      }
#endif
    }

    // Received_CAN_MSG = CAN_MSG_RECEIVE();
    if (Received_CAN_MSG.id == JETSON_MS)
    {
      mission = Received_CAN_MSG.buf[0];
      Mission_Select(mission);
    }

    if (HeartBit + 500 <= millis())
    {
      digitalWrite(HB_LED, !digitalRead(HB_LED));
      digitalWrite(Debug_LED3, !digitalRead(Debug_LED3));
      HeartBit = millis();
    }
    if (digitalRead(IGN_PIN) == 1)
    {
      ignition_signal = 1;
      digitalWrite(Debug_LED2, HIGH);
    }
    else
    {
      ignition_signal = 0;
      digitalWrite(Debug_LED2, LOW);
    }
  }

  detachInterrupt(digitalPinToInterrupt(MS_BUTTON1));
  if (HeartBit + 2000 <= millis())
  {
    digitalWrite(HB_LED, !digitalRead(HB_LED));
    digitalWrite(Debug_LED3, !digitalRead(Debug_LED3));
    HeartBit = millis();
  }
  uint8_t ignition_data[1] = {2};
  CAN_MSG_SEND(IGN_TO_ACU, 1, ignition_data);

  wdt_software.feed();
  reset_debug_leds();

  // detachInterrupt(digitalPinToInterrupt(IGN));
  digitalWrite(Debug_LED4, HIGH);
}

void loop()
{
  checkForResetRequest();
  wdt_software.feed();
#if Pressure_readings_enable
  median_pressures();
#endif
  if (HeartBit + 500 <= millis())
  {
    //Serial2.println("Status: " + String(status_ASSI));
    digitalWrite(HB_LED, !digitalRead(HB_LED));
    digitalWrite(Debug_LED4, !digitalRead(Debug_LED4));
    digitalWrite(Debug_LED6, !digitalRead(Debug_LED4));
    HeartBit = millis();
    uint8_t dummy_data[1] = {1};
    CAN_MSG_SEND(0x99, 1, dummy_data);
  }
  Received_CAN_MSG = CAN_MSG_RECEIVE();

  if(canPrint_Millis + 100 <= millis())
  {
    Serial.println("ID:" + String(Received_CAN_MSG.id) + " Data: " + String(Received_CAN_MSG.buf[0]));
    canPrint_Millis = millis();
  }

  if (Received_CAN_MSG.id == JETSON_AMS)
  {
    //Serial2.println(Received_CAN_MSG.id);
    //Serial2.println(Received_CAN_MSG.buf[0]);
    status_ASSI = Received_CAN_MSG.buf[0];
  }

  if (mission_ign_update + 100 <= millis())
  {
    Mission_Select(mission);

    ASSI(status_ASSI);
    
    if (status_ASSI == 4)
    {
      uint8_t ignition_data[1] = {0};
      CAN_MSG_SEND(VCU_IGN, 1, ignition_data);
      mission_ign_update = millis();
    }
  }

#if SERIAL_DEBUG
  if (DEBUG_TIME + 100 <= millis())
  {
    void sendJson();

    DEBUG_TIME = millis();
  }
#endif
}

void peripheral_init()
{
  pinMode(YELLOW_LEDS, OUTPUT);
  pinMode(BLUE_LEDS, OUTPUT);

  pinMode(MS_BUTTON1, INPUT_PULLUP);

  pinMode(MS_LED1, OUTPUT);
  pinMode(MS_LED2, OUTPUT);
  pinMode(MS_LED3, OUTPUT);
  pinMode(MS_LED4, OUTPUT);
  pinMode(MS_LED5, OUTPUT);
  pinMode(MS_LED6, OUTPUT);
  pinMode(MS_LED7, OUTPUT);
  digitalWrite(MS_LED1, 1);
  digitalWrite(MS_LED2, 1);
  digitalWrite(MS_LED3, 1);
  digitalWrite(MS_LED4, 1);
  digitalWrite(MS_LED5, 1);
  digitalWrite(MS_LED6, 1);
  digitalWrite(MS_LED7, 1);
  pinMode(AS_SW, INPUT);

  pinMode(HB_LED, OUTPUT);
  pinMode(Debug_LED2, OUTPUT);
  pinMode(Debug_LED3, OUTPUT);
  pinMode(Debug_LED4, OUTPUT);
  pinMode(Debug_LED5, OUTPUT);
  pinMode(Debug_LED6, OUTPUT);

  pinMode(EBS_TANK_PRESSURE_A, INPUT);
  pinMode(EBS_TANK_PRESSURE_B, INPUT);
  pinMode(EBS_VALLVE_A, INPUT);
  pinMode(EBS_VALLVE_B, INPUT);

  pinMode(R2D_PIN, INPUT);

  // pinMode(LED_PIN, OUTPUT);
  pinMode(WDT, OUTPUT);

  pinMode(ASMS, INPUT);
  pinMode(IGN_PIN, INPUT);

  Serial2.begin(115200);
  Serial.begin(115200);

#if SERIAL_DEBUG

#endif

// CAN_TIMER.begin(send_can_msg,200000);  // 200ms // tempo em us
#if Pressure_readings_enable
  PRESSURE_TIMER.begin(Pressure_readings, 100000); // 100ms
#endif
 CAN_TO_VCU.begin(send_can_msg, 100000); // 100ms
  // CAN_TO_VCU.begin(send_can_msg, 200000); // 200ms
  // CAN_TO_VCU.begin(send_can_msg, 500000); // 500ms
  // CAN_TO_VCU.begin(send_can_msg, 1000000); // 1s   
}

void MS_INT()
{
  mission_debounce = millis();
  while (millis() - mission_debounce < 150)
  {
   /* if (watchdog_time + 10 <= millis())
    {
      watchdog_time = millis();
      digitalWrite(WDT, !digitalRead(WDT));
      watchdog_time = millis();
    }*/
  }
  if (digitalRead(MS_BUTTON1) == 0)
  {
    mission_flag++;
    if (mission_flag > 6)
    {
      mission_flag = 0;
    }
  }
}

void IGN_INT()
{
  unsigned long IGN_debounce = millis();
  while (millis() - IGN_debounce < 200)
  {
    /*if (watchdog_time + 10 <= millis())
    {
      watchdog_time = millis();
      digitalWrite(WDT, !digitalRead(WDT));
      watchdog_time = millis();
    }*/
  }
  if (digitalRead(IGN_PIN) == 1)
  {
    ignition_signal = 1;
  }
  else
  {
    ignition_signal = 0;
  }
}

/**
 * @brief Reset all debug leds
 */
void reset_debug_leds()
{
  digitalWrite(Debug_LED2, LOW);
  digitalWrite(Debug_LED3, LOW);
  digitalWrite(Debug_LED4, LOW);
  digitalWrite(Debug_LED5, LOW);
  digitalWrite(Debug_LED6, LOW);
}

#if Pressure_readings_enable

void Pressure_readings()
{
  EBS_TANK_PRESSURE_A_values[pointer] = analogRead(EBS_TANK_PRESSURE_A) * 3.3 / 1023 * 2.179;
  EBS_TANK_PRESSURE_B_values[pointer] = analogRead(EBS_TANK_PRESSURE_B) * 3.3 / 1023 * 2.179;
  pointer++;
  if (pointer >= PRESSURE_READINGS)
  {
    pointer = 0;
  }
}
#endif

void median_pressures()
{

#if Pressure_readings_enable
  if (pressure_time + 100 <= millis())
  {
    pressure_time = millis();
    EBS_TANK_PRESSURE_A_value = 0;
    EBS_TANK_PRESSURE_B_value = 0;
    for (int i = 0; i < PRESSURE_READINGS; i++)
    {
      EBS_TANK_PRESSURE_A_value += EBS_TANK_PRESSURE_A_values[i];
      EBS_TANK_PRESSURE_B_value += EBS_TANK_PRESSURE_B_values[i];
    }
    EBS_TANK_PRESSURE_A_value = EBS_TANK_PRESSURE_A_value / PRESSURE_READINGS;
    EBS_TANK_PRESSURE_B_value = EBS_TANK_PRESSURE_B_value / PRESSURE_READINGS;
    EBS_TANK_PRESSURE_A_value = 0.280851064 * EBS_TANK_PRESSURE_A_value - 0.351063830;
    EBS_TANK_PRESSURE_B_value = 0.280851064 * EBS_TANK_PRESSURE_B_value + 0.351063830;
  }
#endif
}

void performSystemReset(uint8_t reason) {
  Serial2.println("PERFORMING SYSTEM RESET - Reason: " + String(reason));
  
  // 1. Visual indication of reset
  for (int i = 0; i < 5; i++) {
    digitalWrite(HB_LED, HIGH);
    digitalWrite(Debug_LED2, HIGH);
    digitalWrite(Debug_LED3, HIGH);
    digitalWrite(Debug_LED4, HIGH);
    digitalWrite(Debug_LED5, HIGH);
    digitalWrite(Debug_LED6, HIGH);
    delay(50);
    digitalWrite(HB_LED, LOW);
    digitalWrite(Debug_LED2, LOW);
    digitalWrite(Debug_LED3, LOW);
    digitalWrite(Debug_LED4, LOW);
    digitalWrite(Debug_LED5, LOW);
    digitalWrite(Debug_LED6, LOW);
    delay(50);
  }
  
  // 2. Send reset notification over CAN
  uint8_t reset_data[2] = {0xAA, reason};
  CAN_MSG_SEND(0x501, 2, reset_data);  // Use appropriate ID
  
  // 3. Safety shutdown - clean state
  ignition_signal = 0;
  ignition_signal_flag = 0;
  ASMS_SIGNAL = 0;
  status_ASSI = 0;
  
  // 4. Final delay to allow CAN messages to send
  delay(100);
  
  // 5. Perform watchdog reset (safer than direct SCB_AIRCR)
  wdt_software.reset();
  
  // 6. If watchdog fails, fall back to system reset
  delay(100);
  SCB_AIRCR = 0x05FA0004;
}

void checkForResetRequest() {
  // Method 1: IGN button long press
  if (digitalRead(IGN_PIN) == 0) {  // Corrected syntax
    if (reset_button_press_time == 0) {
      reset_button_press_time = millis();
    } 
    else if (!reset_in_progress && (millis() - reset_button_press_time > RESET_TIMEOUT)) {
      reset_in_progress = true;
      performSystemReset(1);  // Reason 1: Manual button reset
    }
  } 
  else {
    reset_button_press_time = 0;
    reset_in_progress = false;
  }
  
  // Method 2: CAN command reset
  if (Received_CAN_MSG.id == 0x505) {  // Choose appropriate ID
    if (Received_CAN_MSG.buf[0] == 0x55 && Received_CAN_MSG.buf[1] == 0xAA) {
      performSystemReset(2);  // Reason 2: Remote CAN reset
    }
  }
}

void sendJson()
{
  // Start building the JSON string
  String json = "{";
#if Pressure_readings_enable
  json += "\"PB\": " + String(EBS_TANK_PRESSURE_B_value) + ", ";
  json += "\"PA\": " + String(EBS_TANK_PRESSURE_A_value) + ", ";
#endif
  json += "\"IGN\": " + String(ignition_signal && ignition_signal_flag ? "true" : "false") + ", ";
  json += "\"MISSION\": " + String(mission_flag) + ", ";
  json += "\"ASSI_STATUS\": " + String(status_ASSI) + ", ";
  json += "\"ASMS\": " + String(ASMS_SIGNAL) + ", ";
  json += "\"GO_SIGNAL\": " + String(start_signal ? "true" : "false");
  json += "}";

  // Send the JSON string over Serial2
  Serial2.println(json); // Send the JSON message
}

void send_can_msg(){
  
}