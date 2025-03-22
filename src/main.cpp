#include <Arduino.h>
#include "definitions.h"
#include "ASSI.h"
#include "CAN.h"
#include "Watchdog_t4.h"


#define Pressure_readings_enable 1
#define SERIAL_DEBUG 1



void peripheral_init();
void MS_INT();
void IGN_INT();
void reset_debug_leds();
void desigintion_temporary();
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
int pointer = 0;
unsigned long pressure_time = 0;
#endif

unsigned long DEBUG_TIME = 0;

unsigned long wdt_time_update = 0;

unsigned long mission_ign_update;

CAN_message_t Received_CAN_MSG;
WDT_T4<WDT1> wdt_software;

void wdtCallback()
{
  digitalWrite(Debug_LED6, HIGH);
}

WDT_timings_t config;

void setup()
{
  unsigned long wdt_hardware_time = 0;

  config.trigger = 1;            /* in seconds, 0->128 Warning trigger before timeout */
  config.timeout = 2;            /* in seconds, 0->128 Timeout to reset */
  config.callback = wdtCallback; // Callback function to be called on timeout

  peripheral_init();

  CAN_init();
  ASSI(status_ASSI);
  while (digitalRead(IGN_PIN) == 1)
  {
    delay(50);
  }
  
  wdt_software.begin(config);
  // wait for res
  do
  {
    ASSI(status_ASSI);
    wdt_software.feed();
#if Pressure_readings_enable
    median_pressures();
#endif
    Received_CAN_MSG = CAN_MSG_RECEIVE();

    if(wdt_hardware_time + 10 <= millis()){
      digitalWrite(WDT, !digitalRead(WDT));
      wdt_hardware_time = millis();
    }
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

  desigintion_temporary();
  wdt_software.feed();
#if Pressure_readings_enable
  median_pressures();
#endif
  if (HeartBit + 500 <= millis())
  {
    digitalWrite(HB_LED, !digitalRead(HB_LED));
    digitalWrite(Debug_LED4, !digitalRead(Debug_LED4));
    digitalWrite(Debug_LED6, !digitalRead(Debug_LED4));
    HeartBit = millis();
    uint8_t dummy_data[1] = {1};
    CAN_MSG_SEND(0x99, 1, dummy_data);
  }
  Received_CAN_MSG = CAN_MSG_RECEIVE();
 
  if (Received_CAN_MSG.id == 0x502)
  {
    Serial2.println(Received_CAN_MSG.id);
    Serial2.println(Received_CAN_MSG.buf[0]);
  }
  if (Received_CAN_MSG.id == JETSON_AMS)
  {
    status_ASSI = Received_CAN_MSG.buf[0];
  }
  Mission_Select(mission);
  ASSI(status_ASSI);
  if(mission_ign_update + 100 <= millis()){
    if(status_ASSI == 4){
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

#if SERIAL_DEBUG
  
#endif

// CAN_TIMER.begin(send_can_msg,200000);  // 200ms // tempo em us
#if Pressure_readings_enable
  PRESSURE_TIMER.begin(Pressure_readings, 100000); // 100ms
#endif
}



void MS_INT()
{
  mission_debounce = millis();
  while (millis() - mission_debounce < 150)
  {
    if (watchdog_time + 10 <= millis())
    {
      watchdog_time = millis();
      digitalWrite(WDT, !digitalRead(WDT));
      watchdog_time = millis();
    }
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
    if (watchdog_time + 10 <= millis())
    {
      watchdog_time = millis();
      digitalWrite(WDT, !digitalRead(WDT));
      watchdog_time = millis();
    }
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

void desigintion_temporary()
{
  if (digitalRead(IGN_PIN == 0))
  {
    delay(300);
    if (digitalRead(IGN_PIN == 0))
    {
      SCB_AIRCR = 0x05FA0004; // Software reset
      digitalWrite(Debug_LED2, HIGH);
    }
  }
}







void sendJson() {
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