#include <Arduino.h>
#include "definitions.h"
#include "ASSI.h"
#include "CAN.h"
#include "Watchdog_t4.h"
#include "autonomous_temporary.h"
#include "InternalTemperature.h"

#define Pressure_readings_enable 1
#define SERIAL_DEBUG 0
#define RESET_TIMEOUT 1000  // ms to hold button for reset

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


float voltage_b = 0;
uint8_t ignition_signal_p = 0;
unsigned long watchdog_time = 0;
volatile uint8_t ignition_signal = 0, ignition_signal_flag = 0;
volatile int start_signal = 0;
volatile int status_ASSI = 0;
volatile uint8_t mission = 0, mission_flag = 0;
unsigned long mission_debounce = 0;
unsigned long mission_update = 0;
unsigned long HeartBit = 0;
volatile uint8_t ASMS_SIGNAL = 0;

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

uint8_t emergency_flag = 0; // Flag to indicate emergency state

void wdtCallback()
{
  //digitalWrite(Debug_LED6, HIGH);
}

void send_can_msg();

WDT_timings_t config;
volatile int ign_en = 0; 

void setup()
{
  ignition_signal = 0;
  peripheral_init();
  // At the beginning of setup
  uint8_t resetReason = 0;
  if (CrashReport) {
    resetReason = 0x10;  // Crash-induced reset
    Serial2.println("System recovered from crash!");
  } else {
    resetReason = 0x20;  // Normal power-on reset
    Serial2.println("System boot normally!");
  }
  Serial2.println("System boot - Reset reason: " + String(resetReason));
  

  config.trigger = 1;            /* in seconds, 0->128 Warning trigger before timeout */
  config.timeout = 2;            /* in seconds, 0->128 Timeout to reset */
  config.callback = wdtCallback; // Callback function to be called on timeout

  
  
  CAN_init();
  ASSI(status_ASSI);
  
  while (digitalRead(IGN_PIN) == 1)
  {
    wdt_software.feed();
    Serial2.println("Waiting for ignition signal");
    delay(100);

  }

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
      //digitalWrite(SOLENOID1,!digitalRead(SOLENOID1));
      HeartBit = millis();
     // digitalWrite(Debug_LED2, !digitalRead(Debug_LED2));
    }
    ign_en = 0;
#if SERIAL_DEBUG
    if (DEBUG_TIME + 100 <= millis())
    {

      sendJson();

      DEBUG_TIME = millis();
    }
#endif

  } while (Received_CAN_MSG.id != RES_ID && digitalRead(ASMS) == 0);  // switch to || fo vsv
ign_en = 1;
  //uint8_t sg[] = {0x00};
  //CAN_MSG_SEND(0x00,1, sg); // Send a dummy message to clear the bus
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
      mission_update = millis();
      //digitalWrite(Debug_LED5, !digitalRead(Debug_LED5));
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
      //digitalWrite(Debug_LED3, !digitalRead(Debug_LED3));
      HeartBit = millis();
    }
    if (digitalRead(IGN_PIN) == 1)
    {
      ignition_signal = 1;
      //digitalWrite(Debug_LED2, HIGH);
    }
    else
    {
      ignition_signal = 0;
      //digitalWrite(Debug_LED2, LOW);
    }
  }

  detachInterrupt(digitalPinToInterrupt(MS_BUTTON1));
  if (HeartBit + 2000 <= millis())
  {
    digitalWrite(HB_LED, !digitalRead(HB_LED));
    //digitalWrite(Debug_LED3, !digitalRead(Debug_LED3));
    HeartBit = millis();
  }
  uint8_t ignition_data[1] = {2};
  CAN_MSG_SEND(IGN_TO_ACU, 1, ignition_data);

  wdt_software.feed();
  reset_debug_leds();

  // detachInterrupt(digitalPinToInterrupt(IGN));
  //digitalWrite(Debug_LED4, HIGH);
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

    digitalWrite(HB_LED, !digitalRead(HB_LED));

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

    status_ASSI = Received_CAN_MSG.buf[0];
  }
  else{
    if(Received_CAN_MSG.id == RES_ID){
      if(Received_CAN_MSG.buf[0] == AUTONOMOUS_TEMPORARY_RES_SIGNAL_EMERGENCY_CHOICE)
      status_ASSI =  4; // Emergency
      emergency_flag = 0; // Reset emergency flag
      digitalWrite(Debug_LED4, HIGH); // Turn off emergency LED
      
    }
  }

  if (mission_ign_update + 100 <= millis())
  {
    Mission_Select(mission);

    ASSI(status_ASSI);
    
    if (status_ASSI == 4)
    {
      mission_ign_update = millis();
      emergency_flag = 1; // Set emergency flag
      digitalWrite(Debug_LED4, HIGH); // Turn on emergency LED
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

  pinMode(SOLENOID1, OUTPUT);
  pinMode(SOLENOID2, OUTPUT);

  Serial2.begin(115200);
  Serial.begin(115200);

#if SERIAL_DEBUG

#endif

// CAN_TIMER.begin(send_can_msg,200000);  // 200ms // tempo em us
#if Pressure_readings_enable
  PRESSURE_TIMER.begin(Pressure_readings, 100000); // 100ms
#endif
 CAN_TO_VCU.begin(send_can_msg, 100000);      // 100ms
  // CAN_TO_VCU.begin(send_can_msg, 200000);  // 200ms
  // CAN_TO_VCU.begin(send_can_msg, 500000);  // 500ms
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
  if (digitalRead(IGN_PIN) == 1 && ign_en == 1)
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

void Pressure_readings() {
  EBS_TANK_PRESSURE_A_values[pointer] = analogRead(EBS_TANK_PRESSURE_A);
  EBS_TANK_PRESSURE_B_values[pointer] = analogRead(EBS_TANK_PRESSURE_B);
  pointer++;
  if (pointer >= PRESSURE_READINGS) {
    pointer = 0;
  }
}
#endif

void median_pressures() {
#if Pressure_readings_enable
  if (pressure_time + 300 <= millis()) { 
    pressure_time = millis();
    float chipTemp = InternalTemperature.readTemperatureC();
    Serial2.println("Chip temperature: " + String(chipTemp) + " C");
    // Calculate averages from the array
    float sum = 0;
    for (int i = 0; i < PRESSURE_READINGS; i++) {
      sum += EBS_TANK_PRESSURE_B_values[i];
    }
    EBS_TANK_PRESSURE_B_value = sum / PRESSURE_READINGS;
    
    // Store raw voltage for debugging (convert ADC to voltage)
    float rawVoltage = EBS_TANK_PRESSURE_B_value * 3.3 / 1023; // Read raw voltage from the analog pin
    
    // Use corrected divider value (0.85 instead of 0.66) prev val 0.476
    float actualVoltage = rawVoltage / 0.66;
    
    // Apply formula ONCE with corrected divider
    float pressure = (actualVoltage - 0.5) / 0.4;
    
    Serial2.println("Raw ADC: " + String(EBS_TANK_PRESSURE_B_value));
    Serial2.println("Raw voltage: " + String(rawVoltage));
    Serial2.println("Actual voltage: " + String(actualVoltage));
    Serial2.println("Pressure: " + String(pressure) + " bar");

    if(pressure < TANK_PRESSURE_THRESHOLD){
      digitalWrite(SOLENOID1,HIGH);
      digitalWrite(SOLENOID2,HIGH);
      ignition_signal_p = 0; // Turn off ignition if pressure is below threshold
      emergency_flag = 1; // Set emergency flag
    }
    else{
      digitalWrite(SOLENOID1,LOW);
      digitalWrite(SOLENOID2,LOW);
      ignition_signal_p = 1; // Turn on ignition if pressure is above threshold
    }
    digitalWrite(Debug_LED5, !ignition_signal_p); // Turn off debug LED after reading pressure
    EBS_TANK_PRESSURE_B_value = pressure; // Store the pressure value for further use
  }
#endif
}

void performSystemReset(uint8_t reason) {
  Serial2.println("PERFORMING SYSTEM RESET - Reason: " + String(reason));
  

  
  // 2. Send reset notification over CAN
  uint8_t reset_data[1] = {};
  CAN_MSG_SEND(VCU_IGN ,1, reset_data);  // Use appropriate ID
  
  // 3. Safety shutdown - clean state0
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
      while (EBS_TANK_PRESSURE_B_value < 0.5){
        wdt_software.feed();  // Feed watchdog to prevent reset during pressure check
        Serial2.println("wainting forpressure under 0.5 bar");
      }
      
      performSystemReset(1);  // Reason 1: Manual button reset
    }
  } 
  else {
    reset_button_press_time = 0;
    reset_in_progress = false;
  }
  
  // Method 2: CAN command reset
  if (Received_CAN_MSG.id == IGN_FROM_VCU && status_ASSI != 4){  // Choose appropriate ID
    if (Received_CAN_MSG.buf[0] == 0x00) {
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

void send_can_msg() {
  // Debounce the ignition pin reading
  static uint8_t ignition_readings[3] = {0, 0, 0};
  static uint8_t reading_index = 0;
  
  // Add new reading to the buffer
  ignition_readings[reading_index] = digitalRead(IGN_PIN);
  reading_index = (reading_index + 1) % 3;
  
  // Only change state if all readings agree
  uint8_t stable_reading = (ignition_readings[0] == ignition_readings[1] && 
                           ignition_readings[1] == ignition_readings[2]) ? 
                           ignition_readings[0] : ignition_signal;
  
  // Update global state if it has stabilized
  if (stable_reading != ignition_signal) {
    ignition_signal = stable_reading;
  }
  
  // Prepare message data
  uint8_t ignition_data[3] = {0,ASMS_SIGNAL,emergency_flag};
  
  // Apply the same logic as before
  if (!ignition_signal || ign_en == 0 || emergency_flag == 1) {
    ignition_data[0] = 0;
  } else {
    ignition_data[0] = ignition_signal;
  }
  
  // Send ignition message
  CAN_MSG_SEND(VCU_IGN, 3, ignition_data);
  
  // Also periodically send mission status
  uint8_t mission_data[1] = {mission_flag};
  CAN_MSG_SEND(ACU_MS, 1, mission_data);
}