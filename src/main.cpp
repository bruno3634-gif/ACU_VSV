#include <Arduino.h>
#include "definitions.h"
#include "ASSI.h"
#include "CAN.h"
#include "Watchdog_t4.h"
#include "autonomous_temporary.h"
#include "InternalTemperature.h"
#include <EEPROM.h>

// Global state machine
StateMachine sm = {STATE_INIT, STATE_INIT, 0, 0, false, EVENT_NONE};

// Your existing global variables
WDT_T4<WDT1> wdt_software;
WDT_T4<WDT1> wdt_seq;
uint8_t emergency_flag = 0;
volatile uint8_t ignition_signal = 0;

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
volatile uint8_t ignition_signal_flag = 0;
volatile int start_signal = 0;
volatile int status_ASSI = 0;
volatile uint8_t mission = 0, mission_flag = 0;
unsigned long mission_update = 0;
unsigned long HeartBit = 0;
volatile uint8_t ASMS_SIGNAL = 0;
volatile uint8_t status_ready = 0;





// Function prototypes
void executeStateInit();
void executeStateWaitingActivation();
void executeStatePressureCheck();
void executeStateOperational();
void executeStateEmergencyEntry();
void executeStateEmergencyActive();
void executeStateDepressurizing();
void executeStateRecoveryReady();
void executeStateFault();
void executeStateMaintenance();
void updatePressureReadings();
void updateIgnitionControl();
bool checkCANHealth();





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

unsigned long reset_button_press_time = 0;
bool reset_in_progress = false;

void wdtCallback()
{
  //digitalWrite(Debug_LED6, HIGH);
}

void send_can_msg();

WDT_timings_t config;
volatile int ign_en = 0; 

void setup() {
  Serial2.begin(115200);
  Serial2.println("ACU_VSV State Machine Starting...");
  
  initializeHardware();
  changeState(STATE_INIT);
}

void loop() {
  updateSystemInputs();
  processSystemEvents();
  executeCurrentState();
  handleStateTransitions();
  handleHeartbeat();
  feedWatchdogs();
  
  delay(10);
}

void changeState(SystemState newState) {
  if (sm.currentState != newState) {
    sm.previousState = sm.currentState;
    sm.currentState = newState;
    sm.stateEntryTime = millis();
    sm.stateChanged = true;
    
    Serial2.println("STATE CHANGE: " + getStateName(sm.previousState) + 
                    " -> " + getStateName(sm.currentState));
    
    onStateEntry(newState);
  }
}

void handleHeartbeat() {
  if (millis() - sm.lastHeartbeat >= 1000) {
    sm.lastHeartbeat = millis();
    
    Serial2.println("HEARTBEAT - State: " + getStateName(sm.currentState) + 
                    " | Uptime: " + String(millis()/1000) + "s" +
                    " | Pressure: " + String(EBS_TANK_PRESSURE_B_value) + " bar" +
                    " | Emergency: " + String(emergency_flag ? "ACTIVE" : "CLEAR"));
    
    digitalWrite(HB_LED, !digitalRead(HB_LED));
  }
}

String getStateName(SystemState state) {
  switch (state) {
    case STATE_INIT: return "INIT";
    case STATE_WAITING_ACTIVATION: return "WAITING_ACTIVATION";
    case STATE_PRESSURE_CHECK: return "PRESSURE_CHECK";
    case STATE_OPERATIONAL: return "OPERATIONAL";
    case STATE_EMERGENCY_ENTRY: return "EMERGENCY_ENTRY";
    case STATE_EMERGENCY_ACTIVE: return "EMERGENCY_ACTIVE";
    case STATE_DEPRESSURIZING: return "DEPRESSURIZING";
    case STATE_RECOVERY_READY: return "RECOVERY_READY";
    case STATE_FAULT: return "FAULT";
    case STATE_MAINTENANCE: return "MAINTENANCE";
    default: return "UNKNOWN";
  }
}

void onStateEntry(SystemState state) {
  reset_debug_leds();
  
  switch (state) {
    case STATE_INIT:
      Serial2.println("ENTERING: System Initialization");
      break;
    case STATE_WAITING_ACTIVATION:
      Serial2.println("ENTERING: Waiting for Activation");
      break;
    case STATE_PRESSURE_CHECK:
      Serial2.println("ENTERING: Pressure Validation");
      break;
    case STATE_OPERATIONAL:
      Serial2.println("ENTERING: Normal Operation");
      break;
    case STATE_EMERGENCY_ENTRY:
      Serial2.println("ENTERING: Emergency Entry");
      break;
    case STATE_EMERGENCY_ACTIVE:
      Serial2.println("ENTERING: Emergency Active");
      break;
    case STATE_DEPRESSURIZING:
      Serial2.println("ENTERING: Depressurizing");
      break;
    case STATE_RECOVERY_READY:
      Serial2.println("ENTERING: Recovery Ready");
      break;
    case STATE_FAULT:
      Serial2.println("ENTERING: System Fault");
      break;
    case STATE_MAINTENANCE:
      Serial2.println("ENTERING: Maintenance Mode");
      break;
  }
}

void executeCurrentState() {
  switch (sm.currentState) {
    case STATE_INIT:
      executeStateInit();
      break;
    case STATE_WAITING_ACTIVATION:
      executeStateWaitingActivation();
      break;
    case STATE_PRESSURE_CHECK:
      executeStatePressureCheck();
      break;
    case STATE_OPERATIONAL:
      executeStateOperational();
      break;
    case STATE_EMERGENCY_ENTRY:
      executeStateEmergencyEntry();
      break;
    case STATE_EMERGENCY_ACTIVE:
      executeStateEmergencyActive();
      break;
    case STATE_DEPRESSURIZING:
      executeStateDepressurizing();
      break;
    case STATE_RECOVERY_READY:
      executeStateRecoveryReady();
      break;
    case STATE_FAULT:
      executeStateFault();
      break;
    case STATE_MAINTENANCE:
      executeStateMaintenance();
      break;
  }
}

void executeStateInit() {
  reset_debug_leds();
  digitalWrite(Debug_LED2, HIGH);
  
  ignition_signal = 0;
  digitalWrite(SOLENOID1, LOW);
  digitalWrite(SOLENOID2, LOW);
  emergency_flag = 0;
  
  if (millis() - sm.stateEntryTime > 2000) {
    sm.pendingEvent = EVENT_INIT_COMPLETE;
  }
}

void executeStateWaitingActivation() {
  digitalWrite(Debug_LED3, HIGH);
  
  ignition_signal = 0;
  digitalWrite(SOLENOID1, LOW);
  digitalWrite(SOLENOID2, LOW);
  
  updatePressureReadings();
  
  if (checkCANHealth() && digitalRead(ASMS)) {
    sm.pendingEvent = EVENT_CAN_READY;
  }
}

void executeStatePressureCheck() {
  digitalWrite(Debug_LED4, HIGH);
  updatePressureReadings();
  
  if (EBS_TANK_PRESSURE_B_value >= TANK_PRESSURE_THRESHOLD) {
    sm.pendingEvent = EVENT_PRESSURE_OK;
  } else {
    sm.pendingEvent = EVENT_PRESSURE_LOW;
  }
}

void executeStateOperational() {
  digitalWrite(Debug_LED5, HIGH);
  
  updatePressureReadings();
  updateIgnitionControl();
  
  if (EBS_TANK_PRESSURE_B_value < TANK_PRESSURE_THRESHOLD) {
    sm.pendingEvent = EVENT_EMERGENCY_TRIGGER;
  }
  
  if (!digitalRead(ASMS)) {
    sm.pendingEvent = EVENT_ASMS_INACTIVE;
  }
  
  static unsigned long lastJson = 0;
  if (millis() - lastJson >= 1000) {
    sendJson();
    lastJson = millis();
  }
}

void executeStateEmergencyEntry() {
  digitalWrite(Debug_LED6, HIGH);
  digitalWrite(YELLOW_LEDS, HIGH);
  
  // Log emergency entry details once
  static bool entryLogged = false;
  if (!entryLogged) {
    Serial2.println("=== EMERGENCY ENTRY ACTIVATED ===");
    Serial2.println("Time: " + String(millis()) + "ms");
    Serial2.println("Previous State: " + getStateName(sm.previousState));
    Serial2.println("Pressure A: " + String(EBS_TANK_PRESSURE_A_value) + " bar");
    Serial2.println("Pressure B: " + String(EBS_TANK_PRESSURE_B_value) + " bar");
    Serial2.println("ASMS Status: " + String(digitalRead(ASMS) ? "ACTIVE" : "INACTIVE"));
    Serial2.println("Mission: " + String(mission));
    Serial2.println("===================================");
    entryLogged = true;
  }
  
  // IMMEDIATE safety actions
  ignition_signal = 0;           // Cut ignition immediately
  digitalWrite(SOLENOID1, HIGH); // Open pressure relief
  digitalWrite(SOLENOID2, HIGH); // Open pressure relief
  emergency_flag = 1;            // Set emergency flag
  
  // Visual/audio warnings
  if ((millis() / 200) % 2) {    // Fast blink for urgency
    digitalWrite(Debug_LED6, HIGH);
    digitalWrite(YELLOW_LEDS, HIGH);
  } else {
    digitalWrite(Debug_LED6, LOW);
    digitalWrite(YELLOW_LEDS, LOW);
  }
  
  // Auto-transition after emergency procedures are active
  if (millis() - sm.stateEntryTime > 1000) {
    entryLogged = false; // Reset for next emergency
    changeState(STATE_EMERGENCY_ACTIVE);
  }
}

void executeStateEmergencyActive() {
  if ((millis() / 500) % 2) {
    digitalWrite(Debug_LED6, HIGH);
  } else {
    digitalWrite(Debug_LED6, LOW);
  }
  
  ignition_signal = 0;
  digitalWrite(SOLENOID1, HIGH);
  digitalWrite(SOLENOID2, HIGH);
  emergency_flag = 1;
  
  updatePressureReadings();
  
  if (EBS_TANK_PRESSURE_B_value <= 0.5) {
    sm.pendingEvent = EVENT_PRESSURE_SAFE;
  }
}

void executeStateDepressurizing() {
  if ((millis() / 250) % 2) {
    digitalWrite(Debug_LED5, HIGH);
    digitalWrite(Debug_LED6, LOW);
  } else {
    digitalWrite(Debug_LED5, LOW);
    digitalWrite(Debug_LED6, HIGH);
  }
  
  ignition_signal = 0;
  digitalWrite(SOLENOID1, HIGH);
  digitalWrite(SOLENOID2, HIGH);
  
  updatePressureReadings();
  
  if (EBS_TANK_PRESSURE_B_value <= 0.5 && digitalRead(ASMS)) {
    static unsigned long safeTime = 0;
    if (safeTime == 0) safeTime = millis();
    
    if (millis() - safeTime > 5000) {
      sm.pendingEvent = EVENT_RECOVERY_REQUEST;
    }
  }
}

void executeStateRecoveryReady() {
  digitalWrite(Debug_LED4, HIGH);
  
  ignition_signal = 0;
  digitalWrite(SOLENOID1, LOW);
  digitalWrite(SOLENOID2, LOW);
  emergency_flag = 0;
  
  updatePressureReadings();
  
  if (millis() - sm.stateEntryTime > 3000) {
    changeState(STATE_WAITING_ACTIVATION);
  }
}

void executeStateFault() {
  if ((millis() / 1000) % 2) {
    digitalWrite(Debug_LED2, HIGH);
  } else {
    digitalWrite(Debug_LED2, LOW);
  }
  
  ignition_signal = 0;
  digitalWrite(SOLENOID1, HIGH);
  digitalWrite(SOLENOID2, HIGH);
  
  updatePressureReadings();
  
  if (digitalRead(ASMS) && checkCANHealth()) {
    static unsigned long recoveryTime = 0;
    if (recoveryTime == 0) recoveryTime = millis();
    
    if (millis() - recoveryTime > 10000) {
      changeState(STATE_WAITING_ACTIVATION);
    }
  }
}

void executeStateMaintenance() {
  digitalWrite(Debug_LED3, HIGH);
  digitalWrite(Debug_LED5, HIGH);
  
  updatePressureReadings();
  
  if (digitalRead(MS_BUTTON1) || (millis() - sm.stateEntryTime > 30000)) {
    changeState(STATE_WAITING_ACTIVATION);
  }
}

void handleStateTransitions() {
  switch (sm.currentState) {
    case STATE_INIT:
      if (sm.pendingEvent == EVENT_INIT_COMPLETE) {
        changeState(STATE_WAITING_ACTIVATION);
      }
      break;
      
    case STATE_WAITING_ACTIVATION:
      if (sm.pendingEvent == EVENT_CAN_READY) {
        changeState(STATE_PRESSURE_CHECK);
      }
      break;
      
    case STATE_PRESSURE_CHECK:
      if (sm.pendingEvent == EVENT_PRESSURE_OK) {
        changeState(STATE_OPERATIONAL);
      } else if (sm.pendingEvent == EVENT_PRESSURE_LOW) {
        changeState(STATE_EMERGENCY_ENTRY);
      }
      break;
      
    case STATE_OPERATIONAL:
      if (sm.pendingEvent == EVENT_EMERGENCY_TRIGGER) {
        changeState(STATE_EMERGENCY_ENTRY);
      } else if (sm.pendingEvent == EVENT_ASMS_INACTIVE) {
        changeState(STATE_FAULT);
      }
      break;
      
    case STATE_EMERGENCY_ACTIVE:
      if (sm.pendingEvent == EVENT_PRESSURE_SAFE) {
        changeState(STATE_DEPRESSURIZING);
      }
      break;
      
    case STATE_DEPRESSURIZING:
      if (sm.pendingEvent == EVENT_RECOVERY_REQUEST) {
        changeState(STATE_RECOVERY_READY);
      }
      break;
      
    case STATE_EMERGENCY_ENTRY:
        // Handle emergency entry
        break;
        
    case STATE_RECOVERY_READY:
        // Handle recovery ready
        break;
        
    case STATE_FAULT:
        // Handle fault state
        break;
        
    case STATE_MAINTENANCE:
        // Handle maintenance state
        break;
        
    default:
        break;
  }
  
  sm.pendingEvent = EVENT_NONE;
}

void updateSystemInputs() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 100) {
    Pressure_readings();
    lastUpdate = millis();
  }
}

void processSystemEvents() {
  // Add event processing logic here
}

void updateIgnitionControl() {
  if (digitalRead(IGN_PIN) && ign_en && !emergency_flag) {
    ignition_signal = 1;
  } else {
    ignition_signal = 0;
  }
}

bool checkCANHealth() {
  return true; // Implement CAN health check
}

void feedWatchdogs() {
  static unsigned long lastFeed = 0;
  if (millis() - lastFeed >= 500) {
    wdt_software.feed();
    wdt_seq.feed();
    lastFeed = millis();
  }
}

void initializeHardware() {
  pinMode(SOLENOID1, OUTPUT);
  pinMode(SOLENOID2, OUTPUT);
  pinMode(HB_LED, OUTPUT);
  pinMode(Debug_LED2, OUTPUT);
  pinMode(Debug_LED3, OUTPUT);
  pinMode(Debug_LED4, OUTPUT);
  pinMode(Debug_LED5, OUTPUT);
  pinMode(Debug_LED6, OUTPUT);
  pinMode(YELLOW_LEDS, OUTPUT);
  pinMode(BLUE_LEDS, OUTPUT);
  pinMode(ASMS, INPUT_PULLUP);
  pinMode(IGN_PIN, INPUT_PULLUP);
  pinMode(MS_BUTTON1, INPUT_PULLUP);
  
  digitalWrite(SOLENOID1, LOW);
  digitalWrite(SOLENOID2, LOW);
  digitalWrite(YELLOW_LEDS, LOW);
  digitalWrite(BLUE_LEDS, LOW);
  reset_debug_leds();
}

void updatePressureReadings() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 300) {
    median_pressures();
    lastUpdate = millis();
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
     // emergency_flag = 1; // Set emergency flag
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
  //status_ready = 0;
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
        median_pressures();  // Read pressure values
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
  Serial2.println("Ignition reading: " + String(ignition_readings[reading_index]));
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

  uint8_t status_data[1] = {status_ready};
  CAN_MSG_SEND(IGN_TO_ACU, 1, status_data);

}

void checkGlobalEmergencyConditions() {
  // Priority 1: CAN Emergency Override
  if (Received_CAN_MSG.id == RES_ID && Received_CAN_MSG.buf[1] == 0) {
    Serial2.println("CAN EMERGENCY OVERRIDE - RES byte 1 = 0");
    sm.pendingEvent = EVENT_CAN_EMERGENCY_OVERRIDE;
    return; // Exit immediately - highest priority
  }
  
  // Priority 2: Critical Pressure Drop (works in ALL states)
  if (EBS_TANK_PRESSURE_B_value < TANK_PRESSURE_THRESHOLD) {
    Serial2.println("PRESSURE EMERGENCY - Current: " + String(EBS_TANK_PRESSURE_B_value) + 
                    " bar, Threshold: " + String(TANK_PRESSURE_THRESHOLD) + " bar");
    sm.pendingEvent = EVENT_EMERGENCY_TRIGGER;
    return;
  }
  
  // Priority 3: ASMS Emergency Shutdown (if system is running)
  if (!digitalRead(ASMS) && (sm.currentState == STATE_OPERATIONAL || 
                            sm.currentState == STATE_PRESSURE_CHECK)) {
    Serial2.println("ASMS EMERGENCY SHUTDOWN - Switch deactivated during operation");
    sm.pendingEvent = EVENT_EMERGENCY_TRIGGER;
    return;
  }
  
  // Priority 4: Critical sensor failures
  if (EBS_TANK_PRESSURE_B_value < 0 || EBS_TANK_PRESSURE_B_value > 15.0) {
    Serial2.println("SENSOR EMERGENCY - Invalid pressure reading: " + String(EBS_TANK_PRESSURE_B_value));
    sm.pendingEvent = EVENT_EMERGENCY_TRIGGER;
    return;
  }
  
  // Priority 5: Communication timeout (if system is operational)
  static unsigned long lastCANMessage = 0;
  if (Received_CAN_MSG.id == RES_ID) {
    lastCANMessage = millis();
  }
  
  if (sm.currentState == STATE_OPERATIONAL && 
      millis() - lastCANMessage > 5000) { // 5 second timeout
    Serial2.println("CAN TIMEOUT EMERGENCY - No RES messages for 5 seconds");
    sm.pendingEvent = EVENT_EMERGENCY_TRIGGER;
    return;
  }
  
  // Priority 6: Multiple simultaneous faults
  static uint8_t faultCount = 0;
  faultCount = 0;
  
  if (EBS_TANK_PRESSURE_A_value < TANK_PRESSURE_THRESHOLD) faultCount++;
  if (EBS_TANK_PRESSURE_B_value < TANK_PRESSURE_THRESHOLD) faultCount++;
  if (!digitalRead(ASMS)) faultCount++;
  
  if (faultCount >= 2) {
    Serial2.println("MULTIPLE FAULT EMERGENCY - " + String(faultCount) + " simultaneous faults");
    sm.pendingEvent = EVENT_EMERGENCY_TRIGGER;
    return;
  }
}

