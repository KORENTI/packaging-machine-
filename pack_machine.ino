#include <U8g2lib.h> // Include U8g2 library for OLED display
#include <AiEsp32RotaryEncoder.h> // Include library for rotary encoder
#include <ADS1256.h> // Include ADS1256 library for ADC
#include <FastAccelStepper.h> // Include library for stepper motor control
#include <SPI.h> // Include SPI library for communication
#include <EEPROM.h> // Include EEPROM library for persistent storage
#include <freertos/semphr.h> // Include FreeRTOS semaphore support
#include <CRC32.h> // Include CRC32 library for data integrity
#include "esp_sleep.h" // Include ESP32 sleep functions
#include <esp_task_wdt.h> //WATCH DOG TASK HANDLE FOR ADC 

// Pin definitions
#define OLED_SDA 21  // OLED SDA pin for I2C communication
#define OLED_SCL 22  // OLED SCL pin for I2C communication
#define ENCODER_A 32  // Encoder A pin for rotation detection
#define ENCODER_B 33  // Encoder B pin for rotation detection
#define ENCODER_BTN 25  // Encoder button pin for user input
#define ADS_DRDY 27  // ADS1256 DRDY pin for data ready signal
#define ADS_CS 15  // ADS1256 CS pin for SPI communication
#define ADS_RST 26  // ADS1256 RST pin for reset
#define ADS_SCK 18  // ADS1256 SCK pin for SPI clock (VSPI)
#define ADS_MISO 19  // ADS1256 MISO pin for SPI data input (VSPI)
#define ADS_MOSI 23  // ADS1256 MOSI pin for SPI data output (VSPI)
#define LIMIT_SWITCH 34  // Limit switch pin for position feedback
//#define EMERGENCY_STOP_PIN 35  // Emergency stop pin for safety
#define STEPPER_DIR 2  // Stepper motor direction pin
#define STEPPER_STEP 4  // Stepper motor step pin
#define STEPPER_ENABLE 16  // Stepper motor enable pin

// Stepper configuration
#define STEPS_PER_REV 4000  // Number of steps per revolution for the stepper motor
#define MAX_STEP_FREQ 133333  // Maximum step frequency for the stepper motor
#define MANUAL_MOVE_SPEED 100  // Speed for manual movement of the stepper motor

// EEPROM configuration
#define EEPROM_SIZE 1024 // Size of EEPROM memory to use
#define SCALE_FACTOR_ADDR 20  // Address in EEPROM to store the scale factor

// Load cell constants
const float DIVIDER_RATIO = 6.0f;  // Divider ratio for load cell measurements
const float SENSITIVITY = 2.0f;  // Sensitivity of the load cell
const float EXCITATION_VOLTAGE = 15.0f;  // Excitation voltage for the load cell

// Global variables
volatile float currentWeight = 0.0;  // Current weight measured by the load cell
volatile float tareOffset = 0.0;  // Offset for tare function
volatile float scaleFactor = 1.0;  // Calibration factor for weight calculation
volatile int fillingCycleCount = 0;  // Counter for filling cycles
volatile bool isCalibrated = false;  // Flag to indicate if the system is calibrated
volatile bool newData = false;  // Flag to indicate new ADC data is available
volatile float targetWeight = 3000.0;  // Target weight for filling process
volatile float fastFillSpeed = 500.0;  // Speed for fast filling phase
volatile float fineFillSpeed = 50.0;  // Speed for fine filling phase
volatile float acceleration = 10000.0;  // Acceleration for stepper motor
//volatile bool emergencySwitchEnabled = true;  // Enable emergency switch functionality
volatile bool limitSwitchEnabled = true;  // Enable limit switch functionality
volatile bool motorBootLock = false;  // Lock motor at boot
volatile uint32_t limitSwitchTimeout = 30;  // Timeout for limit switch in seconds
volatile float manualRPM = 100.0;  // RPM for manual movement
volatile bool motorDirectionCW = true;  // Direction for manual movement (true = CW)

// State machine - Simplified states
enum State { 
  IDLE,  // Idle state, no operation
  CALIBRATING,  // Calibration in progress
  FILLING_PROCESS,  // Filling process active
  // EMERGENCY_STOP,  // Emergency stop triggered
  MOTOR_TEST  // Motor test mode
};
volatile State currentState = IDLE;  // Current state of the system

// Filling process substates
enum FillingState {
  FAST_FILLING,  // Fast filling phase
  FINE_FILLING,  // Fine filling phase
  FILLING_COMPLETE,  // Filling completed
  WAITING_FOR_CONTINUE  // Waiting for continuation
};
volatile FillingState fillingState = FAST_FILLING;  // Current filling substate

//ERROR CODE ADDITION OF CODE 
//error levels enums
enum ErrorCode {
  NO_ERROR,           // No error condition
  ADC_ERROR,          // ADC communication or data read failure
  CALIBRATION_ERROR,  // Calibration process failure
  MOTOR_ERROR         // Motor control or operation failure
};
volatile ErrorCode errorCode = NO_ERROR;  // Current error state

// Menu system
enum MenuState {
  MAIN_DISPLAY,  // Main display screen
  MAIN_MENU,  // Main menu
  MOTOR_SETTINGS,  // Motor settings menu
  SYSTEM_SETTINGS,  // System settings menu
  SET_TARGET_WEIGHT,  // Set target weight menu
  SET_FAST_FILL_SPEED,  // Set fast fill speed menu
  SET_FINE_FILL_SPEED,  // Set fine fill speed menu
  SET_ACCELERATION,  // Set acceleration menu
  MANUAL_MOVE_MENU,  // Manual move menu
  CALIBRATION_MENU,  // Calibration menu
  // SET_EMERGENCY_SWITCH,  // Set emergency switch menu
  SET_LIMIT_SWITCH,  // Set limit switch menu
  SET_MOTOR_BOOT_LOCK,  // Set motor boot lock menu
  SET_LIMIT_SWITCH_TIMEOUT,  // Set limit switch timeout menu
  SET_MANUAL_RPM  // Set manual RPM menu
};
MenuState currentMenu = MAIN_DISPLAY;  // Current menu state
int selectedOption = 0;  // Currently selected menu option
int topVisibleOption = 0;  // Top visible option in menu
unsigned long waitingStartTime = 0;  // Start time for waiting state
unsigned long fillingCompleteTime = 0;  // Time when filling completed

// Menu control
bool menuActive = false;  // Flag to indicate if menu is active
unsigned long lastInteractionTime = 0;  // Last interaction time for menu timeout
const unsigned long MENU_TIMEOUT = 10000;  // Menu timeout in milliseconds

// Semaphores and tasks
SemaphoreHandle_t weightMutex;  // Mutex for weight variable
SemaphoreHandle_t settingsMutex;  // Mutex for settings variables
SemaphoreHandle_t dataSemaphore;  // Semaphore for ADC data
SemaphoreHandle_t calibrationSemaphore;  // Semaphore for calibration
TaskHandle_t displayTaskHandle;  // Handle for display task
TaskHandle_t adcTaskHandle;  // Handle for ADC task
TaskHandle_t motorTaskHandle;  // Handle for motor task
// added error mutex
SemaphoreHandle_t errorMutex;  // Mutex for errorCode access

// Critical section protection
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;  // Mutex for critical sections

// Hardware objects
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);  // OLED display object
AiEsp32RotaryEncoder rotaryEncoder(ENCODER_A, ENCODER_B, ENCODER_BTN, -1, 4);  // Rotary encoder object
ADS1256 adc;  // ADS1256 ADC object
FastAccelStepperEngine engine;  // Stepper motor engine
FastAccelStepper *stepper = NULL;  // Stepper motor object

// Persistent data structure
struct PersistentData {
  float targetWeight;  // Target weight
  float fastFillSpeed;  // Fast fill speed
  float fineFillSpeed;  // Fine fill speed
  float acceleration;  // Acceleration
  // bool emergencySwitchEnabled;  // Emergency switch enable flag
  bool limitSwitchEnabled;  // Limit switch enable flag
  bool motorBootLock;  // Motor boot lock flag
  uint32_t limitSwitchTimeout;  // Limit switch timeout
  float manualRPM;  // Manual RPM
  int cycleCount;  // Cycle count
  uint32_t crc;  // CRC for data integrity
};

// Function prototypes
void IRAM_ATTR readEncoderISR();  // ISR for encoder
void initEEPROM();  // Initialize EEPROM
void savePersistentData();  // Save persistent data to EEPROM
float rawToGrams(int32_t raw, int32_t exc_raw);  // Convert raw ADC data to grams
void initADS1256();  // Initialize ADS1256
// void setMotorSpeed(float rpm);  // Set motor speed
// void moveMotorManual(bool clockwise, float rpm);  // Move motor manually
void autoCalibrate();  // Auto calibration
void calibrate3kg();  // Calibrate with 3kg weight
void enterSleepMode();  // Enter sleep mode
void displayTask(void *parameter);  // Display task
void adcTask(void *parameter);  // ADC task
void motorTask(void *parameter);  // Motor task
void runFillingProcess();  // Run filling process
void handleEncoder();  // Handle encoder input
void IRAM_ATTR DRDY_ISR();  // ISR for DRDY
// void IRAM_ATTR emergencyStopISR();  // ISR for emergency stop
void IRAM_ATTR limitSwitchISR();  // ISR for limit switch
void drawMainDisplay();  // Draw main display
void drawMenu();  // Draw menu
void navigateBack();  // Navigate back in menu
void startFillingProcess();  // Start filling process

// Setup function
void setup() {
  Serial.begin(115200);  // Initialize serial communication
 esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 5000,          // 5-second timeout
    .idle_core_mask = (1 << 0) | (1 << 1),  // Monitor both cores
    .trigger_panic = true        // Trigger panic on timeout
  };
  
  esp_err_t err = esp_task_wdt_init(&twdt_config);
  if (err != ESP_OK) {
    Serial.printf("Watchdog init failed: %d\n", err);
  }

    u8g2.begin();
  // Check wake reason
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();  // Get wake reason
  if(wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
    portENTER_CRITICAL(&timerMux);  // Enter critical section
    // currentState = EMERGENCY_STOP;  // Set state to emergency stop
    portEXIT_CRITICAL(&timerMux);  // Exit critical section
  }

  // Initialize EEPROM and load settings
  initEEPROM();  // Initialize EEPROM and load persistent data

  // Rotary encoder setup
  rotaryEncoder.begin();  // Initialize rotary encoder
  rotaryEncoder.setup(readEncoderISR);  // Set up ISR for encoder
  rotaryEncoder.setBoundaries(-1000, 1000, true);  // Set encoder boundaries
  rotaryEncoder.setAcceleration(100);  // Set encoder acceleration
  pinMode(ENCODER_A, INPUT_PULLUP);  // Set encoder A pin as input with pull-up
  pinMode(ENCODER_B, INPUT_PULLUP);  // Set encoder B pin as input with pull-up
  pinMode(ENCODER_BTN, INPUT_PULLUP);  // Set encoder button pin as input with pull-up

  // ADC initialization
  initADS1256();  // Initialize ADS1256 ADC

  // Stepper motor setup
  engine.init();  // Initialize stepper motor engine
  stepper = engine.stepperConnectToPin(STEPPER_STEP);  // Connect stepper to step pin
  if (stepper) {
    stepper->setDirectionPin(STEPPER_DIR);  // Set direction pin
    stepper->setEnablePin(STEPPER_ENABLE);  // Set enable pin
    stepper->setAutoEnable(true);  // Enable auto enable
    stepper->setAcceleration(acceleration);  // Set acceleration
    stepper->setCurrentPosition(0);  // Set current position to 0

    // Apply motor boot lock setting
    if (motorBootLock) {
      stepper->disableOutputs();  // Disable motor outputs
      digitalWrite(STEPPER_ENABLE, HIGH);  // Set enable pin high (active-low)
    }
  }

  // Create synchronization primitives
  weightMutex = xSemaphoreCreateMutex();  // Create mutex for weight
  settingsMutex = xSemaphoreCreateMutex();  // Create mutex for settings
  dataSemaphore = xSemaphoreCreateBinary();  // Create binary semaphore for data
  calibrationSemaphore = xSemaphoreCreateBinary();  // Create binary semaphore for calibration
 
 //ADDED error handling mutex intialization 
 errorMutex = xSemaphoreCreateMutex();  // Initialize mutex for error handling


  // Task allocation - Core 0: Display/ADC, Core 1: Motor (highest priority)
  xTaskCreatePinnedToCore(displayTask, "Display", 4096, NULL, 1, &displayTaskHandle, 0);  // Create display task on core 0
  xTaskCreatePinnedToCore(adcTask, "ADC", 4096, NULL, 2, &adcTaskHandle, 0);  // Create ADC task on core 0
  xTaskCreatePinnedToCore(motorTask, "Motor", 6144, NULL, 3, &motorTaskHandle, 1);  // Create motor task on core 1

  // Pin modes and interrupts
  pinMode(ADS_DRDY, INPUT);  // Set DRDY pin as input
  // pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);  // Set emergency stop pin as input with pull-up
  pinMode(LIMIT_SWITCH, INPUT_PULLUP);  // Set limit switch pin as input with pull-up
  attachInterrupt(digitalPinToInterrupt(ADS_DRDY), DRDY_ISR, FALLING);  // Attach ISR for DRDY
  // attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), emergencyStopISR, FALLING);  // Attach ISR for emergency stop
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH), limitSwitchISR, FALLING);  // Attach ISR for limit switch

//   // Initial calibration
  autoCalibrate();  // Perform auto calibration

  // autcalibration sequences of actions 250-285
{
  portENTER_CRITICAL(&timerMux);  // Enter critical section for state update
  currentState = CALIBRATING;     // Set system state to calibrating
  portEXIT_CRITICAL(&timerMux);   // Exit critical section

  int32_t load_raw, exc_raw;      // Variables for raw ADC readings
  unsigned long startTime = millis();  // Record start time for timeout
  while (millis() - startTime < 1000) {  // 1-second timeout for ADC response
    adc.readDifferentialChannels(&load_raw, &exc_raw);  // Read ADC channels
    if (load_raw != 0 || exc_raw != 0) break;  // Exit if valid data received
    vTaskDelay(pdMS_TO_TICKS(10));    // Delay 10ms between attempts
  }

  if (load_raw == 0 && exc_raw == 0) {  // Check for ADC failure
    xSemaphoreTake(errorMutex, portMAX_DELAY);  // Lock error mutex
    errorCode = ADC_ERROR;              // Set ADC error
    xSemaphoreGive(errorMutex);         // Release error mutex
    return;                             // Exit function on error
  }

  tareOffset = load_raw;                // Set tare offset from ADC reading

  xSemaphoreTake(weightMutex, portMAX_DELAY);  // Lock weight mutex
  currentWeight = 0.0;                  // Reset current weight
  xSemaphoreGive(weightMutex);          // Release weight mutex

  isCalibrated = true;                  // Mark system as calibrated
  xSemaphoreGive(calibrationSemaphore); // Signal calibration complete

  xSemaphoreTake(errorMutex, portMAX_DELAY);  // Lock error mutex
  errorCode = NO_ERROR;                 // Clear any previous error
  xSemaphoreGive(errorMutex);           // Release error mutex

  currentMenu = MAIN_DISPLAY;           // Return to main display
  menuActive = false;                   // Deactivate menu
}


}

// Loop function
void loop() {
  static unsigned long lastActivity = millis();  // Last activity time

  handleEncoder();  // Handle encoder input

  // Update activity timer
  if(currentState != IDLE || rotaryEncoder.readEncoder() != 0) {
    lastActivity = millis();  // Update last activity time
  }

  // Enter sleep after 5 minutes of inactivity
  if(millis() - lastActivity > 300000) {
    enterSleepMode();  // Enter sleep mode
  }

  // Handle filling process
  if (currentState == FILLING_PROCESS) {
    runFillingProcess();  // Run filling process
  }

  vTaskDelay(pdMS_TO_TICKS(10));  // Delay for 10ms
}

// Display task
void displayTask(void *parameter) {
  u8g2.begin();  // Initialize OLED display
  while (1) {
    if (currentMenu == MAIN_DISPLAY) {
      drawMainDisplay();  // Draw main display
    } else {
      drawMenu();  // Draw menu
    }
    vTaskDelay(pdMS_TO_TICKS(50));  // Delay for 50ms (20Hz refresh)
  }
}

// ADC task
void adcTask(void *parameter) {
  const int FILTER_SIZE = 8;  // Filter size for moving average
  float filterBuffer[FILTER_SIZE] = {0};  // Filter buffer
  int filterIndex = 0;  // Filter index

  while (1) {
    
     UBaseType_t stackHWM;  
     esp_task_wdt_reset();
    //  esp_task_wdt_add(NULL);  // Add task to watchdog

    if (xSemaphoreTake(dataSemaphore, portMAX_DELAY)) {  // Take data semaphore
      int32_t load_raw, exc_raw;  // Raw ADC values
      adc.readDifferentialChannels(&load_raw, &exc_raw);  // Read differential channels

      // Calculate weight
      float v_exc = (exc_raw * 2.5f / 0x7FFFFF) * DIVIDER_RATIO;  // Calculate excitation voltage
      float newWeight = (load_raw - tareOffset) * scaleFactor * (EXCITATION_VOLTAGE / v_exc);  // Calculate weight

      // Moving average filter
      filterBuffer[filterIndex] = newWeight;  // Add new weight to buffer
      filterIndex = (filterIndex + 1) % FILTER_SIZE;  // Update index
      float sum = 0;  // Sum of buffer
      for(int i=0; i<FILTER_SIZE; i++) sum += filterBuffer[i];  // Calculate sum

       // Reset watchdog every cycle
    
    
    vTaskDelay(1 / portTICK_PERIOD_MS);  // Yield to other tasks

      xSemaphoreTake(weightMutex, portMAX_DELAY);  // Take weight mutex
      currentWeight = sum / FILTER_SIZE;  // Update current weight
      xSemaphoreGive(weightMutex);  // Give weight mutex

      newData = false;  // Clear new data flag
      
    }
  stackHWM = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("adc stack free: %d\n", stackHWM);
  }
}

// Motor task
void motorTask(void *parameter) {
   UBaseType_t stackHWM;
  
  static unsigned long fillStart = 0;  // Start time for filling
  static float currentPower = 0.3;  // Current power level
  static bool motorTestActive = false;  // Flag for motor test active
  
  

  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for notification

    // Get current settings safely
    // bool localEmergencyEnabled;
    // xSemaphoreTake(settingsMutex, portMAX_DELAY);  // Take settings mutex
    // localEmergencyEnabled = emergencySwitchEnabled;  // Get emergency switch enable
    // xSemaphoreGive(settingsMutex);  // Give settings mutex

    // Emergency stop handling
    // if(localEmergencyEnabled && digitalRead(EMERGENCY_STOP_PIN) == LOW) {
    //   stepper->forceStop();  // Stop stepper motor
    //   portENTER_CRITICAL(&timerMux);  // Enter critical section
    //   currentState = EMERGENCY_STOP;  // Set state to emergency stop
    //   portEXIT_CRITICAL(&timerMux);  // Exit critical section
    //   motorTestActive = false;  // Clear motor test active flag
    //   continue;
    // }

    // Power management
    State localState;
    portENTER_CRITICAL(&timerMux);  // Enter critical section
    localState = currentState;  // Get current state
    portEXIT_CRITICAL(&timerMux);  // Exit critical section

    if (localState == FILLING_PROCESS) {
      FillingState localFillingState;
      portENTER_CRITICAL(&timerMux);  // Enter critical section
      localFillingState = fillingState;  // Get filling state
      portEXIT_CRITICAL(&timerMux);  // Exit critical section

      if (localFillingState == FAST_FILLING) {
        currentPower = (currentPower + 0.05 < 1.0) ? currentPower + 0.05 : 1.0;  // Increase power
      } else if (localFillingState == FINE_FILLING) {
        currentPower = (currentPower - 0.01 > 0.5) ? currentPower - 0.01 : 0.5;  // Decrease power
      } else {
        currentPower = 0.3;  // Reset power
      }
    } else if (localState == MOTOR_TEST) {
      currentPower = 0.7;  // Set power for motor test
    } else {
      currentPower = 0.3;  // Default power
    }

    analogWrite(STEPPER_ENABLE, 255 * currentPower);  // Set enable pin power

    // State machine execution
    switch (localState) {
      case FILLING_PROCESS:
        {
          FillingState localFillingState;
          portENTER_CRITICAL(&timerMux);  // Enter critical section
          localFillingState = fillingState;  // Get filling state
          portEXIT_CRITICAL(&timerMux);  // Exit critical section

          if (localFillingState == FAST_FILLING) {
            if(fillStart == 0) fillStart = millis();  // Set fill start time
            setMotorSpeed(fastFillSpeed);  // Set fast fill speed
            stepper->runForward();  // Run motor forward
          } else if (localFillingState == FINE_FILLING) {
            setMotorSpeed(fineFillSpeed);  // Set fine fill speed
            stepper->runForward();  // Run motor forward
          } else {
            stepper->forceStop();  // Stop motor
            fillStart = 0;  // Reset fill start time
          }
        }
        break;

      case MOTOR_TEST:
        if (!motorTestActive) {
          motorTestActive = true;  // Set motor test active flag
          setMotorSpeed(manualRPM);  // Set manual RPM
          stepper->runForward();  // Run motor forward
        }
        break;

      default:
        stepper->forceStop();  // Stop motor
        fillStart = 0;  // Reset fill start time
        motorTestActive = false;  // Clear motor test active flag
        break;
    }

    // Safety timeout
    if(fillStart > 0 && (millis() - fillStart) > 5000) {
      stepper->forceStop();  // Stop motor
      portENTER_CRITICAL(&timerMux);  // Enter critical section
      // currentState = EMERGENCY_STOP;  // Set state to emergency stop
      portEXIT_CRITICAL(&timerMux);  // Exit critical section
      fillStart = 0;  // Reset fill start time
    }
    stackHWM = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("Motor stack free: %d\n", stackHWM);
  }
}

// Initialize EEPROM
void initEEPROM() {
  EEPROM.begin(EEPROM_SIZE);  // Begin EEPROM with specified size

  PersistentData data;  // Persistent data structure
  EEPROM.get(0, data);  // Get data from EEPROM

  CRC32 crc;  // CRC32 object
  crc.update((uint8_t*)&data, sizeof(data) - sizeof(uint32_t));  // Update CRC

  if (crc.finalize() == data.crc) {  // Check CRC
    targetWeight = data.targetWeight;  // Set target weight
    fastFillSpeed = data.fastFillSpeed;  // Set fast fill speed
    fineFillSpeed = data.fineFillSpeed;  // Set fine fill speed
    acceleration = data.acceleration;  // Set acceleration
    // emergencySwitchEnabled = data.emergencySwitchEnabled;  // Set emergency switch enable
    limitSwitchEnabled = data.limitSwitchEnabled;  // Set limit switch enable
    motorBootLock = data.motorBootLock;  // Set motor boot lock
    limitSwitchTimeout = data.limitSwitchTimeout;  // Set limit switch timeout
    manualRPM = data.manualRPM;  // Set manual RPM
    fillingCycleCount = data.cycleCount;  // Set cycle count

    // Apply settings
    stepper->setAcceleration(acceleration);  // Set stepper acceleration
  }
}

// Save persistent data
void savePersistentData() {
  PersistentData data = {
    targetWeight,  // Target weight
    fastFillSpeed,  // Fast fill speed
    fineFillSpeed,  // Fine fill speed
    acceleration,  // Acceleration
    // emergencySwitchEnabled,  // Emergency switch enable
    limitSwitchEnabled,  // Limit switch enable
    motorBootLock,  // Motor boot lock
    limitSwitchTimeout,  // Limit switch timeout
    manualRPM,  // Manual RPM
    fillingCycleCount,  // Cycle count
    0  // CRC placeholder
  };

  CRC32 crc;  // CRC32 object
  crc.update((uint8_t*)&data, sizeof(data) - sizeof(uint32_t));  // Update CRC
  data.crc = crc.finalize();  // Set CRC

  EEPROM.put(0, data);  // Put data to EEPROM
  EEPROM.commit();  // Commit changes
}

// Convert raw ADC data to grams
float rawToGrams(int32_t raw, int32_t exc_raw) {
  float v_exc = (exc_raw * 2.5f / 0x7FFFFF) * DIVIDER_RATIO;  // Calculate excitation voltage
  return (raw - tareOffset) * scaleFactor * (EXCITATION_VOLTAGE / v_exc);  // Calculate weight
}

// Initialize ADS1256
void initADS1256() {
  adc.pinCS = ADS_CS;  // Set CS pin
  adc.pinDIN = ADS_MOSI;  // Set DIN pin (MOSI)
  adc.pinDOUT = ADS_MISO;  // Set DOUT pin (MISO)
  adc.pinRDY = ADS_DRDY;  // Set DRDY pin
  adc.pinRESET = ADS_RST;  // Set RST pin
  adc.pinSCLK = ADS_SCK;  // Set SCK pin
  adc.speedSPI = 1700000;  // Set SPI speed
  adc.init();  // Initialize ADC
  adc.setPGA(ADS1256_PGA_64);  // Set PGA gain
  adc.setDataRate(ADS1256_DRATE_30000_SPS);  // Set data rate
  adc.sendCommand(ADS1256_SELFCAL);  // Perform self-calibration
}

// Set motor speed
void setMotorSpeed(float rpm) {
  float stepsPerSec = (rpm <= 0) ? 0 : rpm * STEPS_PER_REV / 60.0;  // Calculate steps per second
  stepsPerSec = (stepsPerSec > MAX_STEP_FREQ) ? MAX_STEP_FREQ : stepsPerSec;  // Limit to max frequency
  stepper->setSpeedInHz(stepsPerSec);  // Set speed
}

// Move motor manually
void moveMotorManual(bool clockwise, float rpm) {
  setMotorSpeed(rpm);  // Set speed
  stepper->move(clockwise ? 10000000 : -10000000);  // Move motor
}

// Auto calibration
void autoCalibrate() {
  portENTER_CRITICAL(&timerMux);  // Enter critical section
  currentState = CALIBRATING;  // Set state to calibrating
  portEXIT_CRITICAL(&timerMux);  // Exit critical section

  int32_t load_raw, exc_raw;  // Raw ADC values
  adc.readDifferentialChannels(&load_raw, &exc_raw);  // Read differential channels
  tareOffset = load_raw;  // Set tare offset

  xSemaphoreTake(weightMutex, portMAX_DELAY);  // Take weight mutex
  currentWeight = 0.0;  // Reset current weight
  xSemaphoreGive(weightMutex);  // Give weight mutex

  isCalibrated = true;  // Set calibrated flag
  xSemaphoreGive(calibrationSemaphore);  // Give calibration semaphore

  currentMenu = MAIN_DISPLAY;  // Set menu to main display
  menuActive = false;  // Deactivate menu
}

// Calibrate with 3kg weight
void calibrate3kg() {
  portENTER_CRITICAL(&timerMux);  // Enter critical section for state update
  currentState = CALIBRATING;     // Set system state to calibrating
  portEXIT_CRITICAL(&timerMux);   // Exit critical section

  u8g2.clearBuffer();             // Clear OLED display buffer
  u8g2.drawStr(0, 30, "Place 3kg weight");  // Instruct user to place weight
  u8g2.drawStr(0, 50, "Press encoder btn");  // Instruct user to confirm
  u8g2.sendBuffer();              // Update OLED display

  unsigned long timeout = millis() + 30000;  // Set 30-second timeout
  while (!rotaryEncoder.isEncoderButtonClicked() && millis() < timeout) {
    vTaskDelay(pdMS_TO_TICKS(50));  // Delay 50ms while waiting
  }

  if (millis() >= timeout) {      // Check for timeout
    xSemaphoreTake(errorMutex, portMAX_DELAY);  // Lock error mutex
    errorCode = CALIBRATION_ERROR;  // Set calibration error
    xSemaphoreGive(errorMutex);     // Release error mutex
    return;                         // Exit function on error
  }

  float sum = 0;                  // Sum of weight readings
  for (int i = 0; i < 50; i++) {  // Take 50 samples
    int32_t load_raw, exc_raw;    // Variables for raw ADC readings
    adc.readDifferentialChannels(&load_raw, &exc_raw);  // Read ADC channels
    if (load_raw == 0 && exc_raw == 0) {  // Check for ADC failure
      xSemaphoreTake(errorMutex, portMAX_DELAY);  // Lock error mutex
      errorCode = ADC_ERROR;        // Set ADC error
      xSemaphoreGive(errorMutex);   // Release error mutex
      return;                       // Exit function on error
    }
    xSemaphoreTake(weightMutex, portMAX_DELAY);  // Lock weight mutex
    sum += currentWeight;         // Add current weight to sum
    xSemaphoreGive(weightMutex);  // Release weight mutex
    vTaskDelay(pdMS_TO_TICKS(20));  // Delay 20ms between samples
  }
  float measured3kg = sum / 50.0; // Calculate average weight
  scaleFactor = 3000.0 / (measured3kg - currentWeight);  // Compute scale factor

  EEPROM.put(SCALE_FACTOR_ADDR, scaleFactor);  // Save scale factor to EEPROM
  EEPROM.commit();                // Commit EEPROM changes

  portENTER_CRITICAL(&timerMux);  // Enter critical section for state update
  currentState = IDLE;            // Set system state to idle
  portEXIT_CRITICAL(&timerMux);   // Exit critical section

  xSemaphoreTake(errorMutex, portMAX_DELAY);  // Lock error mutex
  errorCode = NO_ERROR;           // Clear any previous error
  xSemaphoreGive(errorMutex);     // Release error mutex

  currentMenu = MAIN_DISPLAY;     // Return to main display
  menuActive = false;             // Deactivate menu
}


// Enter sleep mode
void enterSleepMode() {
  vTaskSuspend(displayTaskHandle);  // Suspend display task
  vTaskSuspend(adcTaskHandle);  // Suspend ADC task
  vTaskSuspend(motorTaskHandle);  // Suspend motor task

  adc.sendCommand(ADS1256_SLEEP);  // Put ADC to sleep
  stepper->disableOutputs();  // Disable motor outputs

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, LOW);  // Enable wake on encoder button
  // esp_sleep_enable_ext1_wakeup(BIT(EMERGENCY_STOP_PIN), ESP_EXT1_WAKEUP_ALL_LOW);  // Enable wake on emergency stop

  esp_deep_sleep_start();  // Start deep sleep
}

// Updated Start filling process but the loop is intact 
void startFillingProcess() {
  if (!isCalibrated) {            // Check calibration status
    xSemaphoreTake(errorMutex, portMAX_DELAY);  // Lock error mutex
    errorCode = CALIBRATION_ERROR;  // Set calibration error
    xSemaphoreGive(errorMutex);     // Release error mutex
    return;                         // Exit function on error
  }

  int32_t load_raw, exc_raw;      // Variables for raw ADC readings
  adc.readDifferentialChannels(&load_raw, &exc_raw);  // Read ADC channels
  if (load_raw == 0 && exc_raw == 0) {  // Check for ADC failure
    xSemaphoreTake(errorMutex, portMAX_DELAY);  // Lock error mutex
    errorCode = ADC_ERROR;        // Set ADC error
    xSemaphoreGive(errorMutex);   // Release error mutex
    return;                       // Exit function on error
  }

  portENTER_CRITICAL(&timerMux);  // Enter critical section for state update
  currentState = FILLING_PROCESS; // Set system state to filling
  fillingState = FAST_FILLING;    // Set filling substate to fast filling
  portEXIT_CRITICAL(&timerMux);   // Exit critical section
  xTaskNotify(motorTaskHandle, 0, eNoAction);  // Notify motor task
  fillingCycleCount++;            // Increment cycle counter

  xSemaphoreTake(errorMutex, portMAX_DELAY);  // Lock error mutex
  errorCode = NO_ERROR;           // Clear any previous error
  xSemaphoreGive(errorMutex);     // Release error mutex
}


// Run filling process
void runFillingProcess() {
  static bool fineFillingStarted = false;  // Flag for fine filling started
  float localTarget;  // Local target weight

  xSemaphoreTake(settingsMutex, portMAX_DELAY);  // Take settings mutex
  localTarget = targetWeight;  // Get target weight
  xSemaphoreGive(settingsMutex);  // Give settings mutex

  // Get current filling state
  FillingState localFillingState;
  portENTER_CRITICAL(&timerMux);  // Enter critical section
  localFillingState = fillingState;  // Get filling state
  portEXIT_CRITICAL(&timerMux);  // Exit critical section

  switch (localFillingState) {
    case FAST_FILLING:
      if (currentWeight >= 0.9 * localTarget) {
        portENTER_CRITICAL(&timerMux);  // Enter critical section
        fillingState = FINE_FILLING;  // Set filling state to fine filling
        portEXIT_CRITICAL(&timerMux);  // Exit critical section
        xTaskNotify(motorTaskHandle, 0, eNoAction);  // Notify motor task
        fineFillingStarted = true;  // Set fine filling started flag
      }
      break;

    case FINE_FILLING:
      if (currentWeight >= localTarget) {
        portENTER_CRITICAL(&timerMux);  // Enter critical section
        fillingState = FILLING_COMPLETE;  // Set filling state to complete
        fillingCompleteTime = millis();  // Set filling complete time
        portEXIT_CRITICAL(&timerMux);  // Exit critical section
        xTaskNotify(motorTaskHandle, 0, eNoAction);  // Notify motor task
      }
      break;

    case FILLING_COMPLETE:
      if (millis() - fillingCompleteTime > 2000) { // Show completion for 2 seconds
        portENTER_CRITICAL(&timerMux);  // Enter critical section
        fillingState = WAITING_FOR_CONTINUE;  // Set filling state to waiting
        waitingStartTime = millis();  // Set waiting start time
        portEXIT_CRITICAL(&timerMux);  // Exit critical section
      }
      break;

    case WAITING_FOR_CONTINUE:
      if (millis() - waitingStartTime > (limitSwitchTimeout * 1000)) {
        portENTER_CRITICAL(&timerMux);  // Enter critical section
        currentState = IDLE;  // Set state to idle
        fillingState = FAST_FILLING;  // Reset filling state
        portEXIT_CRITICAL(&timerMux);  // Exit critical section
      }
      break;
  }
}

// updated totall Draw main display
void drawMainDisplay() {
  u8g2.clearBuffer();             // Clear OLED display buffer
  u8g2.setFont(u8g2_font_ncenB08_tr);  // Set font for display

  State displayState;             // Local copy of system state
  portENTER_CRITICAL(&timerMux);  // Enter critical section for state read
  displayState = currentState;    // Read current state
  portEXIT_CRITICAL(&timerMux);   // Exit critical section

  const char* stateNames[] = {
    // "IDLE", "CALIBRATING", "FILLING PROCESS", "EMERGENCY STOP", "MOTOR TEST"
    "IDLE", "CALIBRATING", "FILLING PROCESS", "MOTOR TEST"
  };
  u8g2.drawStr(0, 10, stateNames[displayState]);  // Display current state

  char weightStr[20];             // Buffer for weight string
  sprintf(weightStr, "Weight: %07.2fg", currentWeight);  // Format weight
  u8g2.drawStr(0, 25, weightStr); // Display weight

  xSemaphoreTake(errorMutex, portMAX_DELAY);  // Lock error mutex
  ErrorCode currentError = errorCode;  // Read current error
  xSemaphoreGive(errorMutex);         // Release error mutex
  if (currentError != NO_ERROR) {     // Check for error condition
    const char* errorMessages[] = {
      "", "ADC Error", "Calib Error", "Motor Error"
    };
    u8g2.drawStr(0, 40, errorMessages[currentError]);  // Display error message
    u8g2.sendBuffer();                // Update OLED display
    return;                           // Skip other display content on error
  }

  // Existing state-specific displays (FILLING_PROCESS, MOTOR_TEST, IDLE) remain unchanged
  if (displayState == FILLING_PROCESS) {
    FillingState localFillingState; // Local copy of filling state
    portENTER_CRITICAL(&timerMux);  // Enter critical section for state read
    localFillingState = fillingState;  // Read filling state
    portEXIT_CRITICAL(&timerMux);   // Exit critical section

    const char* fillStateNames[] = {
      "FAST FILLING", "FINE FILLING", "COMPLETE!", "CONTINUE? [LS]"
    };
    u8g2.drawStr(0, 40, fillStateNames[localFillingState]);  // Display filling substate

    char targetStr[20];             // Buffer for target string
    sprintf(targetStr, "Target: %.0fg", targetWeight);  // Format target weight
    u8g2.drawStr(0, 55, targetStr); // Display target weight

    if (localFillingState == WAITING_FOR_CONTINUE) {  // Handle waiting state
      char timeStr[20];             // Buffer for time string
      uint32_t remaining = limitSwitchTimeout - ((millis() - waitingStartTime) / 1000);  // Calculate remaining time
      sprintf(timeStr, "Time: %lus", remaining);  // Format time
      u8g2.drawStr(60, 55, timeStr);  // Display remaining time
    }
  } else if (displayState == MOTOR_TEST) {
    char rpmStr[20];                // Buffer for RPM string
    sprintf(rpmStr, "RPM: %.0f", manualRPM);  // Format RPM
    u8g2.drawStr(0, 40, rpmStr);    // Display RPM

    char dirStr[20];                // Buffer for direction string
    sprintf(dirStr, "Direction: %s", motorDirectionCW ? "CW" : "CCW");  // Format direction
    u8g2.drawStr(0, 55, dirStr);    // Display direction
  } else if (displayState == IDLE) {
    char targetStr[20];             // Buffer for target string
    sprintf(targetStr, "Target: %.0fg", targetWeight);  // Format target weight
    u8g2.drawStr(0, 40, targetStr); // Display target weight

    char cyclesStr[20];             // Buffer for cycles string
    sprintf(cyclesStr, "Cycles: %d", fillingCycleCount);  // Format cycle count
    u8g2.drawStr(0, 55, cyclesStr); // Display cycle count
  }

  u8g2.sendBuffer();                // Update OLED display
}

// Draw menu
void drawMenu() {
  u8g2.clearBuffer();  // Clear display buffer
  u8g2.setFont(u8g2_font_ncenB08_tr);  // Set font

  // Menu titles
  const char* menuTitles[] = {
    "Main Menu", "Motor Settings", "System Settings", "Set Target", 
    "Set Fast Fill", "Set Fine Fill", "Set Acceleration", "Manual Move", 
    "Calibration", "Emerg. Switch", "Limit Switch", "Motor Boot Lock",
    "LS Timeout", "Set Manual RPM"
  };
  u8g2.drawStr(0, 10, menuTitles[currentMenu - 1]);  // Draw menu title

  // Display current value for setting menus
  if (currentMenu == SET_TARGET_WEIGHT) {
    u8g2.setCursor(0, 30);  // Set cursor position
    u8g2.print("Value: ");  // Print label
    u8g2.print(targetWeight);  // Print value
    u8g2.print("g");  // Print unit
  } else if (currentMenu == SET_FAST_FILL_SPEED) {
    u8g2.setCursor(0, 30);  // Set cursor position
    u8g2.print("Value: ");  // Print label
    u8g2.print(fastFillSpeed);  // Print value
    u8g2.print(" RPM");  // Print unit
  } else if (currentMenu == SET_FINE_FILL_SPEED) {
    u8g2.setCursor(0, 30);  // Set cursor position
    u8g2.print("Value: ");  // Print label
    u8g2.print(fineFillSpeed);  // Print value
    u8g2.print(" RPM");  // Print unit
  } else if (currentMenu == SET_ACCELERATION) {
    u8g2.setCursor(0, 30);  // Set cursor position
    u8g2.print("Value: ");  // Print label
    u8g2.print(acceleration);  // Print value
    u8g2.print(" steps/s2");  // Print unit
  } else if (currentMenu == SET_MANUAL_RPM) {
    u8g2.setCursor(0, 30);  // Set cursor position
    u8g2.print("Value: ");  // Print label
    u8g2.print(manualRPM);  // Print value
    u8g2.print(" RPM");  // Print unit
  }
  // else if (currentMenu == SET_EMERGENCY_SWITCH) {
  //   u8g2.setCursor(0, 30);  // Set cursor position
  //   u8g2.print("State: ");  // Print label
  //   u8g2.print(emergencySwitchEnabled ? "ON" : "OFF");  // Print state
  // }
   else if (currentMenu == SET_LIMIT_SWITCH) {
    u8g2.setCursor(0, 30);  // Set cursor position
    u8g2.print("State: ");  // Print label
    u8g2.print(limitSwitchEnabled ? "ON" : "OFF");  // Print state
  } else if (currentMenu == SET_MOTOR_BOOT_LOCK) {
    u8g2.setCursor(0, 30);  // Set cursor position
    u8g2.print("State: ");  // Print label
    u8g2.print(motorBootLock ? "ON" : "OFF");  // Print state
  } else if (currentMenu == SET_LIMIT_SWITCH_TIMEOUT) {
    u8g2.setCursor(0, 30);  // Set cursor position
    u8g2.print("Value: ");  // Print label
    u8g2.print(limitSwitchTimeout);  // Print value
    u8g2.print(" seconds");  // Print unit
  } else {
    // Regular menu items
    const char* mainMenuItems[] = {"Motor Settings", "System Settings", "Calibration", "Manual Move", "Back"};
    const char* motorMenuItems[] = {"Fast Fill RPM", "Fine Fill RPM", "Acceleration", "Manual RPM", "Back"};
    const char* systemMenuItems[] = {"Emerg. Switch", "Limit Switch", "Motor Boot Lock", "LS Timeout", "Back"};
    const char* calMenuItems[] = {"Auto Calibrate", "3kg Calibrate", "Back"};
    const char* manualMenuItems[] = {"Start CW", "Start CCW", "Set RPM", "Back"};

    int itemCount = 0;  // Number of menu items
    const char** items = nullptr;  // Menu items
    int visibleItems = 3;  // Number of visible items
    int yPos = 25;  // Y position for drawing

    switch (currentMenu) {
      case MAIN_MENU:
        itemCount = 5;  // Set item count
        items = mainMenuItems;  // Set items
        break;
      case MOTOR_SETTINGS:
        itemCount = 5;  // Set item count
        items = motorMenuItems;  // Set items
        break;
      case SYSTEM_SETTINGS:
        itemCount = 5;  // Set item count
        items = systemMenuItems;  // Set items
        break;
      case CALIBRATION_MENU:
        itemCount = 3;  // Set item count
        items = calMenuItems;  // Set items
        break;
      case MANUAL_MOVE_MENU:
        itemCount = 4;  // Set item count
        items = manualMenuItems;  // Set items
        break;
      default:
        break;
    }

    // Handle scrolling
    if (selectedOption >= topVisibleOption + visibleItems) {
      topVisibleOption = selectedOption - visibleItems + 1;  // Update top visible option
    } else if (selectedOption < topVisibleOption) {
      topVisibleOption = selectedOption;  // Update top visible option
    }

    // Display menu items
    for (int i = topVisibleOption; i < min(topVisibleOption + visibleItems, itemCount); i++) {
      if (i == selectedOption) u8g2.drawStr(0, yPos, ">");  // Draw selection indicator
      u8g2.drawStr(10, yPos, items[i]);  // Draw menu item
      yPos += 15;  // Increment Y position
    }

    // Show scroll indicators
    if (topVisibleOption > 0) u8g2.drawStr(120, 15, "^");  // Draw up arrow
    if (topVisibleOption + visibleItems < itemCount) u8g2.drawStr(120, 60, "v");  // Draw down arrow
  }

  u8g2.sendBuffer();  // Send buffer to display
}

// Navigate back in menu
void navigateBack() {
  switch (currentMenu) {
    case MAIN_MENU:
      currentMenu = MAIN_DISPLAY;  // Set menu to main display
      menuActive = false;  // Deactivate menu
      break;

    case MOTOR_SETTINGS:
    case SYSTEM_SETTINGS:
    case CALIBRATION_MENU:
    case MANUAL_MOVE_MENU:
      currentMenu = MAIN_MENU;  // Set menu to main menu
      selectedOption = 0;  // Reset selected option
      topVisibleOption = 0;  // Reset top visible option
      break;

    case SET_TARGET_WEIGHT:
    case SET_FAST_FILL_SPEED:
    case SET_FINE_FILL_SPEED:
    case SET_ACCELERATION:
    case SET_MANUAL_RPM:
      currentMenu = MOTOR_SETTINGS;  // Set menu to motor settings
      selectedOption = 0;  // Reset selected option
      topVisibleOption = 0;  // Reset top visible option
      break;

    // case SET_EMERGENCY_SWITCH:
    case SET_LIMIT_SWITCH:
    case SET_MOTOR_BOOT_LOCK:
    case SET_LIMIT_SWITCH_TIMEOUT:
      currentMenu = SYSTEM_SETTINGS;  // Set menu to system settings
      selectedOption = 0;  // Reset selected option
      topVisibleOption = 0;  // Reset top visible option
      break;
  }
}

// Handle encoder input
void handleEncoder() {
  static int16_t lastPos = 0;  // Last encoder position
  static unsigned long pressStart = 0;  // Start time for button press
  static unsigned long lastEncoderEvent = 0;  // Last encoder event time

  // Debounce check
  unsigned long currentMillis = millis();  // Get current time
  if (currentMillis - lastEncoderEvent < 100) return;  // Skip if too soon

  int16_t newPos = rotaryEncoder.readEncoder();  // Read encoder position

  // Handle rotation
  if (newPos != lastPos) {
    lastInteractionTime = currentMillis;  // Update last interaction time
    int delta = (newPos > lastPos) ? 1 : -1;  // Calculate delta
    lastPos = newPos;  // Update last position
    lastEncoderEvent = currentMillis;  // Update last event time

    if (menuActive) {
      switch (currentMenu) {
        // Value adjustment menus
        case SET_TARGET_WEIGHT:
          targetWeight += delta * 10.0;  // Adjust target weight
          targetWeight = constrain(targetWeight, 0.0f, 5000.0f);  // Constrain value
          break;

        case SET_FAST_FILL_SPEED:
          fastFillSpeed += delta * 10.0;  // Adjust fast fill speed
          fastFillSpeed = constrain(fastFillSpeed, 0.0f, 2000.0f);  // Constrain value
          break;

        case SET_FINE_FILL_SPEED:
          fineFillSpeed += delta * 10.0;  // Adjust fine fill speed
          fineFillSpeed = constrain(fineFillSpeed, 0.0f, 500.0f);  // Constrain value
          break;

        case SET_ACCELERATION:
          acceleration += delta * 100.0;  // Adjust acceleration
          acceleration = constrain(acceleration, 1000.0f, 50000.0f);  // Constrain value
          stepper->setAcceleration(acceleration);  // Set stepper acceleration
          break;

        case SET_LIMIT_SWITCH_TIMEOUT:
          limitSwitchTimeout += delta;  // Adjust limit switch timeout
          limitSwitchTimeout = constrain(limitSwitchTimeout, 5, 120);  // Constrain value
          break;

        case SET_MANUAL_RPM:
          manualRPM += delta * 10.0;  // Adjust manual RPM
          manualRPM = constrain(manualRPM, 0.0f, 2000.0f);  // Constrain value
          break;

        // Toggle menus
        // case SET_EMERGENCY_SWITCH:
        //   emergencySwitchEnabled = !emergencySwitchEnabled;  // Toggle emergency switch enable
        //   break;

        case SET_LIMIT_SWITCH:
          limitSwitchEnabled = !limitSwitchEnabled;  // Toggle limit switch enable
          break;

        case SET_MOTOR_BOOT_LOCK:
          motorBootLock = !motorBootLock;  // Toggle motor boot lock
          break;

        // Navigation menus
        default:
          selectedOption = (selectedOption + delta + 5) % 5;  // Adjust selected option
          break;
      }
    }
  }

  // Handle button press
  if (rotaryEncoder.isEncoderButtonClicked()) {
    lastInteractionTime = currentMillis;  // Update last interaction time

    if (!menuActive) {
      menuActive = true;  // Activate menu
      currentMenu = MAIN_MENU;  // Set menu to main menu
      selectedOption = 0;  // Reset selected option
      topVisibleOption = 0;  // Reset top visible option
    } else {
      switch (currentMenu) {
        case MAIN_MENU:
          switch (selectedOption) {
            case 0: currentMenu = MOTOR_SETTINGS; break;  // Go to motor settings
            case 1: currentMenu = SYSTEM_SETTINGS; break;  // Go to system settings
            case 2: currentMenu = CALIBRATION_MENU; break;  // Go to calibration menu
            case 3: currentMenu = MANUAL_MOVE_MENU; break;  // Go to manual move menu
            case 4: navigateBack(); break;  // Navigate back
          }
          break;

        case MOTOR_SETTINGS:
          switch (selectedOption) {
            case 0: currentMenu = SET_FAST_FILL_SPEED; break;  // Set fast fill speed
            case 1: currentMenu = SET_FINE_FILL_SPEED; break;  // Set fine fill speed
            case 2: currentMenu = SET_ACCELERATION; break;  // Set acceleration
            case 3: currentMenu = SET_MANUAL_RPM; break;  // Set manual RPM
            case 4: navigateBack(); break;  // Navigate back
          }
          break;

        case SYSTEM_SETTINGS:
          switch (selectedOption) {
            // case 0: currentMenu = SET_EMERGENCY_SWITCH; break;  // Set emergency switch
            case 0: currentMenu = SET_LIMIT_SWITCH; break;  // Set limit switch
            case 1: currentMenu = SET_MOTOR_BOOT_LOCK; break;  // Set motor boot lock
            case 2: currentMenu = SET_LIMIT_SWITCH_TIMEOUT; break;  // Set limit switch timeout
            case 3: navigateBack(); break;  // Navigate back
          }
          break;

        case CALIBRATION_MENU:
          switch (selectedOption) {
            case 0: autoCalibrate(); break;  // Auto calibrate
            case 1: calibrate3kg(); break;  // Calibrate with 3kg
            case 2: navigateBack(); break;  // Navigate back
          }
          break;

        case MANUAL_MOVE_MENU:
          switch (selectedOption) {
            case 0: // Start CW
              motorDirectionCW = true;  // Set direction to CW
              portENTER_CRITICAL(&timerMux);  // Enter critical section
              currentState = MOTOR_TEST;  // Set state to motor test
              portEXIT_CRITICAL(&timerMux);  // Exit critical section
              xTaskNotify(motorTaskHandle, 0, eNoAction);  // Notify motor task
              navigateBack();  // Navigate back
              break;
            case 1: // Start CCW
              motorDirectionCW = false;  // Set direction to CCW
              portENTER_CRITICAL(&timerMux);  // Enter critical section
              currentState = MOTOR_TEST;  // Set state to motor test
              portEXIT_CRITICAL(&timerMux);  // Exit critical section
              xTaskNotify(motorTaskHandle, 0, eNoAction);  // Notify motor task
              navigateBack();  // Navigate back
              break;
            case 2: currentMenu = SET_MANUAL_RPM; break;  // Set manual RPM
            case 3: navigateBack(); break;  // Navigate back
          }
          break;

        // Save settings when exiting parameter menus
        case SET_TARGET_WEIGHT:
        case SET_FAST_FILL_SPEED:
        case SET_FINE_FILL_SPEED:
        case SET_ACCELERATION:
        // case SET_EMERGENCY_SWITCH:
        case SET_LIMIT_SWITCH:
        case SET_MOTOR_BOOT_LOCK:
        case SET_LIMIT_SWITCH_TIMEOUT:
        case SET_MANUAL_RPM:
          savePersistentData();  // Save persistent data
          navigateBack();  // Navigate back
          break;
      }
    }
  }

  // Handle long press for back navigation
  if (rotaryEncoder.isEncoderButtonDown()) {
    if (pressStart == 0) {
      pressStart = millis();  // Set press start time
    } else if (millis() - pressStart > 1000) {
      navigateBack();  // Navigate back
      pressStart = 0;  // Reset press start time
    }
  } else {
    pressStart = 0;  // Reset press start time
  }

  // Menu timeout
  if (menuActive && millis() - lastInteractionTime > MENU_TIMEOUT) {
    navigateBack();  // Navigate back on timeout
  }
}

// ISR for encoder
void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();  // Read encoder in ISR
}

// ISR for DRDY
void IRAM_ATTR DRDY_ISR() {
  newData = true;  // Set new data flag
  xSemaphoreGiveFromISR(dataSemaphore, NULL);  // Give data semaphore
}

// // ISR for emergency stop
// void IRAM_ATTR emergencyStopISR() {
//   // Only handle if enabled
//   if (!emergencySwitchEnabled) return;  // Return if not enabled

//   stepper->forceStop();  // Stop stepper motor
//   portENTER_CRITICAL_ISR(&timerMux);  // Enter critical section
//   currentState = EMERGENCY_STOP;  // Set state to emergency stop
//   portEXIT_CRITICAL_ISR(&timerMux);  // Exit critical section
//   xTaskNotifyFromISR(motorTaskHandle, 0, eNoAction, NULL);  // Notify motor task
// }

// ISR for limit switch
void IRAM_ATTR limitSwitchISR() {
  // Only handle if enabled
  if (!limitSwitchEnabled) return;  // Return if not enabled

  portENTER_CRITICAL_ISR(&timerMux);  // Enter critical section
  // Only start filling process if we're idle
  if (currentState == IDLE && isCalibrated) {
    startFillingProcess();  // Start filling process
  }
  // Continue filling if waiting
  else if (currentState == FILLING_PROCESS && fillingState == WAITING_FOR_CONTINUE) {
    fillingState = FAST_FILLING;  // Set filling state to fast filling
    startFillingProcess();  // Start filling process
  }
  portEXIT_CRITICAL_ISR(&timerMux);  // Exit critical section
}
