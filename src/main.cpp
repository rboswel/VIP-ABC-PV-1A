
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AS7343.h>

// --- TCA9548A I2C MULTIPLEXER DEFINITION ---
#define TCA_ADDR 0x70

// --- SENSOR CHANNEL ASSIGNMENTS ---
#define CHANNEL_PAR0  0  // Open-sky reference sensor (uncovered)
#define CHANNEL_PAR1  1  // Under-panel sensor 1
#define CHANNEL_PAR2  2  // Under-panel sensor 2
#define CHANNEL_PAR3  3  // Under-panel sensor 3

// --- TMC2209 PIN DEFINITIONS ---
#define STEP_PIN    16
#define DIR_PIN     17

// --- MOTOR SETTINGS ---
#define CW          HIGH
#define CCW         LOW
#define STEP_DELAY_US 1000 // Speed of motor steps (microseconds between pulses)
#define SEARCH_STEP_SIZE 200 // Number of steps for a "slight" rotation

// --- TIMING & FILTERING CONSTANTS ---
#define CHECK_INTERVAL_MS 60000 // Take a monitoring reading once per minute (60,000 ms)
#define TARGET_RATIO_PCT  70    // Target under-panel light is 70% of smoothed PAR0
#define LOWER_CUTOFF_PCT  60    // Lower threshold (60% of smoothed PAR0)
#define UPPER_CUTOFF_PCT  80    // Upper threshold (80% of smoothed PAR0)
#define EMA_ALPHA         0.2f  // Smoothing factor for a ~5-minute window at 1 sample/minute

Adafruit_AS7343 as7343;

// Global variable for smoothed open-sky baseline
float smoothedPAR0 = 0.0f;
bool baselineInitialized = false;

// Function prototypes
void tcaSelect(uint8_t channel);
void read_light(uint16_t *readings);
uint32_t isolate_par(uint16_t *readings);
void stepMotor(bool direction, int steps);
uint32_t readSensorOnChannel(uint8_t channel);
uint32_t getOpenSkyPAR();
uint32_t getUnderPanelPAR();

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    delay(2000);

    Wire.begin();

    // Initialize all 4 sensors across multiplexer channels 0 through 3
    for (uint8_t i = 0; i < 4; i++) {
        tcaSelect(i);
        delay(10);
        if (!as7343.begin()) {
            while (true) { delay(10); }
        }
        
        as7343.setGain(AS7343_GAIN_0_5X);
        as7343.setATIME(29); 
        as7343.setASTEP(599);
    }

    // CSV Header row
    Serial.println("Timestamp_ms,PAR0_Raw,SmoothedPAR0,UnderPanel_PAR,Target,LowerCutoff,UpperCutoff");
}

void loop() {
    // 1. Read live open-sky PAR0 and under-panel average
    uint32_t par0 = getOpenSkyPAR();
    uint32_t underPanelLight = getUnderPanelPAR();

    // Initialize smoothing filter instantly on the first reading
    if (!baselineInitialized) {
        smoothedPAR0 = (float)par0;
        baselineInitialized = true;
    } else {
        // Apply Exponential Moving Average (EMA) to open-sky PAR0
        smoothedPAR0 = (EMA_ALPHA * (float)par0) + ((1.0f - EMA_ALPHA) * smoothedPAR0);
    }

    // 2. Calculate dynamic thresholds based on the SMOOTHED baseline (~5 min window)
    uint32_t targetLight   = ((uint32_t)smoothedPAR0 * TARGET_RATIO_PCT) / 100;
    uint32_t lowerCutoff = ((uint32_t)smoothedPAR0 * LOWER_CUTOFF_PCT) / 100;
    uint32_t upperCutoff = ((uint32_t)smoothedPAR0 * UPPER_CUTOFF_PCT) / 100;

    // 3. Print clean CSV row
    Serial.printf("%lu,%u,%.1f,%u,%u,%u,%u\n", millis(), par0, smoothedPAR0, underPanelLight, targetLight, lowerCutoff, upperCutoff);

    // --- 4. LOWER CUTOFF CHECK (Under-panel too dark -> Try to increase light) ---
    if (underPanelLight < lowerCutoff) {
        while (underPanelLight < targetLight) {
            par0 = getOpenSkyPAR();
            underPanelLight = getUnderPanelPAR();
            
            smoothedPAR0 = (EMA_ALPHA * (float)par0) + ((1.0f - EMA_ALPHA) * smoothedPAR0);
            targetLight = ((uint32_t)smoothedPAR0 * TARGET_RATIO_PCT) / 100;
            
            Serial.printf("%lu,%u,%.1f,%u,%u,%u,%u\n", millis(), par0, smoothedPAR0, underPanelLight, targetLight, lowerCutoff, upperCutoff);

            if (underPanelLight >= targetLight) {
                break;
            }

            uint32_t startLight = underPanelLight;

            // --- STEP A: Rotate slightly Clockwise ---
            stepMotor(CW, SEARCH_STEP_SIZE);
            uint32_t cwLight = getUnderPanelPAR();

            if (cwLight >= targetLight) {
                break;
            }

            if (cwLight < startLight) {
                // --- STEP B: CW was worse. Return to start, then try CCW ---
                stepMotor(CCW, SEARCH_STEP_SIZE); 
                stepMotor(CCW, SEARCH_STEP_SIZE); 
                uint32_t ccwLight = getUnderPanelPAR();

                if (ccwLight >= targetLight) {
                    break;
                }

                if (ccwLight < startLight) {
                    // --- STEP C: Both CW and CCW lower than start -> Stay at starting position ---
                    stepMotor(CW, SEARCH_STEP_SIZE); 
                } else {
                    // --- STEP D: CCW got better -> Keep tracking CCW ---
                    uint32_t previousLight = ccwLight;
                    while (true) {
                        stepMotor(CCW, SEARCH_STEP_SIZE);
                        uint32_t nextLight = getUnderPanelPAR();
                        
                        par0 = getOpenSkyPAR();
                        smoothedPAR0 = (EMA_ALPHA * (float)par0) + ((1.0f - EMA_ALPHA) * smoothedPAR0);
                        targetLight = ((uint32_t)smoothedPAR0 * TARGET_RATIO_PCT) / 100;

                        if (nextLight >= targetLight) {
                            break;
                        }

                        if (nextLight <= previousLight) {
                            stepMotor(CW, SEARCH_STEP_SIZE); 
                            break;
                        }
                        previousLight = nextLight;
                    }
                }

            } else {
                // --- STEP E: CW improved -> Keep tracking CW ---
                uint32_t previousLight = cwLight;
                while (true) {
                    stepMotor(CW, SEARCH_STEP_SIZE);
                    uint32_t nextLight = getUnderPanelPAR();

                    par0 = getOpenSkyPAR();
                    smoothedPAR0 = (EMA_ALPHA * (float)par0) + ((1.0f - EMA_ALPHA) * smoothedPAR0);
                    targetLight = ((uint32_t)smoothedPAR0 * TARGET_RATIO_PCT) / 100;

                    if (nextLight >= targetLight) {
                        break;
                    }

                    if (nextLight <= previousLight) {
                        stepMotor(CCW, SEARCH_STEP_SIZE); 
                        break;
                    }
                    previousLight = nextLight;
                }
            }
        }
    }
    // --- 5. UPPER CUTOFF CHECK (Under-panel too bright -> Try to decrease light) ---
    else if (underPanelLight > upperCutoff) {
        while (underPanelLight > upperCutoff) {
            par0 = getOpenSkyPAR();
            underPanelLight = getUnderPanelPAR();
            
            smoothedPAR0 = (EMA_ALPHA * (float)par0) + ((1.0f - EMA_ALPHA) * smoothedPAR0);
            upperCutoff = ((uint32_t)smoothedPAR0 * UPPER_CUTOFF_PCT) / 100;

            Serial.printf("%lu,%u,%.1f,%u,%u,%u,%u\n", millis(), par0, smoothedPAR0, underPanelLight, targetLight, lowerCutoff, upperCutoff);

            if (underPanelLight <= upperCutoff) {
                break;
            }

            uint32_t startLight = underPanelLight;

            // --- STEP A: Rotate slightly Counter-Clockwise (CCW) first ---
            stepMotor(CCW, SEARCH_STEP_SIZE);
            uint32_t ccwLight = getUnderPanelPAR();

            if (ccwLight <= upperCutoff) {
                break;
            }

            if (ccwLight > startLight) {
                // --- STEP B: CCW increased PAR (worse). Return to start, then try CW ---
                stepMotor(CW, SEARCH_STEP_SIZE); 
                stepMotor(CW, SEARCH_STEP_SIZE); 
                uint32_t cwLight = getUnderPanelPAR();

                if (cwLight <= upperCutoff) {
                    break;
                }

                if (cwLight > startLight) {
                    // --- STEP C: Both CCW and CW higher than start -> Stay at starting position ---
                    stepMotor(CCW, SEARCH_STEP_SIZE); 
                } else {
                    // --- STEP D: CW got lower -> Keep tracking CW ---
                    uint32_t previousLight = cwLight;
                    while (true) {
                        stepMotor(CW, SEARCH_STEP_SIZE);
                        uint32_t nextLight = getUnderPanelPAR();

                        par0 = getOpenSkyPAR();
                        smoothedPAR0 = (EMA_ALPHA * (float)par0) + ((1.0f - EMA_ALPHA) * smoothedPAR0);
                        upperCutoff = ((uint32_t)smoothedPAR0 * UPPER_CUTOFF_PCT) / 100;

                        if (nextLight <= upperCutoff) {
                            break;
                        }

                        if (nextLight >= previousLight) {
                            stepMotor(CCW, SEARCH_STEP_SIZE); 
                            break;
                        }
                        previousLight = nextLight;
                    }
                }

            } else {
                // --- STEP E: CCW improved -> Keep tracking CCW ---
                uint32_t previousLight = ccwLight;
                while (true) {
                    stepMotor(CCW, SEARCH_STEP_SIZE);
                    uint32_t nextLight = getUnderPanelPAR();

                    par0 = getOpenSkyPAR();
                    smoothedPAR0 = (EMA_ALPHA * (float)par0) + ((1.0f - EMA_ALPHA) * smoothedPAR0);
                    upperCutoff = ((uint32_t)smoothedPAR0 * UPPER_CUTOFF_PCT) / 100;

                    if (nextLight <= upperCutoff) {
                        break;
                    }

                    if (nextLight >= previousLight) {
                        stepMotor(CW, SEARCH_STEP_SIZE); 
                        break;
                    }
                    previousLight = nextLight;
                }
            }
        }
    }

    delay(CHECK_INTERVAL_MS); // Wait 1 minute before next monitoring check
}

// --- HELPER FUNCTIONS ---

void tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

void stepMotor(bool direction, int steps) {
    digitalWrite(DIR_PIN, direction);
    for (int i = 0; i < steps; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(STEP_DELAY_US / 2);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(STEP_DELAY_US / 2);
    }
}

uint32_t readSensorOnChannel(uint8_t channel) {
    tcaSelect(channel);
    uint16_t readings[18];
    read_light(readings);
    return isolate_par(readings);
}

uint32_t getOpenSkyPAR() {
    return readSensorOnChannel(CHANNEL_PAR0);
}

uint32_t getUnderPanelPAR() {
    delay(50); 
    uint32_t total_par = 0;
    for (uint8_t i = 1; i <= 3; i++) {
        total_par += readSensorOnChannel(i);
    }
    return total_par / 3;
}

void read_light(uint16_t *readings) {
    if (!as7343.readAllChannels(readings)) {
        for(int i = 0; i < 18; i++) readings[i] = 0;
    }
}

uint32_t isolate_par(uint16_t *readings) {
    uint32_t par_sum = 0;
    par_sum += readings[AS7343_CHANNEL_F1];
    par_sum += readings[AS7343_CHANNEL_F2];
    par_sum += readings[AS7343_CHANNEL_FZ];
    par_sum += readings[AS7343_CHANNEL_F3];
    par_sum += readings[AS7343_CHANNEL_F4];
    par_sum += readings[AS7343_CHANNEL_F5];
    par_sum += readings[AS7343_CHANNEL_FY];
    par_sum += readings[AS7343_CHANNEL_FXL];
    par_sum += readings[AS7343_CHANNEL_F6];
    par_sum += readings[AS7343_CHANNEL_F7];
    par_sum += readings[AS7343_CHANNEL_F8];
    return par_sum;
}
/*
//Use this section for base case no rotation testing
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AS7343.h>

// --- TCA9548A I2C MULTIPLEXER DEFINITION ---
#define TCA_ADDR 0x70

Adafruit_AS7343 as7343;

// Function prototypes
void tcaSelect(uint8_t channel);
void read_light(uint16_t *readings);
uint32_t isolate_par(uint16_t *readings);
uint32_t getCleanLightReading();

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    delay(2000);
    Serial.println("Stationary Base Case Data Logger Initialization...");

    Wire.begin();

    // Initialize all 4 sensors across multiplexer channels 0 through 3
    for (uint8_t i = 0; i < 4; i++) {
        tcaSelect(i);
        delay(10); // Short stabilization delay after switching channels

        Serial.printf("Initializing Sensor on Channel %u...\n", i);
        if (!as7343.begin()) {
            Serial.printf("Could not find AS7343 sensor on channel %u!\n", i);
            while (true) { delay(10); }
        }
        
        // --- OUTDOOR OPTIMIZATION FOR BRIGHT SUNLIGHT ---
        as7343.setGain(AS7343_GAIN_0_5X);
        as7343.setATIME(29); 
        as7343.setASTEP(599);
        
        Serial.printf("Sensor on Channel %u initialized successfully!\n", i);
    }

    Serial.println("\nTimestamp_ms,Average_PAR_Value");
}

void loop() {
    // 1. Get current average light level across all 4 sensors for the stationary panel
    uint32_t currentLight = getCleanLightReading();
    
    // 2. Output CSV-friendly format (Millis timestamp, PAR value) for easy logging
    Serial.printf("%lu,%u\n", millis(), currentLight);

    delay(60000); 
}

// --- HELPER FUNCTIONS ---

// Select channel (0-7) on TCA9548A multiplexer
void tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

uint32_t getCleanLightReading() {
    delay(50); // Short settle delay
    
    uint32_t total_par = 0;

    // Loop through all 4 multiplexed channels and average their PAR values
    for (uint8_t i = 0; i < 4; i++) {
        tcaSelect(i);
        uint16_t readings[18];
        read_light(readings);
        total_par += isolate_par(readings);
    }

    return total_par / 4; // Return the average PAR across all 4 sensors
}

void read_light(uint16_t *readings) {
    if (!as7343.readAllChannels(readings)) {
        for(int i = 0; i < 18; i++) readings[i] = 0;
    }
}

uint32_t isolate_par(uint16_t *readings) {
    uint32_t par_sum = 0;
    par_sum += readings[AS7343_CHANNEL_F1];
    par_sum += readings[AS7343_CHANNEL_F2];
    par_sum += readings[AS7343_CHANNEL_FZ];
    par_sum += readings[AS7343_CHANNEL_F3];
    par_sum += readings[AS7343_CHANNEL_F4];
    par_sum += readings[AS7343_CHANNEL_F5];
    par_sum += readings[AS7343_CHANNEL_FY];
    par_sum += readings[AS7343_CHANNEL_FXL];
    par_sum += readings[AS7343_CHANNEL_F6];
    par_sum += readings[AS7343_CHANNEL_F7];
    par_sum += readings[AS7343_CHANNEL_F8];
    return par_sum;
}
*/