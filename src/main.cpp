#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AS7343.h>

// --- TCA9548A I2C MULTIPLEXER DEFINITION ---
#define TCA_ADDR 0x70

// --- TMC2209 PIN DEFINITIONS (Adjust to your wiring on the RP2350B) ---
#define STEP_PIN   2
#define DIR_PIN    3
#define EN_PIN     4   // Optional: TMC2209 enable pin (active low)

// --- MOTOR SETTINGS ---
#define CW        HIGH
#define CCW       LOW
#define STEP_DELAY_US 1000 // Speed of motor steps (microseconds between pulses)
#define SEARCH_STEP_SIZE 50 // Number of steps for a "slight" rotation

Adafruit_AS7343 as7343;

// Function prototypes
void tcaSelect(uint8_t channel);
void read_light(uint16_t *readings);
uint32_t isolate_par(uint16_t *readings);
void stepMotor(bool direction, int steps);
uint32_t getCleanLightReading();

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    delay(2000);
    Serial.println("Multi-Sensor TCA9548A & TMC2209 Initialization...");

    // Configure Motor Pins
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW); // Enable driver (active low)

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

    Serial.println("\nAverage_PAR_Value");
}

void loop() {
    // 1. Get current average light level across all 4 sensors while stationary
    uint32_t currentLight = getCleanLightReading();
    Serial.printf("Current Average PAR: %u\n", currentLight);

    // 2. Check threshold trigger (< 750 average)
    if (currentLight < 750) {
        Serial.println("Average light dropped below 750. Initiating search...");

        uint32_t startLight = currentLight;

        // --- STEP A: Rotate slightly Clockwise ---
        stepMotor(CW, SEARCH_STEP_SIZE);
        uint32_t cwLight = getCleanLightReading();
        Serial.printf("After CW move - Average PAR: %u\n", cwLight);

        if (cwLight < startLight) {
            // --- STEP B: CW was worse. Return to start, then try CCW ---
            Serial.println("CW light is lower. Returning to start and trying CCW...");
            stepMotor(CCW, SEARCH_STEP_SIZE); // Back to start

            stepMotor(CCW, SEARCH_STEP_SIZE); // Move CCW relative to start
            uint32_t ccwLight = getCleanLightReading();
            Serial.printf("After CCW move - Average PAR: %u\n", ccwLight);

            if (ccwLight < startLight) {
                // --- STEP C: Both CW and CCW are lower than start -> Stay at starting position ---
                Serial.println("Both directions lower than start. Returning to starting position.");
                stepMotor(CW, SEARCH_STEP_SIZE); // Return to original position
            } else {
                // --- STEP D: CCW got better -> Keep tracking CCW until light stops increasing ---
                Serial.println("CCW improved. Tracking CCW...");
                uint32_t previousLight = ccwLight;
                while (true) {
                    stepMotor(CCW, SEARCH_STEP_SIZE);
                    uint32_t nextLight = getCleanLightReading();
                    Serial.printf("Tracking CCW - Average PAR: %u\n", nextLight);

                    if (nextLight <= previousLight) {
                        // Light stopped increasing or dropped, step back one increment to peak position
                        stepMotor(CW, SEARCH_STEP_SIZE); 
                        break;
                    }
                    previousLight = nextLight;
                }
            }

        } else {
            // --- STEP E: CW improved or stayed equal -> Keep tracking CW until light stops increasing ---
            Serial.println("CW improved. Tracking CW...");
            uint32_t previousLight = cwLight;
            while (true) {
                stepMotor(CW, SEARCH_STEP_SIZE);
                uint32_t nextLight = getCleanLightReading();
                Serial.printf("Tracking CW - Average PAR: %u\n", nextLight);

                if (nextLight <= previousLight) {
                    // Light stopped increasing or dropped, step back one increment to peak position
                    stepMotor(CCW, SEARCH_STEP_SIZE); 
                    break;
                }
                previousLight = nextLight;
            }
        }
        Serial.println("Search complete. Resuming monitoring...");
    }

    delay(250); // Normal check interval
}

// --- HELPER FUNCTIONS ---

// Select channel (0-7) on TCA9548A multiplexer
void tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

void stepMotor(bool direction, int steps) {
    digitalWrite(DIR_PIN, direction);
    // Motor is moving: NO sensor reads happen here
    for (int i = 0; i < steps; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(STEP_DELAY_US / 2);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(STEP_DELAY_US / 2);
    }
}

uint32_t getCleanLightReading() {
    delay(50); // Short settle delay after stopping motion
    
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