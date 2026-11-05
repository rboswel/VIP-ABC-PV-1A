#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AS7343.h>

// TCA9548A I2C Multiplexer default address
#define TCA_ADDR 0x70

// Create a single sensor instance; we will switch multiplexer channels before talking to it
Adafruit_AS7343 as7343;

// Function to select a specific channel (0-7) on the TCA9548A multiplexer
void tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

void read_light(uint16_t *readings);
uint32_t isolate_par(uint16_t *readings);

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    delay(2000);
    Serial.println("Multi-Sensor AS7343 Initialization...");

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

    // Print multi-column CSV header at startup
    Serial.println("\nSensor_0,Sensor_1,Sensor_2,Sensor_3");
}

void loop() {
    uint32_t par_values[4];

    // Loop through channels 0 to 3 to read each sensor
    for (uint8_t i = 0; i < 4; i++) {
        tcaSelect(i);
        
        uint16_t readings[18];
        read_light(readings);
        par_values[i] = isolate_par(readings);
    }

    // Print all 4 PAR values separated by commas for clean CSV logging
    Serial.printf("%u,%u,%u,%u\n", par_values[0], par_values[1], par_values[2], par_values[3]);

    delay(250);
}

void read_light(uint16_t *readings) {
    if (!as7343.readAllChannels(readings)) {
        // Zero out readings if read fails
        for(int i = 0; i < 18; i++) readings[i] = 0;
    }
}

uint32_t isolate_par(uint16_t *readings) {
    uint32_t par_sum = 0;

    // Aggregate primary visible spectrum channels (F1 through F8)
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