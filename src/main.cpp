#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AS7343.h>

Adafruit_AS7343 as7343;

void read_light(uint16_t *readings);
uint32_t isolate_par(uint16_t *readings);

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    delay(2000);
    Serial.println("Adafruit AS7343 Initialization...");

    Wire.begin();

    if (!as7343.begin()) {
        Serial.println("Could not find AS7343 sensor!");
        while (true) { delay(10); }
    }
    Serial.println("AS7343 found and initialized!");

    // --- OUTDOOR OPTIMIZATION FOR BRIGHT SUNLIGHT ---
    // Lower the analog sensitivity to prevent saturation (Options: 0.5X, 1X, 2X, 4X ... up to 256X)
    as7343.setGain(AS7343_GAIN_0_5X);
    
    // Adjust integration time steps (fewer cycles = shorter exposure time per reading)
    // Default is often higher; setting a lower step count keeps it responsive in direct sun.
    as7343.setATIME(29); 
    as7343.setASTEP(599);

    // Print CSV header once at startup for easy copy-pasting
    Serial.println("\nPAR_Value");
}

void loop() {
    uint16_t readings[18];
    
    read_light(readings);
    uint32_t PAR = isolate_par(readings);
    
    // Print just the raw numeric value for clean CSV logging
    Serial.printf("%u\n", PAR);

    delay(250);
}

void read_light(uint16_t *readings) {
    // Read all 18 channels into the buffer
    if (!as7343.readAllChannels(readings)) {
        Serial.println("Read failed!");
        return;
    }
}

uint32_t isolate_par(uint16_t *readings) {
    uint32_t par_sum = 0;

    // Aggregate primary visible spectrum channels (F1 through F8) to approximate PAR
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