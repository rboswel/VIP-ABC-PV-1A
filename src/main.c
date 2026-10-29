#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Define I2C pins
#define I2C_PORT i2c0
#define PIN_SDA 4
#define PIN_SCL 5


bool as7343_init(i2c_inst_t *i2c);
bool as7343_read_channels(i2c_inst_t *i2c, uint16_t *readings);

int main()
{
    uint16_t readings[18];
    stdio_init_all();
    sleep_ms(2000);
    // 2. Initialize Pico Hardware I2C at 100 kHz
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    const char *filename = "data.csv";
    FILE *fp = fopen(filename, "a");
    if (fp == NULL) {
        printf("Error: Could not open file for appending.\n");
        return 1;
    }
    while(true)
    {
        uint16_t readings[12];
        read_light(readings);
        uint32_t PAR = isolate_par(readings);
        printf(PAR);
        fprintf(fp, "%d\n", PAR);
        //need some testing from here: What amount of PAR does the sensor get during the day at full brightness outdoors?
        //50-70% of that is the target as I recall. From there, we can begin determining when to rotate the panel. 
        //Actual panel solar collection data should be collected and implemented, but this will likely just be recorded
        //and not used to make rotational decisions.
        //TODO: Motor rotation code
        sleep_ms(250); //Readings taken every quarter second
    }
    fclose(fp);
    return(0);
}

//Code to read light sensors, code to get solar data from controller, code to drive motor
//need to find "sweet spot" amount of light to balance plant and panel light, panel should
//adjust to meet this 

void read_light(uint16_t readings[18])
{
    // Brief delay to allow the USB serial monitor to connect

    printf("Adafruit AS7343 Basic Readings (Pico SDK)\n");

    // 3. Initialize the sensor over I2C
    if (!as7343_init(I2C_PORT)) {
        printf("Could not find AS7343 sensor!\n");
        while (true) {
            sleep_ms(10);
        }
    }
    printf("AS7343 found!\n");

    // Configure sensor parameters (Gain, Integration Time cycles, Step size)
    // as7343_set_gain(I2C_PORT, AS7343_GAIN_64X);
    // as7343_set_atime(I2C_PORT, 29);  
    // as7343_set_astep(I2C_PORT, 599); 

    // Main application loop (replaces Arduino loop())

        // Read all spectral channels
    if (!as7343_read_channels(I2C_PORT, readings)) {
        printf("Read failed!\n");
        sleep_ms(500);
        return;
    }

    // Print spectral channels in wavelength order using printf
    printf("\n--- Spectral Readings ---\n");
    printf("F1  (405nm violet):       %u\n", readings[0]);
    printf("F2  (425nm violet-blue):  %u\n", readings[1]);
    printf("FZ  (450nm blue):         %u\n", readings[2]);
    printf("F3  (475nm blue-cyan):    %u\n", readings[3]);
    printf("F4  (515nm green):        %u\n", readings[4]);
    printf("F5  (550nm green-yel):    %u\n", readings[5]);
    printf("FY  (555nm yellow-grn):   %u\n", readings[6]);
    printf("FXL (600nm orange):       %u\n", readings[7]);
    printf("F6  (640nm red):          %u\n", readings[8]);
    printf("F7  (690nm deep red):     %u\n", readings[9]);
    printf("F8  (745nm near-IR):      %u\n", readings[10]);
    printf("NIR (855nm near-IR):      %u\n", readings[11]);
    printf("VIS (clear):              %u\n", readings[12]);


    sleep_ms(500);

}

uint32_t isolate_par(int* readings)
{
    uint32_t par_sum = 0;

    // Aggregate channels F1 through F7 (covering ~400nm to ~750nm)
    par_sum += readings[0];
    par_sum += readings[1];
    par_sum += readings[2];
    par_sum += readings[3];
    par_sum += readings[4];
    par_sum += readings[5];
    par_sum += readings[6];
    par_sum += readings[7];
    par_sum += readings[8];
    par_sum += readings[9];
    par_sum += readings[10];

    printf("Estimated PAR Intensity (400-700nm): %u\n", par_sum);
    return par_sum;
}

