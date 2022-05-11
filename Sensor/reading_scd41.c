#include <stdio.h>
#include <stdint.h> //aggiunta dopo per vedere se cambia printf
#include "boards.h"
#include "nrf.h"
#include "nordic_common.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//SENSIRION's sensor libraries
#include "sensirion_common.h"
#include "sensirion_config.h"
#include "sensirion_i2c.h"
#include "sensirion_i2c_hal.h"
#include "scd4x_i2c.h"
#include "sps30.h"
#include "reading_scd41.h"

void lettura_scd41 (uint8_t campioni)
{
    int16_t error = 0;
    uint16_t status;
    uint16_t target_co2_concentration;
    uint16_t* frc_correction;

    target_co2_concentration = 400;

    scd4x_wake_up();
    scd4x_stop_periodic_measurement();
    //nrf_delay_ms(500);  //dopo stop aspettare almeno 500 ms
    scd4x_reinit();       //sembra non servire, non ho ancora modificato alcun parametro
    nrf_delay_ms(1000);
    
    error = scd4x_start_periodic_measurement();
    //error = scd4x_start_low_power_periodic_measurement();
    //nrf_delay_ms(1000);

    if(error != 0)  printf("errore\n");
    else    printf("Periodic measurement started\n\n");

    printf("Waiting for first measurement... (5 sec)\n\n");

    for (int i = 0; i < campioni; i++) 
    {
        // Read Measurement
        sensirion_i2c_hal_sleep_usec(50000);
        bool data_ready_flag = false;
        nrf_delay_ms(5000);
        error = scd4x_get_data_ready_flag(&data_ready_flag);

        if (error) 
        {
            printf("Error executing scd4x_get_data_ready_flag(): %i\n", error);
            continue;
        }
        if (!data_ready_flag) 
        {
            //nrf_delay_ms(500);
            printf("sono qua  \n");
            continue;
        }
        
        uint16_t co2;
        int32_t temperature;
        int32_t humidity;
        error = scd4x_read_measurement(&co2, &temperature, &humidity);
        if (error) 
        {
            printf("Error executing scd4x_read_measurement(): %i\n", error);
        } 
        else if (co2 == 0) 
        {
            printf("Invalid sample detected, skipping.\n");
        } 
        else 
        {
            printf("Measurement n° %d:\n",i+1);
            printf("CO2: %u ppm\n", co2);
            //printf("Temperature: %d m°C\n", temperature);
            //printf("Humidity: %d mRH\n\n\n", humidity);
        }

    }
    scd4x_stop_periodic_measurement();
    printf("FINE MISURAZIONI\n\n\n");
}