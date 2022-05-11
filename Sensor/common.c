/**
 * Copyright (C) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bme68x.h"
#include "bme68x_defs.h"
#include "common.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


/******************************************************************************/
/*!                 Macro definitions                                         */
/*! BME68X shuttle board ID */
#define BME68X_SHUTTLE_ID  0x93
#define SDA_PIN 18
#define SCL_PIN 19
/******************************************************************************/
/*!                Static variable definition                                 */
static uint8_t dev_addr;
static const nrf_drv_twi_t i2c_instance = NRF_DRV_TWI_INSTANCE(0);


/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */


//se metto dev_addr dice device not found, poi per� stampa valori a caso
//se metto reg_addr dice communication failure

BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr; 
    //int8_t err = nrf_drv_twi_rx(&i2c_instance, dev_addr, reg_data, len);
    //return err;

/*    versione 1: si blocca, non riesce a mettere a dormire il sensore
    for (int i=0; i<len; i++){ //Clears rx buffer
        reg_data[i] = 0;
    }

    ret_code_t err_code = nrf_drv_twi_tx(&i2c_instance, dev_addr, &reg_addr, 1, false);
    if (err_code == NRF_SUCCESS)
    { //NRF_ERROR_INVALID_ADDR
        ret_code_t err_code = nrf_drv_twi_rx(&i2c_instance, dev_addr, reg_data, len);       
        if (err_code == NRF_SUCCESS) 
            return BME68X_OK;
        else 
            return 1;
    }
    else return 1;
*/
   
    ret_code_t ret_code;
    ret_code = nrf_drv_twi_tx(&i2c_instance, dev_addr, &reg_addr,1,false);
    if(ret_code != NRF_SUCCESS)
    {
        return ret_code;
    }

    ret_code = nrf_drv_twi_rx(&i2c_instance,dev_addr, reg_data, len);
    if (ret_code == NRF_SUCCESS) 
        return BME68X_OK;
    else 
        return 1;

/*versione3
    uint8_t error_code;
    uint8_t whoAmIPointer = reg_addr;
    error_code = nrf_drv_twi_tx(&i2c_instance, dev_addr, &whoAmIPointer, 1, true);
    error_code = nrf_drv_twi_rx(&i2c_instance, dev_addr, &reg_data, 1);
    return error_code;  
    */ 
}


/*!
 * I2C write function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    //int8_t err = nrf_drv_twi_tx(&i2c_instance, dev_addr, reg_data, len, false);
    //return err;

/*  versione 1  
    uint8_t send_tmp[10] = {0};
    send_tmp[0] = reg_addr;
    memcpy(send_tmp+1, reg_data, len);
    ret_code_t err_code = nrf_drv_twi_tx(&i2c_instance, dev_addr, send_tmp, len+1, false);
    if (err_code == NRF_SUCCESS) return BME68X_OK;
    else return 1;
*/

    ret_code_t ret_code;
    ret_code = nrf_drv_twi_tx(&i2c_instance, dev_addr, reg_data, len , false); 
    if (ret_code == NRF_SUCCESS) 
        return BME68X_OK;
    else 
        return 1;

    
    /*versione 3
    uint8_t valueBuffer[2];
    valueBuffer[0] = reg_addr;
    valueBuffer[1] = reg_data;
    uint32_t err_code = nrf_drv_twi_tx(&i2c_instance, dev_addr, valueBuffer, sizeof(valueBuffer), false);
    return err_code;
    */
}


/*!
 * Delay function map to COINES platform
 */
void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    nrf_delay_us(period);
}


int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf)
{
    int8_t rslt = BME68X_OK;
    uint8_t reg_addr; 
    const uint8_t *reg_data;
    uint32_t len;
    void *intf_ptr;

    if (bme != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BME68X_I2C_INTF)
        {
            printf("\nI2C Interface\n");
            dev_addr = BME68X_I2C_ADDR_LOW;
            bme->read = bme68x_i2c_read;
            bme->write = bme68x_i2c_write;
            bme->intf = BME68X_I2C_INTF;
            //coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
           /* const nrf_drv_twi_config_t i2c_instance_config = {  .scl = SCL_PIN,
                                                                .sda = SDA_PIN,
                                                                .frequency = NRF_TWI_FREQ_100K,
                                                                .interrupt_priority = 0};
            rslt = nrf_drv_twi_init(&i2c_instance, &i2c_instance_config, NULL, NULL);
            //enable TWI instance
            nrf_drv_twi_enable(&i2c_instance);
    */
        }
        //coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
        //coines_delay_msec(100);
        nrf_delay_ms(100);
        //bme->chip_id = 0x61; //aggiunto per vedere se funziona
        bme->delay_us = bme68x_delay_us;
        bme->intf_ptr = &dev_addr;
        bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
    }
    else
    {
        rslt = BME68X_E_NULL_PTR;
    }

    return rslt;
}


void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:
            printf("API name [%s]: Tutto ok\n", api_name);
            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            printf("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            printf("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            printf("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            printf("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            printf("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            printf("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            break;
        default:
            printf("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

/*
void bme68x_coines_deinit(void)
{
    fflush(stdout);

    coines_set_shuttleboard_vdd_vddio_config(0, 0);
    coines_delay_msec(1000);

    // Coines interface reset
    coines_soft_reset();
    coines_delay_msec(1000);
    coines_close_comm_intf(COINES_COMM_INTF_USB);
}
*/