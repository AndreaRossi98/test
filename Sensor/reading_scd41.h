//reading scd41

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

void lettura_scd41 (uint8_t campioni);