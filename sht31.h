#ifndef SHT31_H
#define SHT31_H

#include "nrf_drv_twi.h"

#define TWI_INSTANCE_ID 0
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);


void twi_init (void);
uint8_t init_tsensor();
int16_t read_i2c_temp();
int16_t read_i2c_hum();


#endif