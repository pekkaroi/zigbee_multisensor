#include "sht31.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_sht31_config = {
       .scl                = NRF_GPIO_PIN_MAP(1,11),
       .sda                = NRF_GPIO_PIN_MAP(1,10),
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_sht31_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}


int16_t read_i2c_temp()
{
      uint8_t buf[3];
      buf[0] = 0xF3;
      nrf_drv_twi_tx(&m_twi, 0x40, buf, 1, 0);
      nrf_delay_us(85000);
      nrf_drv_twi_rx(&m_twi, 0x40, buf, 3);
      uint16_t temp_code = (buf[0] << 8) + (buf[1]&0xFC);

      float temp = -46.85F + 175.72F*(float)(temp_code)/65536.0F;
      NRF_LOG_INFO("Temperature: %d Celsius degrees.", (int)(temp*100));
      return (int16_t)(temp*100);
}
int16_t read_i2c_hum()
{
      uint8_t buf[3];
      buf[0] = 0xF5;
      nrf_drv_twi_tx(&m_twi, 0x40, buf, 1, 0);
      nrf_delay_us(30000);
      nrf_drv_twi_rx(&m_twi, 0x40, buf, 3);
      uint16_t temp_code = (buf[0] << 8) + (buf[1]&0xFC);

      float temp = -6 + 125*(float)(temp_code)/65536.0;
      NRF_LOG_INFO("Humidity: %d percent.", (int)temp);
      return (int16_t)(temp*100);
}