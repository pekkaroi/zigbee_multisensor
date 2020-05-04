#ifndef ZIGBEE_CUSTOM_H__
#define ZIGBEE_CUSTOM_H__

#include "zb_zcl_power_config.h"


typedef struct
{
    zb_int16_t  measure_value;
    zb_int16_t  min_measure_value;
    zb_int16_t  max_measure_value;
    zb_uint16_t tolerance;
} zb_zcl_humidity_measurement_attrs_t;

typedef struct
{
    enum zb_zcl_occupancy_sensing_occupancy_e  occupancy;
    enum zb_zcl_occupancy_sensing_occupancy_sensor_type_e occupancy_sensor_type;
    zb_uint8_t occupancy_sensor_type_bitmap;
} zb_zcl_occupancy_attrs_t;

#define ZB_ZCL_DECLARE_OCCUPANCY_SENSING_ATTRIB_LIST_V2(attr_list, occupancy, occupancy_sensor_type) \
  ZB_ZCL_START_DECLARE_ATTRIB_LIST(attr_list)                                                                                   \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, (occupancy))                                                 \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ID, (occupancy_sensor_type))                         \
  ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST


//this is from: https://github.com/lmahmutov/nrf52840_zigbee_bh175

typedef struct
{
    zb_uint8_t battery_voltage;
    zb_uint8_t battery_remaining_percentage;
    zb_uint32_t alarm_state;
} battery_simplified_attr_t;

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_REMAINING_REPORTING_ID(data_ptr, bat_num) \
{                                                               \
  ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE##bat_num##_REMAINING_ID,     \
  ZB_ZCL_ATTR_TYPE_U8,                                          \
  ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING,  \
  (zb_voidp_t) data_ptr                                         \
}

#define ZB_ZCL_DECLARE_POWER_CONFIG_SIMPLIFIED_ATTRIB_LIST(attr_list,voltage, remaining, alarm_state)  \
  ZB_ZCL_START_DECLARE_ATTRIB_LIST(attr_list)                                             \
  ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID(voltage,),         \
  ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_REMAINING_REPORTING_ID(remaining,),    \
  ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_STATE_ID(alarm_state,), \
  ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST


#ifdef __cplusplus
}
#endif
#endif 