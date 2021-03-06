/*
 * File Name: dis.h
 * Author: ram krishnan (rkris@wisense.in)
 * Created: Nov/16/2017
 *
 *
 * Copyright (c) <2018>, <ram krishnan>
 * All rights reserved.
 *
 * File cannot be copied and/or distributed without the express
 * permission of the author.
 */

#ifndef __DIS_H__
#define __DIS_H__

#define DIS_TLV_TYPE_FIELD_LEN    1
#define DIS_TLV_LENGTH_FIELD_LEN  1

#define DIS_LPWMN_MAC_SHORT_ADDR_LEN  2

#define DIS_TLV_HDR_SZ  (DIS_TLV_TYPE_FIELD_LEN  + DIS_TLV_LENGTH_FIELD_LEN)

#define DIS_SENSOR_ID_ALL  0xff

#define DIS_SENSOR_ID_FIELD_SZ           1
#define DIS_DATA_SCALE_FACTOR_FIELD_SZ   1
#define DIS_SENSOR_TYPE_FIELD_SZ         1
#define DIS_DATA_UNIT_FIELD_SZ           1
#define DIS_SENSOR_CNT_FIELD_SZ          1
#define DIS_SENSOR_OPN_MODE_BIT_MSK_SZ   1

#define DIS_NODE_PWR_SRC_FIELD_SZ        1
#define DIS_BATT_CAP_MAH_FIELD_SZ        2

#define DIS_STS_FIELD_SZ                 1

#define DIS_TIME_IN_NANOSECS_FIELD_SZ    2    // 0 - 999
#define DIS_TIME_IN_MICROSECS_FIELD_SZ   2    // 0 - 999
#define DIS_TIME_IN_MILLISECS_FIELD_SZ   2    // 0 - 999
#define DIS_TIME_IN_SECS_FIELD_SZ        1    // 0 - 59
#define DIS_TIME_IN_MINUTES_FIELD_SZ     1    // 0 - 59
#define DIS_TIME_IN_HOURS_FIELD_SZ       1    // 0 - 23
#define DIS_TIME_IN_DAYS_FIELD_SZ        1    // 0 - 255

#define DIS_SENSOR_COUNT_FIELD_SZ        1    // 0 - 255

#define DIS_SENSOR_ID_TLV_SZ  (DIS_TLV_HDR_SZ + DIS_SENSOR_ID_FIELD_SZ)
#define DIS_DATA_SCALE_FACTOR_TLV_SZ (DIS_TLV_HDR_SZ + DIS_DATA_SCALE_FACTOR_FIELD_SZ)
#define DIS_SENSOR_TYPE_TLV_SZ (DIS_TLV_HDR_SZ + DIS_SENSOR_TYPE_FIELD_SZ)
#define DIS_DATA_UNIT_TLV_SZ (DIS_TLV_HDR_SZ + DIS_DATA_UNIT_FIELD_SZ)
#define DIS_SENSOR_OPN_MODE_TLV_SZ (DIS_TLV_HDR_SZ + DIS_SENSOR_OPN_MODE_BIT_MSK_SZ) 
#define DIS_TIME_IN_NANOSECS_TLV_SZ      (DIS_TLV_HDR_SZ + 2)    // 0 - 999
#define DIS_TIME_IN_MICROSECS_TLV_SZ     (DIS_TLV_HDR_SZ + 2)    // 0 - 999
#define DIS_TIME_IN_MILLISECS_TLV_SZ     (DIS_TLV_HDR_SZ + 2)    // 0 - 999
#define DIS_TIME_IN_SECS_TLV_SZ          (DIS_TLV_HDR_SZ + 1)    // 0 - 59
#define DIS_TIME_IN_MINUTES_TLV_SZ       (DIS_TLV_HDR_SZ + 1)    // 0 - 59
#define DIS_TIME_IN_HOURS_TLV_SZ         (DIS_TLV_HDR_SZ + 1)    // 0 - 23
#define DIS_TIME_IN_DAYS_TLV_SZ          (DIS_TLV_HDR_SZ + 1)    // 0 - 255

#define DIS_STS_TLV_SZ  (DIS_TLV_HDR_SZ + DIS_STS_FIELD_SZ)
#define DIS_SENSOR_CNT_TLV_SZ  (DIS_TLV_HDR_SZ + DIS_SENSOR_CNT_FIELD_SZ)
#define DIS_PWR_SRC_TLV_SZ  (DIS_TLV_HDR_SZ + DIS_NODE_PWR_SRC_FIELD_SZ)
#define DIS_BATT_CAP_MAH_TLV_SZ (DIS_TLV_HDR_SZ + DIS_BATT_CAP_MAH_FIELD_SZ)

#define DIS_SENSOR_COUNT_TLV_SZ          (DIS_TLV_HDR_SZ + DIS_SENSOR_COUNT_FIELD_SZ)

#define DIS_SHORT_ADDR_TLV_SZ   (DIS_TLV_HDR_SZ + DIS_LPWMN_MAC_SHORT_ADDR_LEN)

#define DIS_DIGITAL_IO_PORT_FIELD_SZ  1
#define DIS_DIGITAL_IO_PIN_FIELD_SZ  1
#define DIS_DIGITAL_IO_VAL_FIELD_SZ  1

#define DIS_DIGITAL_IO_PORT_TLV_SZ   (DIS_TLV_HDR_SZ + DIS_DIGITAL_IO_PORT_FIELD_SZ)
#define DIS_DIGITAL_IO_PIN_TLV_SZ    (DIS_TLV_HDR_SZ + DIS_DIGITAL_IO_PIN_FIELD_SZ)
#define DIS_DIGITAL_IO_VAL_TLV_SZ    (DIS_TLV_HDR_SZ + DIS_DIGITAL_IO_VAL_FIELD_SZ)

#define DIS_GPIO_PORT_ID_FIELD_SZ  1
#define DIS_GPIO_PIN_ID_FIELD_SZ  1
#define DIS_GPIO_VAL_FIELD_SZ  1
#define DIS_GPIO_PORT_OP_STATE_FIELD_SZ  1

#define DIS_GPIO_PORT_ID_TLV_SZ   (DIS_TLV_HDR_SZ + DIS_GPIO_PORT_ID_FIELD_SZ)
#define DIS_GPIO_PIN_ID_TLV_SZ    (DIS_TLV_HDR_SZ + DIS_GPIO_PIN_ID_FIELD_SZ)
#define DIS_GPIO_VAL_TLV_SZ    (DIS_TLV_HDR_SZ + DIS_GPIO_VAL_FIELD_SZ)

#define DIS_GPIO_PORT_OP_STATE_TLV_SZ  (DIS_TLV_HDR_SZ + DIS_GPIO_PORT_OP_STATE_FIELD_SZ)

#define DIS_GPIO_CTRL_TLV_SZ (DIS_TLV_HDR_SZ \
                              + DIS_GPIO_PORT_ID_TLV_SZ \
                              + DIS_GPIO_PIN_ID_TLV_SZ \
                              + DIS_GPIO_VAL_TLV_SZ)

#define DIS_ATTR_ID_FIELD_SZ   2  

#define DIS_ATTR_ID_TLV_SZ   (DIS_TLV_HDR_SZ + DIS_ATTR_ID_FIELD_SZ) 

#define DIS_LIST_ATTR_ID_FIELD_SZ   2
#define DIS_LIST_ATTR_ID_TLV_SZ   (DIS_TLV_HDR_SZ + DIS_LIST_ATTR_ID_FIELD_SZ)

#define DIS_LIST_ATTR_IDX_FIELD_SZ   2
#define DIS_LIST_ATTR_IDX_TLV_SZ   (DIS_TLV_HDR_SZ + DIS_LIST_ATTR_IDX_FIELD_SZ)


#define DIS_MSG_TYPE_SZ   0x1

#define DIS_LQI_TLV_FIELD_SZ  1
#define DIS_RSSI_TLV_FIELD_SZ  1
#define DIS_RCVD_PKT_CNT_TLV_FIELD_SZ  2
#define DIS_SECONDS_SINCE_PKT_RX_TLV_FIELD_SZ  2

#define DIS_LQI_TLV_SZ  (DIS_TLV_HDR_SZ + DIS_LQI_TLV_FIELD_SZ)
#define DIS_RSSI_TLV_SZ  (DIS_TLV_HDR_SZ + DIS_RSSI_TLV_FIELD_SZ)
#define DIS_RCVD_PKT_CNT_TLV_SZ  (DIS_TLV_HDR_SZ + DIS_RCVD_PKT_CNT_TLV_FIELD_SZ)
#define DIS_SECONDS_SINCE_PKT_RX_TLV_SZ  (DIS_TLV_HDR_SZ + DIS_SECONDS_SINCE_PKT_RX_TLV_FIELD_SZ)


#define DIS_FU_PAGE_SZ_FIELD_LEN  1

#define DIS_TIME_STAMP_TLV_VAL_LEN   4
#define DIS_TIME_STAMP_TLV_SZ  (DIS_TLV_HDR_SZ + DIS_TIME_STAMP_TLV_VAL_LEN)

#define DIS_FU_PAGE_SZ_TLV_SZ  (DIS_TLV_HDR_SZ + DIS_FU_PAGE_SZ_FIELD_LEN)
#define DIS_UC_FU_STATE_TLV_SZ  (DIS_TLV_HDR_SZ + 1)
#define DIS_UC_FU_SEG_IDX_TLV_SZ  (DIS_TLV_HDR_SZ + 1)
#define DIS_UC_FU_PAGE_IDX_TLV_SZ  (DIS_TLV_HDR_SZ + 2)
#define DIS_UC_FU_RET_CODE_TLV_SZ  (DIS_TLV_HDR_SZ + 1)
#define DIS_FW_SEG_START_TLV_SZ  (DIS_TLV_HDR_SZ + 2)
#define DIS_FW_SEG_LEN_TLV_SZ  (DIS_TLV_HDR_SZ + 2)
#define DIS_FW_SEG_CRC_TLV_SZ  (DIS_TLV_HDR_SZ + 2)
#define DIS_UC_FU_PAGE_CRC_TLV_SZ (DIS_TLV_HDR_SZ + 2)

#define DIS_HOP_COUNT_TLV_SZ  (DIS_TLV_HDR_SZ + 1)

#define DIS_REBOOT_DELAY_TLV_SZ  (DIS_TLV_HDR_SZ + 2)

typedef enum
{
  DIS_MSG_TYPE_GET_NODE_CAP_INFO,
  DIS_MSG_TYPE_NODE_CAP_INFO,
  DIS_MSG_TYPE_GET_SENSOR_OUTPUT,
  DIS_MSG_TYPE_SENSOR_OUTPUT,
  DIS_MSG_TYPE_GET_SENSOR_DETAILS,
  DIS_MSG_TYPE_SENSOR_DETAILS,
  DIS_MSG_TYPE_CFG_SENSOR,
  DIS_MSG_TYPE_CFG_NODE_DATA_PUSH_INTERVAL,
  DIS_MSG_TYPE_ALARM,
  DIS_MSG_TYPE_GET_NODE_SENSOR_COUNT,
  DIS_MSG_TYPE_NODE_SENSOR_COUNT,
  DIS_MSG_TYPE_CTRL_DIGITAL_IO,
  DIS_MSG_TYPE_GET_RT_NEXT_HOP,
  DIS_MSG_TYPE_PING,
  DIS_MSG_TYPE_NODE_REBOOT_REQ,
  DIS_MSG_TYPE_GET_ATTR_VAL,
  DIS_MSG_TYPE_SET_ATTR_VAL,
  DIS_MSG_TYPE_CFG_NODE_DROP_BCN_REQS,
  DIS_MSG_TYPE_CFG_NODE_ACCEPT_BCN_REQS,
  DIS_MSG_TYPE_NODE_BCN_REQ_HANDLING_CFG,
  DIS_MSG_TYPE_GET_GPIO_PORT_OP_STATE,
  DIS_MSG_TYPE_RFD_KEEP_ALIVE_MSG,
  DIS_MSG_TYPE_KA,
  DIS_MSG_TYPE_GET_NEIGHBOR_TBL_ENTRY,
  DIS_MSG_TYPE_UC_FU_INIT_1,
  DIS_MSG_TYPE_UC_FU_INIT_2,
  DIS_MSG_TYPE_UC_FU_IMAGE_PAGE,
  DIS_MSG_TYPE_NODE_UC_FU_MODULE_INFO,
  DIS_MSG_TYPE_SWITCH_TO_SBY_IMAGE,
  DIS_MSG_TYPE_GET_LIST_ATTR_VAL,
  DIS_MSG_TYPE_NO_OP,
  DIS_MSG_TYPE_DELAYED_REBOOT_NODE_REQ,
  DIS_MSG_TYPE_STREAM_SEGMENT,
  DIS_MSG_TYPE_RADAR_RPT_VEH_DETECTED,
  DIS_MSG_TYPE_RADAR_RPT_NO_VEH_DETECTED,
  DIS_MSG_TYPE_ATUAT_TAG_BCN_INFO
} DIS_msgType_t;

typedef enum
{
  DIS_SENSOR_OUTPUT_ANALOG,
  DIS_SENSOR_OUTPUT_DIGITAL
} DIS_sensorOpType_t;

typedef enum
{
  DIS_CMD_TYPE_GET_DEV_DATA,
  DIS_CMD_TYPE_CFG_DEV
} DIS_cmdType_t;

typedef enum
{
  DIS_SENSOR_TYPE_DUMMY,
  DIS_SENSOR_TYPE_TIME,
  DIS_SENSOR_TYPE_BATTERY_VOLTAGE,
  DIS_SENSOR_TYPE_BATTERY_CURRENT,
  DIS_SENSOR_TYPE_POWER,
  DIS_SENSOR_TYPE_ENERGY,
  DIS_SENSOR_TYPE_TEMPERATURE,
  DIS_SENSOR_TYPE_PRESSURE,
  DIS_SENSOR_TYPE_HUMIDITY,
  DIS_SENSOR_TYPE_AMBIENT_LIGHT,
  DIS_SENSOR_TYPE_WEIGHT,
  DIS_SENSOR_TYPE_FORCE,
  DIS_SENSOR_TYPE_SOUND_LEVEL,
  DIS_SENSOR_TYPE_VIBRATION,
  DIS_SENSOR_TYPE_MOTION,
  DIS_SENSOR_TYPE_DISTANCE,
  DIS_SENSOR_TYPE_SPEED,
  DIS_SENSOR_TYPE_DIRECTION,
  DIS_SENSOR_TYPE_ACCELERATION,
  DIS_SENSOR_TYPE_COLOR,
  DIS_SENSOR_TYPE_MOISTURE,
  DIS_SENSOR_TYPE_RAIN,
  DIS_SENSOR_TYPE_RADIATION,
  DIS_SENSOR_TYPE_PROXIMITY,
  DIS_SENSOR_TYPE_COMPASS,
  DIS_SENSOR_TYPE_ALTITUDE,
  DIS_SENSOR_TYPE_DEPTH,
  DIS_SENSOR_TYPE_TILT,
  DIS_SENSOR_TYPE_FIELD,
  DIS_SENSOR_TYPE_IMAGE,
  DIS_SENSOR_TYPE_GLOBAL_POSITION,
  DIS_SENSOR_TYPE_SWITCH,
  DIS_SENSOR_TYPE_PH,
  DIS_SENSOR_TYPE_GAS_CO,
  DIS_SENSOR_TYPE_GAS_CO2,
  DIS_SENSOR_TYPE_GAS_O2,
  DIS_SENSOR_TYPE_GAS_CH4,
  DIS_SENSOR_TYPE_GAS_LPG,
  DIS_SENSOR_TYPE_GAS_OZONE
} DIS_sensorType_t;

typedef enum
{
  DIS_DATA_UNIT_COUNTER,
  DIS_DATA_UNIT_SECOND,
  DIS_DATA_UNIT_VOLT,
  DIS_DATA_UNIT_AMPERE,
  DIS_DATA_UNIT_OHM,
  DIS_DATA_UNIT_WATT,
  DIS_DATA_UNIT_JOULE,
  DIS_DATA_UNIT_CELCIUS,
  DIS_DATA_UNIT_FARENHEIT,
  DIS_DATA_UNIT_GRAM,
  DIS_DATA_UNIT_METER,
  DIS_DATA_UNIT_PASCAL,
  DIS_DATA_UNIT_NEWTON,
  DIS_DATA_UNIT_LUX,    // lux - lumens per square meter
  DIS_DATA_UNIT_ARC_DEGREE,
  DIS_DATA_UNIT_PPM,
  DIS_DATA_UNIT_PERCENT,
  DIS_DATA_UNIT_BINARY
} DIS_measUnit_t;

typedef enum
{
  DIS_DATA_SCALE_TERA,
  DIS_DATA_SCALE_GIGA,
  DIS_DATA_SCALE_MEGA,
  DIS_DATA_SCALE_KILO,
  DIS_DATA_SCALE_HECTO,
  DIS_DATA_SCALE_DEKA,
  DIS_DATA_SCALE_NONE,
  DIS_DATA_SCALE_DECI,
  DIS_DATA_SCALE_CENTI,
  DIS_DATA_SCALE_MILLI,
  DIS_DATA_SCALE_TENTH_MILLI,
  DIS_DATA_SCALE_MICRO,
  DIS_DATA_SCALE_NANO,
  DIS_DATA_SCALE_PICO,
  DIS_DATA_SCALE_FEMTO
} DIS_dataScale_t;


typedef enum
{
  DIS_SENSOR_STATE_ACTIVE,
  DIS_SENSOR_STATE_STANDBY,
  DIS_SENSOR_STATE_SHUTDOWN,
  DIS_SENSOR_STATE_ERROR
} DIS_sensorState_t;

typedef enum
{
  DIS_NODE_POWER_SRC_MAINS,
  DIS_NODE_POWER_SRC_BATTERY,
  DIS_NODE_POWER_SRC_SOLAR
} DIS_nodePowerSrc_t;


/*
 * Note that the operating modes specified below are not mutually exclusive.
 */
typedef enum
{
  DIS_SENSOR_OPN_MODE_PULL          = 0,   // User has to pull this sensor's data from host node
  DIS_SENSOR_OPN_MODE_PUSH_PERIODIC = 1,   // Host node will periodically send sensor data to user
  DIS_SENSOR_OPN_MODE_PUSH_ALARM    = 2    // Host node will send an alarm if sensor data crosses
                                           // configured threshold (low or high)
} DIS_sensorOpnMode_t;

typedef enum
{
  DIS_STS_SUCCESS,
  DIS_STS_INV_PARAM,
  DIS_STS_INV_TLV_SZ,
  DIS_STS_INV_MSG,
  DIS_STS_INV_SENSOR_ID,
  DIS_STS_SENSOR_ID_TLV_MISSING,
  DIS_STS_REQ_IN_PROGRESS,
  DIS_STS_HW_ERROR
} DIS_sts_t;

typedef enum
{
  DIS_TLV_TYPE_UNKNOWN,

  DIS_TLV_TYPE_STATUS,

  DIS_TLV_TYPE_VALUE,
  DIS_TLV_TYPE_DATA_UNIT,
  DIS_TLV_TYPE_DATA_SCALE_FACTOR,
  DIS_TLV_TYPE_DATA_RANGE_MIN,
  DIS_TLV_TYPE_DATA_RANGE_MAX,
  DIS_TLV_TYPE_DATA_THRESHOLD_HIGH,
  DIS_TLV_TYPE_DATA_THRESHOLD_LOW,
  DIS_TLV_TYPE_DATA_TIMESTAMP,
  DIS_TLV_TYPE_DATA_AGE,

  DIS_TLV_TYPE_NODE_CAP_INFO = 11,
  DIS_TLV_TYPE_NODE_PWR_SRC,

  DIS_TLV_TYPE_BATT_CAP_MAH = 13,
 
  DIS_TLV_TYPE_SENSOR_COUNT = 14,
  DIS_TLV_TYPE_ACTUATOR_COUNT = 15,

  DIS_TLV_TYPE_SENSOR_OUTPUT_LIST = 16,
  DIS_TLV_TYPE_SENSOR_OUTPUT = 17,

  DIS_TLV_TYPE_SENSOR_DETAILS,

  DIS_TLV_TYPE_SENSOR_ID_LIST,
  DIS_TLV_TYPE_SENSOR_ID,
  DIS_TLV_TYPE_SENSOR_OPN_MODE,
  DIS_TLV_TYPE_SENSOR_TYPE,
  DIS_TLV_TYPE_SENSOR_STATE,
  DIS_TLV_TYPE_SENSOR_ACTIVE_TIME,
  DIS_TLV_TYPE_SENSOR_ACTIVE_POWER,
  DIS_TLV_TYPE_SENSOR_STANDBY_POWER,
  DIS_TLV_TYPE_SENSOR_PART_NR,
  DIS_TLV_TYPE_SENSOR_MFR,

  DIS_TLV_TYPE_SENSOR_MIN_OUTPUT_VAL,
  DIS_TLV_TYPE_SENSOR_MAX_OUTPUT_VAL,
   
  DIS_TLV_TYPE_PUSH_INTERVAL,

  DIS_TLV_TYPE_ALARM_RATE,
   
  DIS_TLV_TYPE_TIME_IN_NANOSECS,
  DIS_TLV_TYPE_TIME_IN_MICROSECS,
  DIS_TLV_TYPE_TIME_IN_MILLISECS,
  DIS_TLV_TYPE_TIME_IN_SECS,
  DIS_TLV_TYPE_TIME_IN_MINUTES,
  DIS_TLV_TYPE_TIME_IN_HOURS,
  DIS_TLV_TYPE_TIME_IN_DAYS,

  DIS_TLV_TYPE_WALL_CLOCK_TIME,  // Year/month/day/hour/minutes/second/.....
  DIS_TLV_TYPE_CALENDAR_YEAR,    // Example - 2013
  DIS_TLV_TYPE_CALENDAR_MONTH,   // 1 - 12
  DIS_TLV_TYPE_CALENDAR_DAY,     // 1 - 31
  DIS_TLV_TYPE_HOUR,             // 0 - 23
  DIS_TLV_TYPE_MINUTE,           // 0 - 59
  DIS_TLV_TYPE_SECOND,           // 0 - 59

  DIS_TLV_TYPE_SENSOR_SAMPLE_SEQ_NR = 0x2f,

  DIS_TLV_TYPE_SENSOR_CFG_INFO = 0x30,

  DIS_TLV_TYPE_DIGITAL_IO_PORT = 0x31,
  DIS_TLV_TYPE_DIGITAL_IO_PIN = 0x32,
  DIS_TLV_TYPE_DIGITAL_IO_VAL = 0x33,
  DIS_TLV_TYPE_DIGITAL_IO = 0x34,

  DIS_TLV_TYPE_GPIO_PORT_ID = 0x31,
  DIS_TLV_TYPE_GPIO_PIN_ID = 0x32,
  DIS_TLV_TYPE_GPIO_VAL = 0x33,
  DIS_TLV_TYPE_GPIO_CTRL = 0x34,

  DIS_TLV_TYPE_GPIO_PORT_INFO = 0x38,
  DIS_TLV_TYPE_GPIO_PORT_OP_STATE =  0x39,
  DIS_TLV_TYPE_GPIO_PORT_INFO_LIST = 0x3a,


  DIS_TLV_TYPE_SHORT_ADDR = 0x40,

  DIS_TLV_TYPE_ATTR_ID,
  DIS_TLV_TYPE_ATTR_VAL,
  DIS_TLV_TYPE_ATTR_INFO,

  DIS_TLV_TYPE_LIST_ATTR_ID,
  DIS_TLV_TYPE_LIST_ATTR_IDX,
  DIS_TLV_TYPE_LIST_ATTR_INFO,

  DIS_TLV_TYPE_MAC_NEIGHBOR_TBL_ENTRY = 0x4f,
  DIS_TLV_TYPE_LQI  = 0x50,
  DIS_TLV_TYPE_RSSI = 0x51,
  DIS_TLV_TYPE_RCVD_PKT_CNT = 0x52,
  DIS_TLV_TYPE_SECONDS_SINCE_PKT_RX = 0x53,


  DIS_TLV_TYPE_FW_SEG_LIST = 0x60,
  DIS_TLV_TYPE_FW_SEG_INFO = 0x61,
  DIS_TLV_TYPE_FW_SEG_LEN = 0x62,
  DIS_TLV_TYPE_FW_SEG_START = 0x63,
  DIS_TLV_TYPE_FW_SEG_CRC = 0x64,
  DIS_TLV_TYPE_FU_PAGE_SZ = 0x65,
  DIS_TLV_TYPE_UC_FU_SESSION_PARAMS = 0x66,
  DIS_TLV_TYPE_UC_FU_STATE = 0x67,
  DIS_TLV_TYPE_UC_FU_SEG_IDX = 0x68,
  DIS_TLV_TYPE_UC_FU_PAGE_IDX = 0x69,
  DIS_TLV_TYPE_UC_FU_RET_CODE = 0x6a,
  DIS_TLV_TYPE_NODE_UC_FU_MODULE_INFO = 0x6b,
  DIS_TLV_TYPE_UC_FU_PAGE_CRC = 0x6c,

  DIS_TLV_TYPE_TIME_STAMP = 0x70,

  DIS_TLV_TYPE_REBOOT_DELAY = 0x80,
  
  
  DIS_TLV_TYPE_ATUAT_TAG_BCN_INFO = 0x90,
  DIS_TLV_TYPE_ATUAT_TAG_BCN_INFO_LIST = 0x91
} DIS_tlvType_t;


/*
 *
 * Examples :
 *
 * -------------------------------------------------------------------------
 *
 *
 * Type - DIS_TLV_TYPE_GET_SENSOR_DATA_CMD
 * Length - x
 * Value -
 *    Type - DIS_TLV_TYPE_SENSOR_ID
 *    Length - 1
 *    Value - (0 to 254)  // Allows 255 sensors on one node  (255 - all sensors)
 *
 *
 * Type - DIS_TLV_TYPE_SENSOR_DATA
 * Length - n
 * Value -
 *          Type - DIS_TLV_TYPE_SENSOR_ID
 *          Length - 1
 *          Value - (0 to 254)
 * 
 *          Type - DIS_TLV_TYPE_VALUE
 *          Length - N
 *          Value - From sensor
 *
 *          Type - DIS_TLV_TYPE_DATA_SCALE_FACTOR
 *          Length - 1
 *          Value - (Example - DIS_DATA_SCALE_CENTI (1023 => 10.23 deg C))
 *
 *
 * -------------------------------------------------------------------------
 *
 *
 * Type - DIS_TLV_TYPE_GET_DEVICE_CAP_CMD
 * Length - 0
 *
 * Type - DIS_TLV_TYPE_GET_DEVICE_CAP
 * Length - n
 * Value -
 *       Type - DIS_TLV_TYPE_SENSOR_COUNT
 *       Length - 1
 *       Value - Number of sensors on the device
 *
 *       Type - DIS_TLV_TYPE_ACTUATOR_COUNT
 *       Length - 1
 *       Value - Number of actuators on the device
 *
 *
 * -------------------------------------------------------------------------
 *
 *  
 * Type - DIS_TLV_TYPE_GET_SENSOR_DETAILS_CMD
 * Length - 0
 * Value -
 *          Type - DIS_TLV_TYPE_SENSOR_ID
 *          Length - 1
 *          Value - (0 to 254)
 *  
 * Type - DIS_TLV_TYPE_SENSOR_DETAILS
 * Length - 0
 * Value -
 *          Type - DIS_TLV_TYPE_SENSOR_ID
 *          Length - 1
 *          Value - (0 to 254)
 *
 *          Type - DIS_TLV_TYPE_SENSOR_TYPE
 *          Length - 1
 *          Value - (Example - DIS_SENSOR_TYPE_TEMPERATURE)
 *
 *          Type - DIS_TLV_TYPE_DATA_UNIT
 *          Length - 1
 *          Value - (Example - DIS_DATA_UNIT_CELCIUS)
 *
 *          Type - DIS_TLV_TYPE_SENSOR_ACTIVE_TIME,
 *          Length - 0
 *                Type - DIS_TLV_TYPE_TIME_IN_MILLISECS
 *
 *          Type - DIS_TLV_TYPE_SENSOR_MFR,
 *          Length - N
 *          Value - "Text"  (Example - "NXP")
 *
 *          Type - DIS_TLV_TYPE_SENSOR_VERSION,
 *          Length - 0
 *          Value - "Text"
 *
 *          Type - DIS_TLV_TYPE_SENSOR_HIGH_THRESHOLD,
 *          Length - 0
 *          Value - 
 *
 *
 * Type - DIS_TLV_TYPE_GET_SENSOR_POWER_CONSUMPTION
 * Length - 0
 * Value -
 *          Type - DIS_TLV_TYPE_SENSOR_ID
 *          Length - 1
 *          Value - (0 to 254)
 *  
 * Type - DIS_TLV_TYPE_SENSOR_POWER_CONSUMPTION
 * Length - 0
 * Value -
 *          Type - DIS_TLV_TYPE_SENSOR_ID
 *          Length - 1
 *          Value - (0 to 254)
 *
 *          Type - DIS_TLV_TYPE_SENSOR_OPERATING_VOLTAGE
 *          Length - 1
 *          Value - 
 *               Type - DIS_TLV_TYPE_VALUE
 *               Length - n
 *               Value -
 *         
 *               Type - DIS_TLV_TYPE_DATA_SCALE_FACTOR
 *               Length - 1
 *               Value -  
 *
 *          Type - DIS_TLV_TYPE_SENSOR_ACTIVE_POWER
 *          Length - 0
 *          Value -
 *               Type - DIS_TLV_TYPE_VALUE
 *               Length - n
 *               Value -  Watts
 *
 *               Type - DIS_TLV_TYPE_DATA_SCALE_FACTOR
 *               Length - 1
 *               Value -  Example (DIS_DATA_SCALE_MILLI implies value in mW)
 *
 *
 *          Type - DIS_TLV_TYPE_SENSOR_STANDBY_POWER,
 *          Length - 0
 *          Value -
 *               Type - DIS_TLV_TYPE_VALUE
 *               Length - n
 *               Value -  Watts
 *
 *               Type - DIS_TLV_TYPE_DATA_SCALE_FACTOR
 *               Length - 1
 *               Value -  Example (DIS_DATA_SCALE_MILLI implies value in mW)
 *
 * -------------------------------------------------------------------------
 */

#endif
