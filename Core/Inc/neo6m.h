/**
 ******************************************************************************
 * @file           : neo6m.h
 * @brief          : NEO-6M GPS library and NMEA parser interface
 * @author         : Cem Ozan Yilmaz
 * @date           : 16.09.2026 (dd/mm/yyyy)
 ******************************************************************************
 * @details
 *
 * This header defines the public interface and data structures for the
 * STM32 HAL-based NEO-6M GPS library.
 *
 * It provides:
 * - NEO-6M library initialization
 * - UART receive callback handling
 * - NMEA sentence processing
 * - Access to parsed GPS data
 * - RMC position, time, date, speed, and course data
 * - GGA fix quality, satellite count, HDOP, and altitude data
 * - GSA fix type, satellite PRNs, PDOP, HDOP, and VDOP data
 * - GSV satellite PRN, elevation, azimuth, and SNR data
 *
 ******************************************************************************
 */
#ifndef NEO6M_H
#define NEO6M_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

#define NEO6M_MAX_GSV_SATELLITES 16

typedef struct
{
    uint8_t prn;
    uint8_t elevation;
    uint16_t azimuth;
    uint8_t snr;

} NEO6M_Satellite;

typedef struct 
{

    //RMC messages
    uint8_t valid;
    
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    float latitude;
    float longitude;

    float speed_knots;
    float course;

    uint8_t day;
    uint8_t month;
    uint8_t year;

    //GGA messages
    uint8_t fix_quality;
    uint8_t satellites;
    float hdop;
    float altitude;

    //GSA messages
    uint8_t fix_type;
    uint8_t satellite_prn[12];
    uint8_t satellite_prn_count;
    float pdop;
    float vdop;

    //GSV messages
    uint8_t satellites_in_view;

    uint8_t gsv_total_messages;
    uint8_t gsv_message_number;

    NEO6M_Satellite gsv_satellites[NEO6M_MAX_GSV_SATELLITES];
    uint8_t gsv_satellite_count;


} NEO6M_Data;

void NEO6M_Init(UART_HandleTypeDef *huart);
void NEO6M_RxCallback(void);
void NEO6M_Process(void);
NEO6M_Data NEO6M_GetData(void);

#endif
