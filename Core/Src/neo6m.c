/**
 ******************************************************************************
 * @file           : neo6m.c
 * @brief          : NEO-6M GPS library and NMEA parser implementation
 * @author         : Cem Ozan Yilmaz
 * @date           : 16.09.2026 (dd/mm/yyyy)
 ******************************************************************************
 * @details
 *
 * This file implements the NEO-6M GPS library for STM32 HAL.
 *
 * The library:
 * - Receives GPS data using interrupt-driven UART communication
 * - Collects incoming bytes into complete NMEA sentences
 * - Stores complete NMEA sentences in a queue for deferred processing
 * - Parses RMC sentences for position, time, date, speed, and course
 * - Parses GGA sentences for fix quality, satellites used, HDOP, and altitude
 * - Parses GSA sentences for fix type, satellite PRNs, PDOP, HDOP, and VDOP
 * - Parses multi-message GSV sentences for satellites in view
 * - Extracts satellite PRN, elevation, azimuth, and SNR information
 *
 * NMEA parsing is performed outside the UART interrupt to keep interrupt
 * processing short and avoid performing complex operations inside the ISR.
 *
 ******************************************************************************
 */
#include <string.h>
#include <stdlib.h>
#include "neo6m.h"


#define NEO6M_SENTENCE_LENGTH     128
#define NEO6M_QUEUE_SIZE          8


static UART_HandleTypeDef *neo6m_uart;

static uint8_t rx_byte;

/* Current UART receive buffer */
static char rx_buffer[NEO6M_SENTENCE_LENGTH];
static uint16_t rx_index = 0;


/*
 * ============================================================
 * NMEA SENTENCE QUEUE
 * ============================================================
 *
 * UART interrupt stores complete NMEA sentences here.
 *
 * ISR writes using queue_write_index.
 * NEO6M_Process() reads using queue_read_index.
 */
static char sentence_queue[NEO6M_QUEUE_SIZE][NEO6M_SENTENCE_LENGTH];

static volatile uint8_t queue_write_index = 0;
static volatile uint8_t queue_read_index = 0;


/* Current GPS data */
static NEO6M_Data gps;


/*
 * ============================================================
 * INITIALIZATION
 * ============================================================
 */
void NEO6M_Init(UART_HandleTypeDef *huart)
{
    neo6m_uart = huart;

    /* Start receiving one byte using UART interrupt */
    HAL_UART_Receive_IT(neo6m_uart, &rx_byte, 1);
}


/*
 * ============================================================
 * UART RECEIVE CALLBACK
 * ============================================================
 *
 * This function must be called from HAL_UART_RxCpltCallback().
 *
 * Incoming bytes are collected until '\n' is received.
 *
 * When a complete NMEA sentence is received, it is placed
 * into the sentence queue.
 *
 * Parsing is NOT performed inside the interrupt.
 */
void NEO6M_RxCallback(void)
{
    if (rx_byte == '\n')
    {
        /* Terminate received NMEA sentence */
        rx_buffer[rx_index] = '\0';

        /*
         * Calculate next queue position.
         */
        uint8_t next_write_index = (queue_write_index + 1) % NEO6M_QUEUE_SIZE;

        /*
         * If next write position is equal to read position,
         * queue is full.
         *
         * In this case the sentence is discarded.
         */
        if (next_write_index != queue_read_index)
        {
            strcpy(sentence_queue[queue_write_index], rx_buffer);

            queue_write_index = next_write_index;
        }

        /* Prepare receive buffer for next sentence */
        rx_index = 0;
    }
    else
    {
        /*
         * Store incoming byte.
         *
         * Leave one byte for '\0'.
         */
        if (rx_index < NEO6M_SENTENCE_LENGTH - 1)
        {
            rx_buffer[rx_index] = (char)rx_byte;
            rx_index++;
        }
    }

    /* Receive next byte */
    HAL_UART_Receive_IT(neo6m_uart, &rx_byte, 1);
}


/*
 * ============================================================
 * RMC PARSER
 * ============================================================
 */
static void NEO6M_ParseRMC(char *sentence)
{
    uint8_t field_index = 0;

    for (uint16_t buffer_index = 0;
         sentence[buffer_index] != '\0';
         buffer_index++)
    {
        if (sentence[buffer_index] == ',')
        {
            field_index++;


            /*
             * Field 1
             * UTC Time
             */
            if (field_index == 1)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    gps.hour =
                        (sentence[buffer_index + 1] - '0') * 10 +
                        (sentence[buffer_index + 2] - '0');

                    gps.minute =
                        (sentence[buffer_index + 3] - '0') * 10 +
                        (sentence[buffer_index + 4] - '0');

                    gps.second =
                        (sentence[buffer_index + 5] - '0') * 10 +
                        (sentence[buffer_index + 6] - '0');
                }
            }


            /*
             * Field 2
             * GPS Status
             *
             * A = Active
             * V = Void
             */
            if (field_index == 2)
            {
                if (sentence[buffer_index + 1] == 'A')
                {
                    gps.valid = 1;
                }
                else
                {
                    gps.valid = 0;
                }
            }


            /*
             * Field 3
             * Latitude
             *
             * NMEA:
             * ddmm.mmmm
             *
             * Converted:
             * decimal degrees
             */
            if (field_index == 3)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    float raw_latitude =
                        strtof(&sentence[buffer_index + 1], NULL);

                    int latitude_degrees = (int)(raw_latitude / 100.0f);

                    float latitude_minutes = raw_latitude - (latitude_degrees * 100.0f);

                    gps.latitude = latitude_degrees + (latitude_minutes / 60.0f);
                }
            }


            /*
             * Field 4
             * North / South
             */
            if (field_index == 4)
            {
                if (sentence[buffer_index + 1] == 'S')
                {
                    gps.latitude = -gps.latitude;
                }
            }


            /*
             * Field 5
             * Longitude
             *
             * NMEA:
             * dddmm.mmmm
             *
             * Converted:
             * decimal degrees
             */
            if (field_index == 5)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    float raw_longitude = strtof(&sentence[buffer_index + 1], NULL);

                    int longitude_degrees = (int)(raw_longitude / 100.0f);

                    float longitude_minutes = raw_longitude - (longitude_degrees * 100.0f);

                    gps.longitude = longitude_degrees + (longitude_minutes / 60.0f);
                }
            }


            /*
             * Field 6
             * East / West
             */
            if (field_index == 6)
            {
                if (sentence[buffer_index + 1] == 'W')
                {
                    gps.longitude = -gps.longitude;
                }
            }


            /*
             * Field 7
             * Speed over ground
             *
             * Unit:
             * knots
             */
            if (field_index == 7)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    gps.speed_knots = strtof(&sentence[buffer_index + 1], NULL);
                }
            }


            /*
             * Field 8
             * Course over ground
             *
             * Unit:
             * degrees
             */
            if (field_index == 8)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    gps.course = strtof(&sentence[buffer_index + 1], NULL);
                }
            }


            /*
             * Field 9
             * Date
             *
             * DDMMYY
             */
            if (field_index == 9)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    gps.day =
                        (sentence[buffer_index + 1] - '0') * 10 +
                        (sentence[buffer_index + 2] - '0');

                    gps.month =
                        (sentence[buffer_index + 3] - '0') * 10 +
                        (sentence[buffer_index + 4] - '0');

                    gps.year =
                        (sentence[buffer_index + 5] - '0') * 10 +
                        (sentence[buffer_index + 6] - '0');
                }
            }
        }
    }
}


/*
 * ============================================================
 * GGA PARSER
 * ============================================================
 */
static void NEO6M_ParseGGA(char *sentence)
{
    uint8_t field_index = 0;

    for (uint16_t buffer_index = 0;
         sentence[buffer_index] != '\0';
         buffer_index++)
    {
        if (sentence[buffer_index] == ',')
        {
            field_index++;


            /*
             * Field 6
             * Fix Quality
             *
             * 0 = Invalid
             * 1 = GPS fix
             * 2 = DGPS fix
             */
            if (field_index == 6)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    gps.fix_quality =
                        (uint8_t)atoi(&sentence[buffer_index + 1]);
                }
            }


            /*
             * Field 7
             * Number of satellites used
             */
            if (field_index == 7)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    gps.satellites = (uint8_t)atoi(&sentence[buffer_index + 1]);
                }
            }


            /*
             * Field 8
             * HDOP
             */
            if (field_index == 8)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    gps.hdop = strtof( &sentence[buffer_index + 1], NULL);
                }
            }


            /*
             * Field 9
             * Altitude above mean sea level
             *
             * Unit:
             * meters
             */
            if (field_index == 9)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    gps.altitude = strtof(&sentence[buffer_index + 1], NULL);
                }
            }
        }
    }
}


/*
 * ============================================================
 * GSA PARSER
 * ============================================================
 */
static void NEO6M_ParseGSA(char *sentence)
{
    uint8_t field_index = 0;

    /*
     * Reset satellite list before reading
     * the latest GSA sentence.
     */
    gps.satellite_prn_count = 0;

    for (uint16_t buffer_index = 0;
         sentence[buffer_index] != '\0';
         buffer_index++)
    {
        if (sentence[buffer_index] == ',')
        {
            field_index++;


            /*
             * Field 2
             * Fix Type
             *
             * 1 = No fix
             * 2 = 2D fix
             * 3 = 3D fix
             */
            if (field_index == 2)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    gps.fix_type = (uint8_t)atoi(&sentence[buffer_index + 1]);
                }
            }


            /*
             * Fields 3 - 14
             *
             * PRNs of satellites used
             * in position solution.
             */
            if (field_index >= 3 &&
                field_index <= 14)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    if (gps.satellite_prn_count < 12)
                    {
                        gps.satellite_prn[gps.satellite_prn_count] = (uint8_t)atoi(&sentence[buffer_index + 1]);

                        gps.satellite_prn_count++;
                    }
                }
            }


            /*
             * Field 15
             * PDOP
             */
            if (field_index == 15)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    gps.pdop = strtof(&sentence[buffer_index + 1], NULL);
                }
            }


            /*
             * Field 16
             * HDOP
             */
            if (field_index == 16)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    gps.hdop = strtof(&sentence[buffer_index + 1], NULL);
                }
            }


            /*
             * Field 17
             * VDOP
             */
            if (field_index == 17)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    gps.vdop = strtof(&sentence[buffer_index + 1], NULL);
                }
            }
        }
    }
}


/*
 * ============================================================
 * GSV PARSER
 * ============================================================
 *
 * GSV may consist of multiple sentences.
 *
 * Example:
 *
 * $GPGSV,4,1,13,...
 * $GPGSV,4,2,13,...
 * $GPGSV,4,3,13,...
 * $GPGSV,4,4,13,...
 *
 * Each sentence describes a maximum of four satellites.
 */
static void NEO6M_ParseGSV(char *sentence)
{
    uint8_t field_index = 0;

    uint8_t total_messages = 0;
    uint8_t message_number = 0;

    for (uint16_t buffer_index = 0;
         sentence[buffer_index] != '\0';
         buffer_index++)
    {
        if (sentence[buffer_index] == ',')
        {
            field_index++;


            /*
             * Field 1
             * Total number of GSV messages
             */
            if (field_index == 1)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    total_messages = (uint8_t)atoi(&sentence[buffer_index + 1]);

                    gps.gsv_total_messages = total_messages;
                }
            }


            /*
             * Field 2
             * Current message number
             */
            if (field_index == 2)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    message_number = (uint8_t)atoi(&sentence[buffer_index + 1]);

                    gps.gsv_message_number = message_number;

                    /*
                     * Message 1 means that a new
                     * GSV sequence has started.
                     */
                    if (message_number == 1)
                    {
                        gps.gsv_satellite_count = 0;

                        memset(gps.gsv_satellites, 0, sizeof(gps.gsv_satellites));
                    }
                }
            }


            /*
             * Field 3
             * Total satellites in view
             */
            if (field_index == 3)
            {
                if (sentence[buffer_index + 1] != ',')
                {
                    gps.satellites_in_view = (uint8_t)atoi(&sentence[buffer_index + 1]);
                }
            }


            /*
             * Fields 4 - 19
             *
             * Four fields per satellite:
             *
             * PRN
             * Elevation
             * Azimuth
             * SNR
             */
            if (field_index >= 4 &&
                field_index <= 19 &&
                message_number > 0)
            {
                uint8_t satellite_field = (field_index - 4) % 4;

                uint8_t satellite_in_message = (field_index - 4) / 4;

                uint8_t satellite_index = ((message_number - 1) * 4) + satellite_in_message;


                if (satellite_index <
                    NEO6M_MAX_GSV_SATELLITES)
                {
                    /*
                     * PRN
                     */
                    if (satellite_field == 0)
                    {
                        if (sentence[buffer_index + 1] != ',')
                        {
                            gps.gsv_satellites[satellite_index].prn = (uint8_t)atoi(&sentence[buffer_index + 1]);

                            if ((satellite_index + 1) > gps.gsv_satellite_count)
                            {
                                gps.gsv_satellite_count = satellite_index + 1;
                            }
                        }
                    }


                    /*
                     * Elevation
                     *
                     * 0 - 90 degrees
                     */
                    if (satellite_field == 1)
                    {
                        if (sentence[buffer_index + 1] != ',')
                        {
                            gps.gsv_satellites[satellite_index].elevation = (uint8_t)atoi(&sentence[buffer_index + 1]);
                        }
                    }


                    /*
                     * Azimuth
                     *
                     * 0 - 359 degrees
                     */
                    if (satellite_field == 2)
                    {
                        if (sentence[buffer_index + 1] != ',')
                        {
                            gps.gsv_satellites[satellite_index].azimuth = (uint16_t)atoi(&sentence[buffer_index + 1]);
                        }
                    }


                    /*
                     * SNR
                     *
                     * Unit:
                     * dB-Hz
                     *
                     * Empty SNR field = 0
                     */
                    if (satellite_field == 3)
                    {
                        if (sentence[buffer_index + 1] != ',' && sentence[buffer_index + 1] != '*' && sentence[buffer_index + 1] != '\0')
                        {
                            gps.gsv_satellites[satellite_index].snr = (uint8_t)atoi(&sentence[buffer_index + 1]);
                        }
                        else
                        {
                            gps.gsv_satellites[satellite_index].snr = 0;
                        }
                    }
                }
            }
        }
    }
}


/*
 * ============================================================
 * PROCESS NMEA QUEUE
 * ============================================================
 *
 * This function processes ALL complete NMEA sentences
 * currently waiting in the queue.
 *
 * UART interrupt can continue receiving new sentences
 * while parsing is performed here.
 */
void NEO6M_Process(void)
{
    while (queue_read_index != queue_write_index)
    {
        char *sentence = sentence_queue[queue_read_index];


        /*
         * RMC
         */
        if (strncmp(sentence, "$GPRMC", 6) == 0)
        {
            NEO6M_ParseRMC(sentence);
        }


        /*
         * GGA
         */
        else if (strncmp(sentence, "$GPGGA", 6) == 0)
        {
            NEO6M_ParseGGA(sentence);
        }


        /*
         * GSA
         */
        else if (strncmp(sentence, "$GPGSA", 6) == 0)
        {
            NEO6M_ParseGSA(sentence);
        }


        /*
         * GSV
         */
        else if (strncmp(sentence, "$GPGSV", 6) == 0)
        {
            NEO6M_ParseGSV(sentence);
        }


        /*
         * Move to next sentence in queue.
         */
        queue_read_index =
            (queue_read_index + 1) % NEO6M_QUEUE_SIZE;
    }
}


/*
 * ============================================================
 * GET GPS DATA
 * ============================================================
 */
NEO6M_Data NEO6M_GetData(void)
{
    return gps;
}
