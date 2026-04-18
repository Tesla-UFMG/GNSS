#ifndef GNSS_DRIVER_H
#define GNSS_DRIVER_H

#include "main.h"
#include "typedefs.h"

#define KNOTS_TO_KMH_CONVERSION 1.852
#define TIMEZONE_OFFSET 3 * 3600 * (-1)
#define MAX_NMEA_LEN 512

#define LATITUDE_CAN_ID 262
#define LONGITUDE_CAN_ID 263
#define UTC_CAN_ID 264
#define GENERAL_GNSS_CAN_ID 265

typedef enum {
	INVALID_FIX = 0,
    VALID_FIX = 1
} fix_quality_t;

typedef struct {
    uint8_t hh;
    uint8_t mm;
    float ss; // ss.sss
} utc_time_t;

typedef struct {
    uint8_t day;
    uint8_t month;
    uint8_t year;
} utc_date_t;

typedef struct {
    // must be double for 8th decimal place precision
    double latitude;
    double longitude;
    double altitude;
    double ground_speed;

    utc_date_t utc_date;
    utc_time_t utc_time;
    fix_quality_t fix_quality;
} gnss_data_t;

typedef enum {
    NMEA_UNKNOWN = 0,
    NMEA_GPRMC,
    NMEA_GPGGA,
} NMEA_Type;

/* ----- BASE FUNCTIONS AND PARSERS -----  */
error_code_t init(gnss_data_t*);
error_code_t classify_nmea(NMEA_Type* nmea_type, char* sentence);
error_code_t parse_gpgga(gnss_data_t* gnss_data, char* sentence);
error_code_t parse_gprmc(gnss_data_t* gnss_data, char* sentence);
error_code_t save_to_message(gnss_data_t* gnss_data, char* message, int size);

/* ----- CONVERT GNSS DATA TO BYTES FOR CAN TRANSMISSION -----  */
error_code_t save_lat_to_buffer(gnss_data_t* gnss_data, uint8_t* buf);
error_code_t save_lon_to_buffer(gnss_data_t* gnss_data, uint8_t* buf);
error_code_t save_utc_to_buffer(gnss_data_t* gnss_data, uint8_t* buf);
error_code_t save_general_to_buffer(gnss_data_t* gnss_data, uint8_t* buf);

/* ----- CONVERT BYTES FROM CAN TO GNSS DATA FOR CAN RECEPTION -----  */
error_code_t get_latitude_from_buffer(uint8_t* buf, double* lat);
error_code_t get_utc_from_buffer(uint8_t* buf, uint64_t* utc);

#endif // GNSS_DRIVER_H
