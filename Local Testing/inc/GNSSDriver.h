#ifndef GNSS_DRIVER_H
#define GNSS_DRIVER_H

#include "main.h"
#include "typedefs.h"

#define KNOTS_TO_KMH_CONVERSION 1.852

typedef enum {
    VALID_FIX = 0,
    INVALID_FIX
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

error_code_t init(gnss_data_t*);
error_code_t print_data(gnss_data_t*);
error_code_t parse_gpgga(gnss_data_t* gnss_data, char* sentence);
error_code_t parse_gprmc(gnss_data_t* gnss_data, char* sentence);

#endif // GNSS_DRIVER_H