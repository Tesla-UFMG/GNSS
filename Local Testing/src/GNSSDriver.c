#include "../inc/GNSSDriver.h"

// helper local functions
static double nmea_to_decimal(const char *nmea, const char *hemisphere) {
    if (strlen(nmea) < 4) return 0.0;

    // Split degrees and minutes
    int deg_len = (hemisphere[0] == 'N' || hemisphere[0] == 'S') ? 2 : 3;
    char deg_str[4] = {0};
    strncpy(deg_str, nmea, deg_len);
    int degrees = atoi(deg_str);

    double minutes = atof(nmea + deg_len);
    double decimal = degrees + (minutes / 60.0);

    if (hemisphere[0] == 'S' || hemisphere[0] == 'W')
        decimal *= -1.0;

    return decimal;
}

error_code_t init(gnss_data_t* gnss_data) {
    if (gnss_data == NULL) {
        return ERROR_BAD_ARGUMENT;
    }

    utc_date_t default_date = {0, 0, 0};
    utc_time_t default_time = {0, 0, 0.0};

    gnss_data->latitude = 0.0;
    gnss_data->longitude = 0.0;
    gnss_data->altitude = 0.0;
    gnss_data->ground_speed = 0.0; 
    gnss_data->utc_date = default_date; 
    gnss_data->utc_time = default_time; 
    gnss_data->fix_quality = INVALID_FIX; 

    return NO_ERROR;
}

error_code_t print_data(gnss_data_t* gnss_data) {
    if (gnss_data == NULL) {
        return ERROR_BAD_ARGUMENT;
    }

    printf("Latitude: %.8f\n", gnss_data->latitude);
    printf("Longitude: %.8f\n", gnss_data->longitude);
    printf("Altitude: %.2f m\n", gnss_data->altitude);
    printf("Ground Speed: %.2f m/s\n", gnss_data->ground_speed);
    printf("UTC Date: %02d/%02d/%02d\n", gnss_data->utc_date.day, gnss_data->utc_date.month, gnss_data->utc_date.year);
    printf("UTC Time: %02d:%02d:%.3f\n", gnss_data->utc_time.hh, gnss_data->utc_time.mm, gnss_data->utc_time.ss);
    printf("Fix Quality: %s\n", (gnss_data->fix_quality == VALID_FIX) ? "Valid" : "Invalid");

    return NO_ERROR;
}

error_code_t parse_gpgga(gnss_data_t* gnss_data, char* sentence) {
    if (gnss_data == NULL || sentence == NULL) {
        return ERROR_BAD_ARGUMENT;
    }

    // Example: $GPGGA,162429.000,1952.18870,S,04357.77327,W,1,05,1.7,813.99,M,-5.7,M,,*75
    char *token;
    int field = 0;

    token = strtok(sentence, ",");
    while (token != NULL) {
        switch (field) {
            case 1: { // UTC time hhmmss.sss
                int hh, mm;
                float ss;
                sscanf(token, "%2d%2d%f", &hh, &mm, &ss);
                gnss_data->utc_time.hh = (uint8_t)hh;
                gnss_data->utc_time.mm = (uint8_t)mm;
                gnss_data->utc_time.ss = ss;
                break;
            }
            case 2: gnss_data->latitude = nmea_to_decimal(token, strtok(NULL, ",")); field++; break;
            case 4: gnss_data->longitude = nmea_to_decimal(token, strtok(NULL, ",")); field++; break;
            case 6: gnss_data->fix_quality = (token[0] == '0' ? INVALID_FIX : VALID_FIX); break;
            case 9: gnss_data->altitude = atof(token); break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    return NO_ERROR;
}

error_code_t parse_gprmc(gnss_data_t* gnss_data, char *sentence) {
    if (gnss_data == NULL || sentence == NULL) {
        return ERROR_BAD_ARGUMENT;
    }

    // Example: $GPRMC,162429.000,A,1952.18870,S,04357.77327,W,0.6,348.5,190925,,,A*6B
    char *token;
    int field = 0;

    token = strtok(sentence, ",");
    while (token != NULL) {
        switch (field) {
            case 1: { // UTC time
                int hh, mm;
                float ss;
                sscanf(token, "%2d%2d%f", &hh, &mm, &ss);
                gnss_data->utc_time.hh = (uint8_t)hh;
                gnss_data->utc_time.mm = (uint8_t)mm;
                gnss_data->utc_time.ss = ss;
                break;
            }
            case 2: gnss_data->fix_quality = (token[0] == 'A' ? VALID_FIX : INVALID_FIX); break; // A=valid, V=invalid.
            case 3: gnss_data->latitude = nmea_to_decimal(token, strtok(NULL, ",")); field++; break;
            case 5: gnss_data->longitude = nmea_to_decimal(token, strtok(NULL, ",")); field++; break;
            case 8: gnss_data->ground_speed = atof(token) * KNOTS_TO_KMH_CONVERSION;
            case 9: { // Date ddmmyy
                int dd, mm, yy;
                sscanf(token, "%2d%2d%2d", &dd, &mm, &yy);
                gnss_data->utc_date.day = (uint8_t)dd;
                gnss_data->utc_date.month = (uint8_t)mm;
                gnss_data->utc_date.year = (uint8_t)yy;
                break;
            }
        }

        token = strtok(NULL, ",");
        field++;
    }

    return NO_ERROR;
}