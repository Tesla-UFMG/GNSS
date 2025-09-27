#include "inc/main.h"
#include "inc/GNSSDriver.h"

char test_gpgga[] = "$GPGGA,164236.000,1952.28118,S,04357.78719,W,1,10,1.0,819.28,M,-5.7,M,,*7A $GPVTG,0.0,T,,M,0.1,N,0.2,K,A*0E ";
char test_gprmc[] = "$GPRMC,164236.000,A,1952.28118,S,04357.78719,W,0.1,0.0,190925,,,A*6A";

int main() {
    gnss_data_t gnss_data;

    if (init(&gnss_data) != NO_ERROR) {
        fprintf(stderr, "Initialization failed\n");
        return -1;
    }

    if (parse_gprmc(&gnss_data, test_gprmc) != NO_ERROR) {
        fprintf(stderr, "Parsing failed\n");
        return -1;
    }

    if (parse_gpgga(&gnss_data, test_gpgga) != NO_ERROR) {
        fprintf(stderr, "Parsing failed\n");
        return -1;
    }

    if (print_data(&gnss_data) != NO_ERROR) {
        fprintf(stderr, "Printing data failed\n");
        return -1;
    }

    return 0;
}