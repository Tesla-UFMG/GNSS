#include "inc/main.h"

#include "inc/GNSSDriver.h"

#define MAX_NMEA_LEN 128

char test_gpgga[] =
    "$GPGGA,164236.000,1952.28118,S,04357.78719,W,1,10,1.0,819.28,M,-5.7,M,,*"
    "7A $GPVTG,0.0,T,,M,0.1,N,0.2,K,A*0E ";
char test_gprmc[] =
    "$GPRMC,164236.000,A,1952.28118,S,04357.78719,W,0.1,0.0,190925,,,A*6A";

char test_rx_buffer[] =
    ",V,0000.00000,N,00000.00000,E,0.0,0.0,190925,,,N*74\n "
    "$GPGGA,123105.000,0000.00000,N,00000.00000,E,0,02,99.0,100.00,M,0.0,M,,*"
    "6A\n$GPVTG,0.0,T,,M,0.0,";

// ,V,0000.00000,N,00000.00000,E,0.0,0.0,190925,,,N*74
// $GPGGA,123105.000,0000.00000,N,00000.00000,E,0,02,99.0,100.00,M,0.0,M,,*6A
// $GPVTG,0.0,T,,M,0.0,N,0.0,K,N*02
// $GNGSA,A,1,19,11,,,,,,,,,,,99.0,99.0,99.0*16
// $GNGSA,A,1,,,,,,,,,,,,,99.0,99.0,99.0*1E
// $GPGSV,1,1,03,19,00,000,31,11,00,000,42,06,00,000,29,,,,*4B
// $GPGLL,0000.00000,N,00000.00000,E,123105.000,V,N*45
// $PSTMCPU,55.05,-1,

int main() {
  gnss_data_t gnss_data;

  if (init(&gnss_data) != NO_ERROR) {
    fprintf(stderr, "Initialization failed\n");
    return -1;
  }

  NMEA_Type nmea_type = NMEA_UNKNOWN;

  if (classify_nmea(&nmea_type, test_gprmc) != NO_ERROR) {
    fprintf(stderr, "Classification failed\n");
    return -1;
  }

  if (classify_nmea(&nmea_type, test_gpgga)) {
    fprintf(stderr, "Classification failed\n");
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

  char message[MAX_NMEA_LEN] = {'\0'};

  if (save_to_message(&gnss_data, message, MAX_NMEA_LEN) != NO_ERROR) {
    fprintf(stderr, "Save to message failed\n");
    return -1;
  }

  if (print_data(&gnss_data) != NO_ERROR) {
    fprintf(stderr, "Printing data failed\n");
    return -1;
  }

  //   int start_idx = -1, end_idx = -1;
  //   for (int i = 0; i < MAX_NMEA_LEN; ++i) {
  //     char current_char = test_rx_buffer[i];
  //     if (current_char == '$') {
  //       start_idx = i;
  //     }

  //     if (start_idx != -1 && current_char == '\n') {
  //       // terminou a mensagem
  //       end_idx = i;
  //       break;
  //     }
  //   }

  //   if (start_idx == -1 || end_idx == -1) {
  //     // sem mensagem
  //     return 0;
  //   }

  //   // index de inicio e fim determinados. salva no nmea_message
  //   uint8_t message_size = end_idx - start_idx;
  //   char *nmea_message = (char *)malloc(message_size * sizeof(char));

  //   for (int i = 0; i < message_size; ++i) {
  //     nmea_message[i] = test_rx_buffer[start_idx + i];
  //   }

  //   free(nmea_message);
  //   nmea_message = NULL;

  return 0;
}