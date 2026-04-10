/*
  Moving GPS NMEA Sentence Generator
  ----------------------------------
  Simulates a GPS device outputting realistic GPGGA and GPRMC sentences
  every second, with slowly changing position, speed, and time.
*/

unsigned long lastUpdate = 0;
unsigned long secondsSinceStart = 0;

// Starting coordinates (Belo Horizonte, Brazil)
float lat_deg = -19.871353;
float lon_deg = -43.963153;

// Movement parameters
float speed_knots = 5.0;  // ≈ 9.26 km/h
float direction_deg = 90; // eastward (can change to simulate heading)

// Utility to convert decimal degrees to NMEA ddmm.mmmm format
String toNMEA(float value, bool isLat) {
  float absVal = abs(value);
  int degrees = int(absVal);
  float minutes = (absVal - degrees) * 60.0;
  char buffer[16];
  if (isLat)
    sprintf(buffer, "%02d%07.4f", degrees, minutes);
  else
    sprintf(buffer, "%03d%07.4f", degrees, minutes);
  return String(buffer);
}

void setup() {
  Serial.begin(9600);
}

int count = 0;

void loop() {
  count++;
  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdate >= 1000) {
    // send ready sentences
    String rmc_ready = "$GPRMC,164236.000,A,1952.2" + String(count % 10) + "811,S,04357.78719,W,0.1,0.0,190925,,,A*6A";
    String gpgaa_ready = "$GPGGA,164236.000,1952.2" + String(count % 10) + "811,S,04357.78719,W,1,10,1.0,819.28,M,-5.7,M,,*7A";
    String gsa_ready = "$GNGSA,A,3,23,18,12,25,29,05,15,,,,,,1.9,1.0,1.6*24 $GNGSA,A,3,76,86,75,,,,,,,,,,1.9,1.0,1.6*2F";
    String gsv_ready = "$GPGSV,2,1,08,25,83,028,35,12,49,044,33,18,47,276,31,29,45,183,23*76";
    String pst_ready = "$PSTMCPU,100.00,-1,49*7E";

    Serial.println(rmc_ready);
    Serial.println(gpgaa_ready);
    Serial.println(gsa_ready);
    Serial.println(gsv_ready);
    Serial.println(pst_ready);

    return;

    lastUpdate = currentMillis;
    secondsSinceStart++;

    // Simulate motion eastward based on speed and time
    float speed_mps = speed_knots * 0.514444; // knots → m/s
    float distance_m = speed_mps; // distance in 1 second
    float earthRadius = 6371000.0;

    // Update coordinates
    lon_deg += (distance_m / earthRadius) * (180.0 / PI) / cos(lat_deg * PI / 180.0);

    // Format UTC time and date
    int totalSeconds = secondsSinceStart % 86400;
    int hours = (12 + totalSeconds / 3600) % 24;
    int minutes = (totalSeconds % 3600) / 60;
    int seconds = totalSeconds % 60;
    char timeUTC[16];
    sprintf(timeUTC, "%02d%02d%02d.00", hours, minutes, seconds);

    // Fixed date (DDMMYY)
    String date = "161025";

    // Convert position to NMEA format
    String latNMEA = toNMEA(lat_deg, true);
    String latDir = (lat_deg >= 0) ? "N" : "S";
    String lonNMEA = toNMEA(lon_deg, false);
    String lonDir = (lon_deg >= 0) ? "E" : "W";

    // Build GPGGA
    String gga = "$GPGGA," + String(timeUTC) + "," + latNMEA + "," + latDir + "," +
                 lonNMEA + "," + lonDir + ",1,08,0.9,819.3,M,-5.7,M,,";
    gga += "*" + String(calcChecksum(gga), HEX);
    gga.toUpperCase();

    // Build GPRMC
    char speedStr[8], courseStr[8];
    dtostrf(speed_knots, 3, 1, speedStr);
    dtostrf(direction_deg, 3, 1, courseStr);

    String rmc = "$GPRMC," + String(timeUTC) + ",A," + latNMEA + "," + latDir + "," +
                 lonNMEA + "," + lonDir + "," + String(speedStr) + "," +
                 String(courseStr) + "," + date + ",,";

    rmc += "*" + String(calcChecksum(rmc), HEX);
    rmc.toUpperCase();

    // Send both sentences
    // Serial.println(gga);
    // Serial.println(rmc);
  }
}

/*
  Function to calculate NMEA checksum
  (XOR of all characters between '$' and '*')
*/
byte calcChecksum(String sentence) {
  int start = sentence.indexOf('$') + 1;
  int end = sentence.indexOf('*');
  if (end == -1) end = sentence.length();
  byte checksum = 0;
  for (int i = start; i < end; i++) {
    checksum ^= (byte)sentence[i];
  }
  return checksum;
}
