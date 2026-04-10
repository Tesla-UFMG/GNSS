#include <Arduino.h>
#include <SoftwareSerial.h>

// Initialize SoftwareSerial: Pin 2 (RX), Pin 3 (TX)
SoftwareSerial mySerial(2, 3); 

float lat = 1992.08;
float lon = 12293.78;
float speed = 1.5;      // knots
float course = 45.0;    // degrees
float altitude = 1097.5;

unsigned long lastUpdate = 0;

// NMEA checksum
String addChecksum(String sentence) {
    uint8_t checksum = 0;
    for (int i = 1; i < sentence.length(); i++) {
        checksum ^= sentence[i];
    }

    char buf[6];
    sprintf(buf, "*%02X", checksum);
    return sentence + String(buf);
}

// Time hhmmss
String getTime() {
    unsigned long t = millis() / 1000;
    int hh = 13;
    int mm = 35;
    int ss = t % 60;

    char buf[10];
    sprintf(buf, "%02d%02d%02d", hh, mm, ss);
    return String(buf);
}

// Date ddmmyy
String getDate() {
    return "200326";
}

void setup() {
    Serial.begin(115200);
    mySerial.begin(115200); // Software Serial for Module
}

void loop() {
    if (millis() - lastUpdate > 200) {
        lastUpdate = millis();

        // Simulate motion
        lat += 0.01;
        lon += 0.01;
        speed += 0.01;
        course += 1.0;
        altitude += 0.1;
        if (course > 360) course -= 360;

        float speedKmh = speed * 1.852;

        String timeStr = getTime();
        String dateStr = getDate();
        String latStr = String(lat);
        String lonStr = String(lon);

        // --- GPRMC ---
        String rmc = "$GPRMC," + timeStr + ",A," +
                     latStr + ",S," + lonStr + ",W," +
                     String(speed, 2) + "," +
                     String(course, 2) + "," +
                     dateStr + ",,,";

        // --- GPGGA ---
        String gga = "$GPGGA," + timeStr + "," +
                     latStr + "," + lonStr +
                     ",1,08,0.9,800.0,M," + String(altitude) + ",,,";

        // --- GPVTG (Track & Speed) ---
        String vtg = "$GPVTG," +
                     String(course, 2) + ",T,," + ",M," +
                     String(speed, 2) + ",N," +
                     String(speedKmh, 2) + ",K";

        // --- GPGLL (Geographic Position) ---
        String gll = "$GPGLL," +
                     latStr + "," +
                     lonStr + "," +
                     timeStr + ",A";

        // --- GNGSA (GNSS DOP and Active Satellites) ---
        // Example with 5 satellites used
        String gsa = "$GNGSA,A,3,04,14,07,09,16,,,,,,,,1.8,1.0,1.5";

        // Add checksums
        rmc = addChecksum(rmc);
        gga = addChecksum(gga);
        vtg = addChecksum(vtg);
        gll = addChecksum(gll);
        gsa = addChecksum(gsa);

        // Output all
        Serial.println(rmc);
        Serial.println(gga);
        Serial.println(vtg);
        Serial.println(gll);
        Serial.println(gsa);

        mySerial.println(rmc);
        mySerial.println(gga);
        mySerial.println(vtg);
        mySerial.println(gll);
        mySerial.println(gsa);
    }
}