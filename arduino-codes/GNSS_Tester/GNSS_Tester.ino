#include <SPI.h>
#include "mcp_can.h"

// Pino CS (Chip Select) conectado ao MCP2515
const int SPI_CS_PIN = 10;
const int CHANGE_SCREEN_PIN = 3;

// Cria objeto CAN
MCP_CAN CAN(SPI_CS_PIN);

int counter = 0;

double lat = -1;
double lon = -1;
double alt = -1;
double gs = -1;
uint8_t fix_quality = -1;
uint64_t timestamp = 1;

volatile bool flag = false;

bool receiveCANMessage(unsigned long &rxId, uint8_t *rxBuf, uint8_t &len) {
  if (CAN_MSGAVAIL == CAN.checkReceive()) { // se nao funcionar, trocar para -> !digitalRead(SPI_INT_PIN)  
     CAN.readMsgBuf(&rxId, &len, rxBuf);
     return true; 
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  pinMode(CHANGE_SCREEN_PIN, OUTPUT);
  while (!Serial);

  Serial.println("Iniciando MCP2515...");

  // Inicializa o CAN a 500kbps
    while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("Erro ao inicializar MCP2515. Tentando novamente...");
    delay(1000);
  }

  Serial.println("CAN inicializado com sucesso!");

  CAN.setMode(MCP_NORMAL); // Modo normal (para comunicação na rede)

  Serial.println("MCP2515 configurado, esperando mensagens...");


  cli(); // Disable global interrupts

  // Timer1 setup
  TCCR1A = 0;              // Normal mode
  TCCR1B = 0;

  TCNT1 = 0;

  // Compare match value for 200 ms
  // 16 MHz clock / 1024 prescaler = 15625 Hz
  // 200 ms -> 0.2 * 15625 = 3125
  OCR1A = 3125;

  TCCR1B |= (1 << WGM12);  // CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024

  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare interrupt

  sei(); // Enable global interrupts
}

ISR(TIMER1_COMPA_vect) {
  flag = true;  // Set flag (keep ISR short!)
}

void print64(uint64_t n) {
  if (n == 0) {
    Serial.print("0");
    return;
  }
  char buf[21]; // Max uint64_t is 20 digits + null terminator
  char *p = &buf[sizeof(buf) - 1];
  *p = '\0';
  do {
    *--p = '0' + (n % 10);
    n /= 10;
  } while (n > 0);
  Serial.print(p);
}

void loop() {
  uint32_t rxID;
  uint8_t rxData[8];
  uint8_t len = 0;

  counter++;

  if (receiveCANMessage(rxID, rxData, len)) {
    if (rxID == 262) {
      int64_t rawLat = 0;
      memcpy(&rawLat, rxData, sizeof(int64_t));
      lat = rawLat / 1e7;
      // Serial.print("lat = ");
      // Serial.println(lat);
    }

    if (rxID == 263) {
      int64_t rawLon = 0;
      memcpy(&rawLon, rxData, sizeof(int64_t));
      lon = rawLon / 1e7;
      // Serial.print("lon = ");
      // Serial.println(lon);
    }

    if (rxID == 264) {
      memcpy(&timestamp, rxData, sizeof(uint64_t));
      // Serial.print("timestamp = ");
      // print64(timestamp);
      // for (int i = 0; i < 8; ++i) {
      //   Serial.print(rxData[i]);
      //   Serial.print(" ");
      // }
      // Serial.println();
    }

    if (rxID == 265) {
      alt = (((uint16_t)rxData[0] << 8) | rxData[1]) / (double) 100;
      gs = (((uint16_t)rxData[2] << 8) | rxData[3]) / (double) 100;
      fix_quality = rxData[4];

      // Serial.print("alt = ");
      // print64(alt);
      // Serial.print("gs = ");
      // print64(gs);
      // Serial.print("fix_quality = ");
      // print64(fix_quality);
    }

    if (flag) {
      Serial.print(lat, 8);
      Serial.print(" ");
      Serial.print(lon, 8);
      Serial.print(" ");
      Serial.print(alt);
      Serial.print(" ");
      Serial.print(gs);
      Serial.print(" ");
      print64(timestamp);
      Serial.print(" ");
      Serial.print(fix_quality);
      Serial.print('\r');
      Serial.print('\n');

      counter = 0;
      flag = false;
    }

    // Serial.println("Received!");
    // Serial.println(rxID);
    // for (int i = 0; i < 8; ++i) {
    //   Serial.print(rxData[i]);
    //   Serial.print(" ");
    // }
    // Serial.println();
  }

  return;

}


