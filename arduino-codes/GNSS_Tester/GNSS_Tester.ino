#include <SPI.h>
#include "mcp_can.h"

// Pino CS (Chip Select) conectado ao MCP2515
const int SPI_CS_PIN = 10;
const int CHANGE_SCREEN_PIN = 3;

// Cria objeto CAN
MCP_CAN CAN(SPI_CS_PIN);


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
}

void print64(uint64_t n) {
  if (n == 0) {
    Serial.println("0");
    return;
  }
  char buf[21]; // Max uint64_t is 20 digits + null terminator
  char *p = &buf[sizeof(buf) - 1];
  *p = '\0';
  do {
    *--p = '0' + (n % 10);
    n /= 10;
  } while (n > 0);
  Serial.println(p);
}

void loop() {
  uint32_t rxID;
  uint8_t rxData[8];
  uint8_t len = 0;

  double lat = -1;
  double lon = -1;
  double alt = -1;
  double gs = -1;
  uint8_t fix_quality = -1;
  uint64_t timestamp = 1;

  if (receiveCANMessage(rxID, rxData, len)) {
    if (rxID == 262) {
      int64_t rawLat = 0;
      memcpy(&rawLat, rxData, sizeof(int64_t));
      lat = rawLat / 1e7;
      Serial.print("lat = ");
      Serial.println(lat);
    }

    if (rxID == 263) {
      int64_t rawLon = 0;
      memcpy(&rawLon, rxData, sizeof(int64_t));
      lon = rawLon / 1e7;
      Serial.print("lon = ");
      Serial.println(lon);
    }

    if (rxID == 264) {
      memcpy(&timestamp, rxData, sizeof(uint64_t));
      Serial.print("timestamp = ");
      print64(timestamp);
    }

    if (rxID == 265) {
      alt = (((uint16_t)rxData[0] << 8) | rxData[1]) / (double) 100;
      gs = (((uint16_t)rxData[2] << 8) | rxData[3]) / (double) 100;
      fix_quality = rxData[4];

      Serial.print("alt = ");
      print64(alt);
      Serial.print("gs = ");
      print64(gs);
      Serial.print("fix_quality = ");
      print64(fix_quality);
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


