#include <SPI.h>
#include "mcp_can.h"

// Pino CS (Chip Select) conectado ao MCP2515
const int SPI_CS_PIN = 10;
const int CHANGE_SCREEN_PIN = 3;

#define ACCEL_ID 0x123
#define GYRO_ID 0x124

// Cria objeto CAN
MCP_CAN CAN(SPI_CS_PIN);

void setup() {
  Serial.begin(115200);
  pinMode(CHANGE_SCREEN_PIN, OUTPUT);
  while (!Serial);

  Serial.println("Iniciando MCP2515...");

  // Inicializa o CAN a 500kbps
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN inicializado com sucesso!");
  } else {
    Serial.println("Erro ao inicializar CAN");
    while (1);
  }

  CAN.setMode(MCP_NORMAL); // Modo normal (para comunicação na rede)

  Serial.println("MCP2515 configurado, esperando mensagens...");
}

int counter = 0;
bool isPositive = false;
bool prevPress = false;

int fastValue = 0;
int slowValue = 0;

int missedCounter = 0;

void loop() {
  // Check if data is available
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    long unsigned int canId;
    unsigned char len = 0;
    unsigned char buf[8];

    // Read the message
    CAN.readMsgBuf(&canId, &len, buf);

    Serial.print("CAN ID: ");
    Serial.print(canId);
    Serial.print(" Data: ");
    for (int i = 0; i < len; i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    missedCounter = 0;
  } else {
    missedCounter++;
  }

  if (missedCounter % 60000 == 0) {
    Serial.print("CAN ID: 0000");
    Serial.print(" Data: ");
    for (int i = 0; i < 8; i++) {
      Serial.print(0, HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}


