#include <SPI.h>
#include "mcp_can.h"

// Pino CS (Chip Select) conectado ao MCP2515
const int SPI_CS_PIN = 10;
const int CHANGE_SCREEN_PIN = 3;

#define ACCEL_ID 0x123
#define GYRO_ID 0x124

// #define UPDATED_PROTOCOL

struct OldProtocol {
  byte id79[8];
};

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

int counter = 0;
bool isPositive = false;
bool prevPress = false;

int fastValue = 0;
int slowValue = 0;
int zeroFourValue = 0;
int zeroOneValue = 0;
int speedValue = 0;

int shouldBrake = 0;

struct OldProtocol oldProtocol;

void loop() {
  uint32_t rxID;
  uint8_t rxData[8];
  uint16_t output[8];
  uint8_t len = 0;

  if (receiveCANMessage(rxID, rxData, len)) {
    Serial.println("Received!");
    Serial.println(rxID);
    for (int i = 0; i < 8; ++i) {
      Serial.print(rxData[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
  // byte testData[8] = {10,20,30,40,50,60,70,80};
  // CAN.sendMsgBuf(201, 0, 8, testData);
  delay(10);
  return;

  delay(100);   // send data per 10ms
  counter++;

  if (counter > 60000) {
    counter = 0;
  }

  if (counter % 10 == 0) { // 100 ms
    fastValue++;
  }

  if (counter % 100 == 0) { // 1s
    shouldBrake = shouldBrake == 1 ? 0 : 1;
    slowValue++;
    zeroFourValue++;
    zeroOneValue++;
    speedValue++;

    if (zeroFourValue > 4) {
      zeroFourValue = 0;
    }
    if (zeroOneValue > 1) {
      zeroOneValue = 0;
    }

    if (speedValue > 15) {
      speedValue = 0;
    }
  }

  if (fastValue > 250) {
    fastValue = 0;
  }

  if (slowValue > 250) {
    slowValue = 0;
  }

  byte fastData8[8] = { fastValue, 0, 0, 0, 0, 0, 0, 0 };
  byte fastData8_100[8] = { fastValue % 100, 0, 0, 0, 0, 0, 0, 0 };
  byte fastData16[8] = { fastValue, 2, 0, 0, 0, 0, 0, 0 };

  byte slowData8[8] = { slowValue, 0, 0, 0, 0, 0, 0, 0 };
  byte slowData16[8] = { slowValue, 2, 0, 0, 0, 0, 0, 0 };

  byte sndStat = 0;

  #ifdef UPDATED_PROTOCOL
    /* DRIVER PAGE */
    CAN.sendMsgBuf(201, 0, 8, fastData8_100);
    CAN.sendMsgBuf(241, 0, 8, fastData8_100);
    CAN.sendMsgBuf(242, 0, 8, fastData8_100);
    CAN.sendMsgBuf(207, 0, 8, fastData16);
    CAN.sendMsgBuf(112, 0, 8, fastData8_100);
    CAN.sendMsgBuf(211, 0, 8, fastData8_100);
    CAN.sendMsgBuf(101, 0, 8, fastData16);
    CAN.sendMsgBuf(103, 0, 8, fastData16);
    sndStat = CAN.sendMsgBuf(212, 0, 8, fastData16);

    /* CONTROL PAGE */
    CAN.sendMsgBuf(231, 0, 8, fastData16);
    CAN.sendMsgBuf(232, 0, 8, fastData16);
    CAN.sendMsgBuf(226, 0, 8, fastData8_100);
    CAN.sendMsgBuf(227, 0, 8, fastData16);
    CAN.sendMsgBuf(225, 0, 8, fastData16);

    CAN.sendMsgBuf(245, 0, 8, fastData16);
    CAN.sendMsgBuf(246, 0, 8, fastData16);
    CAN.sendMsgBuf(240, 0, 8, fastData8_100);
    // CAN.sendMsgBuf(241, 0, 8, fastData16);
    CAN.sendMsgBuf(239, 0, 8, fastData16);
    CAN.sendMsgBuf(292, 0, 8, fastData16);
    CAN.sendMsgBuf(291, 0, 8, fastData16);
    CAN.sendMsgBuf(202, 0, 8, fastData8_100);

    /* SAFETY PAGE */
    CAN.sendMsgBuf(317, 0, 8, fastData8);
    CAN.sendMsgBuf(337, 0, 8, fastData8);
    CAN.sendMsgBuf(357, 0, 8, fastData8);
    CAN.sendMsgBuf(377, 0, 8, fastData8);
    CAN.sendMsgBuf(397, 0, 8, fastData8);
    CAN.sendMsgBuf(417, 0, 8, fastData8);
    CAN.sendMsgBuf(108, 0, 8, fastData8);
    CAN.sendMsgBuf(109, 0, 8, fastData8);
    CAN.sendMsgBuf(110, 0, 8, fastData8);
    CAN.sendMsgBuf(111, 0, 8, fastData8);
  #else
    /* ECU */
    byte old_data[8] = {0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00};
    // sndStat = CAN.sendMsgBuf(76, 0, 8, id76data);

    // byte id85data[8] = {0x02, 0x00, 0x02, 0x00, 0x02, 0x05, 0x02, 0x00};
    // sndStat = CAN.sendMsgBuf(85, 0, 8, id85data);
    // byte id95data[8] = {0x02, 0x00, 0x02, 0x00, 0x02, 0x05, 0x02, 0x00};
    // sndStat = CAN.sendMsgBuf(95, 0, 8, id95data);

    byte id76_data[8] = {fastValue, 0x00, 0x00, 0x00, fastValue, 0x00, shouldBrake, 0x00};
    byte id77_data[8] = {zeroFourValue, 0x00, 0x00, 0x00, fastValue, 0x00, 0x02, 0x00};
    byte id78_data[8] = {zeroFourValue, 0x00, 0x00, 0x00, fastValue, 0x01, 0x02, 0x00};
    byte id79_data[8] = {fastValue, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00};
    byte id306_data[8] = {fastValue, 0x60, 0x00, 0x90, fastValue, 0x02, fastValue, 0x00};
    byte id300_data[8] = {fastValue, 0x60, fastValue, 0x90, fastValue, 0x00, fastValue, 0x00};
    byte id307_data[8] = {zeroOneValue, 0x00,zeroOneValue, 0x00,zeroOneValue, 0x00,zeroOneValue, 0x00};
    byte id95_data[8] = {fastValue, fastValue, zeroOneValue, 0x00,zeroOneValue, 0x00,zeroOneValue, 0x00};

    CAN.sendMsgBuf(76, 0, 8, id76_data);
    CAN.sendMsgBuf(77, 0, 8, id77_data);
    CAN.sendMsgBuf(78, 0, 8, id78_data);
    CAN.sendMsgBuf(79, 0, 8, id79_data);
    CAN.sendMsgBuf(81, 0, 8, old_data);
    CAN.sendMsgBuf(85, 0, 8, old_data);
    CAN.sendMsgBuf(86, 0, 8, old_data);
    CAN.sendMsgBuf(87, 0, 8, old_data);
    CAN.sendMsgBuf(88, 0, 8, old_data);
    CAN.sendMsgBuf(95, 0, 8, id95_data);
    CAN.sendMsgBuf(96, 0, 8, old_data);
    CAN.sendMsgBuf(97, 0, 8, old_data);
    CAN.sendMsgBuf(98, 0, 8, old_data);
    CAN.sendMsgBuf(300, 0, 8, id300_data);
    CAN.sendMsgBuf(301, 0, 8, id300_data);
    CAN.sendMsgBuf(302, 0, 8, id300_data);
    CAN.sendMsgBuf(303, 0, 8, id300_data);
    CAN.sendMsgBuf(304, 0, 8, id300_data);
    CAN.sendMsgBuf(305, 0, 8, id300_data);
    CAN.sendMsgBuf(306, 0, 8, id306_data);
    CAN.sendMsgBuf(307, 0, 8, id307_data);
  #endif

  // Serial.print(sndStat);
  if(sndStat == CAN_OK){
    // Serial.println("Message Sent Successfully!");
  } else {
    if (counter % 10 == 0) {
      Serial.println("Error Sending Message...");
    }
  }
  // if (counter % 10 == 0) {
  //     Serial.println(counter);
  //   }
}


