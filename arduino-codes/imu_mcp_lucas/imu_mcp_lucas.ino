#include <SPI.h>
#include <string.h>
#include "mcp_can.h"
#include <stddef.h> 

// Defines
#define SPI_CS_PIN 10
#define SPI_INT_PIN 2 

// Objeto CAN
MCP_CAN CAN(SPI_CS_PIN);

bool receiveCANMessage(unsigned long &rxId, uint8_t *rxBuf, uint8_t &len) {
  if (CAN_MSGAVAIL == CAN.checkReceive()) { // se nao funcionar, trocar para -> !digitalRead(SPI_INT_PIN)  
     CAN.readMsgBuf(&rxId, &len, rxBuf);
     return true; 
  }
  return false;
}

unsigned long last_time = millis();

void setup() {
  Serial.begin(115200);

  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("Erro ao inicializar MCP2515. Tentando novamente...");
    delay(1000);
  }

  Serial.println("MCP2515 inicializado com sucesso!");
  CAN.setMode(MCP_NORMAL); 
  pinMode(SPI_INT_PIN, INPUT);  
}

void loop() {
  uint32_t rxID;
  uint8_t rxData[8];
  uint16_t output[8];
  uint8_t len = 0;

  if (receiveCANMessage(rxID, rxData, len)) {
    // dado recebido: salva o tempo
    // int current_time = millis();
    // // Limpa o buffer de output
    // memset(output, 0, 8);

    // // Faz os deslocamentos de bit e converte para 16 bits com sinaç
    // output[0] = (int16_t)(rxData[1] << 8 | rxData[0]);
    // output[1] = (int16_t)(rxData[3] << 8 | rxData[2]);
    // output[2] = (int16_t)(rxData[5] << 8 | rxData[4]);
    // output[3] = (int16_t)(rxData[6] << 8 | rxData[5]);
    // output[4] = (int16_t)(rxData[8] << 8 | rxData[7]);

    // // Imprime no formato da reconstrução (id,valor0,valor1,...,valor7) <- sem espaço entre as virgulas
    // Serial.print(rxID, HEX);
    // Serial.print(",");
    // for (size_t i = 0; i < len; i++){
    //     Serial.print(output[i]);
    //     if (i < len - 1) Serial.print(",");
    // }
    // Serial.print(current_time - last_time);
    // last_time = current_time;
    // Serial.print(",");
    // Serial.println();


    unsigned long current_time = millis();
    Serial.print(rxID);
    Serial.print(",");
    Serial.print(current_time);
    Serial.print(",");
    for (size_t i = 0; i < 8; i++){
        Serial.print(rxData[i]);
        if (i < len - 1) Serial.print(",");
    }
    Serial.println();
  }

}



