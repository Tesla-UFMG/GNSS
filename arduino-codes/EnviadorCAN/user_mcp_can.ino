/*
  User functions to send and reveive can messages 
  Autor: Gabriel Luiz
*/

#include "mcp_can.h"

uint32_t Identifier;
uint8_t Len;
uint8_t buffer[8];

void STATUS_erro(uint8_t status, uint8_t ID) {
  switch (status) {
    case CAN_OK:
      Serial.print("Send successfully || ID: ");
      Serial.println(ID);
      break;
    case CAN_GETTXBFTIMEOUT:
      Serial.print("ERRO: Get Tx message TIMEOUT || ID: ");
      Serial.println(ID);
      break;
    case CAN_SENDMSGTIMEOUT:
      Serial.print("ERRO: Send message TIMEOUT || ID: ");
      Serial.println(ID);
      break;
  }
}
void CAN_Init(MCP_CAN *hCAN) {
  // init can bus, baudrate: 500k
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.

  while ((*hCAN).begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("Error initializing MCP2515.");
  }
  Serial.println("MCP2515 initialized successfully.");
  (*hCAN).setMode(MCP_NORMAL);  // Change to normal mode to allow messages to be transmitted
}

//Envia apenas um unico buffer de dados da CAN, para apenas uma ID
bool CAN_SendMessage(MCP_CAN *hCAN, uint16_t ID, uint8_t DATA[4]) {

  uint8_t can_msg[8] = { DATA[0], DATA[0] >> 8, DATA[1], DATA[1] >> 8,
                         DATA[2], DATA[2] >> 8, DATA[3], DATA[3] >> 8 };

  STATUS_erro((*hCAN).sendMsgBuf(ID, 8, can_msg), ID);
}

//Envia apenas um unico buffer de dados da CAN, para varias ID's
bool CAN_SendMessage(MCP_CAN *hCAN, uint16_t ID[], uint8_t size_ID, uint8_t DATA[4]) {

  uint8_t can_msg[8] = { DATA[0], DATA[0] >> 8, DATA[1], DATA[1] >> 8,
                         DATA[2], DATA[2] >> 8, DATA[3], DATA[3] >> 8 };

  for (uint8_t i = 0; i < size_ID; i++)
    STATUS_erro((*hCAN).sendMsgBuf(ID[i], 8, can_msg), ID[i]);
}

//Envia varios buffers de dados da CAN para as ID's correspondentes no vetor

//EXPLICAR OS PARAMETROS
bool CAN_SendMessage_multBuf(MCP_CAN *hCAN, uint16_t ID[], uint16_t size_ID,
                             uint8_t DATA[][4], uint16_t size_DATA) {
  //  EXPLICAR O FUNCIONAMENTO INTERNO
  uint16_t size;
  if (size_DATA < size_ID)
    size = size_DATA;
  if (size_DATA > size_ID)
    size = size_ID;
  if (size_DATA == size_ID)
    size = size_DATA;

  for (uint8_t i = 0; i < size; i++) {
    uint8_t can_msg[8] = { DATA[i][0], DATA[i][0] >> 8, DATA[i][1], DATA[i][1] >> 8,
                           DATA[i][2], DATA[i][2] >> 8, DATA[i][3], DATA[i][3] >> 8 };
    STATUS_erro((*hCAN).sendMsgBuf(ID[i], 8, can_msg), ID[i]);
  }
}

// void CAN_MessageReceive_IT(MCP_CAN *hCAN, uint16_t CAN_Vector[][]) {
//   while ((*hCAN).checkReceive() == CAN_MSGAVAIL) {
//     (*hCAN).readMsgBuf(&Identifier, &Len, buffer);
//     for (int i = 0; i < Len; i++) {
//       CAN_Vector[Identifier][i] = buffer[i] + (buffer[i] >> 8);
//     }
//   }
// }