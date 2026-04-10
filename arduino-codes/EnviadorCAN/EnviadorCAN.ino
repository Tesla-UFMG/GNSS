#include "mcp_can.h"
#include <SPI.h>

#define tdelay 50  // Delay não necessario no código, mas que ajuda a visualizar a interrupção

MCP_CAN CAN(10);  // Set CS to pin 10

//Define se vai enviar um unico buffer de dados da CAN (word 0, 1, 2, e 3), ou varios
bool MODE_singleBuffer = true;

//Formato de buffer para o modo de unico buffer
uint8_t data_buffer_single[4] = { 200, 1, 3, 143 };  // word_0, word_1, word_2, word_3

/*
Formato de buffer para o modo de varios buffers, o tamanho da variavel é igual ao produto
do tamanho do tipo de variável (2, "uint16_t"), vezes a quantidade de buffers (x), vezes a quantidade
de posições por buffer (4). Ou seja, size = 2 * [x]*[4]
*/
uint16_t data_buffer[][4] = { { 200, 1, 3, 143 },     //  ID: 76 || VEL_MED, VOLANTE, ACELERADOR, FREIO
                              { 45, 12, 100, 143 },   //  ID: 77 || MODO_ECU, GAIN_TORQUE, HODOM_PARC, HODOMM_TOT
                              { 200, 1, 3, 143 },     //  ID: 78 || ECU_EVET_1, ECU_EVENT_2, REF_TOR_MD, REF_TOR_ME
                              { 45, 12, 100, 143 },   //  ID: 79 || VEL_RODA_DE, VEL_RODA_DD, VEL_RODA_TD, VEL_RODA_TD
                              { 200, 1, 3, 143 },     //  ID: 80 || REDU_TOR_RE, REDU_TOR_RD, EXTRA, EXTRA
                              { 45, 12, 100, 143 },   //  ID: 81 || FREN_REG, ***** , FLAG_IMU_BSE, FLAG_IMU_VEL
                              { 200, 1, 3, 143 },     //  ID: 85 || VELOC_ME, TORQUE_ME, POTEN_ME, CORRENTE_ME
                              { 45, 12, 100, 143 },   //  ID: 86 || ENERGIA_ME, SOBRECAR_ME, TEMP1_MOSF_ME, TEMP2_MOSF_ME
                              { 200, 1, 3, 143 },     //  ID: 87 || MSG_PERD_ME, CONT_BUSOFF_ME, STATE_CAN_ME, *****
                              { 45, 12, 100, 143 },   //  ID: 88 || STATE_INV_ME, FALHA_ME, ALARME_ME, *****
                              { 200, 1, 3, 143 },     //  ID: 95 || VELOC_MD, TORQUE_MD, POTEN_MD, CORRENTE_MD
                              { 45, 12, 100, 143 },   //  ID: 96 || ENERGIA_MD, SOBRECAR_MD, TEMP1_MOSF_MD, TEMP2_MOSF_MD
                              { 200, 1, 3, 143 },     //  ID: 97 || MSG_PERD_MD, CONT_BUSOFF_MD, STATE_CAN_MD, *****
                              { 45, 12, 100, 143 },   //  ID: 98 || STATE_INV_MD, FALHA_MD, ALARME_MD, *****
                              { 45, 12, 100, 143 },   //  ID: 100 || ECU_DEBUG0, ECU_DEBUG1, ECU_DEBUG2, *****
                              { 200, 1, 3, 143 },     //  ID: 151 || BEACON_VIR, ***** , ***** , *****
                              { 45, 12, 100, 143 },   //  ID: 152 || SENS_PRESS_D, SENS_PRESS_T, BRAKE_BIAS, COL_HODOM
                              { 200, 1, 3, 143 },     //  ID: 153 || DATALOG_STATUS, ***** , ***** , *****
                              { 45, 12, 100, 143 },   //  ID: 154 || MLX_DE_1, MLX_DE_2, MLX_DE_3, MLX_DE_4
                              { 200, 1, 3, 143 },     //  ID: 155 || MLX_DE_5, MLX_DE_6, MLX_DE_7, MLX_DE_8
                              { 45, 12, 100, 143 },   //  ID: 156 || MLX_DE_9, MLX_DE_10, MLX_DE_11, MLX_DE_12
                              { 200, 1, 3, 143 },     //  ID: 157 || MLX_DE_13, MLX_DE_14, MLX_DE_15, MLX_DE_16
                              { 45, 12, 100, 143 },   //  ID: 158 || MLX_DD_1, MLX_DD_2, MLX_DD_3, MLX_DD_4
                              { 200, 1, 3, 143 },     //  ID: 159 || MLX_DD_5, MLX_DD_6, MLX_DD_7, MLX_DD_8
                              { 45, 12, 100, 143 },   //  ID: 160 || MLX_DD_9, MLX_DD_10, MLX_DD_11, MLX_DD_12
                              { 200, 1, 3, 143 },     //  ID: 161 || MLX_DD_13, MLX_DD_14, MLX_DD_15, MLX_DD_16
                              { 45, 12, 100, 143 },   //  ID: 162 || MLX_TE_1, MLX_TE_2, MLX_TE_3, MLX_TE_4
                              { 200, 1, 3, 143 },     //  ID: 163 || MLX_TE_5, MLX_TE_6, MLX_TE_7, MLX_TE_8
                              { 45, 12, 100, 143 },   //  ID: 164 || MLX_TE_9, MLX_TE_10, MLX_TE_11, MLX_TE_12
                              { 200, 1, 3, 143 },     //  ID: 165 || MLX_TE_13, MLX_TE_14, MLX_TE_15, MLX_TE_16
                              { 200, 1, 3, 143 },     //  ID: 166 || MLX_TD_1, MLX_TD_2, MLX_TD_3, MLX_TD_4
                              { 200, 1, 3, 143 },     //  ID: 167 || MLX_TD_5, MLX_TD_6, MLX_TD_7, MLX_TD_8
                              { 45, 12, 100, 143 },   //  ID: 168 || MLX_TD_9, MLX_TD_10, MLX_TD_11, MLX_TD_12
                              { 200, 1, 3, 143 },     //  ID: 169 || MLX_TD_13, MLX_TD_14, MLX_TD_15, MLX_TD_16
                              { 200, 1, 3, 143 },     //  ID: 200 || AQS_DEBUG0, AQS_DEBUG1, AQS_DEBUG2, *****
                              { 45, 12, 100, 143 },   //  ID: 226 || TEN_MAX, TEN_MIN, VAR_TEN, TEMP_MAX
                              { 200, 1, 3, 143 },     //  ID: 227 || MODO_BMS, FLAG_ERRO, STATE_CONTA, TEN_TRATI
                              { 45, 12, 100, 143 },   //  ID: 230 || SENS_CORR_0, SENS_CORR_1, SENS_CORR_2, SENS_CORR_3
                              { 200, 1, 3, 143 },     //  ID: 231 || CARGA_ATUAL, SOC_INICIAL, CARGA_INTE, TEMP_MED
                              { 45, 12, 100, 143 },   //  ID: 232 || VALOR_SOC, ***** , ***** , *****
                              { 200, 1, 3, 143 },     //  ID: 250 || SS_DEBUG0, SS_DEBUG1, SS_DEBUG2, *****
                              { 45, 12, 100, 143 },   //  ID: 291 || ACELE_X, ACELE_Y, ACELE_Z, IMU_TEMP
                              { 200, 1, 3, 143 },     //  ID: 292 || GIROS_X, GIROS_Y, GIROS_Z, IMU_ERRO
                              { 45, 12, 100, 143 },   //  ID: 301 || PCK1_TEN_CEL1, PCK1_TEN_CEL2, PCK1_TEN_CEL3, PCK1_TEN_CEL4
                              { 200, 1, 3, 143 },     //  ID: 302 || PCK1_TEN_CEL5, PCK1_TEN_CEL6, PCK1_TEN_CEL7, PCK1_TEN_CEL8
                              { 45, 12, 100, 143 },   //  ID: 303 || PCK1_TEN_CEL9, PCK1_TEN_CEL10, PCK1_TEN_CEL11, PCK1_TEN_CEL12
                              { 200, 1, 3, 143 },     //  ID: 304 || PCK1_TERMI1, PCK1_TERMI2, PCK1_TERMI3, PCK1_TERMI4
                              { 45, 12, 100, 143 },   //  ID: 305 || PCK1_TERMI5, PCK1_TEN_TOT, PCK1_TEN_REF, PCK1_FLAG_BAL
                              { 200, 1, 3, 143 },     //  ID: 306 || PCK2_TEN_CEL1, PCK2_TEN_CEL2, PCK2_TEN_CEL3, PCK2_TEN_CEL4
                              { 200, 1, 3, 143 },     //  ID: 307 || PCK2_TEN_CEL5, PCK2_TEN_CEL6, PCK2_TEN_CEL7, PCK2_TEN_CEL8
                              { 45, 12, 100, 143 },   //  ID: 308 || PCK2_TEN_CEL9, PCK2_TEN_CEL10, PCK2_TEN_CEL11, PCK2_TEN_CEL12
                              { 200, 1, 3, 143 },     //  ID: 309 || PCK2_TERMI1, PCK2_TERMI2, PCK2_TERMI3, PCK2_TERMI4
                              { 45, 12, 100, 143 },   //  ID: 310 || PCK2_TERMI5, PCK2_TEN_TOT, PCK2_TEN_REF, PCK2_FLAG_BAL
                              { 45, 12, 100, 143 },   //  ID: 311 || PCK3_TEN_CEL1, PCK3_TEN_CEL2, PCK3_TEN_CEL3, PCK3_TEN_CEL4
                              { 200, 1, 3, 143 },     //  ID: 312 || PCK3_TEN_CEL5, PCK3_TEN_CEL6, PCK3_TEN_CEL7, PCK3_TEN_CEL8
                              { 45, 12, 100, 143 },   //  ID: 313 || PCK3_TEN_CEL9, PCK3_TEN_CEL10, PCK3_TEN_CEL11, PCK3_TEN_CEL12
                              { 200, 1, 3, 143 },     //  ID: 314 || PCK3_TERMI1, PCK3_TERMI2, PCK3_TERMI3, PCK3_TERMI4
                              { 45, 12, 100, 143 },   //  ID: 315 || PCK3_TERMI5, PCK3_TEN_TOT, PCK3_TEN_REF, PCK3_FLAG_BAL
                              { 200, 1, 3, 143 },     //  ID: 316 || PCK4_TEN_CEL1, PCK4_TEN_CEL2, PCK4_TEN_CEL3, PCK4_TEN_CEL4
                              { 200, 1, 3, 143 },     //  ID: 317 || PCK4_TEN_CEL5, PCK4_TEN_CEL6, PCK4_TEN_CEL7, PCK4_TEN_CEL8
                              { 45, 12, 100, 143 },   //  ID: 318 || PCK4_TEN_CEL9, PCK4_TEN_CEL10, PCK4_TEN_CEL11, PCK4_TEN_CEL12
                              { 200, 1, 3, 143 },     //  ID: 319 || PCK4_TERMI1, PCK4_TERMI2, PCK4_TERMI3, PCK4_TERMI4
                              { 45, 12, 100, 143 } };  //  ID: 320 || PCK4_TERMI5, PCK4_TEN_TOT, PCK4_TEN_REF, PCK4_FLAG_BAL

/*
Vetor de armazenamento de ID's, o tamanho é 2*x, onde 2 é o tamanho do tipo de variável (uint16_t), e "x",
a quantidade de posições no vetor
*/
uint16_t id[] = { 76, 77, 78, 79, 80,
                  81, 85, 86, 87, 88,
                  95, 96, 97, 98, 100,
                  151, 152, 153, 154,
                  155, 156, 157, 158,
                  159, 160, 161, 162,
                  163, 164, 165, 166,
                  167, 168, 169, 200,
                  226, 227, 230, 231,
                  232, 250, 291, 292,
                  301, 302, 303, 304,
                  305, 306, 307, 308,
                  309, 310, 311, 312,
                  313, 314, 315, 316,
                  317, 318, 319, 320 };  // ID: 76, ID: 77, ID: 78, ID: 79, ...

void setup() {
  Serial.begin(115200);
  CAN_Init(&CAN);
}

void loop() {
  if (MODE_singleBuffer == true)
    CAN_SendMessage(&CAN, id, sizeof(id) / 2, data_buffer_single);  // A id é dividida por 2, para pegar a quantidade de posições
  //ou "CAN_SendMessage(&CAN, 'ID', data_buffer_single);" for single ID
//  else
//    CAN_SendMessage_multBuf(&CAN, id, sizeof(id) / 2, data_buffer, sizeof(data_buffer) / 2 * 4);
  // O data_buffer é dividida por 2*4, para pegar a quantidade de buffers independentes

  // CAN_MessageReceive_IT(&CAN, can_vector);
  delay(500);
}
