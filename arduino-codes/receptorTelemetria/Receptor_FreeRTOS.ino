#include <SPI.h>
#include <LoRa.h>
#include <string.h> // Para memset e strtok
#include <stdlib.h> // Para strtol

// Lora SPI
#define SS_PIN    18
#define RST_PIN   14
#define DIO0_PIN  26
#define PACKET_SIZE 36

SemaphoreHandle_t loraSem;
void IRAM_ATTR onDio0ISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(loraSem, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

// LED PIN
#define LED_PIN   25
bool ledState = false;

typedef struct {
  uint32_t id;
  uint8_t data[8];
} CanMessage_t;
QueueHandle_t canTxQueue;

enum PrintMode {PM_INT, PM_UINT, PM_FLOAT};
PrintMode print_mode = PM_INT;
uint32_t uint_result[8];
int32_t  int_result[8];
float    float_result[8];

static inline uint16_t u16(uint8_t hi, uint8_t lo) {
  return (uint16_t)hi << 8 | (uint16_t)lo;
}
static inline int16_t s16(uint8_t hi, uint8_t lo) {
  return (int16_t)u16(hi, lo);
}

void toggleLED() {
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState);
}

void setup() {
  // Serial
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Receiver (Heltec V2) - Otimizado");

  // Led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(DIO0_PIN, INPUT);

  // Lora
  SPI.begin();
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(915E6)) {
    Serial.println("Erro: não inicializou LoRa!");
    while (1);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(500E3);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(20);
  loraSem = xSemaphoreCreateBinary();
  attachInterrupt(digitalPinToInterrupt(DIO0_PIN), onDio0ISR, RISING);

  canTxQueue = xQueueCreate(1000, sizeof(CanMessage_t));
  if (canTxQueue == NULL) {
    Serial.println("Out of memory");
    toggleLED();
    while (1);
  }
  LoRa.receive();

  Serial.println("LoRa inicializado em 915 MHz, aguardando pacotes...");

  xTaskCreatePinnedToCore(
    LoraRxTask,       // Função da Tarefa Lora
    "LoraRx",         // Nome
    10000,           // Tamanho da pilha
    NULL,            // Parâmetro
    10,               // Prioridade (Maior que o LoRa, para garantir a captura CAN)
    NULL,            // Handler
    1.               // Core 1
  );

  xTaskCreatePinnedToCore(
    PrintTask,       // Função da Tarefa Lora
    "PrintRx",         // Nome
    10000,           // Tamanho da pilha
    NULL,            // Parâmetro
    5,               // Prioridade (Maior que o LoRa, para garantir a captura CAN)
    NULL,            // Handler
    1.               // Core 1
  );
}

void LoraRxTask(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(loraSem, portMAX_DELAY) == pdTRUE) {
      if (LoRa.parsePacket()) {
        CanMessage_t msg;
        LoRa.readBytes((uint8_t*)&msg.id, 4);
        LoRa.readBytes(msg.data, 8);
        xQueueSend(canTxQueue, &msg, 0);
        toggleLED();
      }
      LoRa.receive();
    }
  }
}
void PrintTask(void *pvParameters) {
  CanMessage_t message;
  while (1) {
    if (xQueueReceive(canTxQueue, &message, portMAX_DELAY) == pdPASS) { 
      switch (message.id) {
        case 76:
          memset(int_result,  0, sizeof(int_result));
          int_result[0] = (int32_t) s16(message.data[1], message.data[0]); // SPEED_AVG
          int_result[1] = (int32_t) s16(message.data[3], message.data[2]); // STEERING_WHEEL
          int_result[2] = (int32_t) s16(message.data[5], message.data[4]); // THROTTLE
          int_result[3] = (int32_t) s16(message.data[7], message.data[6]); // BRAKE
          print_mode = PM_INT;
          break;

        case 77:
          memset(uint_result, 0, sizeof(uint_result));
          uint_result[0] = (uint32_t) u16(message.data[1], message.data[0]); // MODE
          uint_result[1] = (uint32_t) u16(message.data[3], message.data[2]); // TORQUER_GAIN
          uint_result[2] = (uint32_t) u16(message.data[5], message.data[4]); // DISTANCE_P_ODOM
          uint_result[3] = (uint32_t) u16(message.data[7], message.data[6]); // DISTANCE_T_ODOM
          print_mode = PM_UINT;
          break;

        case 78:
          memset(uint_result, 0, sizeof(uint_result));
          uint_result[0] = (uint32_t) u16(message.data[1], message.data[0]); // CONTROL_EVENT_FLAG_1
          uint_result[1] = (uint32_t) u16(message.data[3], message.data[2]); // CONTROL_EVENT_FLAG_2
          uint_result[2] = (uint32_t) u16(message.data[5], message.data[4]); // REF_TORQUE_R_MOTOR
          uint_result[3] = (uint32_t) u16(message.data[7], message.data[6]); // REF_TORQUE_L_MOTOR
          print_mode = PM_UINT;
          break;

        case 79:
          memset(int_result,  0, sizeof(int_result));
          int_result[0] = (int32_t) s16(message.data[1], message.data[0]); // SPEED_LF
          int_result[1] = (int32_t) s16(message.data[3], message.data[2]); // SPEED_LR
          int_result[2] = (int32_t) s16(message.data[5], message.data[4]); // SPEED_RL
          int_result[3] = (int32_t) s16(message.data[7], message.data[6]); // SPEED_RR
          print_mode = PM_INT;
          break;

        case 80:
          memset(int_result,  0, sizeof(int_result));
          int_result[0] = (int32_t) s16(message.data[1], message.data[0]); // ID_PANEL_DEBUG_1
          int_result[1] = (int32_t) s16(message.data[3], message.data[2]); // ID_PANEL_DEBUG_2
          int_result[2] = (int32_t) s16(message.data[5], message.data[4]); // ID_PANEL_DEBUG_3
          int_result[3] = (int32_t) s16(message.data[7], message.data[6]); // ID_PANEL_DEBUG_4
          print_mode = PM_INT;
          break;

        case 81:
          memset(uint_result, 0, sizeof(uint_result));
          uint_result[0] = (uint32_t) u16(message.data[1], message.data[0]); // ID_REGEN_BRAKE_STATE
          print_mode = PM_UINT;
          break;

        case 85:
          memset(int_result,  0, sizeof(int_result));
          int_result[0] = (int32_t) s16(message.data[1], message.data[0]); // SPEED_L_MOTOR
          int_result[1] = (int32_t) s16(message.data[3], message.data[2]); // TORQUE_L_MOTOR
          int_result[2] = (int32_t) s16(message.data[5], message.data[4]); // POWER_L_MOTOR
          int_result[3] = (int32_t) s16(message.data[7], message.data[6]); // CURRENT_L_MOTOR
          print_mode = PM_INT;
          break;

        case 86:
          memset(int_result,  0, sizeof(int_result));
          int_result[0] = (int32_t) s16(message.data[1], message.data[0]); // ENERGY_L_MOTOR
          int_result[1] = (int32_t) s16(message.data[3], message.data[2]); // OVERLOAD_L_MOTOR
          int_result[2] = (int32_t) s16(message.data[5], message.data[4]); // TEMPERATURE1_L
          int_result[3] = (int32_t) s16(message.data[7], message.data[6]); // TEMPERATURE2_L
          print_mode = PM_INT;
          break;

        case 87:
          memset(uint_result, 0, sizeof(uint_result));
          uint_result[0] = (uint32_t) u16(message.data[1], message.data[0]); // ID_LOST_MSG_L_MOTOR
          uint_result[1] = (uint32_t) u16(message.data[3], message.data[2]); // ID_BUS_OFF_L_MOTOR
          uint_result[2] = (uint32_t) u16(message.data[5], message.data[4]); // ID_CAN_STATE_L_MOTOR
          print_mode = PM_UINT;
          break;

        case 88:
          memset(uint_result, 0, sizeof(uint_result));
          uint_result[0] = (uint32_t) u16(message.data[1], message.data[0]); // ID_INV_STATE_L_MOTOR
          uint_result[1] = (uint32_t) u16(message.data[3], message.data[2]); // ID_FAILURE_L_MOTOR
          uint_result[2] = (uint32_t) u16(message.data[5], message.data[4]); // ID_ALARM_L_MOTOR
          print_mode = PM_UINT;
          break;

        case 95:
          memset(int_result,  0, sizeof(int_result));
          int_result[0] = (int32_t) s16(message.data[1], message.data[0]); // ID_SPEED_R_MOTOR
          int_result[1] = (int32_t) s16(message.data[3], message.data[2]); // ID_TORQUE_R_MOTOR
          int_result[2] = (int32_t) s16(message.data[5], message.data[4]); // ID_POWER_R_MOTOR
          int_result[3] = (int32_t) s16(message.data[7], message.data[6]); // ID_CURRENT_R_MOTOR
          print_mode = PM_INT;
          break;

        case 96:
          memset(int_result,  0, sizeof(int_result));
          int_result[0] = (int32_t) s16(message.data[1], message.data[0]); // ID_ENERGY_R_MOTOR
          int_result[1] = (int32_t) s16(message.data[3], message.data[2]); // ID_OVERLOAD_R_MOTOR
          int_result[2] = (int32_t) s16(message.data[5], message.data[4]); // ID_TEMPERATURE1_R
          int_result[3] = (int32_t) s16(message.data[7], message.data[6]); // ID_TEMPERATURE2_R
          print_mode = PM_INT;
          break;

        case 97:
          memset(uint_result, 0, sizeof(uint_result));
          uint_result[0] = (uint32_t) u16(message.data[1], message.data[0]); // ID_LOST_MSG_R_MOTOR
          uint_result[1] = (uint32_t) u16(message.data[3], message.data[2]); // ID_BUS_OFF_R_MOTOR
          uint_result[2] = (uint32_t) u16(message.data[5], message.data[4]); // ID_CAN_STATE_R_MOTOR
          print_mode = PM_UINT;
          break;

        case 98:
          memset(uint_result, 0, sizeof(uint_result));
          uint_result[0] = (uint32_t) u16(message.data[1], message.data[0]); // ID_INV_STATE_R_MOTOR
          uint_result[1] = (uint32_t) u16(message.data[3], message.data[2]); // ID_FAILURE_R_MOTOR
          uint_result[2] = (uint32_t) u16(message.data[5], message.data[4]); // ID_ALARM_R_MOTOR
          print_mode = PM_UINT;
          break;

        case 259:
          memset(int_result,  0, sizeof(int_result));
          int_result[0] = (int32_t) s16(message.data[1], message.data[0]); // AcelX
          int_result[1] = (int32_t) s16(message.data[3], message.data[2]); // AcelY
          int_result[2] = (int32_t) s16(message.data[5], message.data[4]); // AcelZ
          print_mode = PM_INT;
          break;

        case 260:
          memset(int_result,  0, sizeof(int_result));
          int_result[0] = (int32_t) s16(message.data[1], message.data[0]); // GyroX
          int_result[1] = (int32_t) s16(message.data[3], message.data[2]); // GyroY
          int_result[2] = (int32_t) s16(message.data[5], message.data[4]); // GyroZ
          print_mode = PM_INT;
          break;

        case 261:
          memset(int_result,  0, sizeof(int_result));
          int_result[0] = (int32_t) s16(message.data[1], message.data[0]); // Temp
          print_mode = PM_INT;
          break;

        case 361:
          memset(int_result,  0, sizeof(int_result));
          int_result[0] = (int32_t) s16(message.data[1], message.data[0]); // ID_PANEL_DEBUG_1
          int_result[1] = (int32_t) s16(message.data[3], message.data[2]); // ID_PANEL_DEBUG_2
          int_result[2] = (int32_t) s16(message.data[5], message.data[4]); // ID_PANEL_DEBUG_3
          int_result[3] = (int32_t) s16(message.data[7], message.data[6]); // ID_PANEL_DEBUG_4
          print_mode = PM_INT;
          break;

        case 300: // STACK 1
          memset(float_result, 0, sizeof(float_result));
          float_result[0] = (float) u16(message.data[1], message.data[0]) / 10000.0f; // CELL_1
          float_result[1] = (float) u16(message.data[3], message.data[2]) / 10000.0f; // CELL_2
          float_result[2] = (float) u16(message.data[5], message.data[4]) / 10000.0f; // CELL_3
          float_result[3] = (float) u16(message.data[7], message.data[6]) / 10000.0f; // CELL_4
          print_mode = PM_FLOAT;
          break;

        case 301: // STACK 2
          memset(float_result, 0, sizeof(float_result));
          float_result[0] = (float) u16(message.data[1], message.data[0]) / 10000.0f; // CELL_1
          float_result[1] = (float) u16(message.data[3], message.data[2]) / 10000.0f; // CELL_2
          float_result[2] = (float) u16(message.data[5], message.data[4]) / 10000.0f; // CELL_3
          float_result[3] = (float) u16(message.data[7], message.data[6]) / 10000.0f; // CELL_4
          print_mode = PM_FLOAT;
          break;

        case 302: // STACK 3
          memset(float_result, 0, sizeof(float_result));
          float_result[0] = (float) u16(message.data[1], message.data[0]) / 10000.0f; // CELL_1
          float_result[1] = (float) u16(message.data[3], message.data[2]) / 10000.0f; // CELL_2
          float_result[2] = (float) u16(message.data[5], message.data[4]) / 10000.0f; // CELL_3
          float_result[3] = (float) u16(message.data[7], message.data[6]) / 10000.0f; // CELL_4
          print_mode = PM_FLOAT;
          break;

        case 303: // STACK 4
          memset(float_result, 0, sizeof(float_result));
          float_result[0] = (float) u16(message.data[1], message.data[0]) / 10000.0f; // CELL_1
          float_result[1] = (float) u16(message.data[3], message.data[2]) / 10000.0f; // CELL_2
          float_result[2] = (float) u16(message.data[5], message.data[4]) / 10000.0f; // CELL_3
          float_result[3] = (float) u16(message.data[7], message.data[6]) / 10000.0f; // CELL_4
          print_mode = PM_FLOAT;
          break;

        case 304: // STACK 5
          memset(float_result, 0, sizeof(float_result));
          float_result[0] = (float) u16(message.data[1], message.data[0]) / 10000.0f; // CELL_1
          float_result[1] = (float) u16(message.data[3], message.data[2]) / 10000.0f; // CELL_2
          float_result[2] = (float) u16(message.data[5], message.data[4]) / 10000.0f; // CELL_3
          float_result[3] = (float) u16(message.data[7], message.data[6]) / 10000.0f; // CELL_4
          print_mode = PM_FLOAT;
          break;

        case 305: // STACK 6
          memset(float_result, 0, sizeof(float_result));
          float_result[0] = (float) u16(message.data[1], message.data[0]) / 10000.0f; // CELL_1
          float_result[1] = (float) u16(message.data[3], message.data[2]) / 10000.0f; // CELL_2
          float_result[2] = (float) u16(message.data[5], message.data[4]) / 10000.0f; // CELL_3
          float_result[3] = (float) u16(message.data[7], message.data[6]) / 10000.0f; // CELL_4
          print_mode = PM_FLOAT;
          break;

        case 306: // ACCUMULATOR PARAMS
          memset(float_result, 0, sizeof(float_result));
          memset(int_result,   0, sizeof(int_result));
          float_result[0] = (float) u16(message.data[1], message.data[0]) / 10000.0f; // MIN_VOLTAGE
          float_result[1] = (float) u16(message.data[3], message.data[2]) / 10000.0f; // MAX_VOLTAGE
          float_result[2] = (float) u16(message.data[5], message.data[4]) / 10.0f;   // TOTAL_VOLTAGE
          int_result[3]   = (int32_t) u16(message.data[7], message.data[6]);         // SHUNT CURRENT (mantive como inteiro)
          print_mode = PM_FLOAT;
          break;

        case 307: // BMS PARAMS
          memset(int_result,  0, sizeof(int_result));
          int_result[0] = (int32_t) s16(message.data[1], message.data[0]); // BMS_MODE
          int_result[1] = (int32_t) s16(message.data[3], message.data[2]); // BMS_ERROR
          int_result[2] = (int32_t) s16(message.data[5], message.data[4]); // AIR_P
          int_result[3] = (int32_t) s16(message.data[7], message.data[6]); // AIR_N
          print_mode = PM_INT;
          break;

        default:
          memset(int_result,  0, sizeof(int_result));
          int_result[0] = (int32_t) s16(message.data[1], message.data[0]); // ID_PANEL_DEBUG_1
          int_result[1] = (int32_t) s16(message.data[3], message.data[2]); // ID_PANEL_DEBUG_2
          int_result[2] = (int32_t) s16(message.data[5], message.data[4]); // ID_PANEL_DEBUG_3
          int_result[3] = (int32_t) s16(message.data[7], message.data[6]); // ID_PANEL_DEBUG_4
          print_mode = PM_INT;
          break;
        }


        Serial.print(message.id);
        Serial.print(",");

        if (print_mode == PM_UINT) {
          for (size_t i = 0; i < 8; i++) {
            Serial.print(uint_result[i]);
            if (i < 7) Serial.print(",");
          }
        } else if (print_mode == PM_INT) {
          for (size_t i = 0; i < 8; i++) {
            Serial.print(int_result[i]);
            if (i < 7) Serial.print(",");
          }
        } else { // PM_FLOAT
          for (size_t i = 0; i < 8; i++) {
            Serial.print(float_result[i], 2);
            if (i < 7) Serial.print(",");
          }
        }
        Serial.println();
      }
    }
  }



void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}