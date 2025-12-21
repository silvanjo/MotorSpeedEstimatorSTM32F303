/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "arm_math.h"
#include "stm32f3xx_hal_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_PACKET_SIZE 512
#define ADC_BUFFER_SIZE 2048
#define RINGBUFFER_SIZE 2048
#define FFT_SIZE ADC_BUFFER_SIZE

#define COBS_MAX_PACKET_SIZE UART_PACKET_SIZE
#define COBS_PACKET_QUEUE_SLOTS 8

/* Spectrum transmission parameters */
#define SPECTRUM_BINS 200          /* Number of bins to send (first 200 bins) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

bool input_adc = false;   /* If true the input buffer will be filled with data from the adc */
bool input_uart = true;   /* If true the input buffer will be filled with data from uart */

/* 
This is the input buffer for the MOPA algorithm. It will either be filled using
ADC or with data send from UART.
*/
uint16_t adc_input_buffer[ADC_BUFFER_SIZE];    /* Input buffer filled by ADC DMA */
uint16_t uart_input_buffer[ADC_BUFFER_SIZE];   /* Input buffer filled from UART1 packets */
volatile uint16_t uart_input_write_idx = 0;    /* Write index for UART input buffer */

volatile uint8_t buffer_half_full_flag = 0;
volatile uint8_t buffer_full_flag = 0;
volatile uint8_t uart_buffer_half_full_flag = 0;  /* Flag: UART buffer first half ready */
volatile uint8_t uart_buffer_full_flag = 0;       /* Flag: UART buffer completely full */
volatile uint8_t uart_buffer_initialized = 0;     /* Flag: UART buffer has been filled at least once */

// FFT buffers and instance
float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE * 2];  // Complex output: real and imaginary
float32_t fft_magnitude[FFT_SIZE];
float32_t hann_window[FFT_SIZE];  // Hann window coefficients

arm_rfft_fast_instance_f32 fft_instance;

// Voltage measurement variables
#define VREF 3.3f  // Reference voltage in volts
#define ADC_MAX 4095.0f  // 12-bit ADC max value (2^12 - 1)

// UART DMA receive buffers and flags
uint8_t uart1_rx_buffer[UART_PACKET_SIZE];
uint8_t uart2_rx_buffer[UART_PACKET_SIZE];
uint8_t uart3_rx_buffer[UART_PACKET_SIZE];

uint8_t uart1_tx_buffer[UART_PACKET_SIZE];
uint8_t uart2_tx_buffer[UART_PACKET_SIZE];
uint8_t uart3_tx_buffer[UART_PACKET_SIZE];

volatile uint8_t uart1_rx_flag = 0;
volatile uint8_t uart2_rx_flag = 0;
volatile uint8_t uart3_rx_flag = 0;

volatile uint8_t uart1_tx_complete = 1;
volatile uint8_t uart2_tx_complete = 1;
volatile uint8_t uart3_tx_complete = 1;

volatile uint16_t uart1_rx_len = 0;
volatile uint16_t uart2_rx_len = 0;
volatile uint16_t uart3_rx_len = 0;

/* COBS Packet Queue - stores complete decoded packets */
typedef struct {
  uint8_t data[COBS_PACKET_QUEUE_SLOTS][COBS_MAX_PACKET_SIZE];
  uint16_t lengths[COBS_PACKET_QUEUE_SLOTS];
  volatile uint8_t write_idx;
  volatile uint8_t read_idx;
  volatile uint8_t count;
} cobs_packet_queue_t;

cobs_packet_queue_t cobs_rx_queue;

/* COBS packet reception state machine */
typedef enum {
  COBS_STATE_WAIT_START_DELIMITER,  /* Waiting for 0x00 start */
  COBS_STATE_RECEIVING,              /* Receiving packet data */
} cobs_rx_state_t;

cobs_rx_state_t cobs_rx_state = COBS_STATE_WAIT_START_DELIMITER;
uint8_t cobs_rx_temp_buffer[COBS_MAX_PACKET_SIZE + 2]; /* Temp buffer for incoming COBS data */
uint16_t cobs_rx_temp_idx = 0;

/* Spectrum transmission buffer and state */
uint8_t spectrum_tx_buffer[SPECTRUM_BINS * 2 + 20];  /* uint16 data + header + COBS overhead */
volatile uint8_t spectrum_ready_to_send = 0;
volatile uint16_t spectrum_packet_len = 0;

volatile uint8_t cobs_ready_to_receive = 1; /* Flag: ready to receive new packet */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void prepare_spectrum_packet(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
DEBUGGING FUNCTIONS
*/

/* When a message is received on a UART port it will be send back with a ACK. */
void poll_uart() {
  // Handle UART1 received message
  if (uart1_rx_flag && uart1_tx_complete)
  {
    uart1_rx_flag = 0;
    uart1_tx_complete = 0;
    // Echo back with ACK using DMA
    int len = sprintf((char*)uart1_tx_buffer, "ACK UART1 (%d bytes): %.*s\r\n", uart1_rx_len, uart1_rx_len, uart1_rx_buffer);
    HAL_UART_Transmit_DMA(&huart1, uart1_tx_buffer, len);
  }

  // Handle UART2 received message
  if (uart2_rx_flag && uart2_tx_complete)
  {
    uart2_rx_flag = 0;
    uart2_tx_complete = 0;
    // Echo back with ACK using DMA
    int len = sprintf((char*)uart2_tx_buffer, "ACK UART2 (%d bytes): %.*s\r\n", uart2_rx_len, uart2_rx_len, uart2_rx_buffer);
    HAL_UART_Transmit_DMA(&huart2, uart2_tx_buffer, len);
  }

  // Handle UART3 received message
  if (uart3_rx_flag && uart3_tx_complete)
  {
    uart3_rx_flag = 0;
    uart3_tx_complete = 0;
    // Echo back with ACK using DMA
    int len = sprintf((char*)uart3_tx_buffer, "ACK UART3 (%d bytes): %.*s\r\n", uart3_rx_len, uart3_rx_len, uart3_rx_buffer);
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buffer, len);
  }
}

// Calculate and send voltage measurement via UART
void send_voltage_measurement_uart(UART_HandleTypeDef* uart_port, uint16_t* adc_data, uint32_t offset)
{
  // Calculate average ADC value over half buffer size (new data only)
  uint32_t half_size = FFT_SIZE / 2;
  uint32_t adc_sum = 0;
  uint16_t min_val = 4095;
  uint16_t max_val = 0;

  for (uint32_t i = 0; i < half_size; i++)
  {
    uint16_t val = adc_data[offset + i];
    adc_sum += val;
    if (val < min_val) min_val = val;
    if (val > max_val) max_val = val;
  }

  float avg_adc = (float)adc_sum / (float)half_size;

  // Convert ADC value to voltage
  float voltage = (avg_adc / ADC_MAX) * VREF;
  float min_voltage = ((float)min_val / ADC_MAX) * VREF;
  float max_voltage = ((float)max_val / ADC_MAX) * VREF;

  // Send voltage via UART
  char msg[150];
  int adc_int = (int)avg_adc;
  int voltage_mv = (int)(voltage * 1000.0f);  // Convert to millivolts
  int min_mv = (int)(min_voltage * 1000.0f);
  int max_mv = (int)(max_voltage * 1000.0f);

  int len = sprintf(msg, "ADC: %d | Voltage: %d.%03d V | Min: %d.%03d V | Max: %d.%03d V\r\n",
                    adc_int,
                    voltage_mv / 1000, voltage_mv % 1000,
                    min_mv / 1000, min_mv % 1000,
                    max_mv / 1000, max_mv % 1000);
  HAL_UART_Transmit(uart_port, (uint8_t*)msg, len, 1000);
}

/*
HELPER FUNCTIONS
*/

// Initialize Hann window coefficients
void init_hann_window(void)
{
  const float32_t pi = 3.14159265358979323846f;
  for (uint32_t i = 0; i < FFT_SIZE; i++)
  {
    // Hann window: w(n) = 0.5 * (1 - cos(2*pi*n/(N-1)))
    hann_window[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * pi * i / (FFT_SIZE - 1)));
  }
}

/*
COBS PACKET QUEUE
*/

static inline void cobs_packet_queue_init(cobs_packet_queue_t* q) {
  q->write_idx = 0;
  q->read_idx = 0;
  q->count = 0;
}

static inline uint8_t cobs_packet_queue_has_space(const cobs_packet_queue_t* q) {
  return q->count < COBS_PACKET_QUEUE_SLOTS;
}

static inline uint8_t cobs_packet_queue_has_packet(const cobs_packet_queue_t* q) {
  return q->count > 0;
}

static inline int cobs_packet_queue_push(cobs_packet_queue_t* q, const uint8_t* data, uint16_t len) {
  if (!cobs_packet_queue_has_space(q)) return -1;
  if (len > COBS_MAX_PACKET_SIZE) return -1;
  memcpy(q->data[q->write_idx], data, len);
  q->lengths[q->write_idx] = len;
  q->write_idx = (q->write_idx + 1) % COBS_PACKET_QUEUE_SLOTS;
  q->count++;
  return 0;
}

static inline uint16_t cobs_packet_queue_pop(cobs_packet_queue_t* q, uint8_t* out, uint16_t max_len) {
  if (!cobs_packet_queue_has_packet(q)) return 0;
  uint16_t len = q->lengths[q->read_idx];
  if (len > max_len) len = max_len;
  memcpy(out, q->data[q->read_idx], len);
  q->read_idx = (q->read_idx + 1) % COBS_PACKET_QUEUE_SLOTS;
  q->count--;
  return len;
}

/*
COBS
*/

/* COBS encode: transforms data so it contains no 0x00 bytes */
static uint16_t cobs_encode(const uint8_t* input, uint16_t len, uint8_t* output) {
  uint16_t read_idx = 0;
  uint16_t write_idx = 1;
  uint16_t code_idx = 0;
  uint8_t code = 1;

  while (read_idx < len) {
    if (input[read_idx] == 0x00) {
      output[code_idx] = code;
      code_idx = write_idx++;
      code = 1;
    } else {
      output[write_idx++] = input[read_idx];
      code++;
      if (code == 0xFF) {
        output[code_idx] = code;
        code_idx = write_idx++;
        code = 1;
      }
    }
    read_idx++;
  }
  output[code_idx] = code;
  return write_idx;
}

/* COBS decode: restores original data from COBS-encoded data */
static int16_t cobs_decode(const uint8_t* input, uint16_t len, uint8_t* output) {
  uint16_t read_idx = 0;
  uint16_t write_idx = 0;

  while (read_idx < len) {
    uint8_t code = input[read_idx++];
    if (code == 0) return -1; // Invalid COBS

    for (uint8_t i = 1; i < code; i++) {
      if (read_idx >= len) return -1; // Truncated
      output[write_idx++] = input[read_idx++];
    }

    if (code < 0xFF && read_idx < len) {
      output[write_idx++] = 0x00;
    }
  }

  // Remove trailing zero if present
  if (write_idx > 0 && output[write_idx - 1] == 0x00) {
    write_idx--;
  }

  return write_idx;
}

/*
COBS FLOW CONTROL
*/

static uint8_t cobs_ready_msg[] = {0x00, 0x01, 0x00};  /* COBS-encoded empty "ready" packet */

static void cobs_send_ready(void) {
  if (uart1_tx_complete && cobs_packet_queue_has_space(&cobs_rx_queue)) {
    uart1_tx_complete = 0;
    HAL_UART_Transmit_DMA(&huart1, cobs_ready_msg, sizeof(cobs_ready_msg));
  }
}

static void cobs_process_received_data(const uint8_t* data, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    uint8_t byte = data[i];

    switch (cobs_rx_state) {
      case COBS_STATE_WAIT_START_DELIMITER:
        if (byte == 0x00) {
          cobs_rx_state = COBS_STATE_RECEIVING;
          cobs_rx_temp_idx = 0;
        }
        break;

      case COBS_STATE_RECEIVING:
        if (byte == 0x00) {
          /* End delimiter - decode and store packet */
          if (cobs_rx_temp_idx > 0 && cobs_packet_queue_has_space(&cobs_rx_queue)) {
            uint8_t decoded[COBS_MAX_PACKET_SIZE];
            int16_t decoded_len = cobs_decode(cobs_rx_temp_buffer, cobs_rx_temp_idx, decoded);
            if (decoded_len > 0) {
              cobs_packet_queue_push(&cobs_rx_queue, decoded, decoded_len);
            }
          }
          cobs_rx_state = COBS_STATE_WAIT_START_DELIMITER;
          cobs_rx_temp_idx = 0;
        } else {
          /* Store byte if buffer not full */
          if (cobs_rx_temp_idx < COBS_MAX_PACKET_SIZE + 2) {
            cobs_rx_temp_buffer[cobs_rx_temp_idx++] = byte;
          } else {
            /* Buffer overflow - reset state machine */
            cobs_rx_state = COBS_STATE_WAIT_START_DELIMITER;
            cobs_rx_temp_idx = 0;
          }
        }
        break;
    }
  }
}

/*
MOPA FUNCTIONS
*/

// Process FFT on ADC buffer segment with Hann windowing
void process_fft(uint16_t* input, uint32_t offset)
{
  uint32_t half_size = FFT_SIZE / 2;
  uint32_t old_data_offset;

  // Determine the offset of the old data 
  if (offset == 0) {
      old_data_offset = half_size;
  } else {
      old_data_offset = 0;
  }

  /* Construct Input Buffer: [OLD DATA] + [NEW DATA] */

  // First, copy data without windowing to calculate DC offset
  float32_t dc_sum = 0.0f;
  for (uint32_t i = 0; i < half_size; i++)
  {
      fft_input[i] = (float32_t) input[old_data_offset + i];
      dc_sum += fft_input[i];
  }
  for (uint32_t i = 0; i < half_size; i++)
  {
      fft_input[i + half_size] = (float32_t) input[offset + i];
      dc_sum += fft_input[i + half_size];
  }

  // Calculate and remove DC offset
  float32_t dc_offset = dc_sum / (float32_t)FFT_SIZE;

  // Remove DC offset and apply Hann window
  for (uint32_t i = 0; i < FFT_SIZE; i++)
  {
      fft_input[i] = (fft_input[i] - dc_offset) * hann_window[i];
  }

  // Perform FFT
  arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);

  // Calculate magnitude spectrum
  arm_cmplx_mag_f32(fft_output, fft_magnitude, FFT_SIZE / 2);

  // Normalize the magnitude spectrum
  float32_t norm_factor = 2.0f / (float32_t)FFT_SIZE;
  for (uint32_t i = 0; i < FFT_SIZE / 2; i++) {
    fft_magnitude[i] *= norm_factor;
  }

  // Prepare spectrum packet for transmission
  prepare_spectrum_packet();
}

/* Prepare spectrum data for transmission via UART3 */
static void prepare_spectrum_packet(void) {
  /* Packet format: [0x01=spectrum_id] [bin_count_low] [bin_count_high] [uint16 data...] */
  uint8_t raw_packet[SPECTRUM_BINS * 2 + 3];
  raw_packet[0] = 0x01;  /* Packet type: spectrum */
  raw_packet[1] = SPECTRUM_BINS & 0xFF;
  raw_packet[2] = (SPECTRUM_BINS >> 8) & 0xFF;

  /* Convert float magnitudes to uint16 */
  for (uint16_t i = 0; i < SPECTRUM_BINS; i++) {
    float32_t mag = fft_magnitude[i];
    uint16_t val = (mag > 65535.0f) ? 65535 : (uint16_t)mag;
    raw_packet[3 + i * 2] = val & 0xFF;
    raw_packet[3 + i * 2 + 1] = (val >> 8) & 0xFF;
  }

  /* COBS encode and frame */
  spectrum_tx_buffer[0] = 0x00;  /* Start delimiter */
  uint16_t encoded_len = cobs_encode(raw_packet, SPECTRUM_BINS * 2 + 3, &spectrum_tx_buffer[1]);
  spectrum_tx_buffer[1 + encoded_len] = 0x00;  /* End delimiter */

  spectrum_packet_len = encoded_len + 2;
  spectrum_ready_to_send = 1;
}

/*
INTERRPUT ROUTINES
*/

// DMA half-transfer complete callback
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    buffer_half_full_flag = 1;
  }
}

// DMA transfer complete callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    buffer_full_flag = 1;
  }
}

// UART TX complete callback (called when DMA transmission finishes)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart1_tx_complete = 1;
  }
  else if (huart->Instance == USART2)
  {
    uart2_tx_complete = 1;
  }
  else if (huart->Instance == USART3)
  {
    uart3_tx_complete = 1;
  }
}

// UART RX Event callback (called on IDLE line detection or buffer full)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1)
  {
    /* Process COBS-encoded data */
    cobs_process_received_data(uart1_rx_buffer, Size);
    /* Restart DMA reception */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer, UART_PACKET_SIZE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
  else if (huart->Instance == USART2)
  {
    uart2_rx_len = Size;
    uart2_rx_flag = 1;
    // Restart DMA reception for next message
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_buffer, UART_PACKET_SIZE);
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);  // Disable half-transfer interrupt
  }
  else if (huart->Instance == USART3)
  {
    uart3_rx_len = Size;
    uart3_rx_flag = 1;
    // Restart DMA reception for next message
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_buffer, UART_PACKET_SIZE);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);  // Disable half-transfer interrupt
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize COBS packet queue
  cobs_packet_queue_init(&cobs_rx_queue);

  // Start UART DMA reception with IDLE line detection
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer, UART_PACKET_SIZE);
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);  // Disable half-transfer interrupt

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_buffer, UART_PACKET_SIZE);
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);  // Disable half-transfer interrupt

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_buffer, UART_PACKET_SIZE);
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);  // Disable half-transfer interrupt

  uint8_t uart_ready_msg[] = "UART DMA IDLE detection started\r\n";
  HAL_UART_Transmit(&huart2, uart_ready_msg, sizeof(uart_ready_msg) - 1, 100);

  // Initialize FFT instance
  arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);

  // Initialize Hann window coefficients for spectral leakage reduction
  init_hann_window();
  uint8_t window_msg[] = "Hann window initialized\r\n";
  HAL_UART_Transmit(&huart2, window_msg, sizeof(window_msg) - 1, 100);

  // Calibrate ADC
  HAL_StatusTypeDef cal_status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  if (cal_status == HAL_OK)
  {
    uint8_t cal_ok[] = "ADC calibration successful\r\n";
    HAL_UART_Transmit(&huart2, cal_ok, sizeof(cal_ok) - 1, 100);
  }
  else
  {
    uint8_t cal_err[] = "ADC calibration FAILED!\r\n";
    HAL_UART_Transmit(&huart2, cal_err, sizeof(cal_err) - 1, 100);
  }

  // Start ADC with DMA
  HAL_StatusTypeDef adc_status = HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_input_buffer, ADC_BUFFER_SIZE);
  if (adc_status == HAL_OK)
  {
    uint8_t adc_ok[] = "ADC DMA started successfully\r\n";
    HAL_UART_Transmit(&huart2, adc_ok, sizeof(adc_ok) - 1, 100);
  }
  else
  {
    uint8_t adc_err[] = "ADC DMA start FAILED!\r\n";
    HAL_UART_Transmit(&huart2, adc_err, sizeof(adc_err) - 1, 100);
  }

  // Send initial ready message to indicate we can receive COBS packets
  cobs_send_ready();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Send spectrum data on UART3 when ready
    if (spectrum_ready_to_send && uart3_tx_complete) {
      spectrum_ready_to_send = 0;
      uart3_tx_complete = 0;
      HAL_UART_Transmit_DMA(&huart3, spectrum_tx_buffer, spectrum_packet_len);

      // Debug: log spectrum transmission with first bytes
      char dbg[100];
      int dbg_len = sprintf(dbg, "TX spectrum: %d bytes [%02X %02X %02X %02X %02X...]\r\n",
                            spectrum_packet_len,
                            spectrum_tx_buffer[0], spectrum_tx_buffer[1],
                            spectrum_tx_buffer[2], spectrum_tx_buffer[3],
                            spectrum_tx_buffer[4]);
      HAL_UART_Transmit(&huart2, (uint8_t*)dbg, dbg_len, 100);
    }

    // Process incoming COBS packets from queue 
    if (cobs_packet_queue_has_packet(&cobs_rx_queue)) {
      uint8_t packet[COBS_MAX_PACKET_SIZE];
      uint16_t packet_len = cobs_packet_queue_pop(&cobs_rx_queue, packet, COBS_MAX_PACKET_SIZE);
      if (packet_len > 0) {
        // Check packet type
        uint8_t packet_type = packet[0];

        if (packet_type == 0x02) {
          // Packet type 0x02: UART input data 
          // Format: [0x02] [sample0_low] [sample0_high] [sample1_low] [sample1_high] ...
          uint16_t sample_count = (packet_len - 1) / 2;  // Calculate from packet length

          // Copy samples to UART input buffer
          for (uint16_t i = 0; i < sample_count; i++) {
            uint16_t idx = 1 + i * 2;
            uart_input_buffer[uart_input_write_idx] = packet[idx] | (packet[idx + 1] << 8);
            uart_input_write_idx++;

            // Check if buffer is half full 
            if (uart_input_write_idx == ADC_BUFFER_SIZE / 2) {
              uart_buffer_half_full_flag = 1;
            }
            // Check if buffer is completely full
            else if (uart_input_write_idx >= ADC_BUFFER_SIZE) {
              uart_buffer_full_flag = 1;
              uart_input_write_idx = 0;  // Reset for next batch
            }
          }

          char dbg[64];
          int dbg_len = sprintf(dbg, "UART data: %d samples, idx=%d\r\n", sample_count, uart_input_write_idx);
          HAL_UART_Transmit(&huart2, (uint8_t*)dbg, dbg_len, 100);
        } else {
          // Unknown packet type
          char dbg[64];
          int dbg_len = sprintf(dbg, "Unknown packet type 0x%02X, %d bytes\r\n", packet_type, packet_len);
          HAL_UART_Transmit(&huart2, (uint8_t*)dbg, dbg_len, 100);
        }
      }
      // Send ready/ACK on UART1
      cobs_send_ready();
    }

    // Check if ADC buffer is half full 
    if (input_adc && buffer_half_full_flag)
    {
      buffer_half_full_flag = 0;
      process_fft(adc_input_buffer, 0);
    }

    // Check if ADC buffer is full 
    if (input_adc && buffer_full_flag)
    {
      buffer_full_flag = 0;
      process_fft(adc_input_buffer, ADC_BUFFER_SIZE / 2);
    }

    // Check if UART buffer is half full
    // Only process if buffer has been initialized (filled at least once before)
    if (input_uart && uart_buffer_half_full_flag)
    {
      uart_buffer_half_full_flag = 0;
      if (uart_buffer_initialized) {
        // Process FFT: old data from second half [1024..2047], new data from first half [0..1023]
        HAL_UART_Transmit(&huart2, (uint8_t*)"FFT half\r\n", 10, 100);
        process_fft(uart_input_buffer, 0);
      }
    }

    // Check if UART buffer is full
    if (input_uart && uart_buffer_full_flag)
    {
      uart_buffer_full_flag = 0;
      // Mark buffer as initialized after first complete fill
      uart_buffer_initialized = 1;
      // Process FFT: old data from first half [0..1023], new data from second half [1024..2047]
      HAL_UART_Transmit(&huart2, (uint8_t*)"FFT full\r\n", 10, 100);
      process_fft(uart_input_buffer, ADC_BUFFER_SIZE / 2);
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
