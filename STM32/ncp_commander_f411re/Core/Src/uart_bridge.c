#include "uart_bridge.h"

#include "stm32f4xx_hal.h"
#include <string.h>

#define BUFFER_SIZE 128

typedef struct {
    uint8_t buffer[BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer;

static RingBuffer uart1_rx_buf = {0};
static RingBuffer uart2_rx_buf = {0};

static uint8_t rx1_byte;
static uint8_t rx2_byte;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

static void RingBuffer_Write(RingBuffer* rb, uint8_t data)
{
    uint16_t next = (rb->head + 1) % BUFFER_SIZE;
    if (next != rb->tail) {  // avoid overwrite
        rb->buffer[rb->head] = data;
        rb->head = next;
    }
}

static int RingBuffer_Read(RingBuffer* rb, uint8_t* data)
{
    if (rb->tail == rb->head) return 0; // empty
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % BUFFER_SIZE;
    return 1;
}

void UARTBridge_Init()
{
    HAL_UART_Receive_IT(&huart1, &rx1_byte, 1);
    HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        RingBuffer_Write(&uart1_rx_buf, rx1_byte);
        HAL_UART_Receive_IT(&huart1, &rx1_byte, 1);
    }
    else if (huart->Instance == USART2)
    {
        RingBuffer_Write(&uart2_rx_buf, rx2_byte);
        HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
    }
}

void UARTBridge_Process()
{
    uint8_t byte;

    while (RingBuffer_Read(&uart1_rx_buf, &byte)) {
        HAL_UART_Transmit(&huart2, &byte, 1, 10);
    }

    while (RingBuffer_Read(&uart2_rx_buf, &byte)) {
        HAL_UART_Transmit(&huart1, &byte, 1, 10);
    }
}
