#include <time.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "uart_rx.pio.h"
#include "uart_tx.pio.h"
#include "quadrature_encoder.pio.h"
#include <string.h>

// Global variables
uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
unsigned char *p;       // Pointer declaration for the new received data
unsigned char incomingByte;
unsigned char incomingBytePrev;
#define START_FRAME 0xABCD // [-] Start frme definition for reliable serial communication

// state machine
const uint smEnc0 = 0;
const uint smEnc1 = 1;
const uint smTX = 0;
PIO pioEnc = pio0;
PIO pioTX = pio1;

PIO pioRX = pio1;
uint smRX = 1;

#define PIO_RX_PIN 1

typedef struct
{
    uint16_t start;
    int16_t steer;
    int16_t speed;
    uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct
{
    uint16_t start;
    int16_t cmd1;
    int16_t cmd2;
    int16_t speedR_meas;
    int16_t speedL_meas;
    int16_t batVoltage;
    int16_t boardTemp;
    uint16_t cmdLed;
    uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

clock_t clock()
{
    return (clock_t)time_us_64() / 1000;
}

void Send(int16_t uLeft, int16_t uRight)
{
    uint8_t high_byte;
    uint8_t low_byte;

    // Start Frame
    uart_tx_program_putc(pioTX, smTX, (char)0xCD);
    uart_tx_program_putc(pioTX, smTX, (char)0xAB);

    // Left Motor
    high_byte = (uint8_t)(uLeft >> 8);
    low_byte = (uint8_t)(uLeft & 0xFF);
    uart_tx_program_putc(pioTX, smTX, low_byte);
    uart_tx_program_putc(pioTX, smTX, high_byte);

    // Right Motor
    high_byte = (uint8_t)(uRight >> 8);
    low_byte = (uint8_t)(uRight & 0xFF);
    uart_tx_program_putc(pioTX, smTX, low_byte);
    uart_tx_program_putc(pioTX, smTX, high_byte);

    // Checksum
    uint16_t checksum = 0xABCD ^ uLeft ^ uRight;
    high_byte = (uint8_t)(checksum >> 8);
    low_byte = (uint8_t)(checksum & 0xFF);
    uart_tx_program_putc(pioTX, smTX, low_byte);
    uart_tx_program_putc(pioTX, smTX, high_byte);
}

void Receive()
{
    incomingByte = uart_rx_program_getc(pioRX, smRX);
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;

    if (bufStartFrame == START_FRAME)
    { // Initialize if new data is detected
        p = (unsigned char *)&NewFeedback;
        *p++ = incomingBytePrev;
        *p++ = incomingByte;
        idx = 2;
    }
    else if (idx >= 2 && idx < sizeof(SerialFeedback))
    { // Save the new received data
        *p++ = incomingByte;
        idx++;
    }

    if (idx == sizeof(SerialFeedback))
    {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum)
        {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            // Print data to built-in Serial
            printf("1: %d 2: %d 3: %d 4: %d 5: %d 6: %d 7: %d\n", Feedback.cmd1, Feedback.cmd2, Feedback.speedR_meas, Feedback.speedL_meas, Feedback.batVoltage, Feedback.boardTemp, Feedback.cmdLed);
        }
        else
        {
            printf("Non-valid data skipped");
        }
        idx = 0; // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

int main()
{

    stdio_init_all();

    int new_value1, delta1, old_value1 = 0;
    int new_value2, delta2, old_value2 = 0;

    uint offset = pio_add_program(pioEnc, &quadrature_encoder_program);

    // Base pin to connect the A phase of the encoder.
    // The B phase must be connected to the next pin
    quadrature_encoder_program_init(pioEnc, smEnc0, offset, 16, 0);
    quadrature_encoder_program_init(pioEnc, smEnc1, offset, 14, 0);

    const uint PIN_TX = 0;
    // This is the same as the default UART baud rate on Pico
    const uint SERIAL_BAUD = 115200;

    uint offset2 = pio_add_program(pioTX, &uart_tx_program);
    uart_tx_program_init(pioTX, smTX, offset2, PIN_TX, SERIAL_BAUD);

    Command.start = (uint16_t)0xABCD;
    Command.steer = (int16_t)0;
    Command.speed = (int16_t)150;
    Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

    unsigned long iTimeSend = 0;
    int iTest = 0;
    int iStep = 5;

    uint offsetRX = pio_add_program(pioRX, &uart_rx_program);
    uart_rx_program_init(pioRX, smRX, offsetRX, PIO_RX_PIN, SERIAL_BAUD);

    //    sleep_ms(1000);

    clock_t currentTime = clock();
    unsigned long lastSendTime = clock();

    while (1)
    {
        Receive();

        currentTime = clock();
        if (clock() - lastSendTime >= 100)
        {
            Send(0, 0);
            lastSendTime = currentTime;
        }
    }
}

/*
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

const uint LED_PIN = 25;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data++;
}

int main()
{
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);

    msg.data = 0;



    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
*/