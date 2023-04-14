#include <time.h>
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "uart_rx.pio.h"
#include "uart_tx.pio.h"
#include "quadrature_encoder.pio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float64.h>
#include <geometry_msgs/msg/vector3_stamped.h>
#include <geometry_msgs/msg/vector3.h>
#include <nav_msgs/msg/odometry.h>
#include <rmw_microros/rmw_microros.h>
#include "pico_uart_transports.h"
#include <math.h>

const uint LED_PIN = 25;

// counts per rev = 11500
const double counts_per_rev = 11500.0;

rcl_publisher_t publisher;
rcl_publisher_t batteryPublisher;
rcl_publisher_t speedPublisher;
rcl_publisher_t odomPublisher;

std_msgs__msg__Int32 msg;
std_msgs__msg__Float64 batteryMsg;
geometry_msgs__msg__Vector3Stamped speedMsg;
nav_msgs__msg__Odometry odom_msg;

// nav_msgs__msg__Odometry odom_msg_;
double x_pos = 0.0;
double y_pos = 0.0;
double heading = 0.0;

unsigned long prev_odom_update = 0;

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

int new_value1, delta1, old_value1 = 0;
int new_value2, delta2, old_value2 = 0;

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

typedef struct
{
    double linear_x;
    double linear_y;
    double angular_z;
} VelocityVector3;
VelocityVector3 current_vel;

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

            // Print data to serial
            // printf("1: %d 2: %d 3: %d 4: %d 5: %d 6: %d 7: %d\n", Feedback.cmd1, Feedback.cmd2, Feedback.speedR_meas, Feedback.speedL_meas, Feedback.batVoltage, Feedback.boardTemp, Feedback.cmdLed);
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

const void euler_to_quat(float roll, float pitch, float yaw, float *q)
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;
}

void kinematicsUpdate(float rpm1, float rpm2)
{
    // Kinematics::velocities vel;
    float wheel_circumference_ = 0.16;
    float wheels_y_distance_ = 0.32;

    float average_rps_x;
    float average_rps_a;

    // convert average revolutions per minute to revolutions per second
    average_rps_x = ((float)(rpm1 + rpm2) / 2) / 60.0;           // RPM
    current_vel.linear_x = average_rps_x * wheel_circumference_; // m/s

    current_vel.linear_y = 0;

    // convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(-rpm1 + rpm2) / 2) / 60.0;
    current_vel.angular_z = (average_rps_a * wheel_circumference_) / (wheels_y_distance_ / 2.0); //  rad/s
}

void odomUpdate(float vel_dt)
{
    float delta_heading = current_vel.angular_z * vel_dt; // radians
    float cos_h = cos(heading);
    float sin_h = sin(heading);
    float delta_x = (current_vel.linear_x * cos_h - current_vel.linear_y * sin_h) * vel_dt; // m
    float delta_y = (current_vel.linear_x * sin_h + current_vel.linear_y * cos_h) * vel_dt; // m

    // calculate current position of the robot
    x_pos += delta_x;
    y_pos += delta_y;
    heading += delta_heading;

    // calculate robot's heading in quaternion angle
    // ROS has a function to calculate yaw in quaternion angle
    float q[4];
    euler_to_quat(0, 0, heading, q);

    // robot's position in x,y, and z
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;

    // robot's heading in quaternion
    odom_msg.pose.pose.orientation.x = (double)q[1];
    odom_msg.pose.pose.orientation.y = (double)q[2];
    odom_msg.pose.pose.orientation.z = (double)q[3];
    odom_msg.pose.pose.orientation.w = (double)q[0];

    odom_msg.pose.covariance[0] = 0.001;
    odom_msg.pose.covariance[7] = 0.001;
    odom_msg.pose.covariance[35] = 0.001;

    // linear speed from encoders
    odom_msg.twist.twist.linear.x = current_vel.linear_x;
    odom_msg.twist.twist.linear.y = current_vel.linear_y;
    odom_msg.twist.twist.linear.z = 0.0;

    // angular speed from encoders
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = current_vel.angular_z;

    odom_msg.twist.covariance[0] = 0.0001;
    odom_msg.twist.covariance[7] = 0.0001;
    odom_msg.twist.covariance[35] = 0.0001;
}

struct timespec ts;
extern int clock_gettime(clockid_t unused, struct timespec *tp);

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    new_value1 = quadrature_encoder_get_count(pioEnc, smEnc0);
    delta1 = new_value1 - old_value1;
    old_value1 = new_value1;

    new_value2 = quadrature_encoder_get_count(pioEnc, smEnc1);
    delta2 = new_value2 - old_value2;
    old_value2 = new_value2;

    unsigned long now = clock();

    double vel_dt = (now - prev_odom_update) / 1000.0; // seconds
    double dtm = (double)(now - prev_odom_update) / (60000.0); // minutes

    prev_odom_update = now;
    
    
    double currentRPM_L = (delta1 / counts_per_rev) / dtm;
    double currentRPM_R = (delta2 / counts_per_rev) / dtm;

    kinematicsUpdate(currentRPM_L, currentRPM_R);
    odomUpdate(vel_dt);

    clock_gettime(CLOCK_REALTIME, &ts);
    odom_msg.header.stamp.sec = ts.tv_sec;
    odom_msg.header.stamp.nanosec = ts.tv_nsec;

    rcl_ret_t odom = rcl_publish(&odomPublisher, &odom_msg, NULL);

    // speedMsg.header.stamp.sec = ts.tv_sec;
    // speedMsg.header.stamp.nanosec = ts.tv_nsec;
    // speedMsg.vector.x = delta1;
    // speedMsg.vector.y = delta2;

    //  rcl_ret_t speed = rcl_publish(&speedPublisher, &speedMsg, NULL);
    msg.data = currentRPM_L;
    rcl_ret_t test = rcl_publish(&publisher, &msg, NULL);

    batteryMsg.data = Feedback.batVoltage / 100.0;
    rcl_ret_t bat = rcl_publish(&batteryPublisher, &batteryMsg, NULL);
}

int main()
{
    stdio_init_all();

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

    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_node_t batteryNode;

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

    rclc_publisher_init_default(
        &batteryPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        "hover_battery");

    rclc_publisher_init_default(
        &speedPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3Stamped),
        "hover_speed");

    rclc_publisher_init_default(
        &odomPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom_unfiltered");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);

    batteryMsg.data = 0;
    msg.data = 0;

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        Receive();
        currentTime = clock();
        if (clock() - lastSendTime >= 100)
        {
            Send(0, 0);
            lastSendTime = currentTime;
        }
    }
}