#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>
#include <string.h>
#include <time.h>
#include "hardware/timer.h"
#include "hardware/pio.h"
#include "uart_rx.pio.h"
#include "uart_tx.pio.h"
#include "quadrature_encoder.pio.h"
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64.h>
#include <geometry_msgs/msg/vector3_stamped.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include "pico_uart_transports.h"
#include "pid_controller.h"

#define PIO_RX_PIN 1
#define PIO_TX_PIN 0
#define LED_PIN 25
#define COUNTS_PER_REV 11500.0
#define START_FRAME 0xABCD
#define SERIAL_BAUD 115200
#define RIGHT_SLOPE 0.5495
#define LEFT_SLOPE 0.5375
#define RIGHT_MIN_SPEED 46
#define LEFT_MIN_SPEED 49

#define WHEELS_Y_DISTANCE 0.53
#define WHEELS_CIRCUMFERENCE 0.5432
#define MAX_RPM 400

#define KP 0.5
#define KI 5.0
#define KD 0.0
#define DT 0.02

PID pid_control_left, pid_control_right, *pid_ctrl_ptr_left, *pid_ctrl_ptr_right;

// Publishers
rcl_publisher_t publisher, batteryPublisher, speedPublisher, odomPublisher;

// Subscribers
rcl_subscription_t cmd_subscriber;

// Messages
std_msgs__msg__Int32 msg;
std_msgs__msg__Float64 batteryMsg;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist twist_msg;
geometry_msgs__msg__Twist wheel_speed_msg;

// odom data
double x_pos, y_pos, heading = 0.0;
double leftReqRPM, rightReqRPM = 0.0;
double currentRPM_R, currentRPM_L = 0.0;
unsigned long prev_odom_update, prev_cmd_time = 0;
int new_value1, delta1, old_value1, new_value2, delta2, old_value2 = 0;

// HoverSerial
uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
unsigned char *p;       // Pointer declaration for the new received data
unsigned char incomingByte, incomingBytePrev;

bool sleep = true;

// PIO state machines
const uint smEnc0 = 0;
const uint smEnc1 = 1;
const uint smTX = 0;
const uint smRX = 1;
PIO pioEnc = pio0;
PIO pioTX = pio1;
PIO pioRX = pio1;

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
            // printf("Non-valid data skipped");
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
    float average_rps_x;
    float average_rps_a;

    // convert average revolutions per minute to revolutions per second
    average_rps_x = ((float)(rpm1 + rpm2) / 2) / 60.0;           // RPM
    current_vel.linear_x = average_rps_x * WHEELS_CIRCUMFERENCE; // m/s

    current_vel.linear_y = 0;

    average_rps_a = ((float)(-rpm1 + rpm2) / 2) / 60.0;
    current_vel.angular_z = (average_rps_a * WHEELS_CIRCUMFERENCE) / (WHEELS_Y_DISTANCE / 2.0); //  rad/s
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

    odom_msg.pose.covariance[0] = 0.1;
    odom_msg.pose.covariance[7] = 0.1;
    odom_msg.pose.covariance[35] = 0.1;

    // linear speed from encoders
    odom_msg.twist.twist.linear.x = current_vel.linear_x;
    odom_msg.twist.twist.linear.y = current_vel.linear_y;
    odom_msg.twist.twist.linear.z = 0.0;

    // angular speed from encoders
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = current_vel.angular_z;

    odom_msg.twist.covariance[0] = 0.005;
    odom_msg.twist.covariance[7] = 0.005;
    odom_msg.twist.covariance[35] = 0.005;
}

struct timespec ts;
extern int clock_gettime(clockid_t unused, struct timespec *tp);

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    new_value1 = quadrature_encoder_get_count(pioEnc, smEnc0);
    new_value2 = quadrature_encoder_get_count(pioEnc, smEnc1);
    delta1 = new_value1 - old_value1;
    delta2 = new_value2 - old_value2;
    old_value1 = new_value1;
    old_value2 = new_value2;

    unsigned long now = clock();

    double vel_dt = (now - prev_odom_update) / 1000.0;         // seconds
    double dtm = (double)(now - prev_odom_update) / (60000.0); // minutes

    prev_odom_update = now;

    currentRPM_R = (-delta1 / COUNTS_PER_REV) / dtm;
    currentRPM_L = (-delta2 / COUNTS_PER_REV) / dtm;

    wheel_speed_msg.linear.x = currentRPM_L;
    wheel_speed_msg.angular.x = currentRPM_R;

    kinematicsUpdate(currentRPM_L, currentRPM_R);
    odomUpdate(vel_dt);

    clock_gettime(CLOCK_REALTIME, &ts);
    odom_msg.header.stamp.sec = ts.tv_sec;
    odom_msg.header.stamp.nanosec = ts.tv_nsec;
    odom_msg.header.frame_id.data = "base_footprint";
    odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
    // odom_msg.header.child_frame_id.data = "base_link";
    // odom_msg.header.child_frame_id.size = strlen(odom_msg.header.child_frame_id.data);

    rcl_ret_t odom = rcl_publish(&odomPublisher, &odom_msg, NULL);
    rcl_ret_t wheel_speed = rcl_publish(&speedPublisher, &wheel_speed_msg, NULL);

    msg.data = currentRPM_L;
    rcl_ret_t test = rcl_publish(&publisher, &msg, NULL);

    batteryMsg.data = Feedback.batVoltage / 100.0;
    rcl_ret_t bat = rcl_publish(&batteryPublisher, &batteryMsg, NULL);
}

void calculateRPM()
{
    float tangential_vel = twist_msg.angular.z * (WHEELS_Y_DISTANCE / 2.0);

    // convert m/s to m/min
    float linear_vel_x_mins = twist_msg.linear.x * 60.0;

    // convert rad/s to rad/min
    float tangential_vel_mins = tangential_vel * 60.0;

    float x_rpm = linear_vel_x_mins / WHEELS_CIRCUMFERENCE;
    float tan_rpm = tangential_vel_mins / WHEELS_CIRCUMFERENCE;

    // calculate for the target motor RPM and direction
    leftReqRPM = x_rpm - tan_rpm;
    rightReqRPM = x_rpm + tan_rpm;

    // leftReqRPM = constrain(leftReqRPM, -MAX_RPM, MAX_RPM);
    // rightReqRPM = constrain(rightReqRPM, -MAX_RPM, MAX_RPM);

    wheel_speed_msg.linear.y = leftReqRPM;
    wheel_speed_msg.angular.y = rightReqRPM;
}

void twistCallback(const void *msgin)
{
    gpio_put(LED_PIN, !gpio_get(LED_PIN));
    prev_cmd_time = clock();
    sleep = false;
    // msg.data++;
}

int getSign(int number)
{
    if (number > 0)
        return 1; // Positive number
    else if (number < 0)
        return -1; // Negative number
    else
        return 0; // Zero
}

int main()
{
    stdio_init_all();

    PID_Coefficents(&pid_control_right, KP, KI, KD, DT, 1000);
    PID_Coefficents(&pid_control_left, KP, KI, KD, DT, 1000);

    pid_ctrl_ptr_right = &pid_control_right;
    pid_ctrl_ptr_left = &pid_control_left;

    uint offset = pio_add_program(pioEnc, &quadrature_encoder_program);

    // Base pin to connect the A phase of the encoder.
    // The B phase must be connected to the next pin
    quadrature_encoder_program_init(pioEnc, smEnc0, offset, 16, 0);
    quadrature_encoder_program_init(pioEnc, smEnc1, offset, 14, 0);

    uint offsetTX = pio_add_program(pioTX, &uart_tx_program);
    uart_tx_program_init(pioTX, smTX, offsetTX, PIO_TX_PIN, SERIAL_BAUD);

    uint offsetRX = pio_add_program(pioRX, &uart_rx_program);
    uart_rx_program_init(pioRX, smRX, offsetRX, PIO_RX_PIN, SERIAL_BAUD);

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
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
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

    rclc_subscription_init_default(
        &cmd_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 2, &allocator);

    rclc_executor_add_subscription(
        &executor,
        &cmd_subscriber,
        &twist_msg,
        &twistCallback,
        ON_NEW_DATA);

    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);

    batteryMsg.data = 0;
    msg.data = 0;
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        Receive();
        currentTime = clock();
        absolute_time_t curr_time = get_absolute_time();

        // brake if there's no command received
        if (((clock() - prev_cmd_time) >= 500))
            sleep = true;

        if (clock() - lastSendTime >= 20) // 50 hz
        {
            if (sleep)
            {
                pid_ctrl_ptr_right->setpoint = 0;
                pid_ctrl_ptr_left->setpoint = 0;
                pid_ctrl_ptr_right->measured = currentRPM_R;
                pid_ctrl_ptr_left->measured = currentRPM_L;

                PID_Compute(curr_time, pid_ctrl_ptr_left);
                PID_Compute(curr_time, pid_ctrl_ptr_right);

                twist_msg.linear.x = 0.0;
                twist_msg.linear.y = 0.0;
                twist_msg.angular.z = 0.0;

                Send(0, 0);
                gpio_put(LED_PIN, 1);
            }
            else
            {
                // convert twist to rpm
                calculateRPM();

                pid_ctrl_ptr_right->setpoint = rightReqRPM;
                pid_ctrl_ptr_left->setpoint = leftReqRPM;
                pid_ctrl_ptr_right->measured = currentRPM_R;
                pid_ctrl_ptr_left->measured = currentRPM_L;

                int leftPIDCmd = PID_Compute(curr_time, pid_ctrl_ptr_left);
                int rightPIDCmd = PID_Compute(curr_time, pid_ctrl_ptr_right);

                int leftCmd = (leftPIDCmd / LEFT_SLOPE) + (getSign(leftReqRPM) * RIGHT_MIN_SPEED);
                int rightCmd = (rightPIDCmd / RIGHT_SLOPE) + (getSign(rightReqRPM) * LEFT_MIN_SPEED);

                wheel_speed_msg.linear.z = leftCmd;
                wheel_speed_msg.angular.z = rightCmd;

                Send(leftCmd, rightCmd);

                lastSendTime = currentTime;
            }
        }
    }
}
