/**
 * RCTR Robot STM32 Firmware
 * Handles low-level motor control, encoder reading, and servo control
 * Communication with Raspberry Pi via UART
 */

#include "mbed.h"
#include "PwmOut.h"
#include "InterruptIn.h"
#include "Serial.h"
#include <string>
#include <sstream>

// Pin Definitions - Update based on your exact wiring
// DC Motor 1 (Left Track)
PwmOut motor1_pwm(D3);      // PWM signal to L298N IN1
DigitalOut motor1_dir(D4);   // Direction control
InterruptIn encoder1_A(D5);  // Encoder channel A
InterruptIn encoder1_B(D6);  // Encoder channel B

// DC Motor 2 (Right Track)
PwmOut motor2_pwm(D9);      // PWM signal to L298N IN3
DigitalOut motor2_dir(D10);  // Direction control
InterruptIn encoder2_A(D11); // Encoder channel A
InterruptIn encoder2_B(D12); // Encoder channel B

// I2C for PCA9685 servo driver
I2C i2c(D14, D15);  // SDA, SCL
const int PCA9685_ADDR = 0x40 << 1;  // Default address

// UART communication with Raspberry Pi
Serial raspi_serial(USBTX, USBRX, 115200);

// Encoder variables
volatile int encoder1_count = 0;
volatile int encoder2_count = 0;
int encoder1_last_B = 0;
int encoder2_last_B = 0;

// Control variables
float motor1_speed = 0.0;  // -1.0 to 1.0
float motor2_speed = 0.0;  // -1.0 to 1.0
int servo_positions[4] = {90, 90, 90, 90};  // Servo angles in degrees

// PID variables for velocity control
float kp = 0.5, ki = 0.1, kd = 0.05;  // Tune these values
float motor1_integral = 0, motor2_integral = 0;
float motor1_last_error = 0, motor2_last_error = 0;
float motor1_target_velocity = 0, motor2_target_velocity = 0;

// Timing
Timer control_timer;
float last_control_time = 0;

/**
 * Encoder interrupt handlers
 * Quadrature decoding for accurate position/velocity measurement
 */
void encoder1_A_handler() {
    if (encoder1_A.read() == encoder1_B.read()) {
        encoder1_count++;
    } else {
        encoder1_count--;
    }
}

void encoder2_A_handler() {
    if (encoder2_A.read() != encoder2_B.read()) {
        encoder2_count++;
    } else {
        encoder2_count--;
    }
}

/**
 * Initialize PCA9685 servo driver
 */
void init_PCA9685() {
    char cmd[2];
    
    // Reset
    cmd[0] = 0x00;  // MODE1 register
    cmd[1] = 0x00;  // Normal mode
    i2c.write(PCA9685_ADDR, cmd, 2);
    
    // Set PWM frequency to 50Hz for servos
    cmd[0] = 0xFE;  // Prescale register
    cmd[1] = 121;   // 50Hz
    i2c.write(PCA9685_ADDR, cmd, 2);
    
    // Wake up
    cmd[0] = 0x00;
    cmd[1] = 0x20;  // Auto-increment
    i2c.write(PCA9685_ADDR, cmd, 2);
}

/**
 * Set servo position (0-180 degrees)
 * @param channel Servo channel (0-3 for 4 servos)
 * @param angle Angle in degrees (0-180)
 */
void set_servo(int channel, int angle) {
    // Limit angle
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Convert angle to PWM values (assuming 1ms-2ms pulse for 0-180 degrees)
    int pulse_length = 1000 + (angle * 1000 / 180);  // microseconds
    int pwm_value = pulse_length * 4096 / 20000;     // 20ms period
    
    char cmd[5];
    int reg_base = 0x06 + (channel * 4);
    
    // Set ON time (0)
    cmd[0] = reg_base;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    i2c.write(PCA9685_ADDR, cmd, 3);
    
    // Set OFF time
    cmd[0] = reg_base + 2;
    cmd[1] = pwm_value & 0xFF;
    cmd[2] = (pwm_value >> 8) & 0xFF;
    i2c.write(PCA9685_ADDR, cmd, 3);
}

/**
 * Set motor speed and direction
 * @param motor Motor number (1 or 2)
 * @param speed Speed from -1.0 to 1.0 (negative for reverse)
 */
void set_motor(int motor, float speed) {
    // Limit speed
    if (speed > 1.0) speed = 1.0;
    if (speed < -1.0) speed = -1.0;
    
    if (motor == 1) {
        motor1_dir = (speed >= 0) ? 1 : 0;
        motor1_pwm = fabs(speed);
    } else if (motor == 2) {
        motor2_dir = (speed >= 0) ? 1 : 0;
        motor2_pwm = fabs(speed);
    }
}

/**
 * Velocity PID control
 * Maintains desired velocity using encoder feedback
 */
void velocity_control() {
    float current_time = control_timer.read();
    float dt = current_time - last_control_time;
    
    if (dt > 0.01) {  // 100Hz control loop
        // Calculate current velocities (counts per second)
        float motor1_velocity = encoder1_count / dt;
        float motor2_velocity = encoder2_count / dt;
        
        // Reset encoder counts
        encoder1_count = 0;
        encoder2_count = 0;
        
        // PID for motor 1
        float error1 = motor1_target_velocity - motor1_velocity;
        motor1_integral += error1 * dt;
        float derivative1 = (error1 - motor1_last_error) / dt;
        float output1 = kp * error1 + ki * motor1_integral + kd * derivative1;
        motor1_last_error = error1;
        
        // PID for motor 2
        float error2 = motor2_target_velocity - motor2_velocity;
        motor2_integral += error2 * dt;
        float derivative2 = (error2 - motor2_last_error) / dt;
        float output2 = kp * error2 + ki * motor2_integral + kd * derivative2;
        motor2_last_error = error2;
        
        // Apply control outputs
        set_motor(1, motor1_speed + output1);
        set_motor(2, motor2_speed + output2);
        
        last_control_time = current_time;
    }
}

/**
 * Parse and execute commands from Raspberry Pi
 * Command format: "CMD:param1,param2,..."
 * Examples:
 * - "MOTOR:0.5,-0.3" - Set motor speeds
 * - "SERVO:0,90" - Set servo 0 to 90 degrees
 * - "PID:0.5,0.1,0.05" - Update PID gains
 * - "STOP:" - Emergency stop
 */
void parse_command(string cmd) {
    size_t colon_pos = cmd.find(':');
    if (colon_pos == string::npos) return;
    
    string command = cmd.substr(0, colon_pos);
    string params = cmd.substr(colon_pos + 1);
    
    if (command == "MOTOR") {
        // Parse motor speeds
        size_t comma_pos = params.find(',');
        if (comma_pos != string::npos) {
            motor1_speed = stof(params.substr(0, comma_pos));
            motor2_speed = stof(params.substr(comma_pos + 1));
            
            // Update target velocities (assuming max 1000 counts/sec at full speed)
            motor1_target_velocity = motor1_speed * 1000;
            motor2_target_velocity = motor2_speed * 1000;
        }
    }
    else if (command == "SERVO") {
        // Parse servo command
        size_t comma_pos = params.find(',');
        if (comma_pos != string::npos) {
            int channel = stoi(params.substr(0, comma_pos));
            int angle = stoi(params.substr(comma_pos + 1));
            if (channel >= 0 && channel < 4) {
                servo_positions[channel] = angle;
                set_servo(channel, angle);
            }
        }
    }
    else if (command == "PID") {
        // Update PID gains
        stringstream ss(params);
        char comma;
        ss >> kp >> comma >> ki >> comma >> kd;
    }
    else if (command == "STOP") {
        // Emergency stop
        motor1_speed = 0;
        motor2_speed = 0;
        motor1_target_velocity = 0;
        motor2_target_velocity = 0;
        set_motor(1, 0);
        set_motor(2, 0);
    }
    else if (command == "STATUS") {
        // Send status back to Raspberry Pi
        raspi_serial.printf("STATUS:%d,%d,%.2f,%.2f\n",
                           encoder1_count, encoder2_count,
                           motor1_speed, motor2_speed);
    }
}

int main() {
    // Initialize hardware
    motor1_pwm.period_ms(20);  // 50Hz PWM
    motor2_pwm.period_ms(20);
    
    // Attach encoder interrupts
    encoder1_A.rise(&encoder1_A_handler);
    encoder1_A.fall(&encoder1_A_handler);
    encoder2_A.rise(&encoder2_A_handler);
    encoder2_A.fall(&encoder2_A_handler);
    
    // Initialize servo driver
    init_PCA9685();
    
    // Set initial servo positions (neutral)
    for (int i = 0; i < 4; i++) {
        set_servo(i, 90);
    }
    
    // Start control timer
    control_timer.start();
    
    // Command buffer
    string cmd_buffer = "";
    
    raspi_serial.printf("RCTR STM32 Ready\n");
    
    while (true) {
        // Check for serial commands
        if (raspi_serial.readable()) {
            char c = raspi_serial.getc();
            if (c == '\n') {
                parse_command(cmd_buffer);
                cmd_buffer = "";
            } else {
                cmd_buffer += c;
            }
        }
        
        // Run velocity control
        velocity_control();
        
        // Small delay
        wait_ms(1);
    }
}
