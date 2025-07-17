/**
 * RCTR Robot STM32 Firmware - Teleoperation Version
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

// IMU sensor (BNO055)
const int BNO055_ADDR = 0x28 << 1;

// UART communication with Raspberry Pi
Serial raspi_serial(USBTX, USBRX, 115200);

// Encoder variables
volatile int encoder1_count = 0;
volatile int encoder2_count = 0;

// Control variables
float motor1_speed = 0.0;  // -1.0 to 1.0
float motor2_speed = 0.0;  // -1.0 to 1.0
int servo_positions[4] = {90, 90, 90, 90};  // Servo angles in degrees

// IMU data
float roll = 0.0, pitch = 0.0, yaw = 0.0;

// Timing
Timer status_timer;
float last_status_time = 0;

/**
 * Encoder interrupt handlers
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
 * Initialize BNO055 IMU
 */
void init_BNO055() {
    char cmd[2];
    
    // Reset
    cmd[0] = 0x3F;  // SYS_TRIGGER register
    cmd[1] = 0x20;  // Reset
    i2c.write(BNO055_ADDR, cmd, 2);
    wait_ms(650);   // Wait for reset
    
    // Set to NDOF mode
    cmd[0] = 0x3D;  // OPR_MODE register
    cmd[1] = 0x0C;  // NDOF mode
    i2c.write(BNO055_ADDR, cmd, 2);
    wait_ms(20);
}

/**
 * Read Euler angles from IMU
 */
void read_IMU() {
    char cmd[1] = {0x1A};  // Euler angle data register
    char data[6];
    
    i2c.write(BNO055_ADDR, cmd, 1);
    i2c.read(BNO055_ADDR, data, 6);
    
    // Convert to degrees
    yaw = ((int16_t)(data[1] << 8) | data[0]) / 16.0;
    roll = ((int16_t)(data[3] << 8) | data[2]) / 16.0;
    pitch = ((int16_t)(data[5] << 8) | data[4]) / 16.0;
}

/**
 * Set servo position
 */
void set_servo(int channel, int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    int pulse_length = 1000 + (angle * 1000 / 180);
    int pwm_value = pulse_length * 4096 / 20000;
    
    char cmd[5];
    int reg_base = 0x06 + (channel * 4);
    
    cmd[0] = reg_base;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    i2c.write(PCA9685_ADDR, cmd, 3);
    
    cmd[0] = reg_base + 2;
    cmd[1] = pwm_value & 0xFF;
    cmd[2] = (pwm_value >> 8) & 0xFF;
    i2c.write(PCA9685_ADDR, cmd, 3);
}

/**
 * Set motor speed and direction
 */
void set_motor(int motor, float speed) {
    if (speed > 1.0) speed = 1.0;
    if (speed < -1.0) speed = -1.0;
    
    if (motor == 1) {
        motor1_dir = (speed >= 0) ? 1 : 0;
        motor1_pwm = fabs(speed);
        motor1_speed = speed;
    } else if (motor == 2) {
        motor2_dir = (speed >= 0) ? 1 : 0;
        motor2_pwm = fabs(speed);
        motor2_speed = speed;
    }
}

/**
 * Parse and execute commands from Raspberry Pi
 * Command format: "CMD:param1,param2,..."
 */
void parse_command(string cmd) {
    size_t colon_pos = cmd.find(':');
    if (colon_pos == string::npos) return;
    
    string command = cmd.substr(0, colon_pos);
    string params = cmd.substr(colon_pos + 1);
    
    if (command == "MOTOR") {
        // Direct motor control for teleoperation
        size_t comma_pos = params.find(',');
        if (comma_pos != string::npos) {
            float left_speed = stof(params.substr(0, comma_pos));
            float right_speed = stof(params.substr(comma_pos + 1));
            set_motor(1, left_speed);
            set_motor(2, right_speed);
        }
    }
    else if (command == "SERVO") {
        // Servo control for reconfiguration
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
    else if (command == "RECONFIG") {
        // Predefined reconfigurations
        if (params == "NEUTRAL") {
            for (int i = 0; i < 4; i++) set_servo(i, 90);
        } else if (params == "CLIMB_UP") {
            set_servo(0, 45); set_servo(1, 135);
            set_servo(2, 90); set_servo(3, 90);
        } else if (params == "CLIMB_DOWN") {
            set_servo(0, 135); set_servo(1, 45);
            set_servo(2, 90); set_servo(3, 90);
        } else if (params == "TURN_LEFT") {
            set_servo(0, 90); set_servo(1, 90);
            set_servo(2, 45); set_servo(3, 135);
        } else if (params == "TURN_RIGHT") {
            set_servo(0, 90); set_servo(1, 90);
            set_servo(2, 135); set_servo(3, 45);
        } else if (params == "BRIDGE") {
            set_servo(0, 60); set_servo(1, 120);
            set_servo(2, 90); set_servo(3, 90);
        }
    }
    else if (command == "STOP") {
        // Emergency stop
        set_motor(1, 0);
        set_motor(2, 0);
    }
    else if (command == "STATUS") {
        // Send telemetry data
        read_IMU();
        raspi_serial.printf("TELEMETRY:%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f\n", 
                           encoder1_count, encoder2_count,
                           motor1_speed, motor2_speed,
                           roll, pitch, yaw);
    }
}

/**
 * Send periodic telemetry
 */
void send_telemetry() {
    float current_time = status_timer.read();
    if (current_time - last_status_time > 0.1) {  // 10Hz
        read_IMU();
        raspi_serial.printf("TELEMETRY:%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f\n", 
                           encoder1_count, encoder2_count,
                           motor1_speed, motor2_speed,
                           roll, pitch, yaw);
        last_status_time = current_time;
    }
}

int main() {
    // Initialize hardware
    motor1_pwm.period_ms(20);
    motor2_pwm.period_ms(20);
    
    // Attach encoder interrupts
    encoder1_A.rise(&encoder1_A_handler);
    encoder1_A.fall(&encoder1_A_handler);
    encoder2_A.rise(&encoder2_A_handler);
    encoder2_A.fall(&encoder2_A_handler);
    
    // Initialize I2C devices
    init_PCA9685();
    init_BNO055();
    
    // Set initial servo positions
    for (int i = 0; i < 4; i++) {
        set_servo(i, 90);
    }
    
    // Start status timer
    status_timer.start();
    
    // Command buffer
    string cmd_buffer = "";
    
    raspi_serial.printf("RCTR STM32 Teleoperation Ready\n");
    
    while (true) {
        // Check for commands
        if (raspi_serial.readable()) {
            char c = raspi_serial.getc();
            if (c == '\n') {
                parse_command(cmd_buffer);
                cmd_buffer = "";
            } else {
                cmd_buffer += c;
            }
        }
        
        // Send periodic telemetry
        send_telemetry();
        
        wait_ms(1);
    }
}
