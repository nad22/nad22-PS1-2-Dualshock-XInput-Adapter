/*
 * PlayStation 1/2 Controller to XInput USB Adapter
 * 
 * Hardware Requirements:
 * - Arduino Pro Micro (ATmega32U4, 5V/16MHz)
 * - Logic Level Shifter (5V ↔ 3.3V)
 * - Step-Down Module (5V → 3.3V)
 * - PlayStation DualShock Controller
 * 
 * Power Configuration:
 * - Controller Logic: 3.3V (via step-down module)
 * - Rumble Motors: 5V (8V recommended for optimal performance)
 * 
 * Features:
 * - Full Xbox 360 Controller emulation via XInput
 * - Independent rumble motor control
 * - All buttons and analog sticks supported
 * - Custom bit-banging SPI for reliable communication
 * - MiSTer FPGA compatible
 */

#include <XInput.h>

// PlayStation Controller Pin Definitions (Manual SPI Implementation)
const int PS2_ATT = 10;  // Attention/Select (SS)
const int PS2_CMD = 16;  // Command (MOSI)
const int PS2_DAT = 14;  // Data (MISO)  
const int PS2_CLK = 15;  // Clock (SCK)
const int PS2_ACK = 9;   // Acknowledge (optional)

const uint8_t CMD_POLL = 0x42;
const uint8_t CMD_CONFIG_MODE = 0x43;
const uint8_t CMD_SET_MODE = 0x44;
const uint8_t CMD_VIBRATION = 0x4D;
const uint8_t CMD_EXIT_CONFIG = 0x43;

const unsigned long POLL_INTERVAL = 8;  // 125Hz for minimum latency (was 20ms/50Hz)

uint8_t ps2Data[21];
unsigned long lastPollTime = 0;
uint16_t lastButtons = 0xFFFF;
uint8_t rumbleLarge = 0;
uint8_t rumbleSmall = 0;

// Performance optimization: Track analog stick changes
static uint8_t lastAnalogLX = 128, lastAnalogLY = 128;
static uint8_t lastAnalogRX = 128, lastAnalogRY = 128;
static uint8_t pollCounter = 0;

const uint16_t PS2_SELECT   = 0x0001;
const uint16_t PS2_L3       = 0x0002;
const uint16_t PS2_R3       = 0x0004;
const uint16_t PS2_START    = 0x0008;
const uint16_t PS2_UP       = 0x0010;
const uint16_t PS2_RIGHT    = 0x0020;
const uint16_t PS2_DOWN     = 0x0040;
const uint16_t PS2_LEFT     = 0x0080;
const uint16_t PS2_L2       = 0x0100;
const uint16_t PS2_R2       = 0x0200;
const uint16_t PS2_L1       = 0x0400;
const uint16_t PS2_R1       = 0x0800;
const uint16_t PS2_TRIANGLE = 0x1000;
const uint16_t PS2_CIRCLE   = 0x2000;
const uint16_t PS2_CROSS    = 0x4000;
const uint16_t PS2_SQUARE   = 0x8000;

/**
 * Manual bit-banging SPI transfer for PlayStation controller communication
 * 
 * Uses software-controlled SPI instead of hardware SPI for better compatibility
 * between Arduino IDE and PlatformIO implementations.
 * 
 * @param data Byte to transmit to controller
 * @return Received byte from controller
 */
uint8_t spiTransfer(uint8_t data) {
    uint8_t result = 0;
    
    // Optimized timing: Reduced from 2µs to 1µs for faster communication
    for (uint8_t i = 0; i < 8; i++) {
        // Set clock low
        digitalWrite(PS2_CLK, LOW);
        delayMicroseconds(1);  // Reduced delay for speed
        
        // Transmit bit (LSB first)
        digitalWrite(PS2_CMD, (data & (1 << i)) ? HIGH : LOW);
        delayMicroseconds(1);  // Reduced delay for speed
        
        // Set clock high and read response bit
        digitalWrite(PS2_CLK, HIGH);
        delayMicroseconds(1);  // Reduced delay for speed
        
        // Read incoming bit from controller
        if (digitalRead(PS2_DAT)) {
            result |= (1 << i);
        }
        delayMicroseconds(1);  // Reduced delay for speed
    }
    
    // Keep clock high (idle state)
    digitalWrite(PS2_CLK, HIGH);
    delayMicroseconds(5);  // Reduced inter-byte delay
    
    return result;
}

/**
 * Enable analog mode and rumble functionality on PlayStation controller
 * 
 * Sends configuration commands to enable:
 * - Analog stick mode (for analog stick readings)
 * - Rumble motor functionality
 * - Locks the configuration to prevent accidental mode changes
 */
void enableAnalogMode() {
    // Enter configuration mode
    digitalWrite(PS2_ATT, LOW);
    delayMicroseconds(10);
    spiTransfer(0x01);               // Command start byte
    spiTransfer(CMD_CONFIG_MODE);    // Config mode command
    spiTransfer(0x00);
    spiTransfer(0x01);               // Enter config mode
    spiTransfer(0x00);
    spiTransfer(0x00);
    spiTransfer(0x00);
    spiTransfer(0x00);
    spiTransfer(0x00);
    digitalWrite(PS2_ATT, HIGH);
    delay(20);
    
    // Enable analog mode and lock settings
    digitalWrite(PS2_ATT, LOW);
    delayMicroseconds(10);
    spiTransfer(0x01);               // Command start byte
    spiTransfer(CMD_SET_MODE);       // Set mode and lock command
    spiTransfer(0x00);
    spiTransfer(0x01);               // Enable analog mode
    spiTransfer(0x03);               // Lock configuration settings
    spiTransfer(0x00);
    spiTransfer(0x00);
    spiTransfer(0x00);
    spiTransfer(0x00);
    digitalWrite(PS2_ATT, HIGH);
    delay(20);
    
    // Enable vibration/rumble mapping
    digitalWrite(PS2_ATT, LOW);
    delayMicroseconds(10);
    spiTransfer(0x01);               // Command start byte
    spiTransfer(CMD_VIBRATION);      // Vibration mapping command
    spiTransfer(0x00);
    spiTransfer(0x00);
    spiTransfer(0x01);               // Enable vibration mapping
    spiTransfer(0xFF);               // Motor 1 mapping
    spiTransfer(0xFF);               // Motor 2 mapping
    spiTransfer(0xFF);
    spiTransfer(0xFF);
    digitalWrite(PS2_ATT, HIGH);
    delay(20);
    
    // Exit configuration mode
    digitalWrite(PS2_ATT, LOW);
    delayMicroseconds(10);
    spiTransfer(0x01);               // Command start byte
    spiTransfer(CMD_EXIT_CONFIG);    // Exit config mode command
    spiTransfer(0x00);
    spiTransfer(0x00);
    spiTransfer(0x5A);
    spiTransfer(0x5A);
    spiTransfer(0x5A);
    spiTransfer(0x5A);
    spiTransfer(0x5A);
    digitalWrite(PS2_ATT, HIGH);
    delay(20);
}

/**
 * Poll PlayStation controller for button states and send rumble commands
 * 
 * Performs the main 9-byte communication sequence:
 * - Sends poll command and rumble motor values
 * - Receives button states and analog stick positions
 * - Updates global ps2Data array with controller state
 */
void pollController() {
    // Start communication (attention signal) - optimized timing
    digitalWrite(PS2_ATT, LOW);
    delayMicroseconds(5);  // Reduced from 10µs for faster polling
    
    // Exchange 9 bytes with controller
    ps2Data[0] = spiTransfer(0x01);        // Start byte
    ps2Data[1] = spiTransfer(CMD_POLL);    // Poll command (0x42)
    ps2Data[2] = spiTransfer(0x00);        // Reserved
    ps2Data[3] = spiTransfer(rumbleSmall); // Small motor (right motor on/off)
    ps2Data[4] = spiTransfer(rumbleLarge); // Large motor (left motor PWM 0-255)
    ps2Data[5] = spiTransfer(0x00);        // Digital buttons byte 1
    ps2Data[6] = spiTransfer(0x00);        // Digital buttons byte 2
    ps2Data[7] = spiTransfer(0x00);        // Right analog stick X
    ps2Data[8] = spiTransfer(0x00);        // Right analog stick Y
    
    // End communication - optimized timing
    delayMicroseconds(5);  // Reduced from 10µs for faster response
    digitalWrite(PS2_ATT, HIGH);
}

/**
 * Arduino setup function - initializes pins, XInput, and controller
 * 
 * Configures:
 * - PlayStation controller communication pins
 * - XInput library for Xbox 360 controller emulation
 * - Controller analog mode and rumble functionality
 * - Performs startup rumble test (0.5 seconds)
 */
void setup() {
    // Configure PlayStation controller communication pins
    pinMode(PS2_ATT, OUTPUT);        // Attention signal (chip select)
    pinMode(PS2_CMD, OUTPUT);        // Command data (MOSI)
    pinMode(PS2_DAT, INPUT_PULLUP);  // Controller data (MISO) with pull-up
    pinMode(PS2_CLK, OUTPUT);        // Clock signal
    pinMode(PS2_ACK, INPUT);         // Acknowledge signal (unused)
    
    // Set pins to idle state
    digitalWrite(PS2_ATT, HIGH);     // Attention inactive (high)
    digitalWrite(PS2_CMD, HIGH);     // Command line idle high
    digitalWrite(PS2_CLK, HIGH);     // Clock idle high
    
    // Initialize system components
    delay(500);                     // Allow power to stabilize
    XInput.begin();                 // Initialize XInput (Xbox 360 controller emulation)
    delay(100);                     // Wait for XInput initialization
    enableAnalogMode();             // Configure PlayStation controller for analog mode and rumble
    delay(500);                     // Allow configuration to complete
    
    // Perform startup rumble test - sequential motor test
    // First test right motor (small motor) for 200ms
    rumbleSmall = 0x01;            // Enable small motor (right motor)
    rumbleLarge = 0x00;            // Keep large motor off
    for (int i = 0; i < 25; i++) { // 200ms = 25 × 8ms polling cycles
        pollController();
        delay(8);
    }
    
    // Then test left motor (large motor) for 200ms  
    rumbleSmall = 0x00;            // Turn off small motor
    rumbleLarge = 0xFF;            // Enable large motor at full intensity (left motor)
    for (int i = 0; i < 25; i++) { // 200ms = 25 × 8ms polling cycles
        pollController();
        delay(8);
    }
    
    // Turn off both motors
    rumbleSmall = 0x00;            // Disable rumble motors
    rumbleLarge = 0x00;
}

/**
 * Main loop - polls PlayStation controller and updates XInput state
 * 
 * Runs at 50Hz (20ms intervals) to:
 * - Poll PlayStation controller for button/stick states
 * - Handle separate rumble motor control (left motor variable, right motor on/off)
 * - Update XInput button mappings and analog stick positions
 * - Map PlayStation controller data to Xbox 360 controller format
 */
void loop() {
    // Reduce millis() calls: only check every few iterations
    if (++pollCounter >= 4) {  // Check timing every 4th iteration
        pollCounter = 0;
        unsigned long currentTime = millis();
        
        // Poll controller at 125Hz (every 8ms) for minimum latency
        if (currentTime - lastPollTime >= POLL_INTERVAL) {
            lastPollTime = currentTime;
            
            // Get rumble values before polling (reduces XInput calls)
            uint8_t leftMotor = XInput.getRumbleLeft();
            uint8_t rightMotor = XInput.getRumbleRight();
            rumbleSmall = (rightMotor > 0) ? 0x01 : 0x00;
            rumbleLarge = leftMotor;
            
            pollController();
            
            // Parse button states from PlayStation controller response
            uint16_t buttons = (ps2Data[4] << 8) | ps2Data[3];
            
            // Update XInput button states only when changes occur
            if (buttons != lastButtons) {
                lastButtons = buttons;
                
                // Map PlayStation buttons to Xbox layout - optimized boolean logic
                XInput.setButton(BUTTON_A, !(buttons & PS2_CROSS));
                XInput.setButton(BUTTON_B, !(buttons & PS2_CIRCLE));
                XInput.setButton(BUTTON_X, !(buttons & PS2_SQUARE));
                XInput.setButton(BUTTON_Y, !(buttons & PS2_TRIANGLE));
                XInput.setButton(BUTTON_LB, !(buttons & PS2_L1));
                XInput.setButton(BUTTON_RB, !(buttons & PS2_R1));
                XInput.setButton(BUTTON_L3, !(buttons & PS2_L3));
                XInput.setButton(BUTTON_R3, !(buttons & PS2_R3));
                XInput.setButton(BUTTON_BACK, !(buttons & PS2_SELECT));
                XInput.setButton(BUTTON_START, !(buttons & PS2_START));
                
                // D-pad mapping
                XInput.setDpad(!(buttons & PS2_UP), !(buttons & PS2_DOWN), 
                             !(buttons & PS2_LEFT), !(buttons & PS2_RIGHT));
                
                // Shoulder triggers
                XInput.setTrigger(TRIGGER_LEFT, (buttons & PS2_L2) ? 0 : 255);
                XInput.setTrigger(TRIGGER_RIGHT, (buttons & PS2_R2) ? 0 : 255);
            }
            
            // Optimized analog stick processing: Only update when values change
            uint8_t analogLX = ps2Data[7], analogLY = ps2Data[8];
            uint8_t analogRX = ps2Data[5], analogRY = ps2Data[6];
            
            if (analogLX != lastAnalogLX || analogLY != lastAnalogLY) {
                lastAnalogLX = analogLX; lastAnalogLY = analogLY;
                // Fast mapping: ((value - 128) * 512) for -32768 to +32767 range
                int16_t lx = ((int16_t)analogLX - 128) << 8;
                int16_t ly = -((int16_t)analogLY - 128) << 8;  // Inverted Y
                XInput.setJoystick(JOY_LEFT, lx, ly);
            }
            
            if (analogRX != lastAnalogRX || analogRY != lastAnalogRY) {
                lastAnalogRX = analogRX; lastAnalogRY = analogRY;
                int16_t rx = ((int16_t)analogRX - 128) << 8;
                int16_t ry = -((int16_t)analogRY - 128) << 8;  // Inverted Y
                XInput.setJoystick(JOY_RIGHT, rx, ry);
            }
        }
    }
}
