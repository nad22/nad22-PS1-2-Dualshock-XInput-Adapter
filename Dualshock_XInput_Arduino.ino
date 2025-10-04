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

const unsigned long POLL_INTERVAL = 20;

uint8_t ps2Data[21];
unsigned long lastPollTime = 0;
uint16_t lastButtons = 0xFFFF;
uint8_t rumbleLarge = 0;
uint8_t rumbleSmall = 0;

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
    
    // Transmit 8 bits (LSB first, as per PlayStation protocol)
    for (int i = 0; i < 8; i++) {
        // Set clock low
        digitalWrite(PS2_CLK, LOW);
        delayMicroseconds(2);
        
        // Transmit bit (LSB first)
        digitalWrite(PS2_CMD, (data & (1 << i)) ? HIGH : LOW);
        delayMicroseconds(2);
        
        // Set clock high and read response bit
        digitalWrite(PS2_CLK, HIGH);
        delayMicroseconds(2);
        
        // Read incoming bit from controller
        if (digitalRead(PS2_DAT)) {
            result |= (1 << i);
        }
        delayMicroseconds(2);
    }
    
    // Keep clock high (idle state)
    digitalWrite(PS2_CLK, HIGH);
    delayMicroseconds(10);
    
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
    // Start communication (attention signal)
    digitalWrite(PS2_ATT, LOW);
    delayMicroseconds(10);
    
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
    
    // End communication
    delayMicroseconds(10);
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
    
    // Perform startup rumble test (0.5 seconds)
    // Tests both motors to verify functionality
    rumbleSmall = 0x01;            // Enable small motor (right motor)
    rumbleLarge = 0xFF;            // Enable large motor at full intensity (left motor)
    for (int i = 0; i < 25; i++) { // 0.5 seconds = 25 × 20ms polling cycles
        pollController();
        delay(20);
    }
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
    unsigned long currentTime = millis();
    
    // Poll controller at 50Hz (every 20ms)
    if (currentTime - lastPollTime >= POLL_INTERVAL) {
        lastPollTime = currentTime;
        pollController();
        
        // Handle separate rumble motor control from XInput host
        uint8_t leftMotor = XInput.getRumbleLeft();   // Large motor (variable intensity 0-255)
        uint8_t rightMotor = XInput.getRumbleRight(); // Small motor (on/off only)
        
        // Map XInput rumble to PlayStation controller format
        rumbleSmall = (rightMotor > 0) ? 0x01 : 0x00;  // Small motor: digital on/off
        rumbleLarge = leftMotor;                        // Large motor: PWM value 0-255
        
        // Parse button states from PlayStation controller response
        uint16_t buttons = (ps2Data[4] << 8) | ps2Data[3];
        
        // Update XInput button states only when changes occur
        if (buttons != lastButtons) {
            lastButtons = buttons;
            
            // Map PlayStation face buttons to Xbox layout
            XInput.setButton(BUTTON_A, !(buttons & PS2_CROSS));      // Cross → A
            XInput.setButton(BUTTON_B, !(buttons & PS2_CIRCLE));     // Circle → B
            XInput.setButton(BUTTON_X, !(buttons & PS2_SQUARE));     // Square → X
            XInput.setButton(BUTTON_Y, !(buttons & PS2_TRIANGLE));   // Triangle → Y
            
            // Map shoulder buttons
            XInput.setButton(BUTTON_LB, !(buttons & PS2_L1));        // L1 → Left Bumper
            XInput.setButton(BUTTON_RB, !(buttons & PS2_R1));        // R1 → Right Bumper
            
            // Map analog stick clicks
            XInput.setButton(BUTTON_L3, !(buttons & PS2_L3));        // L3 → Left Stick
            XInput.setButton(BUTTON_R3, !(buttons & PS2_R3));        // R3 → Right Stick
            
            // Map menu buttons
            XInput.setButton(BUTTON_BACK, !(buttons & PS2_SELECT));  // Select → Back
            XInput.setButton(BUTTON_START, !(buttons & PS2_START));  // Start → Start
            
            // Map D-pad buttons
            XInput.setDpad(
                !(buttons & PS2_UP),     // D-pad Up
                !(buttons & PS2_DOWN),   // D-pad Down
                !(buttons & PS2_LEFT),   // D-pad Left
                !(buttons & PS2_RIGHT)   // D-pad Right
            );
            
            // Map shoulder triggers (L2/R2 as digital buttons → analog triggers)
            uint8_t lt = (buttons & PS2_L2) ? 0 : 255;    // L2 pressed = full trigger
            uint8_t rt = (buttons & PS2_R2) ? 0 : 255;    // R2 pressed = full trigger
            XInput.setTrigger(TRIGGER_LEFT, lt);
            XInput.setTrigger(TRIGGER_RIGHT, rt);
        }
        
        // Update analog stick positions (always update for smooth movement)
        int16_t lx = map(ps2Data[7], 0, 255, -32768, 32767);  // Left stick X
        int16_t ly = map(ps2Data[8], 0, 255, 32767, -32768);  // Left stick Y (inverted)
        int16_t rx = map(ps2Data[5], 0, 255, -32768, 32767);  // Right stick X
        int16_t ry = map(ps2Data[6], 0, 255, 32767, -32768);  // Right stick Y (inverted)
        
        XInput.setJoystick(JOY_LEFT, lx, ly);   // Set left analog stick
        XInput.setJoystick(JOY_RIGHT, rx, ry);  // Set right analog stick
    }
}
