# PlayStation 1/2 Controller to XInput USB Adapter

A complete hardware and software solution to convert PlayStation DualShock controllers to XInput-compatible USB controllers for use with MiSTer FPGA, retro gaming systems, and modern PCs.

## Features

- **Full Xbox 360 Controller Emulation** via XInput protocol
- **Complete Button Mapping** - All PS2 buttons mapped to Xbox equivalents
- **Dual Analog Stick Support** - Both sticks with full range
- **Force Feedback/Rumble** - Independent control of both motors
- **MiSTer FPGA Compatible** - Perfect for retro gaming with rumble support
- **Arduino IDE Compatible** - Easy to build and customize

## Hardware Requirements

### Components
- **Arduino Pro Micro** (5V/16MHz with ATmega32U4)
- **Logic Level Shifter** (Bidirectional 5V ↔ 3.3V)
- **Step-Down Module** (5V → 3.3V regulator)
- **PlayStation Controller** (DualShock 1 or 2)
- **Breadboard/PCB** for connections
- **Jumper Wires**

### Power Supply Configuration
- **Controller Data Lines & VCC**: 3.3V (via step-down module)
- **Rumble Motors**: 5V (adequate performance)
- **Optimal Rumble Power**: 8V (requires additional power source for best motor performance)

### Pin Connections

```
PlayStation Controller → Arduino Pro Micro
=====================================
Pin 1 (DATA)        → Pin 14 (MISO) via Level Shifter
Pin 2 (CMD)         → Pin 16 (MOSI) via Level Shifter  
Pin 3 (VCC-rumble)  → 5V or better 8V for Rumble Motors
Pin 4 (GND)         → GND
Pin 5 (VCC)         → 3.3V from Step-Down Module
Pin 6 (ATT)         → Pin 10 (SS) via Level Shifter
Pin 7 (CLK)         → Pin 15 (SCK) via Level Shifter
Pin 8 (n/c)         → Not Connected
Pin 9 (ACK)         → Pin 9 via Level Shifter
```

## Software Setup

### Arduino IDE Configuration

1. **Install XInput AVR Boards Package**
   - File → Preferences → Additional Boards Manager URLs
   - Add: `https://raw.githubusercontent.com/dmadison/ArduinoXInput_Boards/master/package_dmadison_xinput_index.json`
   - Tools → Board → Boards Manager
   - Search "XInput" and install "XInput AVR Boards"

2. **Install XInput Library**
   - Sketch → Include Library → Manage Libraries
   - Search "XInput" by dmadison
   - Install "XInput by dmadison"

3. **Board Selection**
   - Tools → Board → XInput AVR Boards → SparkFun Pro Micro (16MHz)
   - Tools → Port → Select your COM port

### Compilation & Upload

1. Open `Dualshock_XInput_Arduino.ino` in Arduino IDE
2. Verify/Compile (Ctrl+R)
3. Upload (Ctrl+U)

## Button Mapping

| PlayStation Button | Xbox 360 Equivalent |
|-------------------|---------------------|
| Cross (✕)         | A Button            |
| Circle (○)        | B Button            |
| Square (□)        | X Button            |
| Triangle (△)      | Y Button            |
| L1                | Left Bumper (LB)    |
| R1                | Right Bumper (RB)   |
| L2                | Left Trigger (LT)   |
| R2                | Right Trigger (RT)  |
| L3                | Left Stick Click    |
| R3                | Right Stick Click   |
| Select            | Back Button         |
| Start             | Start Button        |
| D-Pad             | D-Pad               |
| Left Analog       | Left Stick          |
| Right Analog      | Right Stick         |

## Force Feedback (Rumble)

The adapter supports independent control of both PlayStation rumble motors:

- **Large Motor** (Left Motor): Variable intensity (0-255)
- **Small Motor** (Right Motor): On/Off control

### Rumble Power Notes
- **5V Supply**: Adequate performance for most games
- **8V Supply**: Optimal motor performance (requires additional power source)
- The controller will accept 5V rumble power but motors will be less intense

## Technical Implementation

### Communication Protocol
- **Interface**: Custom bit-banging SPI implementation
- **Clock Speed**: ~125kHz (software-controlled timing)
- **Data Format**: 9-byte PlayStation controller protocol
- **Polling Rate**: 50Hz (20ms intervals)

### Why Custom SPI?
The Arduino IDE's hardware SPI implementation differs from PlatformIO, causing communication issues with PlayStation controllers. This implementation uses manual bit-banging for reliable cross-platform compatibility.

## Compatibility

### Tested Systems
- ✅ **MiSTer FPGA** (Full functionality including rumble)
- ✅ **Windows PC** (Recognized as Xbox 360 Controller)
- ✅ **HTML5 Gamepad API** (Web browsers)
- ✅ **RetroArch** (All cores with XInput support)

### Supported Controllers
- PlayStation DualShock 1
- PlayStation DualShock 2
- Third-party PS1/PS2 controllers with standard protocol

## Troubleshooting

### Controller Not Detected
- Check all wiring connections
- Verify 3.3V power supply to controller
- Ensure level shifter is properly connected
- Try different PlayStation controller

### No Analog Mode
- Verify step-down module provides stable 3.3V
- Check DATA line connection (Pin 1)
- Ensure controller supports analog mode

### Rumble Not Working
- Verify 5V power to controller Pin 5
- Check rumble motor connections inside controller
- Test with different games/applications

### Buttons Not Responding
- Verify all data lines through level shifter
- Check ground connections
- Test controller on original PlayStation console

## Development

### Project Structure
```
Dualshock_XInput_Arduino/
├── Dualshock_XInput_Arduino.ino    # Main Arduino sketch
├── README.md                       # This documentation
└── ANLEITUNG.md                    # German setup guide
```

### Key Functions
- `enableAnalogMode()`: Configures controller for analog sticks and rumble
- `pollController()`: Reads controller state via custom SPI protocol  
- `spiTransfer()`: Manual bit-banging SPI implementation
- `loop()`: Main update cycle with XInput communication

## License

This project is open-source. Use and modify freely for personal and educational purposes.

## Credits

- **XInput Library**: [dmadison/ArduinoXInput](https://github.com/dmadison/ArduinoXInput)
- **PlayStation Protocol**: Based on community reverse-engineering efforts
- **Hardware Design**: Inspired by various DIY controller adapter projects

## Contributing

Contributions welcome! Please submit issues and pull requests for:
- Additional controller compatibility
- Hardware design improvements  
- Code optimizations
- Documentation enhancements

---

**Enjoy your PlayStation controllers on modern systems with full XInput compatibility!** 🎮