# Embedded-Spatial-Scanner

An embedded system that uses a time-of-flight sensor and a rotary mechanism to perform 360° spatial scans. It captures distance measurements within a plane, stores the data onboard, and transmits it to a PC for 3D reconstruction and visualization.

## Setup & Usage Instructions

### Hardware Setup

1. **ToF Sensor Setup**
   - Gently remove cover from over the sensor
   - Attach ToF sensor to stepper motor's rotary part
   - Ensure nothing blocks/covers the sensor or its path

2. **Stepper Motor Wiring** (Driver Board → MCU)
   - + to 5V
   - - to GND  
   - IN1 to PH0
   - IN2 to PH1
   - IN3 to PH2
   - IN4 to PH3

3. **ToF Sensor Wiring** (Sensor → MCU)
   - VIN to 3V3
   - GND to GND
   - SDA to PB3
   - SCL to PB2

4. **PC Connection**
   - USB-A to micro-USB

### Software Setup

1. **MCU Programming**
   - Open the Keil project file
   - Select MCU target and verify target/debugger settings
   - Translate and build target
   - Flash to the board
   - Press the 'restart' push button on MCU

2. **MATLAB Setup**
   - Open the MATLAB file
   - Run the MATLAB file

### Scanning Procedure

1. **Preparation**
   - Place device on suitable elevated object for steady scans
   - Ensure consistent position and direction throughout scans
   - Verify nothing is obscuring the ToF sensor or its path
   - Ensure MATLAB file is running
   - In MATLAB command window, look for: "Press Enter to start communication..."

2. **Start Scan**
   - Click in MATLAB's command window
   - Press Enter on PC
   - Immediately press pushbutton PJ0 on MCU
   - Verify LED D2 is on (scan in progress)
   - Note: LED D1 and D3 will flash during individual scans

3. **During Scan**
   - Wait for stepper motor to stop rotating (or LED D2 turns off)
   - Steadily move device forward by 300mm without changing orientation
   - Repeat scan steps for maximum of 10 revolutions

4. **View Results**
   - View MATLAB window with 3D scan: 'Final 3D Visualization of ToF Sensor Data'
   - For additional scans, repeat the procedure

## Technical Specifications

### Hardware Components

| Component | Specifications |
|-----------|----------------|
| **Microcontroller** | MSP-EXP432E401Y, ARM Cortex-M4, 16MHz |
| **ToF Sensor** | VL53L1X, 3.6m range, I²C address 0x29 |
| **Stepper Motor** | 28BYJ-48, 2048 steps/revolution, 0.175°/step |
| **Communication** | I²C: 100 kbps, UART: 115200 bps |

### Coordinate System

- **Y-axis**: Horizontal plane (left-right)
- **Z-axis**: Vertical plane (up-down) 
- **X-axis**: Displacement direction (forward movement)
- **Displacement increment**: 300mm per revolution

### Data Format

UART output format: `RangeStatus, Distance, SignalRate, AmbientRate, SpadNum`

## Performance Characteristics

### Timing Analysis
- ToF measurement time: ~33ms per reading
- Stepper motor step delay: 1.5ms per step
- Total revolution time: ~10.368 seconds
- Measurements per revolution: 128 samples

### Accuracy
- ToF sensor resolution: 1mm
- Maximum quantization error: ±0.5mm
- Angular resolution: 2.8125° per measurement

## Troubleshooting

### Common Issues

1. **No Sensor Data**
   - Verify I²C wiring (SDA to PB3, SCL to PB2)
   - Check sensor power (3V3 and GND)
   - Ensure sensor cover is removed

2. **Motor Not Rotating**
   - Verify motor driver wiring (IN1-IN4 to PH0-PH3)
   - Check power connections (5V and GND)
   - Confirm step delay settings

3. **MATLAB Communication Issues**
   - Verify COM port settings (typically COM4)
   - Check baud rate: 115200
   - Ensure proper line termination (CR/LF)

### LED Indicators
- **LED D2**: Scan in progress
- **LED D1**: Measurement activity
- **LED D3**: Additional status

## File Structure

Embedded-Spatial-Scanner/
├── firmware/ # MCU source code
│ ├── main.c # Main application
│ ├── motor_control.c # Stepper motor functions
│ └── sensor_interface.c # ToF sensor communication
├── matlab/
│ └── visualization.m # 3D plotting and data processing
├── docs/
│ ├── schematics/ # Circuit diagrams
│ └── datasheets/ # Component specifications
└── media/ # Images and diagrams


## Dependencies

### Software Requirements
- Texas Instruments Code Composer Studio or Keil MDK
- MATLAB R2023a+ with Instrument Control Toolbox
- Embedded C compiler for ARM Cortex-M4

### Hardware Requirements
- MSP-EXP432E401Y LaunchPad
- VL53L1X Time-of-Flight sensor
- 28BYJ-48 stepper motor with ULN2003 driver
- USB cable for programming and data transfer

## Applications

- 3D environment mapping
- Room dimension measurement
- Object detection and profiling
- Educational demonstrations of embedded systems
- Prototyping for autonomous navigation systems

## Limitations & Considerations

1. **Processing Constraints**
   - No hardware FPU on microcontroller
   - Trigonometric calculations performed in MATLAB
   - Limited onboard RAM (256KB)

2. **Physical Constraints**
   - Manual displacement required between scans
   - Maximum reliable scanning distance: 3.6m
   - Requires stable mounting platform

3. **Environmental Factors**
   - Performance may vary with lighting conditions
   - Reflective surfaces can affect distance measurements
   - Ambient infrared sources may interfere with readings

## Future Enhancements

- Automated displacement mechanism
- Wireless data transmission
- Real-time web visualization
- Multiple sensor integration for improved coverage
- Edge computing for on-device processing

## Support

For technical issues or questions about this project, please refer to the documentation in the `/docs` folder or check the component datasheets for detailed specifications.

---
*Project developed for embedded systems applications and 3D spatial mapping*