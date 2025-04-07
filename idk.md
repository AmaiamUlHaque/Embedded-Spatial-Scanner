# Embedded-Spatial-Scanner

An embedded system that uses a time-of-flight sensor and a rotary mechanism to perform 360° spatial scans. It captures distance measurements within a plane, stores the data onboard, and transmits it to a PC or web app for 3D reconstruction and visualization. #EmbeddedSystems #LiDAR #3DReconstruction

## Device Overview

The device is an embedded spatial mapping system that uses a Time-of-Flight (ToF) sensor (VL53L1X) mounted on a stepper motor to perform 360° scans of its surroundings. It measures distances in the y-z plane at fixed angular intervals and combines these with manual displacements along the x-axis to generate 3D spatial data. The data is transmitted to a PC via UART and visualized using MATLAB.

### Features 

- **Bus Speed**: 16MHz
- **Operating voltage**: 2.97V - 3.63V
  - MCU: 2.97V - 3.63V (regulated from 5V PC's USB supply)
  - ToF Sensor: 2.8V (regulated from 3.3V MCU supply)
  - Stepper Motor: 5V (from MCU supply)
- **Cost**: $94.26 CAD total
- **Serial communication**:
  - I²C: ToF to MCU (Address: 0x29, Speed: 100 kbps)
  - UART: MCU to PC (Baud: 115200 bps, CSV format)
- **Programming**:
  - C: Embedded Firmware (Texas Instruments Code Composer Keil Studio)
  - MATLAB: PC Visualization and coordinate conversion
- **Memory**: 256KB onboard RAM for measurement buffers

### System Description

- **Signal Acquisition**: VL53L1X emits 940 nm laser pulses and measures time for reflected photons using SPAD array
- **Distance Calculation**: $d = \frac{1}{2} \bigtriangleup t \times c$
- **Coordinate Conversion**: Polar to Cartesian conversion with precomputed lookup tables
- **Data Flow**: ToF Sensor → I²C → MCU → UART → PC → MATLAB → 3D Visualization

### Block Diagram

<img src="./media/image1.png" style="width:6.90625in;height:1.20638in" />

## Hardware Specifications

| Component | Specifications |
|-----------|----------------|
| **Microcontroller MSP-EXP432E401Y** | ARM Cortex-M4, 16MHz<br>Pins: PB2-3 (ToF), PH0-3 (Stepper), PF4/PN1/PN0 (LEDs), PJ0 (Button) |
| **VL53L1X ToF Sensor** | Long mode (3.6m range), I²C communication<br>Pins: VIN, GND, SDA, SCL |
| **Stepper Motor 28BYJ-48 + ULN2003** | 2048 steps/revolution, 0.175°/step, 1ms/step<br>Pins: +, -, IN1, IN2, IN3, IN4 |
| **Communication** | I²C: 100 kbps (ToF→MCU)<br>UART: 115200 bps (MCU→PC) |

## Technical Implementation

### Distance Measurement & Coordinate Conversion

The system converts raw ToF distance measurements (d) and stepper motor angles (θ) into Cartesian coordinates:

**Formulas:**
- *θ* = *θ*<sub>*step*</sub> ⋅ (*i* − 1)
- *y* = *d* ⋅ sin(*θ*)
- *z* = *d* ⋅ cos(*θ*) 
- *x* = *j* ⋅ △*x*

**Where:**
- *d* = distance measured by ToF sensor (mm)
- *θ*<sub>*step*</sub> = 2.8125° (stepper motor angle per step)
- *i* = current measurement index (1-128)
- *j* = current revolution index
- △*x* = 300mm (displacement increment between revolutions)

**Example Calculation:**
For 55th measurement of 3rd revolution with d=172mm:
- *θ* = 2.8125(55−1) = 151.875°
- *y* = 172⋅sin(151.875) = 151.58mm
- *z* = 172⋅cos(151.875) = 81.28mm  
- *x* = 3⋅300 = 900mm
- Result: (900mm, 151.58mm, 81.28mm)

### Data Processing Pipeline

1. Pushbutton pressed → motor rotates and pauses for measurements
2. ToF sensor measures distance → I²C → MCU → UART → PC
3. MATLAB parses UART data and validates
4. Polar to Cartesian coordinate conversion
5. Real-time 3D plotting
6. Manual displacement by 300mm after each revolution
7. Process repeats for 10 revolutions

### Visualization (MATLAB)

```matlab
% UART Setup
s = serialport('COM4', 115200, 'Timeout', 30);
configureTerminator(s, "CR/LF");

% Coordinate Conversion
angle = 2.8125 * (i-1);
y_coord = distance * sind(angle);
z_coord = distance * cosd(angle); 
x_coord = j * 300;

% Real-time Plotting
plot3([x_prev, x_coord], [y_prev, y_coord], [z_prev, z_coord], '-b');
plot3(x_coord, y_coord, z_coord, 'ro', 'MarkerSize', 6);
drawnow;