# AxisController_STM32F103C8T6
3 Axis XYZ with multiple button to manual control and UART
# STM32-Based 3-Axis CNC Controller Firmware


---

## ✨ Key Features

* **3-Axis Stepper Motor Control**: Smooth and precise control for X, Y, and Z axes using Timer PWM interrupts.
* **Dual Control Modes**: Seamlessly switch between manual jogging and automated G-code execution.
* **Manual Jogging**: Intuitive control using an analog joystick for X/Y axes and dedicated buttons for Z-axis movement.
* **G-Code Execution**: Run G-code files from your favorite sender software via a standard UART interface.
* **Real-time DRO**: Get live position feedback (Digital Read-Out) just like professional CNC systems (GRBL-compatible format).
* **On-the-fly Configuration**: Set and update preset work positions (A, B, C, D) using simple UART commands without needing to re-flash the firmware.
* **Modular Codebase**: Well-structured libraries for axis control (`Axis_v2`), G-code parsing, and UART communication make the code easy to understand and extend.

---

## 🛠️ Hardware Requirements

* **Microcontroller**: STM32F103C8T6 "Blue Pill" Board
* **Stepper Drivers**: 3x A4988, DRV8825, or similar stepper motor drivers.
* **Stepper Motors**: 3x NEMA 17 or similar stepper motors.
* **Input Devices**:
    * 1x Analog Joystick (2-axis) for X/Y jogging.
    * Push buttons for manual control (Z-up, Z-down, Left, Right, Go to A/B/C/D, Stop).
* **Power Supply**: A suitable power supply for your motors (e.g., 12V or 24V).
* **Connectivity**: A USB-to-Serial (FTDI) adapter for UART communication with a PC.

---

## 💾 Software and Setup

### 1. Required Software
* [**STM32CubeIDE**](https://www.st.com/en/development-tools/stm32cubeide.html)
* A G-code Sender application like [**UGS (Universal Gcode Sender)**](https://winder.github.io/ugs_website/) or [**ioSender**](https://github.com/grblHAL/ioSender).
* A serial terminal program like PuTTY or CoolTerm for sending `SET` commands.

### 2. Flashing the Firmware
1.  Clone this repository to your local machine.
2.  Open STM32CubeIDE and import the project (`File` > `Import...` > `General` > `Existing Projects into Workspace`).
3.  Connect your STM32 board via an ST-Link programmer.
4.  Build the project by clicking the "Hammer" icon.
5.  Flash the firmware to the board by clicking the "Play" (Run) icon.

---

## 🕹️ How to Use

The controller operates in two main states: **IDLE** and **BUSY**. All manual and G-code commands can only be issued when the machine is in the `IDLE` state.

### Manual Control (IDLE State)

When the machine is idle, you can use the connected hardware for manual control:
* **Joystick**: Move the joystick to jog the X and Y axes.
* **Z-UP / Z-DOWN Buttons**: Press to move the Z-axis up or down in small increments.
* **L / R Buttons**: Press to move the X-axis left or right in larger increments.
* **A, B, C, D Buttons**: Press to automatically move the machine to the corresponding preset position.

### G-Code and Command Control via UART

Connect to the STM32's UART pins using a serial adapter.

* **Baud Rate**: `9600`
* **Data Bits**: `8`
* **Parity**: `None`
* **Stop Bits**: `1`

#### Real-time Position Feedback (DRO)
The controller automatically sends a status report every 200ms, which can be displayed by your G-code sender.
* **Format**: `<State|WPos:X.XXX,Y.YYY,Z.ZZZ>`
* **Example (Idle)**: `<Idle|WPos:10.000,25.500,0.000>`
* **Example (Running)**: `<Run|WPos:15.123,30.789,-1.500>`

#### Supported Commands

| Command Type      | Command Syntax                        | Description                                                                                             |
| ----------------- | ------------------------------------- | ------------------------------------------------------------------------------------------------------- |
| **Set Position** | `SET <ID> [Coordinates]`              | Updates a preset position. `<ID>` must be A, B, C, or D. Coordinates are in **mm**.                          |
| **Rapid Move** | `G0 X<val> Y<val> Z<val>`             | Moves the machine at maximum speed to the target coordinates.                                          |
| **Linear Move** | `G1 X<val> Y<val> Z<val> F<rate>`     | Moves in a straight line at a specified feed rate (`F`). Feed rate is in **mm/minute**.                |

**`SET` Command Examples:**
* Set position `A` to X=10, Y=20.5, Z=-1.0:
    ```
    SET A X10 Y20.5 Z-1.0
    ```
* Update only the Z-coordinate of position `C`:
    ```
    SET C Z-5.0
    ```

**G-Code Example Workflow:**
1.  Connect with UGS. You will start seeing `<Idle|WPos:...>` messages.
2.  Send `G0 X10 Y10`.
3.  The machine moves, and the status report changes to `<Run|WPos:...>`.
4.  When the move is complete, the machine state returns to `Idle` and you can send the next command.

---
