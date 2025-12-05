# Rotary Inverted Pendulum

This repository contains the source code and documentation for a **Rotary Inverted Pendulum**.  
The system balances a free-swinging pendulum on a rotary arm using a **NEMA 17 stepper motor** and a **high-resolution rotary encoder**.

The project features:
- A custom **Swing-Up algorithm (Energy Shaping)**
- A **PID / LQR Controller**
- Runs on an **Arduino Mega 2560**

---

## 🖼️ Figure 1: The complete Rotary Inverted Pendulum setup

![Pendulum Setup](https://github.com/user-attachments/assets/d7564fd5-6e1b-46b8-8208-ef1c2332b197)

---

# 🚀 Features

- **Auto-Swing Up** — Pumps energy into the pendulum to lift it to upright.
- **PID Stabilization** — Smooth balancing with PID control.
- **High-Speed Stepper Control** — Direct port manipulation + interrupts (40k+ steps/sec).
- **Safety Cutoff** — Driver disables if pendulum falls to prevent overheating.

---

# 🛠️ Hardware Requirements

### **Microcontroller**
- Arduino Mega 2560

### **Motor**
- NEMA 17 Stepper Motor

### **Driver**
- DRV8825 or A4988 Stepper Driver

### **Sensor**
- Optical Rotary Encoder (600 PPR or higher recommended)

### **Power**
- 12V 2A+ power supply

### **Misc**
- 2 × 4.7kΩ Pull-up resistors  
- 100µF capacitor (optional power filtering)

---

## 🖼️ Figure 2: Electronics and Wiring Close-Up

![Electronics](https://github.com/user-attachments/assets/1d9bc8fe-3394-4782-8f4b-28bb53a5a75b)

---

# 🔌 Wiring & Connections (Arduino Mega)

## **1. Stepper Motor Driver (A4988 / DRV8825)**

| Driver Pin | Arduino Pin | Description |
|-----------|-------------|-------------|
| STEP      | Pin 5       | Step pulse signal |
| DIR       | Pin 6       | Direction signal |
| SLEEP     | Pin 8       | Sleep mode control (Active High) |
| RESET     | Pin 9       | Reset control (Active High) |
| VMOT      | Ext. 12V +  | Motor power |
| GND       | Ext. 12V −  | Motor power ground (common ground required) |
| VDD       | 5V          | Logic power |
| GND       | GND         | Logic ground |
| 1A/1B     | —           | Stepper Motor Coil A |
| 2A/2B     | —           | Stepper Motor Coil B |

---

## **2. Rotary Encoder**

> Pins 2 and 3 correspond to **Port E** on the ATmega2560 for fast port access.

| Encoder Wire | Arduino Pin | Notes |
|--------------|-------------|-------|
| Phase A (Green) | Pin 2 | Interrupt pin — use 4.7kΩ pull-up |
| Phase B (White) | Pin 3 | Interrupt pin — use 4.7kΩ pull-up |
| VCC (Red)       | 5V    | Encoder power |
| GND (Black)     | GND   | Encoder ground |

---

# 🔍 Noise Reduction (Recommended)

To prevent jitter or false readings:

- Connect **4.7kΩ** resistor between Pin **2 → 5V**
- Connect **4.7kΩ** resistor between Pin **3 → 5V**

---

# ⚙️ Installation & Usage

### **1. Clone the Repo**
```bash
git clone https://github.com/TheAstrophile-KingK/Rotary_Inverted_Pendulum.git


