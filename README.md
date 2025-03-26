# **BotaniCare** 🌱🔬  

## **Description**  
BotaniCare is a smart gardening IoT device that helps monitor and protect plants using a combination of sensors, AI, and automation. It collects environmental data and detects unwanted animal intrusions to ensure optimal plant growth and safety.  

## **Table of Contents**  
- [Features](#features)  
- [Hardware Components](#hardware-components)  
- [Software Components](#software-components)  
- [Installation](#installation)  
- [Usage](#usage)  
- [Configuration](#configuration)  
- [Contributing](#contributing)  
- [License](#license)  
- [Contact](#contact)  

## **Features**  
- 📊 Monitors **soil moisture, light presence, temperature, and air humidity necessary to plant growth**.  
- 🦊 Detects **animal presence** using a motion sensor.  
- 🎥 Runs an **animal detection model** on the Raspberry Pi.  
- 🚨 Activates a **buzzer** to scare away unwanted animals.  
- 🔄 Wireless communication between Arduino and Raspberry Pi via **Bluetooth**.
- ☁️ **Stores sensor data in Firebase** for real-time access and monitoring.

## **Hardware Components**  
- **Arduino Uno**  
- **Raspberry Pi 4**
- **Sensors:**  
  - Soil moisture sensor  
  - Light sensor  
  - Temperature & humidity sensor  
  - Motion detector (PIR sensor)  
- **USB Camera** (for animal detection)  
- **Buzzer** (for animal deterrence)  
- **Bluetooth module** (e.g., HC-05 for Arduino)  

## **Software Components**  
- **Arduino**: Handles sensor readings and Bluetooth communication.  
- **Raspberry Pi**: Runs the AI model for animal detection, controls the buzzer response and sends data to .  
- **Python** (for Raspberry Pi)  
- **OpenCV & TensorFlow** (for animal detection model)  
- **Bluetooth communication libraries** 

## **Installation**  

### **1. Setting up Arduino**  
1. Install the **Arduino IDE**.  
2. Upload the `sensors_script.ino` sketch to your Arduino with the dht files.  
3. Connect the sensors and Bluetooth module to the Arduino respecting the pins mentionned in the script.  

### **2. Setting up Raspberry Pi**  
1. Install Python and required libraries:  
   ```sh
   sudo apt update && sudo apt upgrade
   sudo apt install python3-pip
   pip3 install opencv-python ultralytics pybluez firebase-admin

