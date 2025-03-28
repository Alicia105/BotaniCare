# **BotaniCare** 🌱🔬  

## **Description**  
BotaniCare is a smart gardening IoT device that helps monitor and protect plants using a combination of sensors, AI, and automation. It collects environmental data and detects unwanted animal intrusions to ensure optimal plant growth and safety.  

## **Table of Contents**  
- [Features](#features)  
- [Hardware Components](#hardware-components)  
- [Software Components](#software-components)
- [Yolov8 model](#yolov8-model)
- [Setup](#setup)    
 

## **Features**  
- 📊 Monitors **soil moisture, light, temperature, and air humidity** necessary to plant growth.  
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
- **Bluetooth module** (HC-02 for Arduino)

![BotaniCare System Architecture](images/setup.jpg)

## **Software Components**  
- **Arduino**: Handles sensor readings and Bluetooth communication.  
- **Raspberry Pi**: Runs the AI model for animal detection, controls the buzzer response and sends data to .  
- **Python** (for Raspberry Pi and AI model training)  
- **OpenCV** (for animal detection model)  
- **Bluetooth communication libraries** 

![BotaniCare Software flowchart](images/chart.jpg)

## **Yolov8 model** 
- **Animals**: Currently trained to detect some of the most common animals in North american gardens : rats, rabbits and squirrels.  
- **Dataset**: Trained on a restraint dataset of 208 images. Created with Roboflow.
![Dataset repartition](images/dataset.jpg)
- **Results**: We obtained the following results on the training :
![Training confusion matrix](images/confusionMatrix.png)
![Training metrics](images/metrics.png)
![Exemple of prediction](images/rabbit.jpg)


## **Setup**
![Full setup](images/plantWithSensors.jpg)
![Plant setup](images/plant.jpg)
![Sensors setup](images/sensors.jpg)
![Raspberry and camera setup](images/raspberry.jpg)

