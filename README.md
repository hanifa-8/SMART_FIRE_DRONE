# Smart Fire Drone

CNN Powered Autonomous Fire Detection and Suppression System.
BACKGROUND:The smart fire drone offers a transformative solution to the challenges of traditional firefighting methods. By autonomously responding to distress SOS alerts and swiftly navigating to the scene, it overcomes the delays caused by traffic congestion and navigation difficulties. Equipped with advanced technology, including computer vision algorithms, it rapidly detects fires and initiates firefighting measures, all without endangering human lives. In essence, the smart fire drone serves as a proactive guardian against the devastation caused by fires. Its rapid response, precise detection, autonomous operation, efficient suppression capabilities help minimize property damage, save lives, and protect the environment, making it a crucial asset in firefighting operations.

## Features
1. Autonomous Navigation: Utilizes Pixhawk 2.4.8 flight controller for precise movement and GPS data for location tracking.
2. Emergency SMS Alerts: Receives and processes SOS alerts via GSM SIM800L module.
3. Fire Detection: Equipped with a webcam and a Convolutional Neural Network (CNN) model running on Raspberry Pi for accurate fire detection.
4. 360-degree Scanning: Performs a comprehensive sweep by rotating in all four directions.
5. Fire Suppression: Deploys an ABC dry chemical via a servo-controlled mechanism upon fire detection.
6. Live Video Streaming: Shares real-time video feed with the fire department for remote situational awareness.

## Process

1. Receive SOS Alert: Upon receiving the SOS alert from any device, the drone extracts the coordinates from the SOS message using the GSM SIM800L module.
2. Navigate to Location: The Pixhawk 2.4.8 flight controller navigates the drone to the specified location using GPS data.
3. Activate Webcam: Upon arrival at the location, the drone activates its webcam and starts capturing video.
4. Fire Detection: The webcam feed is processed using a CNN model running on the Raspberry Pi to detect fire.
5. Live Feed: Simultaneously, the Raspberry Pi camera is activated to share a live feed with the fire department for situational awareness.
6. 360-degree Scan: The drone rotates in all four directions to ensure comprehensive scanning of the premises.
7. Advance to Fire: Upon detecting fire in any direction, the drone moves forward a predetermined distance.
8. Adjust Altitude: The drone then adjusts to reach its target altitude.
9. Activate Servo: Upon reaching the target altitude, the servo mechanism is activated.
10. Fire Suppression: The firefighting nozzle is engaged to discharge the ABC dry chemical for fire suppression.
11. Monitor Area: The drone continues to monitor the area and provide feedback until the fire is fully suppressed or additional assistance arrives.

## Hardware Components
- Pixhawk 2.4.8 Flight Controller
- Raspberry Pi 4B
- GSM SIM800L Module
- Webcam
- Raspberry Pi Camera
- Servo Motor
- ABC Dry Chemical Nozzle
- Hexa Frame
- Motors and Propellers
- Electronic Speed Controllers (ESC)
- Battery
- GPS Module
- Rx/Tx (Receiver/Transmitter)

## Software Components
- CNN Model for Fire Detection
- Raspberry Pi OS
- Flight Control Software for Pixhawk
- GSM Module Interface Code

