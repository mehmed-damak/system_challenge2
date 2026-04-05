# SafeShift
SafeShift is a system meant to be retrofitted on new and/or old industrial vehicles. The Goal of Safeshift is to provide assistance in preventative maintenance and in safety (and limited power tracking to where it applies). SafeShift is meant to be modular and be a plug and play module, that can work on any vehicle both new and old.

## Physical Installation
The main SafeShift board is to be placed in space inside the drivers cabin. and the sensors respective connectors will be plugged into them. The sensors are to be bolted/fasted to their repective areas (defined below). _NOTE: not ALL sensors must be connected to use SafeShift_
<img width="1721" height="965" alt="image" src="https://github.com/user-attachments/assets/0dbe2eb0-9e8e-4bf4-827d-cef882a1213e" />

<img width="1730" height="1142" alt="image" src="https://github.com/user-attachments/assets/1c289df4-5d90-434b-9a91-b1259d5d4591" />

## Connectivity and Dashboard
The Preventative Maintenance Part of the systam connects to a dashboard through wifi or bluetooth, and provides sensor reading, both past and present. A simple learning model from previous data shows the current status of the system based on the data gathered. <img width="1784" height="930" alt="image" src="https://github.com/user-attachments/assets/e17b7cbf-8435-4a08-8fa3-0644ea47b524" />

## Safety 
SafeShift uses a combination of sensors to ensure real-time safety monitoring and intelligent decision-making in industrial vehicles. It integrates JSN-SR04T (ultrasonic sensor) for proximity and obstacle detection, RCWL-0516 (microwave radar sensor) for motion detection, ADXL345 (MEMS accelerometer) for vibration and tilt monitoring, DS18B20 (digital temperature sensor) for thermal monitoring, WCS1500 (Hall-effect current sensor) for system current measurement, and A3144 (Hall-effect sensor) for position and magnetic detection. These sensors enable features such as blind spot detection, pedestrian awareness, and environment-adaptive responses in dynamic industrial settings. Together, this multi-sensor approach provides reliable, low-cost, and scalable safety coverage, significantly improving operational safety and system awareness.

## Preventative Maintenance
SafeShift tracks the following parameters in Industrial Vehicles: Vibration in motor pumps, temperature of motor-pumps. it uses a simple DS18B20 to track temperature sensors and an ADXL345 for vibrations. There are 4 of each applicable to this system.

## Energy Tracking 
If the vehicle is equipped with an electrical system, consumption is monitored via real-time current data shown on the dashboard through the current sensor.
Note: The voltage is to be set by the user following the vehicles specifications.

## Usage
Today at 8:20 AM
8:20 AM
Asma
SafeShift is designed for simple and intuitive usage, requiring the operator to download and launch the mobile application and connect to the system via WiFi or Bluetooth.

run app

Once connected, the dashboard provides real-time monitoring of vehicle status and safety conditions.

If proximity is detected, a buzzer alarm is activated. 
If a pedestrian is detected, a buzzer alarm is also triggered. 
If the seatbelt is not fastened, an LED indicator is activated. 
If the door is not properly closed, an LED indicator is turned on. 
If the load is too heavy, a pressure LED is activated.
