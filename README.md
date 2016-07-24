# gpstest
This is a open source program that integrates CAN (J1939), GPS, WiFi, Gyroscope functions in one project.  
This project is based on Arduino Mega, however, currently I am using ESP8266 as wifi router to send message to cloud
server via TCP/IP protocol. The ESP8266 have its seperate firmware running and Arduino Mega communicate with ESP8266 via
Serial port No.2

Functions Used:
1. Real Time Clock (updated via GPS).
2. GPS
3. CAN bus (J1939)
4. Gyroscope (Accelerometer and Yaw, Pitch, Roll)
5. WiFi

