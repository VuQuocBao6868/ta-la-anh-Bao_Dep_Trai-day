import os
# Set the path for the QT plugin (only necessary in your environment)
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/home/bao/my_env/lib/python3.12/site-packages/PySide2/Qt/plugins/platforms'
import cv2
import numpy as np
import serial
import time
import paho.mqtt.client as mqtt

#Setup fot MQTT
broker = "192.168.213.184"
port   = 1883
topic = "opencv/esp32"


client = mqtt.Client()
client.connect(broker, port)


# Setup to connect to the Arduino
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=0.1)


# Open the camera

cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("Can't open the camera")
    exit()

red_BGR = [0, 0, 255]

def get_limits(color):
    c = np.uint8([[color]])  # BGR values
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    hue = hsvC[0][0][0]  # Get the hue value

    # Handle red hue wrap-around
    if hue >= 165:  # Upper limit for divided red hue
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:  # Lower limit for divided red hue
        lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit

# Define the range for red and yellow
lower_red, upper_red = get_limits(red_BGR)
lower_yellow = np.array([20, 50, 50])
upper_yellow = np.array([30, 255, 255])

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't get the frame (The program is ended!)")
        break

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks for red and yellow
    red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)
    yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
    combined_mask = cv2.bitwise_or(red_mask, yellow_mask)

    # Find contours
    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) > 1500:
            mask = np.zeros_like(hsv_frame[:, :, 0])
            cv2.drawContours(mask, [contour], -1, 255, -1)
            mean_val = cv2.mean(hsv_frame, mask=mask)[:3]

            if (0 <= mean_val[0] < 20 or 160 <= mean_val[0] <= 180):  # Red color detection
                color = "Red"
                arduino.write(b'1')  # Send '1' to Arduino

            elif 20 <= mean_val[0] <= 30:  # Yellow color detection
                color = "Yellow"
                arduino.write(b'2')  # Send '2' to Arduino
                
            else:
                color = "Unknown"
                arduino.write(b'0')  # Send '0' to Arduino for unknown color

            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(frame, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    
    if arduino.in_waiting > 0:
        arduino_data = arduino.readline().decode('utf-8').strip()
        print(f"Data from Arduino: {arduino_data}")

        # Only proceed if the received data is either 'R' or 'Y'
        if arduino_data in ['R', 'Y']:
            message = arduino_data
            print(f"Publishing to MQTT: {message}")
            client.publish(topic, message)


    cv2.imshow("Frame", frame)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


client.disconnect()
cap.release()
cv2.destroyAllWindows()
