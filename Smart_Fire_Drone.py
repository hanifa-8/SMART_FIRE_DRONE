from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import socket
import argparse
import geopy.distance
import serial
import numpy as np
import cv2
import tflite_runtime.interpreter as tflite
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
servo_pin = 17
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

interpreter = tflite.Interpreter(model_path="/home/pi/Downloads/fire_detection_model.tflite")
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()


def send_command(command, ser):
    ser.write(command.encode() + b'\r\n')
    time.sleep(1)
    response = ser.read_all().decode()
    print(response)

def setup_sim800l(ser):
    send_command('AT', ser)
    send_command('AT+CMGF=1', ser)
    send_command('AT+CNMI=1,2,0,0,0', ser)

def read_serial(ser):
    message_list = []
    message = ""
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        message += line
        if line:
            message_list.append(message)
            message = ""
    return message_list

def extract_sender_details(message):
    start_index = message.find("[")
    end_index = message.find("]")
    if start_index != -1 and end_index != -1:
        sender_details = message[start_index + 1:end_index]
        return sender_details
    return "Sender details not found"

def extract_location_coordinates(message):
    start_index = message.find("loc:") + len("loc:")
    end_index = message.find(",", start_index)
    latitude = message[start_index:end_index]
    start_index = end_index + 1
    end_index = message.find(")", start_index)
    longitude = message[start_index:end_index]
    return latitude, longitude

def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = args.connect
    baud_rate = 57600
    print("\nConnecting to vehicle on: %s" % connection_string)
    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    return vehicle

def arm_and_takeoff(aTargetAltitude, vehicle):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(3)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def get_distance(cord1, cord2):
    return geopy.distance.geodesic(cord1, cord2).meters

def goto_location(to_lat, to_long, vehicle):
    print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    curr_alt = vehicle.location.global_relative_frame.alt
    to_point = LocationGlobalRelative(to_lat, to_long, curr_alt)
    vehicle.simple_goto(to_point, groundspeed=3)
    to_cord = (to_lat, to_long)
    while True:
        curr_lat = vehicle.location.global_relative_frame.lat
        curr_lon = vehicle.location.global_relative_frame.lon
        curr_cord = (curr_lat, curr_lon)
        print("curr location: {}".format(curr_cord))
        distance = get_distance(curr_cord, to_cord)
        print("distance remaining {}".format(distance))
        if distance <= 2:
            print("Reached within 2 meters of target location...")
            break
        time.sleep(1)

def preprocess_image(image):
    img = cv2.resize(image, (227, 227))
    img = img.astype(np.float32) / 255.0
    img = np.expand_dims(img, axis=0)
    return img

def condition_yaw(heading, relative, yaw_speed, vehicle):
    if relative:
        is_relative = 1
    else:
        is_relative = 0

    msg = vehicle.message_factory.command_long_encode(
        0, 0,  
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0,  
        heading, 
        yaw_speed, 
        1,  
        is_relative,  
        0, 0, 0  
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def rotate_90_degrees(vehicle, yaw_speed, val):
    print(f"Rotation {val+1}: Rotating 90 degrees at {yaw_speed} deg/s")
    condition_yaw(90, True, yaw_speed, vehicle) 
    time.sleep(10) 
    print(f"Rotation {val+1}: Completed")
    

def move_forward(distance_meters, vehicle):
    distance_cm = distance_meters * 100.0
    current_heading = vehicle.heading

    vx = distance_cm / 100.0 * math.cos(math.radians(current_heading))
    vy = distance_cm / 100.0 * math.sin(math.radians(current_heading))

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,      
        0, 0,    
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  
        0b0000111111000111, 
        0, 0, 0, 
        vx, vy, 0,  
        0, 0, 0,  
        0, 0)    

    vehicle.send_mavlink(msg)
    vehicle.flush()

    time.sleep(5)  

    print(f"Moved forward {distance_meters} meters.")

def descend_to_target(current_altitude, target_altitude, vehicle):
    descent_step = 0.25
    while current_altitude > target_altitude:
        new_target_altitude = max(target_altitude, current_altitude - descent_step)
        vehicle.simple_goto(LocationGlobalRelative(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, new_target_altitude))
        time.sleep(1)
        current_altitude = vehicle.location.global_relative_frame.alt
    print("Reached target altitude.")

def set_angle(angle):
    duty = angle / 18 + 2
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1) 
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

def main():
    global cord
    cord = []
    ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=5)
    time.sleep(1)
    setup_sim800l(ser)
    z = 0
    while z == 0:
        messages = read_serial(ser)
        for message in messages:
            print(message)
            sender_details = extract_sender_details(message)
            if "google" in message:
                latitude, longitude = extract_location_coordinates(message)
                cord.append(float(latitude))
                cord.append(float(longitude))
                print("Latitude:", latitude)
                print("Longitude:", longitude)
            if "google" in message:
                z = 1
                break
    ser.close()
    if cord and len(cord) >= 2:
        vehicle = connectMyCopter()
        time.sleep(1)
        ht = 10
        arm_and_takeoff(ht, vehicle)
        time.sleep(1)
        goto_location(cord[0], cord[1], vehicle)
        time.sleep(2)
        cap = cv2.VideoCapture(0)
        C = ['fire', 'non fire']
        i=0
  
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            preprocessed_frame = preprocess_image(frame)
            interpreter.set_tensor(input_details[0]['index'], preprocessed_frame)
            interpreter.invoke()
            output_data = interpreter.get_tensor(output_details[0]['index'])
            predictions = np.squeeze(output_data)
            prediction = np.argmax(predictions)
            label = C[prediction]
            cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.imshow('Fire Detection', frame)
            
            if prediction == 0:

                time.sleep(1)

                move_forward(3, vehicle)  
                time.sleep(2)

                current_altitude = vehicle.location.global_relative_frame.alt
                target_altitude = 5
                descend_to_target(current_altitude, target_altitude, vehicle)
                time.sleep(2)

                try:
                    set_angle(90)
                    time.sleep(5)
                    set_angle(0)
                finally:
                    pwm.stop()
                    GPIO.cleanup()
                    break

            else:
                print("No fire detected!")
                if i<4:
                    rotate_90_degrees(vehicle, 10 , i)
                    i += 1  
                else:
                    print("Unable to rotate further!.No fire detected in all four directions:)")
                    break  

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        print("Returning to Launch")
        time.sleep(2)
        vehicle.mode = VehicleMode("RTL")
        vehicle.close()  
        
    else:
        print("Insufficient values to go to location")

if __name__ == "_main_":
    main()
    print(cord)