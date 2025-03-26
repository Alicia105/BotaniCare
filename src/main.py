import bluetooth  # needed for bluetooth communication
import pyrebase  # needed for firebase
import datetime
import time
import json
import os
import cv2
from PIL import Image
from ultralytics import YOLO

import random  # for testing

def readComputerVision():
    #settings
    dark_net=False#Choose YOLO model
    frameJump=5 #Choose the number of frame to jump
    #we jump 1/frameJump frames in the video--> help manage delay in real-time video processing  
    saveResult=True #Save video processing result
    showResult=True #Show prediction result window
    mp4VideoFormat=False #Choose the video format ouput if not .mp4 then .avi
    usbWebcam=False #Choose input video stream
    videoIn=f"../videos/rabbit1.mp4" #Choose video to process if not from USB webcam
    counter=0 #Number of detection
    n=15 #Number minimal of frames an animal needs to be seen
    minu=1 #Number minimal of minutes considered as an appropriate processing time 
    r=0 #Detection status
    i=0
    quit_=False #Satisfying conditions to quit the detection loop

    #YOLO model detection choice
    if(dark_net):#Choose modelsv4 model
        # Load class names
        with open('../models/coco.names', 'r') as f:
            classes = f.read().splitlines()

        # Load the YOLO model
        net = cv2.dnn.readNetFromDarknet('../models/yolov4/yolov4.cfg', '../models/yolov4/yolov4.weights')
        if net.empty():
            raise IOError("Could not load the YOLO configuration or weights files.")

        # Configure the model
        model = cv2.dnn_DetectionModel(net)
        model.setInputParams(scale=1 / 255, size=(416, 416), swapRB=True)  
    else:#Choose our Animal detection YOLOv8 model
        # Load class names
        with open('../models/yolov8/animal.names', 'r') as f:
            classes = f.read().splitlines()

        # Load the YOLO model
        model = YOLO("../models/yolov8/yoloAnimalDetection.pt")

    # Choose input video stream
    if (usbWebcam) :
        # Try opening the video stream from USB webcam
        usb_cam_index = 0  #webcam on video0
        cap = cv2.VideoCapture(usb_cam_index)
    else :
        cap = cv2.VideoCapture(videoIn)

    if not cap.isOpened():
        raise IOError(f"Cannot open video stream from USB webcam with index {usb_cam_index}.")

    # Get video properties
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))

    # Get the current timestamp
    current_timestamp = datetime.datetime.now()

    # Format the timestamp
    timestamp_str = current_timestamp.strftime("%Y%m%d_%H%M%S")

    if(saveResult):
        if(mp4VideoFormat):
            # Generate the filename
            resultVideoName = f"videoOutput_{timestamp_str}.mp4"
            # Define the codec and create VideoWriter object
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')

        else:
            # Generate the filename
            resultVideoName = f"videoOutput_{timestamp_str}.avi"
            # Define the codec and create VideoWriter object
            fourcc = cv2.VideoWriter_fourcc(*'XVID')

        # Create the results directory if it doesn't exist
        results_dir = "../results"
        if not os.path.exists(results_dir):
            os.makedirs(results_dir)

        # Generate the filename with the path
        filename = os.path.join(results_dir, resultVideoName)

        print(f"Video file created: {resultVideoName}")
        out = cv2.VideoWriter(filename, fourcc, fps, (frame_width, frame_height))

    res=["",""] #output format 
    #res[0]=animal type detected, res[1]=animal detection condition is met 

    while True:
        i+=1
        if(i%frameJump==0): #condition to jump frame and manage processing delay
            continue
        
        else:
            # Get the current timestamp
            new_timestamp = datetime.datetime.now()

            ret, frame = cap.read()
            if not ret:
                print("Failed to read frame from video stream")
                break

            if(dark_net):
                # Perform the detection
                classIds, scores, boxes = model.detect(frame, confThreshold=0.6, nmsThreshold=0.4)

                res[0]="none"
                # Draw bounding boxes
                for (classId, score, box) in zip(classIds, scores, boxes):
                    cv2.rectangle(frame, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), color=(0, 255, 0), thickness=2)
                    
                    text = '%s: %.2f' % (classes[classId], score)
                    cv2.putText(frame, text, (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0, 255, 0), thickness=2)
                    if(classes[classId]=='person'):
                        counter+=1
                        res[0]=classes[classId]
                
            else:
                # Convert BGR image to RGB for the model
                img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                # Run object detection
                results = model.predict(source=img_rgb)

                # Extract detection results
                boxes = results[0].boxes.xyxy.cpu().numpy()  # Extract bounding boxes
                scores = results[0].boxes.conf.cpu().numpy()  # Extract confidence scores
                class_ids = results[0].boxes.cls.cpu().numpy().astype(int)  # Extract class IDs

                res[0]="none"
                # Draw detections on the image
                for (class_id, score, box) in zip(class_ids, scores, boxes):
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color=(0, 255, 0), thickness=2)
                    text = f'{classes[class_id]}: {score:.2f}'
                    cv2.putText(frame, text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    if(classes[class_id]=='rabbit'or classes[class_id]=='rat'or classes[class_id]=='squirrel'):
                        counter+=1
                        res[0]=classes[class_id]
                    else :
                        res[0]="none"
                
            if(saveResult):
                # Write the frame into the output video file 
                out.write(frame)

            if(showResult):
                # Display the frame with detections
                cv2.imshow('Video Stream', frame)

            #Set boolean to quit derection loop
            if res[0]=='rabbit'or res[0]=='rat'or res[0]=='squirrel'or res[0]=='person'or res[0]=='none' :
                if new_timestamp.minute-current_timestamp.minute>=minu or counter>n :
                    r=1
                    quit_=True

            # Break the loop if 'q' key is pressed or satisfactory conditions met
            if cv2.waitKey(1) & 0xFF == ord('q') or quit_==True: 
                break
    res[1]=str(r)
    # Release the video capture and writer objects, and close all OpenCV windows
    cap.release()
    out.release()
    cv2.destroyAllWindows()

    print(counter)
    print(r)
    print(res)
    return res

def sanitize_json_data(data):
    sanitized_data = {}
    
    # List of expected keys and their types
    expected_keys = {
         "motion": float,
         "temperature": float,
         "humidity": float,
         "timestamp": str,
         "soilMoisture": float,
         "sunlight": float,
         "animal-detected": str,
         "animal-type": str
    } 
    
    # Sanitize and add expected keys
    for key, expected_type in expected_keys.items():
        if key in data and isinstance(data[key], expected_type):
            sanitized_data[key] = data[key]
        else:
            # Provide a default value if key is missing or incorrect type
            print("Wrong Data Type Detected in ", key)
            if expected_type == int:
                sanitized_data[key] = 0
            elif expected_type == float:
                sanitized_data[key] = 0.0
    
    return sanitized_data

def main():
    bluetooth_addr = "00:22:02:01:06:70"  # The address from the HC-06 sensor
    bluetooth_port = 1  # Channel 1 for RFCOMM
    bluetoothSocket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    bluetoothSocket.connect((bluetooth_addr, bluetooth_port))

    # to connect to firebase
    config = {
        "apiKey": "AIzaSyA1icqadoGaEPZO8jgxZsyfghkmw1bP32w",
        "authDomain": "botanicare-b3545.firebaseapp.com",
        "projectId": "botanicare-b3545",
        "storageBucket": "botanicare-b3545.appspot.com",
        "messagingSenderId": "1056535007014",
        "appId": "1:1056535007014:web:e4e1c51b28db79140efbe7",
        "measurementId": "GG-S3DFFGRS38",
        "databaseURL": "https://botanicare-b3545-default-rtdb.firebaseio.com"
    }
    firebase = pyrebase.initialize_app(config)
    database = firebase.database()

    while True:
        try:
            detected_animal="0"
            received_data = bluetoothSocket.recv(87)  # Assuming the size of the data in bytes
            json_data = json.loads(received_data.decode('utf-8'))
            print("Received Data:", json_data)

            # add timestamp here
            current_timestamp = datetime.datetime.now()
            timestamp_str = current_timestamp.strftime("%Y-%m-%d %H:%M:%S")
            print(timestamp_str)
            
            json_data["timestamp"] = timestamp_str            
            # Check if motion is greater than 0 and call readComputerVision if it is
            if 'motion' in json_data and json_data['motion'] > 0:
                res=readComputerVision()
                detected_animal = res[1]
                if detected_animal != "0":
                    json_data["animal-detected"] =detected_animal
                    json_data["animal-type"] = res[0]

            # send to database
            database.child("RPi@BotaniCare").child("Tester").update(json_data)
            
            # Send feedback signal to the Arduino if animal detected
            if detected_animal != "0":            
                detectionStatus= "1"
                bluetoothSocket.send(detectionStatus.encode('utf-8'))
            
            time.sleep(5)  # send data every one minute for real-time use. 5 for testing?
            
            if cv2.waitKey(1) & 0xFF == ord('q') : 
                print("keyboard interrupt detected")
                break
        
        except KeyboardInterrupt:
            print("keyboard interrupt detected")
            break
        except json.JSONDecodeError:
            print("Failed to decode JSON from received data")
        except Exception as e:
            print(f"An error occurred: {e}")

    bluetoothSocket.close()
    return

main()
