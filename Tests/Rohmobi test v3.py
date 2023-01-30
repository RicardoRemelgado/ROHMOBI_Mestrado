import sys
from pynput import keyboard
import serial.tools.list_ports
import cv2
import numpy as np
from timeit import default_timer as timer
import matplotlib.pyplot as plt
import csv
import math

serialInst = serial.Serial()
serialInst.baudrate = 115200
serialInst.port = 'COM3'
serialInst.open()
x_key = ""

# Read video from external camera
video = cv2.VideoCapture(1)
name = str(input("Name the test: "))
frame_width = int(video.get(3))
frame_height = int(video.get(4))
write_video = cv2.VideoWriter(name + '.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10,
                              (frame_width, frame_height))

# Exit if video is not
if not video.isOpened():
    print("Could not open video")
    sys.exit()

#start_time = timer()
Velocities = []
Avg_velocity = []
Centers = []
Distances = []
Times = []
Total_time = []
Data = []
Data_csv = []
center = (0, 0)
old_center = (0, 0)
w = 1 #width of bbox, avoid erros by dividing by zero
delay_time = 0
start_time = timer()

# Graphs cracteristics
fig1 = plt.figure(1)
ax1 = plt.axes(title='Velocidade instantânea', xlabel='Time [s]', ylabel='Speed [mm/s]')
fig2 = plt.figure(2)
ax2 = plt.axes(title='Velocidade média', xlabel='Time [s]', ylabel='Speed [mm/s]')

#Boolean variables
bol_test = False
first_cycle = True
init_comms = True

def on_press(key):
    global x_key
    try:
        # print( key.char)
        x_key = key.char
    except:
        print("Invalid Key!")


# Collect events until released
listener = keyboard.Listener(on_press=on_press)
listener.start()

while True:
    serialInst.write(x_key.encode('utf-8'))
    if (x_key == 'f'):
        break
    elif (x_key == 'w'):
        delay_time = timer()
        start_time = 0
        bol_test = True
    x_key = ""

    # Stop if command s (sem isto o vídeo não corre)
    if cv2.waitKey(1) & 0xFF == ord('s'):
        print('Stop')

    # Read a new frame
    ret, frame = video.read()
    if not ret:
        break

    image = frame
    # Convert to hsv color space
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    # Lower and upper bound of HSV green color spectre
    lower_bound = np.array([28, 40, 215])
    upper_bound = np.array([88, 255, 255])

    # Find the colors within the boundaries
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # Define kernel size
    kernel = np.ones((7, 7), np.uint8)

    # Remove unnecessary noise from the mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # removes black noise from white regions
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # removes white noise from black regions

    # Segment only the detected region
    segmented_img = cv2.bitwise_and(image, image, mask=mask)  # applies the mask on the image to obtain only the colors that we want

    # Find contours from the mask
    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    output = cv2.drawContours(image, contours, -1, (255, 0, 0), 3)

    # Drawing the bbox and the center around the object
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 40: #remove noise from the image
            M = cv2.moments(cnt)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(output, center, 5, (0, 0, 255), -1)
            Centers.append(center)
            w = math.sqrt(area) #the blob is a square so the width is the square root of the blob, more consistent then creating the bounding box

    # Drawing the trajectory
    for i in range(len(Centers)):
        if i > 0:
            start = Centers[i - 1]
            end = Centers[i]
            cv2.line(output, start, end, (0, 0, 255), 2)

            if len(contours) == 0:  # if object out of frame stop tracking
                Centers = []
                break

    # Get the elapsed time
    end_time = timer() - delay_time
    time = end_time - start_time

    if bol_test == True:
        # Time
        Times.append(time)
        Total_time.append(sum(Times))
        #Distance
        #distance = np.sqrt((center[0] - old_center[0]) ** 2 + (center[1] - old_center[1]) ** 2)
        distance = old_center[0] - center[0]
        distance_mm = (distance * 30) / w
        # Velocity in mm/s
        velocity = distance_mm / time
        Velocities.append(velocity)

        cv2.putText(output, "Instant velocity: " + str(int(velocity)) + ' mm/s', (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, (0, 0, 0), 2);
        cv2.putText(output, "Elapsed time: " + str(int(start_time)) + ' s', (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, (0, 0, 0), 2);

        if serialInst.in_waiting and first_cycle == False:
            data_pack = serialInst.readline()
            arduino_info = data_pack.decode('utf-8')

            #Average velocity, avoid initial peack
            Distances.append(distance_mm)
            avg_velocity = sum(Distances) / sum(Times)
            Avg_velocity.append((avg_velocity))
            cv2.putText(output, "Average velocity: " + str(int(avg_velocity)) + ' mm/s', (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        0.75, (0, 0, 0), 2);

            if arduino_info.count('_') == 3:

                arduino_info = arduino_info.split("_")

                Data.append(str(round(start_time, 2)))
                Data.append(str(round(velocity, 2)))
                Data.append(str(round(avg_velocity, 2)))
                Data.append(str(float(arduino_info[0])/100))
                Data.append(str(float(arduino_info[1])/100))
                Data.append(str(float(arduino_info[2])/100))
                Data.append(str(float(arduino_info[3])/100))

                Data_csv.append(Data.copy())
                Data.clear()

        # update the variables
        old_center = center
        start_time = end_time

        #don't consider the first cycle
        first_cycle = False

    else:
        cv2.putText(output, "Waiting test to start ", (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, (0, 0, 0), 2);

    # Showing the output
    cv2.line(output, (0, 100), (639, 100), (255, 255, 255), 1)
    cv2.imshow("Tracking object", output)
    write_video.write(output)


print(Data_csv)
print(Centers)
print(Distances)
print(Times)

with open("C:/Universidade/10º Semestre - Tese de mestrado/Python code/Rohmobi test/Tests/" + name + ".csv", mode = 'w', encoding='UTF8', newline='') as info:
    data_write = csv.writer(info, delimiter=',', quotechar = ' ', quoting=csv.QUOTE_MINIMAL)
    header = ['Time', 'Velocity', 'Avg Vellocity', 'AccelerationX', 'AccelerationY', 'AngularAccX', 'AngularAccY']
    data_write.writerow(header) # only writes on time
    data_write.writerows(Data_csv)

ax1.plot(Total_time[1:], Velocities[1:])
ax2.plot(Total_time[1:], Avg_velocity)
plt.show()
video.release()
write_video.release()
cv2.destroyAllWindows()
