#
# A program to calibrate gyro on Create 1 - reads DB25 pin 4 analog value and prints it
#
# Run it on Turtlebot's on-board Raspberry Pi: "cd ~/launch; python3 roomba.py"
#
# Adjust gyro_offset - see https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1
#

import time
from serial import *

tty = Serial(port="/dev/ttyUSB0", baudrate=57600, timeout=0.5)

def sendToRoomba(numbers):

    send_bytes=bytes(numbers)
    tty.write(send_bytes)
    time.sleep(2)

def bytes_to_int(byte_array):
    return int.from_bytes(byte_array, byteorder='big')

def askRoomba():

    try:

        print("Sending Reset")

        sendToRoomba([7])   # reset

        line = tty.readline().decode('utf-8').strip()
        print(line)

        print("Sending Start")

        sendToRoomba([128]) # start Open Interface

        print("Flushing serial")

        num_bytes_to_read = 200
        data = tty.read(num_bytes_to_read)
        print(data)  # Prints the raw bytes

        #print("Trying beep")
        #sendToRoomba([140,3,1,64,16,141,3]) # beep

        print("Reading DB25 pin 4:")

        while True:
            sendToRoomba([142,33]) # analog read DB25 pin 4

            num_bytes_to_read = 2
            data = tty.read(num_bytes_to_read)
            #print(data)  # Prints the raw bytes

            val = bytes_to_int(data)
            print(val)

    except:
        print("exception caught")

askRoomba()
tty.close()
