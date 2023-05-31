import serial
import time 
import json

port = '/dev/ttyACM0'
baud_rate=115200
timeout=1
sleep_time = 2

if __name__ == "__main__":
    ser = serial.Serial(port, baud_rate, timeout=timeout)
    ser.flush()
    #Read from the serial connection to get the json for odometry and sonar and print it 
    while 1: 
     line = ser.readline()
     if line == "":
      time.sleep(sleep_time)
      msg = json.loads(line)
     print(msg)