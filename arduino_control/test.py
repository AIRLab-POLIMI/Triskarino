import serial
import time 
import json

port = '/dev/ttyACM0'
baud_rate=115200
timeout=1
sleep_time = 1

if __name__ == "__main__":
    print("Connection starting...")
    ser = serial.Serial(port, baud_rate, timeout=timeout)
    ser.flush()
    print("Connection is " + str(ser))
    #Read from the serial connection to get the json for odometry and sonar and print it 
    while 1: 
        time.sleep(sleep_time)
        print("Restarting Loop")
        line = ser.readline()
        line = line.decode("utf-8")
        print("line is..." + str(line))
      #  try:
      #      msg = json.loads(line)
     #       print(msg)
     #   except:
     #       print("First Line got truncated probably")
        speedX = float(input("Insert speedX.."))
        speedY = float(input("Insert speedY.."))
        speedTh = float(input("Insert speedTh.."))
        twist_msg = {"twist": [speedX, speedY, speedTh]}
        serialized_twist_msg = json.dumps(twist_msg)
        print("Serialized message is " + serialized_twist_msg)
        ser.write(bytes((serialized_twist_msg), encoding='utf-8'))
       