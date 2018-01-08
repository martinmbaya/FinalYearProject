#Libraries
import RPi.GPIO as GPIO
import time
import socket
import array
import struct

#Set up tcp server for transmission
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM, proto=0)
port = 12346
s.bind(("192.168.43.153", port))


#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

#set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

GPIO.output(GPIO_TRIGGER, False)
print("Waiting for sensor to settle..")
time.sleep(10)

def distance():
    print("Attempting..")
    #set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    #set Trigger after 1ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    #save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    #save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    #time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    #multiply with the sonic speed (34300 cm/s)
    #and divide by 2, because it's to and from
    distance = (TimeElapsed * 34300) / 2
    return distance

if __name__ == '__main__':
    try:
        s.listen(5)
        while True:
            dist = distance()
            print ("Measured Distance = %.1f cm" % dist)
            client, addr = s.accept()
            dist = str(dist)
            tdist = dist.encode()
            client.send(tdist)
            client.close()
            time.sleep(1)
            

    #Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
