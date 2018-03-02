import os
import serial
from inotify_simple import INotify, flags


ser = serial.Serial('/dev/ttyUSB0',9600)

inotify = INotify()
watch_flags = flags.CREATE | flags.DELETE | flags.MODIFY | flags.DELETE_SELF
wd = inotify.add_watch('/var/www/html/ugrs', watch_flags)
x=[]

while(1):
    
    for event in inotify.read():
        print(event)
        for flag in flags.from_mask(event.mask):
            print('    ' + str(flag))
        f = open("/var/www/html/ugrs/values.txt", "r")
        x = f.readlines()
        x=[y.strip() for y in x]
        for i in range(6):            
            print ("Motor " + str(i+1) +": " + x[i])
            x[i] += 'x'
            
            
            ser.write(x[i].encode())

            print(ser.readline())
    
    
    
