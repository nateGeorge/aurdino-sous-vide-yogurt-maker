import serial, re, csv, os
from datetime import datetime

ser = serial.Serial(10, timeout = 3)
windowSize = 5000
kp = 50
ki = 0.5
kd = 0.1
setpoint = 40

datafile = datetime.strftime(datetime.now(),'%Y-%m-%d %H-%M-%S') + ' temperature data.csv'
output = None
if not os.path.isfile(datafile):
    with open(datafile,'wb') as csvfile:
        arduinocsv = csv.writer(csvfile, delimiter = ',')
        arduinocsv.writerow(['time (iso)', 'output % on', 'temp'])

with open(datafile,'wb') as csvfile:
    arduinocsv = csv.writer(csvfile, delimiter = ',')
    while True:
        line = ser.readline()
        if re.search('output time on setpoint:', line):
            output = re.search('\d+\.\d+', line).group(0)
            print 'output:', output
        if re.search('temp:', line):
            temp = re.search('\d+\.\d+', line).group(0)
            print 'temp:', temp
        if output!=None:
            arduinocsv.writerow(['time (iso)', 'output % on', 'temp'])