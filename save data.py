import serial, re, csv, os, requests, time
from datetime import datetime

postAddr = "https://data.sparkfun.com/input/VGAgDL18xGu1y3lmnpAZ?private_key=9YAZyKkVdYFq7D9k0Xn1&datetime="

lastTime = datetime.now()
ser = serial.Serial(10, timeout = 3)
windowSize = 5000
kp = 50 # was 10 for second time through
ki = 0.5 # set to 5 third time
kd = 0.1 # set to 0 third time
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
            timeNow = datetime.now().isoformat()
            arduinocsv.writerow([timeNow, output, temp])
            if (datetime.now()-lastTime).total_seconds() >= 15:
                try:
                    r = requests.post(postAddr + timeNow + '&output=' + output + '&temp_c=' + temp)
                    print r.status_code, r.reason
                    lastTime = datetime.now()
                except Exception as e: # need to implement better code for storing data and sending if can't connect
                    print 'coulnd\'t connect, skipping data'
                    print e
                    continue
            output = None
            