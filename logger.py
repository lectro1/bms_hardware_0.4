import serial
import datetime

port = "/dev/ttyUSB0"
baud = 19200
fileName = "logger.csv"

ser = serial.Serial(port, baud)
print("Connected to port: " + port)
file = open(fileName, "a")
print("Created file!")

getData = str(ser.readline())
data = getData[2:][:-5]
print(data)

file = open(fileName, "a")
file.write(data + "\n")

file.close()

samples = 10  # how many samples to collect
print_labels = False
line = 0  # start at 0 because our header is 0 (not real data)
while True:
    # incoming = ser.read(9999)
    # if len(incoming) > 0:
    if print_labels:
        if line == 0:
            print("Printing Column Headers")
        else:
            print("Line " + str(line) + ": writing...")
    getData = str(ser.readline())
    st = datetime.datetime.now().strftime('%H:%M:%S,')
    data = st + getData[2:][:-5]
    print(data)

    file = open(fileName, "a")
    file.write(data + "\n")  # write data with a newline
    line = line+1

print("Data collection complete!")
file.close()
