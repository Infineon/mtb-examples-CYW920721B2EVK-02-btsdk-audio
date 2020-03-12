#!/usr/bin/python -tt
# Copyright Cypress 2018
# Test Program to test the Audio Insert feature
# This program sends a WICED HCI command to test this feature
# This program has been designed to run under Cygwin.
# Cygwin use /dev/ttySx instead of COMy to access Com Ports.
# Note that 'x' is 'y-1' => COM20 = /dev/ttyS19

# To execute this test script (COM18, 3000000bps, 1 second)
#$./audio-insert-test.py -d /dev/ttyS17 -b 3000000 -p 1

# The following Python modules are required
# To install the 'serial' package, you can use $python -m pip install pyserial
import serial
import time
import sys

ser=None

# Open Serial Com Port
def serial_port_open(COMport,BaudRate=115200):
    global ser

    if ser!=None:
        ser.close()
        ser=None
    COMportOpen=True
    print 'Opening', COMport, 'at', BaudRate, 'bps'
    try:
        ser = serial.Serial(port=COMport,
                            baudrate=BaudRate,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS,
                            rtscts=True,
                            timeout=1)
    except serial.SerialException:
        COMportOpen=False

    return COMportOpen

# Read data from Serial Port
def serial_port_read(response_len=0,Timeout=1):
    response_finished=False
    start_time=time.time()
    response=''

    while (len(response)<response_len):
        response=response+ser.read(ser.inWaiting())
        if ((time.time()-start_time)>Timeout):
            return response
    print('UART RX<<%s'%' '.join('{:02X}'.format(ord(c)) for c in response))
    return response

# Send Audio Insert Test command
def audio_insert(duration):
    message='\x19\x21\xD0\x01\x00'+chr(duration)
    print('UART TX>>%s'%' '.join('{:02X}'.format(ord(c)) for c in message))
    ser.write(message)
    return serial_port_read(6,1)=='\x19\xFF\xD0\x01\x00\x00'

# Wait 1 second to receive the, optional, Device Started event
def device_started():
    response=serial_port_read(5,1)
    return response=='\x19\x05\x00\x00\x00'

# Check the parameters (passed on the Command Line)
def check_parameter(param):
    try:
        sys.argv.index(param)
        return True
    except:
        return False

# Exit
def exit(error, comPortOpen):

    if comPortOpen:
        ser.close()

    if error:
        sys.exit(1)
    else:
        sys.exit(0)

# Some default values
success = False
serialport = '/dev/ttyS1'
baudrate=3000000
param_duration=1

# Main function

# Check parameters
if check_parameter('-b'):
    baudrate=sys.argv[sys.argv.index('-b')+1]

if check_parameter('-d'):
    serialport=sys.argv[sys.argv.index('-d')+1]

if check_parameter('-p'):
    param_duration=sys.argv[sys.argv.index('-p')+1]

#open Com Port
comPortOpen = serial_port_open(serialport,baudrate)
if comPortOpen:
    # Wait to receive the, optional, Device Started event
    if device_started():
            print('device_started(optional): received')
    else:
            print('device_started(optional): not received')

    # Send the Audio Insert command
    if audio_insert(int(param_duration)):
            print('audio_insert(): Success')
    else:
            print('audio_insert(): Fail')
#    print('audio_insert(): %s'%audio_insert(int(param_duration)))

else:
    print("Couldn't open serial port %s", serialport)
    exit(True, comPortOpen)
