import keyboard
import serial
import time
import sys

arduinoPort ='/dev/ttyUSB0'
arduinoBaudRate = 9600

try:
    arduinoData = serial.Serial(arduinoPort, arduinoBaudRate)
except serial.SerialException as e:
    print(f"Error: {e}")
    sys.exit(1)

print('Press q to open claw, a to close claw, esc to exit')

try:
    delay_between_commands = 0.05  # Adjust as needed

    while True:
        if keyboard.is_pressed('q'):
            print('Opening claw')
            arduinoData.write("OPEN\r".encode())
            time.sleep(delay_between_commands)
        elif keyboard.is_pressed('a'):
            print('Closing claw')
            arduinoData.write("CLOSE\r".encode())
            time.sleep(delay_between_commands)
        elif keyboard.is_pressed('esc'):
            print('Exiting')
            break

finally:
    arduinoData.close()