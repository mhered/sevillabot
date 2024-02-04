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

print('Press Q to aim higher, A to aim lower, L to toggle laser, SPACE to fire, ESC to exit')

try:
    delay_between_commands = 0.05  # Adjust as needed

    while True:
        if keyboard.is_pressed('q'):
            print('aiming higher')
            arduinoData.write("UP\r".encode())
            time.sleep(delay_between_commands)
        elif keyboard.is_pressed('a'):
            print('aiming lower')
            arduinoData.write("DOWN\r".encode())
            time.sleep(delay_between_commands)
        elif keyboard.is_pressed('space'):
            print('firing!')
            arduinoData.write("FIRE\r".encode())
            time.sleep(delay_between_commands)
        elif keyboard.is_pressed('l'):
            print('toggling laser')
            arduinoData.write("LASER\r".encode())
            time.sleep(delay_between_commands)
        elif keyboard.is_pressed('esc'):
            print('Exiting')
            break

finally:
    arduinoData.close()
