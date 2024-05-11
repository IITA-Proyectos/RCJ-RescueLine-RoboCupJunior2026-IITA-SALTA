import serial
import time
ser = serial.Serial('COM4', 9600)
caracter= "-"
while True:
    caracter= "a"
    ser.write(caracter.encode('utf-8'))
    print("encendido")
    time.sleep(2)
    caracter= "b"
    ser.write(caracter.encode('utf-8'))
    print("apagdo")
    time.sleep(2)
 