import RPi.GPIO as GPIO
import time

LED_PIN_1=20
LED_PIN_2=21

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(LED_PIN_1,GPIO.OUT)
GPIO.setup(LED_PIN_2,GPIO.OUT)

for x in range(0,3):
	print "LED on"
	GPIO.output(LED_PIN_1,GPIO.HIGH)
	GPIO.output(LED_PIN_2,GPIO.HIGH)
	time.sleep(0.5)
	print "LED off"
	GPIO.output(LED_PIN_1,GPIO.LOW)
	GPIO.output(LED_PIN_2,GPIO.LOW)
	time.sleep(0.5)
