import keyboard
import time

while True:
	if keyboard.is_pressed('up arrow'):
		a = 10
		print(a)
		#break
	elif keyboard.is_pressed('down arrow'):
		b = 11
		print(b)
		#break
	elif keyboard.is_pressed('left arrow'):
		c = 12
		print(c)
		#break
	elif keyboard.is_pressed('right arrow'):
		d = 13
		print(d)
		#break
	else:
		pass
	time.sleep(0.001)


