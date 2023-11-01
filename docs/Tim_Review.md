- What is the input voltage range? Should annotate that on schematic (voltage min/max, current rating
-- added
- Inductor in line of power input?
-- have not done yet
- "If it's latched up, it won't be executing any code." -Pete 
	- This is not true. A latch up does not necessarily cause the CPU has crashed. In fact it generally does not, and a software protection that detects increased current and power cycles the chip is totally valid.
	- If you want to actually do this, you should do some analysis of the timing. A common way to do this is to have a capacitor on the enable pin, with a weak pull-up, and then an N-FET to ground. When you turn on the GPIO, it turns on the FET, which drains the cap really fast (nanoseconds). It take microseconds to re-charge, giving the power rails time to turn fully off, and giving the latch-up a chance to clear
- You should break out all the GPIO, even the ones the teensy does not. Just leave them as test points.

- These too (the ones not going to the JTAG header):
	 ![[Pasted image 20231021010133.png|300]]
- I would add test points to the SD card interface to help debug/software
- SENSORS
	- You should have a power switch to power cycle the sensors. I would say you should put two of each sensor, and have two I2C busses, each with one of each sensor.
	- I2C isolation is not actually super practical, imo. The bus is too complicated to be able to guarantee two devices are isolated. I'd just aim for reliability by redundancy.
	-- will do both, eventually. For now, just multiple devices
	- You should break out more pins of the GPS. At least the PPS pin, which you might want to use. 
	-- added a note to schematic, not gonna do for now
	- What's the deal with this splitter thing? Why not just a splitter, or two GPS's? seems like extra points of failure on zero fault tolerant system.
	-- resolved this