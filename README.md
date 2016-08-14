
			GEM Telescope Mount Control with an Arduino Mega -- Direct output version

GEM Telescope Mount Control with an Arduino Mega -- Direct output version


By: Ray Wells, forked from Mathew Watts' fine work.
------------------------------------------------

 Following up on the Work of Matthew Watts "https://github.com/mwvent/arduino_telescope_mount" I started making changes to better suit the hardware I selected. My design is mounted on a Celestron Omni CG-4 nonmotorized equatorial mount with tight accurate tracking for unguided Astrophotography being the primary focus. 

The software provides the following features
   Control of the mounts axis using DC motors with encoder feedback
   Largely LX200 compatible interface on the Serial ports of the Arduino for PC control
   A user interface using an LCD and Wii nunchuck connected to the I2C Bus, this is optional/easily changed see notes below
   Tracking

And I am working on: 
- Moving the fine and course jogging controls to be active outside the tracking menu.
- Setting backlight control to only blind me during setup.
- Changing the blue on white LCD to a red on black one with dimming - library change needed
- sorting out the meridian flip control for different geolocations - still gets confused by southwestern goto calls.
- An additional RA encoder may be added to the other side of the slow motion control shaft to enable PEC 
- Guiding

I will be following upstream progress on these functions with interest in possible later pulls:
   Periodic error correction for RA Axis encoders that are indexed
   An INDILib driver for finer control

This code uses the following hardware:

Required
--------
Arduino Mega 2560 
LM298 H-bridge driver board for for dual axis bidirectional motor drive output 
		https://smile.amazon.com/RioRand-H-Bridge-Arduino-Stepper-Control/dp/B00JCJ9QPU/ref=sr_1_1

DS3231 Real Time Clock on I2C Bus (works with existing ds1307 library but has better clock)
		https://smile.amazon.com/Holdding-AT24C32-Precision-Temperature-compensated-Oscillator/dp/B00LZCTMJM/ref=sr_1_fkmr0_1

2 DC Motors with quadrature encoders connected to the mounts 2 axis --high Ratio gearheads needed. I swapped the 515:1 gearheads from the first motor set to the second motor which has a shaft mounted encoder to get the right ratios. 

		https://smile.amazon.com/High-Torque-Reversable-Electric-Motor/dp/B01517X404/ref=sr_1_fkmr0_4
		https://smile.amazon.com/SainSmart-29-Gearmotor-37Dx52L-Encoder/dp/B008BGLO00/ref=sr_1_1

LCD Screen with I2C Backpack for communications - I suggest a red on black or other dark screen. Will be changing mine soon.

		https://smile.amazon.com/SainSmart-Serial-Module-Shield-Arduino/dp/B0080DYTZQ/ref=sr_1_1

BC06 bluetooth to serial module connected to SPI-1 

		https://smile.amazon.com/KEDSUM®-Arduino-Wireless-Bluetooth-Transceiver/dp/B0093XAV4U/ref=sr_1_1

A 6 or 12VDC power source for the motor boards. 
		
Ninendo Wii nunchuck* (see below for other options) (was on hand)

Optional
--------
DC Motor conencted to telescope focuser.

Some kind of outdoor box to hold it all.

Getting started
---------------
-  Always have an E-stop switch - (said no sci-fi movie ever.) 
- Wire the power source. I routed wires to the l298 through a fuse and switch from the solder lugs of the power jack on the mega and use a 7ah SLA battery outdoors, or a wallwart for testing. This way the motors can be shut down leaving the cpu active for debugging.
- Using the I/O list in settings.h Hookup your supply and motors to the L298 board.
- Hookup your encoders, the directions are now reversable in software to make up for motor side/opposite side encoder mountings by way of a compiler directive. 
  
- If your RA Encoder has a z-index hook it up at A12. An RA Encoder with an index that is coupled to the RA shaft will enable Periodic error correction in later editions of this code (rbw: Code for this is currently disabled in my version due to lack of a Z output on my encoders.)

- Hook up the DC1307, LCD Screen and Wii nunchuck. If you want to drive the telescope purely from the serial port with LX200 and not use the LCD / Wii controller  do this by commenting out the lines UI_c* UI; - UI = new UI_c(mount, DS1307_RTC, EEPROMHandler); and UI->update();

- Setup variables are in settings.h and include motor and encoder direction and speed limits. You will also need to enter the PPR (pulses per revolution) for each encoder, taking into account the gearhead if it is motorshaft mounted, and the amount of gear teeth on your axis (or rather the amount of revolutions the worm gear would make to fully rotate the axis). 

- Once you have things moving in the right directions, the LCD screen will display nested menus for realtime setup changes as well as timing and position readouts.  The PID settings as well as a fine tune of the RA gear ratio can be found there for realtime tuning of tracking speeds. Be careful not to get the proportional gain too high as it can result in some fairly agressive motor speed oscillation. The bluetooth module works well with Stellarium's telescope goto plugin using /dev/rfcomm0 as the port. 



A video of the Mathew's original version of this setup in action, I'll try to make something similar sometime soon.
https://www.youtube.com/watch?v=mgYvCZ1bOII 

Some Photo's of my version:
https://goo.gl/photos/Z6ZzgSoEpCQbiZXB6
