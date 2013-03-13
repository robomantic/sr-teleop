
-----------------------------------------
	1. INSTRUCTIONS:
-----------------------------------------

In order to start the cybergloveplus please do the following steps:

Step 1. Connect and power on the cyberglove

Step 2. Make sure that the button is switched off on the cyberglove (no red light visible)

Step 3. Start the cybergloveplus and cybergloveplusui by running
	roslaunch cybergloveplus control.launch

Step 4. Do training movements while button is switched off in order to identify the limits of each of the cyberglove sensors.

Step 5. Press the button in order to start sending commands to the Shadow Hand


-----------------------------------------
	2. LAUNCH PARAMETERS
-----------------------------------------

For example see cybergloveplus/launch/control.launch 

Serial port can be defined via the serialport parameter:
	<param name="serialport" type="string" value="/dev/ttyUSB0" />

The frequency is defined via the frequency parameter:
	<param name="frequency" type="int" value="100" />

Each joint can be enabled or disabled by using using the <joint>_active parameters:
	<param name="ffj0_active" type="bool" value="true" />

Parameter is_biotac tells if the Shadow Hand has Biotac fingers or not:
	<param name="is_biotac" type="bool" value="false" />

The suffix of the type of controller to be used to control the hand is given by 
using the controller_suffix parameter

	<param name="controller_suffix" type="string" value="_position_controller/command" />

The whole topic name is reconstructed automatically inside the node by using the 
following logic:

	"sh_" + joint_name + controller_suffix


-----------------------------------------
	3. GOOD TO KNOW
-----------------------------------------

- In case of emergency, just press the button to stop the glove from sending commands 
	to the Shadow Hand.

- After doing the training movements (step 4), we recommend to disable the auto 
	calibration by unchecking all Auto Calibration checkbox and pressing the Send 
	command button. This will stop the cyberglove software from autocalibrating during 
	usage.


-----------------------------------------
    4. HOW TO USE THE GUI
-----------------------------------------

The GUI consits of a grid with 5 columns
- Sensor Name		- the name of the sensor
- Min value			- the minimum value of the value (the range is from 0 to 1)
- Max value			- maximum value (the range is from 0 to 1)
- Auto calibrate	- checkbox that enabled or disables the autocalibration
- Select			- always disabled. if enable can be used to control only certain sensors

While auto calibration is enabled for a sensor, the GUI listens to the incoming raw
glove values, and updates the minimum and maximum values accordingly. See step 4
("do training movements") in chapter 1 above :-)

The Min and Max values spinboxes are always disabled if the autocalibration is enabled
for a sensor. During this time data is automatically displayed from the cybergloveplus
autocalibration that runs in the background.

Once autocalibration is disabled for a sensor, the cybergloveplus algorithm will not 
automatically update the limits based on the values recorded from the glove. If a value
is larger than the maximum value set, it will be automatically truncated to it. Same
thing for minimum.

Once the autocalibration is disabled, one can fine tune the parameters by changing the
values inside the Min and Max spinboxes. Once the values are modified, the user must
press the Send button, in order to send a calibration command to the cybergloveplus.

You can use the Load/Save buttons to import/export the configuration (min-max values
for a given sensor) from/to a file.



-----------------------------------------
    5. SENSOR - JOINT MAPPING
-----------------------------------------

The mapping implemented inside cyberglove control is as follows:

A. FOR SHADOW HAND WITH BIOTAC

JOINT	|	SENSOR
===========================
FFJ0 	| G_IndexPIJ
FFJ3	| G_IndexMPJ
FFJ4	| G_MiddleIndexAb
----------------------------
MFJ0 	| G_MiddlePIJ
MFJ3	| G_MiddleMPJ
MFJ4	| 0.5
----------------------------
RFJ0 	| G_RingPIJ
RFJ3	| G_RingMPJ
RFJ4	| G_RingMiddleAb
----------------------------
LFJ0 	| G_PinkiePIJ
LFJ3	| G_PinkieMPJ
LFJ4	| G_PinkieRingAb
LFJ5	| <not available>
----------------------------
THJ1	| <not available>
THJ2	| G_ThumbIJ
THJ3	| 0.5
THJ4	| G_ThumbAb
THJ5	| G_ThumbRotate
----------------------------
WRJ1	| G_WristPitch
WRJ2	| G_WristYaw

B. FOR SHADOW HAND WITHOUT BIOTAC

JOINT	|	SENSOR
====================================
FFJ0 	| G_IndexPIJ + G_IndexDIJ
FFJ3	| G_IndexMPJ
FFJ4	| G_MiddleIndexAb
------------------------------------
MFJ0 	| G_MiddlePIJ + G_MiddleDIJ
MFJ3	| G_MiddleMPJ
MFJ4	| 0.5
-----------------------------------
RFJ0 	| G_RingPIJ + G_RingDIJ
RFJ3	| G_RingMPJ
RFJ4	| G_RingMiddleAb
-----------------------------------
LFJ0 	| G_PinkiePIJ + G_PinkieDIJ
LFJ3	| G_PinkieMPJ
LFJ4	| G_PinkieRingAb
LFJ5	| <not available>
-----------------------------------
THJ1	| G_ThumbIJ
THJ2	| G_ThumbMJ
THJ3	| 0.5
THJ4	| G_ThumbAb
THJ5	| G_ThumbRotate
-----------------------------------
WRJ1	| G_WristPitch
WRJ2	| G_WristYaw



-----------------------------------------
    6. INSIDE "THE CAN"
-----------------------------------------

The TAMS Cyberglove stack consists of two packages:
- cybergloveplus 	: the main node that collects data from the cyberglove and writes 
						commands to joint controllers
- cybergloveplusui 	: configuration ui that allows fine tuning of the calibration parameters

The communication between the cybergloveplus and the ui is done via 2 topics:
- /cybergloveplus/calibration/status		- cybergloveplus sends all time the calibration 
												configurations for each sensor
- /cybergloveplus/calibration/command		- cybergloveplus listens for calibration messages

The calibration algorithm is as follows:

1. Raw level:
	- inside cybergloveplus : src/cybergloveraw.cpp
		= reads data from serial and publishes it into /cybergloveplus/raw/joint_states

2. Calibration level:
	- inside cybergloveplus : src/cyberglove_calib_min_max.cpp
		= moves the value space from the interval [min,max] of the raw values
			  into [0,1], where 0 corresponds to min and 1 to max
		= if autocalibration is enabled, everytime new data is received it is
			  compared to the minimum and maximum value found before in order to maintain
			  the scaling. If a smaller than min or larger than max value is found, the
			  min and max are updated accordingly
	- inside cybergloveplusui :
		= realtime values for the min, max and current value for each joint is displayed
		= commands can be send to cybergloveplus in order to disable the autocalibration
			and modify current min or max, or to reenable the autocalibration

3. Control level:
	- inside cybergloveplus : src/cyberglove_control.cpp
		= here lies the mapping between the cyberglove sensors and the Shadow Hand commands
		= each joint has limits defined in degrees
		= depending if the Shadow Hand has Biotac Sensors or not, the limits and mapping
			are modified accordingly
