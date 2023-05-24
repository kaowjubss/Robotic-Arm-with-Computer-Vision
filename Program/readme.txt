Program have 2 parts
1.Arduino
	- StandardFirmata
		Standard program from Firmata library
2.Python
	- Camera.py
		Main program get position of object from camera and send a mission to MyRobot class. the mission contain color and position. To pick at the right position and place in the color area.
	- MyRobot.py
		Recieve position from camera, and Do inverse kinematic to get angle of each servo motor. move servo motor to do the mission. Then wait for next mission.
		
		