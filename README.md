1.       If compile in DEV and no status message returned, probably are not connected to wifi.  Particle-DEV does not work over GE network.

2.       Wifi connection problems at work usually fixed by changing Odyssey from BLUESSO to Internet.   Particle-DEV does not work over GE network.

4.       Photon does not connect for flash:   need to get closer to Internet wifi.   Photon wifi a little weak.

5.       Coolterm “Port not found.”  Photon USB not visible.  Restart Coolterm.   Unplug Photon USB and replug.   Rescan ports in Coolterm-Connection-Options.   Prevent this by disconnecting CoolTerm before servicing Photon.

6.       Red error window pops in DEV.  Restart DEV.


Compile Photon.
	- use MS Visual Studio IDE
	- ctr-Shift-B #to build
	- ctrl-P task flash #to flash
Communicate with photon real time
	- Cool Term - Connection - Send String
		b    # bare photon toggle
		t    # close loop with model toggle
		c    # closed loop toggle
		P 20 # request potThrottle 20 deg, open or closed work
		V 	 # run VECT toggle
		R 	 # run RAMP toggle
		f 	 # run freq resp toggle
	
