Configuring Visual Studio Code IDE to run with Particle Photon
  https://www.hackster.io/gusgonnet/use-visual-studio-code-for-your-particle-development-needs-9e23bc
1.  Update particle IDE
    https://binaries.particle.io/cli/installer/windows/ParticleCLISetup.exe
	npm install -g particle-cli
2.  Download/install VS Code
	a.   Install from https://code.visualstudio.com/
	b.  Execute from command line providing proect path:
		$ code /home/gustavo/myParticleProject
		This can also be done from File menu of VS IDE
	c.  Install C/C++ Extension from Getting Started of https://code.visualstudio.com/docs/languages/cpp
	d.	Configure the c_cpp_properties.json file following instruction from same place of previous line
		browse to a .cpp file that has a #include.   You will see squiggly green line underneath.  Click on the text that is undelined.
		click the lightbulb that appears.   Click "Add include path to settings"
	e.  Generate a tasks.json file following instructions same place
		open Command Palette (ctrl-shift-P)
		select Tasks:  Configure Task Runner
		select Others
		Change "command" to the cmd line expression used to build ""
In the tasks.json (.vscode folder in MYWINDCODE) make it look like following (for usb flashing)
{ 
"version": "0.1.0", 
"command": "particle", 
"isShellCommand": true, 
"args": [], 
"showOutput": "always", 
"echoCommand": true, 
"tasks": [ 
{ 
    "taskName": "compile", 
    "suppressTaskName": false, 
    "isBuildCommand": true,
    "args": ["photon", "${workspaceRoot}", "--saveTo", "${workspaceRoot}/firmware.bin"]             
}, 
{ 
    "taskName": "flash",             
    "suppressTaskName": false, 
    "isTestCommand": true, 
    "args": ["--usb", "firmware.bin"] 
}    
]     
} 


	f.  Format your code by right-click - Format Code
	g.  Build by ctrl-shift-B
	h.  Flash by ctrl-P, task - <space> - <flash>
		Photon must be blinking yellow by pushing both buttons simultaneously, releasing RESET, wait for purple then yellow
	
	


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
	
