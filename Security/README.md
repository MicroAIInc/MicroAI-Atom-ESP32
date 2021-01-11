# MicroAI Security for the ESP32


### Overview
This includes source code, AI Engine library, Security library, and documentation specific to the ESP32 platform.
This data shall be used to generate the SDK for distribution.


### Library Files
  components/AiEngine/AiEngine.h	- header file for library
  
  components/AiEngine/libAiEngine.a	- static library

  components/SecurityChannels/securitychannels.h	- header file for library
  
  components/SecurityChannels/libSecurityChannels.a	- static library


### Configure the project

Open the project configuration menu (`idf.py menuconfig`). 

Then go into `Example Connection Configuration` menu.  Enter the proper WiFi SSID and Password to gain access to the local network.



### Build and Flash

Enter `idf.py build` to build project, 

or `idf.py -p PORT flash` to build and flash the project.



### Run and Monitor the program

There are 2 methods to run and execute the project:

1. Enter `idf.py -p PORT monitor` to begin running and monitor the project.

  (to exit the serial monitor, type ``Ctrl-]``)

2. Connect the USB com port to any terminal program (like TeraTerm).  Apply power or press the reset button and the program will begin.


### Customize the program

Read the [SDK PDF](https://github.com/ONE-Tech-Inc/MicroAI-Atom-ESP32/edit/master/Security/microAI%20ESP32%20Security%20SDK%20Version%201.0.pdf) document for additional detail on program operation and how to modify the program for options such as sending alert messages to an MQTT server.

