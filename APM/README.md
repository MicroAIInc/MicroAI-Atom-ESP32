# MicroAI for the ESP32


### Overview
This includes source code, AI Engine library, and documentation specific to the ESP32 platform.



### Library Files
  components/AiEngine/AiEngine.h	- header file for library
  
  components/AiEngine/libAiEngine.a	- static library



### Configure the project

For the initial build, flash, and run of the program, no special configuration is required.  However, this initial configuration provides input values of zero and no outbound communication.  

After the initial test of the program, refer to the [SDK PDF](https://github.com/ONE-Tech-Inc/MicroAI-Atom-ESP32/edit/master/APM/microAI%20ESP32%20APM%20SDK%20Version%201.0.pdf) document to make configuration changes for input sensor channels as well as outgoing messages.



### Build and Flash

Enter `idf.py build` to build project, 

or `idf.py -p PORT flash` to build and flash the project.



### Run and Monitor the program

There are 2 methods to run and execute the project:

1. Enter `idf.py -p PORT monitor` to begin running and monitor the project.

  (to exit the serial monitor, type ``Ctrl-]``)

2. Connect the USB com port to any terminal program (like TeraTerm).  Apply power or press the reset button and the program will begin.



### Customize the program

Read the [SDK PDF](https://github.com/ONE-Tech-Inc/MicroAI-Atom-ESP32/edit/master/APM/microAI%20ESP32%20APM%20SDK%20Version%201.0.pdf) document for additional detail on program operation and how to modify the program for options such as sending alert messages to an MQTT server.

