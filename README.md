# esp-fso
ESP32 based fine-scanning system for FSO. Codes have been written in Arduino IDE.

A fine scanning method employed using two ESP32s to find a Free Space Optics or FSO link.
The codes have been tested over 150m outdoor FSO link few times, still the project is under development.

# motorESP
Code for the tx-side motor ESP32.
Supports remote reset via WiFi from a button press on adcESP.
Moves the two motors in yaw-pitch direction according to the coordinates in terms of motor steps recieved.

# adcESP
Code for the rx-side ESP32.
Controls the scanning pattern and determines when to stop upon finding the FSO link.

Currently two ways of scanning have been implemented,
1. FineSpiral: The fine scan pattern is same as coarse scan pattern, both are equally spaced spiral scan patterns.
2. SGD: The fine scanning here is done using Steepest Gradient Descend method, coarse scan is same as previous one.
