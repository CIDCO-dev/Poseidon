# Documentation for water_pump node package

This ros package is used to automate the detection of water leak in a boat. 

Material used : 
- Numato 8 channel USB gpio
- Water leak detector
- computer or rpi

# How does it works
The python scripts is reading the numato port each miliseconds to see if it has received a positive signal from the water leak detector. In either cases, it writes a message to a topic in which 1 means there is a leak or 0 no leak.

