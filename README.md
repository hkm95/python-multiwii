# python-multiwii (Quadcopter)
Get *sensor data from Arduino (Uno/Nano) running MultWii firmware using Python v2.x (I used Raspberry Pi).

Information about Multiwii can be found on: http://www.multiwii.com/wiki/index.php?title=Main_Page.

Arduino and Raspberry Pi are communicating over serial port.

Example: To run the application issue the command:-  sudo python ptest1.py

(To know your python version type:  python -V)

*You can get any type of data supported by MultiWii Serial Protocol (You can even fly directly from WiFi without using any Standard RC transmitter and reciever...although the performance would be poor, but its great for autonomous control): http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol

**Future revisions would include the use of python's threading library and a Flask web server to make a web based controllable Quadcopter...more on that can be found here: https://github.com/hkm95/web-multiwii




***I have used some of the function names and overall structure from MultiWii-GUI code written in processing language.
