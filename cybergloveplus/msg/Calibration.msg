# this message contains information about the cyberglove only. name is filled with cyberglove sensor name
# auto_calibration - tells if autocalibration is enabled for the given sensor. Autocalibration means that
# 					the minimum and maximum values are updated in realtime
# min - minimum value of the sensor that is used for 0..1 rescaling
# max - maximum value of the sensor that is used for 0..1 rescaling

string[] name
bool[] auto_calibration
float64[] min
float64[] max
