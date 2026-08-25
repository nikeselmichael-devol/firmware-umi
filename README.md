# firmware-umi
 main flow how UDP sender works 

 Get sensor(IMU) and encoder data > Use CBOR to structure the data for packet sending > send packet using UDP + Wifi > data get sent to PC (Linux) 

# Pending
* LPF for IMU
* Library for WIFI, UDP
* Structure files for this 
