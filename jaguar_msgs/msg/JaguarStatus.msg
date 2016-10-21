Header header

#The below fields are inside arrays. One value for each battery pack. (The battery packs above 2 have a broken status value.)

# Status of the motor controller ESTOP/OK
uint32[] status

# Internal chip temperature
float32[] internal_chip_temp

#Driver in channel 1 temperature
float32[] driver1_temp

#Driver in channel 1 temperature
float32[] driver2_temp

#Main battery voltage
float32[] battery_voltage

#12V power
float32[] vol12V 

#12V power
float32[] vol5V

# Digital input
uint32[] dinput

#Digital output
uint32[] doutput

#Received command + for correct - for incorrect?(I need to investigate what this does exactly)
int32[] ack