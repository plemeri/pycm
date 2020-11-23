from pycm import CM
import time
import socket
# 1 - Open CarMaker with option -cmdport
'''
    For example: on a Windows system with CarMaker 8.0.2 installed on the default
    folder send the command C:\IPG\carmaker\win64-8.0.2\bin\CM.exe -cmdport 16660
'''

IP_ADDRESS = 'localhost' 
PORT = 16600
cm = CM(IP_ADDRESS, PORT)

# 4 - Connect to CarMaker

# 5 - Subscribe to vehicle speed
# Create a Quantity instance for vehicle speed (vehicle speed is a float type variable)

# Initialize with negative speed to indicate that value was not re"adCM

# Subscribe (TCP socket need to be connected)

# Let's also read the simulation status (simulation status is not a quantity but a command
# so the command parameter must be set to True)

# 6 - Read all subscribed quantities. In this example, vehicle speed and simulation status
# For some reason, the first two reads will be incomplete and must be ignored
# You will see 2 log errors like this: [ ERROR]   CarMaker: Wrong read
cm.read()
cm.read()
time.sleep(0.1)
c = 100000
while(c > 0):
    c = c - 1
    # Read data from carmaker
    cm.read()
    print(cm.quantity)
    # print("Simulation status: " + ("Running" if sim_status.data >=
                                #    0 else cm.status_dic.get(sim_status.data)))
    time.sleep(0.01)
