# CAB500
A python code for finding and configure LEMs CAB500 current sensors

Needs packages in requirement.txt to be installed <pip install -r requirements.txt">
## cab_search.py 
usage: python cab_search.py

Will display, and log if a folder "data" is present in current directory, all available CAB500 sensors.

## cab_info.py
usage: py .\cab_info.py --udsClientID 0x06A2 --subF "canSpeed"

Will change the bit rate of the sensor with the uds client ID of 0x06A2. 

usage: python .\cab_info.py --udsClientID "0x06A2" --subF canID --clientID "0x0602" --serverID "0x0601" --IpID "0x0221"

Will change IDs of the sensor 0x06A2.

The sub functions to chose from are: canID, filterFreq, CANspeed, framePeriod and reboot.

