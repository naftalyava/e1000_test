To get better understanding on how linux kernel nic drivers work, I took a functional e1000 driver and removed all the non core functionality from it.\
Support for all adapters is removed except from 82545.\
Support for all the fancy features [flow control, ethtools, VLAN, jumbo frames etc] is removed as well.\
Driver code was reduced from 17K LOC to 6.5K LOC.
