To get better understanding on how linux kernel nic driver work, I took a functional e1000 driver and removed all the non core functionality from it.
Support for all adapters is removed except to 82545 together with all the fancy features [flow control, ethtools, VLAN, jumbo frames etc].

Drive code was reduced from 17K LOC to 6.5K LOC.
