import modbus_cnc

# Open port
mb = modbus_cnc.ModbusCNC('COM30')

# Setup M-function 38 as output 2 on function
#mb.writeRegister(100, 0x260005)

# Setup M-function 38 as pause function
mb.writeRegister(100, 0x260001)

# Setup inputs
mb.writeRegister(136, 11) # input 5 - homing
mb.writeRegister(137, 12) # input 6 - start program (number in reg90)
mb.writeRegister(90, 2) # reg90 = program 2
mb.writeRegister(138, 23) # input 7 - stop
mb.writeRegister(139, 24) # input 8 - block

# Setup outputs
mb.writeRegister(140, 25) # output 6 - error
mb.writeRegister(141, 26) # output 7 - motion
