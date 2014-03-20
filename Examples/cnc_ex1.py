import modbus_cnc

# Open port
mb = modbus_cnc.ModbusCNC('COM30')

# Setup M-function 38 as output 2 on function
mb.writeRegister(100, 0x260005)

# Setup inputs
mb.writeRegister(136, 11) # input 4 - homing
mb.writeRegister(137, 12) # input 5 - start program from reg90
mb.writeRegister(90, 2) # reg90 = program 2
