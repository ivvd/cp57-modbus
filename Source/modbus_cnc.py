import serial

DEFAULT_BAUDRATE = 115200
FUNC_SEND_MSG = 102
FUNC_READ_MSG = 101
FUNC_READ_HOLD_REG = 3
FUNC_WRITE_MULT_REG = 16
READ_BYTE_NUMB = 100
cnc_addr = 1;

class ModbusCNC:
    """ Class for operation with PT-CNC via ModbusRTU """

    def __init__(self, port_name):
        """ Open COM-port and setup it """
        self.port = serial.Serial(port_name, DEFAULT_BAUDRATE,
                             parity=serial.PARITY_NONE,
                             stopbits=serial.STOPBITS_ONE,
                             timeout=0.5)

    def sendMessage(self, msg):
        """ Send 8 byte message to CNC """
        frame = []
        crc = []
        #Slave address
        frame.append(cnc_addr)
        #Function code
        frame.append(FUNC_SEND_MSG)
        #Bytes of message
        for item in msg:
            frame.append(item)
        #Calculate CRC
        crc = self.calcCRC16(frame)
        frame.append(crc[0])
        frame.append(crc[1])
        #Send frame
        self.port.write(frame)
        for c in frame:
            print(hex(c), end=' ')
        print()
    
    def calcCRC16(self, data):
        """ Calculate modbus CRC16 with polynom 0xA001.
            Input data must be a list of byte (0-255) numbers
        """
        polynom = 0xA001
        crc16 = 0xFFFF
        ret_crc16 = []

        for byte in data:
            crc16 ^= byte
            crc16 &= 0xFFFF
            for i in range(8):
                if (crc16 & 0x0001) != 0:
                    crc16 >>= 1
                    crc16 ^= polynom
                else:
                    crc16 >>= 1
            crc16 &= 0xFFFF
        ret_crc16.append(crc16 & 0x00FF) #LSB
        ret_crc16.append(crc16 >> 8) #MSB
        return ret_crc16

    def setState(self, new_state):
        """ Set CNC state """
        MSG_CODE = 1
        state_lo = new_state & 0xFF
        state_hi = (new_state >> 8) & 0xFF
        msg = [MSG_CODE, state_lo, state_hi, 0, 0, 0, 0, 0]
        self.sendMessage(msg)
        answ = []
        answ = self.port.read(READ_BYTE_NUMB)
        print(len(answ))
        for byte in answ:
            print(hex(byte), end=' ')
        print()

    def runProgram(self, prg_numb, ref):
        """ Run CNC program """
        if ref == 0:
            #run from zero position
            MSG_CODE = 4
        else:
            #run from actual position
            MSG_CODE = 5
        prg_lo = prg_numb & 0xFF
        prg_hi = (prg_numb >> 8) & 0xFF
        msg = [MSG_CODE, prg_lo, prg_hi, 0, 0, 0, 0, 0]
        self.sendMessage(msg)
        answ = []
        answ = self.port.read(READ_BYTE_NUMB)
        print(len(answ))
        for byte in answ:
            print(hex(byte), end=' ')
        print()

    def stopProgram(self, prg_numb, param):
        """ Run CNC program """
        MSG_CODE = 6
        param_lo = param & 0xFF
        param_hi = (param >> 8) & 0xFF
        msg = [MSG_CODE, param_lo, param_hi, 0, 0, 0, 0, 0]
        self.sendMessage(msg)
        answ = []
        answ = self.port.read(READ_BYTE_NUMB)
        print(len(answ))
        for byte in answ:
            print(hex(byte), end=' ')
        print()

    def setWorkCoord(self):
        """ Set actual coordinates as work coordinates """
        MSG_CODE = 9
        msg = [MSG_CODE, 0, 0, 0, 0, 0, 0, 0]
        self.sendMessage(msg)
        answ = []
        answ = self.port.read(READ_BYTE_NUMB)
        print(len(answ))
        for byte in answ:
            print(hex(byte), end=' ')
        print()

    def moveRefPos(self, param):
        """ Move to absolute zero or work zero position """
        if param == 0:
            MSG_CODE = 7 #absolute zero
        else:
            MSG_CODE = 8 #work zero
        msg = [MSG_CODE, 0, 0, 0, 0, 0, 0, 0]
        self.sendMessage(msg)
        answ = []
        answ = self.port.read(READ_BYTE_NUMB)
        print(len(answ))
        for byte in answ:
            print(hex(byte), end=' ')
        print()

    def doneMFunc(self, m_func):
        """ Confirmation of M-function execution """
        MSG_CODE = 12
        m_func_lo = m_func & 0xFF
        m_func_hi = (m_func >> 8) & 0xFF
        msg = [MSG_CODE, m_func_lo, m_func_hi, 0, 0, 0, 0, 0]
        self.sendMessage(msg)
        answ = []
        answ = self.port.read(READ_BYTE_NUMB)
        print(len(answ))
        for byte in answ:
            print(hex(byte), end=' ')
        print()
        
    def readMessage(self):
        """ Read message from CNC """
        frame = []
        crc = []
        #Slave address
        frame.append(cnc_addr)
        #Function code
        frame.append(FUNC_READ_MSG)
        #Calculate CRC
        crc = self.calcCRC16(frame)
        frame.append(crc[0])
        frame.append(crc[1])
        #Send frame
        self.port.write(frame)
        #Print transmitted frame
        for c in frame:
            print(hex(c), end=' ')
        print()
        #Read answer
        answ = []
        answ = self.port.read(READ_BYTE_NUMB)
        #Print received frame
        for c in answ:
            print(hex(c), end=' ')
        print()
        return answ

    def procInputMessage(self):
        """ Process input message from CNC """
        in_msg = self.readMessage()
        if len(in_msg) != 0:
            if in_msg[1] == FUNC_READ_MSG:
                if in_msg[2] == 1:
                    #Device status
                    stat_word = (in_msg[6] << 24) | (in_msg[5] << 16) \
                                | (in_msg[4] << 8) | in_msg[3]
                    print('Status word: ' + hex(stat_word))
                elif in_msg[2] == 2:
                    #Emergency message
                    err_code = (in_msg[4] << 8) | in_msg[3]
                    sub_err_code = (in_msg[6] << 8) | in_msg[5]
                    print('Error code: ' + hex(err_code))
                    print('Sub-error code: ' + hex(sub_err_code))
                elif in_msg[2] == 3:
                    #M code for processing
                    m_code = (in_msg[4] << 8) | in_msg[3]
                    print('M-code: ' + str(m_code))
                    self.doneMFunc(m_code)
                elif in_msg[2] == 4:
                    #Program number
                    prg_num = (in_msg[4] << 8) | in_msg[3]
                    str_num = (in_msg[8] << 24) | (in_msg[7] << 16) \
                              | (in_msg[6] << 8) | in_msg[5]
                    print('Executing program: ' + str(prg_num), end=' ')
                    print(', string ' + str(str_num))
                elif in_msg[2] == 5:
                    #Program done
                    prg_num = (in_msg[4] << 8) | in_msg[3]
                    print('Program done, # ' + str(prg_num))

    def readRegister(self, reg):
        """ Read CNC register """
        if reg > 0:
            reg -= 1;
        else:
            return
        frame = []
        crc = []
        #Slave address
        frame.append(cnc_addr)
        #Function code
        frame.append(FUNC_READ_HOLD_REG)
        #Starting register address
        reg *= 2
        reg += 508
        addr_lo = reg & 0xFF
        addr_hi = (reg >> 8) & 0xFF
        frame.append(addr_hi)
        frame.append(addr_lo)
        #Quantity of registers = 2
        frame.append(0)
        frame.append(2)
        #Calculate CRC
        crc = self.calcCRC16(frame)
        frame.append(crc[0])
        frame.append(crc[1])
        #Send frame
        self.port.write(frame)
        for c in frame:
            print(hex(c), end=' ')
        print()
        #Read answer
        answ = []
        answ = self.port.read(READ_BYTE_NUMB)
        #Print received frame
        for c in answ:
            print(hex(c), end=' ')
        print()
        value = answ[6] | (answ[5] << 8) | (answ[4] << 16) | (answ[3] << 24)
        return value

    def writeRegister(self, reg, value):
        """ Write CNC register """
        if reg > 0:
            reg -= 1;
        else:
            return
        frame = []
        crc = []
        #Slave address
        frame.append(cnc_addr)
        #Function code
        frame.append(FUNC_WRITE_MULT_REG)
        #Starting register address
        reg *= 2
        reg += 508
        addr_lo = reg & 0xFF
        addr_hi = (reg >> 8) & 0xFF
        frame.append(addr_hi)
        frame.append(addr_lo)
        #Quantity of registers = 2
        frame.append(0)
        frame.append(2)
        #Byte count
        frame.append(4)
        #Data bytes
        frame.append((value >> 24) & 0xFF)
        frame.append((value >> 16) & 0xFF)
        frame.append((value >> 8) & 0xFF)
        frame.append(value & 0xFF)
        #Calculate CRC
        crc = self.calcCRC16(frame)
        frame.append(crc[0])
        frame.append(crc[1])
        #Send frame
        self.port.write(frame)
        for c in frame:
            print(hex(c), end=' ')
        print()
        #Read answer
        answ = []
        answ = self.port.read(READ_BYTE_NUMB)
        #Print received frame
        for c in answ:
            print(hex(c), end=' ')
        print()

    def moveTo(self, x, y, z):
        """ Move to desired point """
        #Write desired positions to CNC registers
        xreg = 1000
        yreg = 1001
        zreg = 1002
        self.writeRegister(xreg, x)
        self.writeRegister(yreg, y)
        self.writeRegister(zreg, z)
        #Send message with command
        MSG_CODE = 2
        xreg_lo = xreg & 0xFF
        xreg_hi = (xreg >> 8) & 0xFF
        yreg_lo = yreg & 0xFF
        yreg_hi = (yreg >> 8) & 0xFF
        zreg_lo = zreg & 0xFF
        zreg_hi = (zreg >> 8) & 0xFF
        msg = [MSG_CODE, xreg_lo, xreg_hi, yreg_lo, yreg_hi, zreg_lo, zreg_hi, 0]
        self.sendMessage(msg)
        answ = []
        answ = self.port.read(READ_BYTE_NUMB)
        print(len(answ))
        for byte in answ:
            print(hex(byte), end=' ')
        print()
