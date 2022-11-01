import serial
from BaseCommunication import BaseCommunication

class SerialCommunication(BaseCommunication):
    def listen(self):
        serialport = self.__connect()
        while True:
            data = serialport.read()
            if data == False or data == "":
                continue
            print("received: " + data.decode('utf-8'))
            self.get_callback()(data.decode('utf-8'), self.get_extra_parameters())

    def __connect(self):
        serialport = serial.Serial(self.get_endpoint(), 115200, timeout=0.5)

        return serialport