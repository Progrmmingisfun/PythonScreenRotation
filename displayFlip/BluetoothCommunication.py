from BaseCommunication import BaseCommunication
import bluetooth

class BluetoothCommunication(BaseCommunication):

    def listen(self):
        btDevice = self.__connect()
        while True:
            data = btDevice.recv(10)
            if data == False:
                continue
            print("received: " + data)
            self.get_callback()(data, self.get_extra_parameters())

    def __connect(self):
        btDevice = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        btDevice.settimeout(None)
        btDevice.connect((self.get_endpoint(), 1))

        return btDevice