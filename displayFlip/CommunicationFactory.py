from BluetoothCommunication import BluetoothCommunication
from SerialCommunication import SerialCommunication


class CommunicationFactory:
    @staticmethod
    def create_communicator(type):
        if type == 'bluetooth':
            return BluetoothCommunication()
        elif type == 'serial':
            return SerialCommunication()
        else:
            raise Exception('Unkonwn comminicator type', type)