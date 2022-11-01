class BaseCommunication:
    def __init__(self):
        self.__endpoint = None
        self.__callback = None
        self.__extra_parameters = None

    def set_endpoint(self, endpoint):
        self.__endpoint = endpoint

    def get_endpoint(self):
        return self.__endpoint

    def set_callback(self, callback):
        self.__callback = callback

    def get_callback(self):
        return self.__callback

    def set_extra_parameters(self, parameters):
        self.__extra_parameters = parameters

    def get_extra_parameters(self):
        return self.__extra_parameters