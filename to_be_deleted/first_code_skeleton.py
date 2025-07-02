class Game:
    # will include walls, gadgets (which gadget on which wall if possible and necessary)
    # i.e. two lists or a map from walls to gadgets
    # will keep commands to comm. with MCs
    # listen sensors
    # must include a universal code game logic API
    # can convert API language to a main loop
    pass


class Wall:
    def __init__(self):
        self.actuator_gadgets = []
        self.sensor_gadgets = []


class Gadget:
    # every actuator has a microcontroller inside
    # every sensor connects to a microcontroller by daisy-chaining

    # in Game, when an MC first communicates it will send some string, parse and identify it
    def __init__(self, handShakeString):
        self.commands = {}  # and parameters, most possibly be used to send actuate commands
        self.id = None
        self.name = None
        self.parseHandShake(handShakeString)

    def parseHandShake(self, string):
        # do something
        out = [string]
        self.id = out[0]
        self.name = out[1]
        self.commands = out[2]

    def getCommands(self):
        return self.commands

    def sendCommand(self, command, *params):
        # send command to MC through serial or wireless
        pass


class SensorGadget(Gadget):
    def __init__(self, handShakeString):
        super().__init__(handShakeString)


class ActuatorGadget(Gadget):
    def __init__(self, handShakeString):
        super().__init__(handShakeString)
        self.connected_actuators = []  # must be ordered to define a proper chain
