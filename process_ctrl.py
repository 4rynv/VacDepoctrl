from enum import Enum

class SystemState(Enum):

    IDLE = "IDLE"
    PUMP_DOWN = "PUMP DOWN"
    TURBO_SPINUP = "TURBO SPINUP"
    READY = "READY"
    SPUTTERING = "SPUTTERING"
    VENTING = "VENTING"
    FAULT = "FAULT"

class ProcessController:

    def __init__(self):
        self.state = SystemState.IDLE
    def get_state(self):
        return self.state
    def set_state(self, state):
        self.state = state
    def update(self, pirani_data, mfc_data):
        #
        # IDLE
        #
        if self.state == SystemState.IDLE:
            return
        #
        # PUMP DOWN
        #
        elif self.state == SystemState.PUMP_DOWN:
            if pirani_data["adc"] < 9000:
                self.state = SystemState.TURBO_SPINUP
        #
        # TURBO SPINUP
        #
        elif self.state == SystemState.TURBO_SPINUP:
            pass
        #
        # READY
        #
        elif self.state == SystemState.READY:
            pass
        #
        # SPUTTERING
        #
        elif self.state == SystemState.SPUTTERING:
            pass
        #
        # VENTING
        #
        elif self.state == SystemState.VENTING:
            pass
        #
        # FAULT
        #
        elif self.state == SystemState.FAULT:
            pass