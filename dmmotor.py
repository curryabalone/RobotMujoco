import numpy as np
class motor:
    def __init__(self, mu_static, mu_viscious, mu_coulumb):
        self.mu_static = mu_static
        self.mu_viscious = mu_viscious
        self.mu_coulumb = mu_coulumb
    def friction_torque(self, velocity, shaft_load, external_torque):
        if velocity == 0 and external_torque < self.mu_static * shaft_load:
            return self.mu_static * shaft_load * np.sign(external_torque) #static friction should always oppose the external torque
        else:
            return 0 #change this later