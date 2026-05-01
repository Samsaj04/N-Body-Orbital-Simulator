class Body:
    def __init__(self, position, velocity, mass):
        self.position = position
        self.velocity = velocity
        self.mass = mass

class Propulsion:
    def __init__(self, tf, dVx, dVy, dVz):
        self.tf = tf
        self.dVx = dVx
        self.dVy = dVy
        self.dVz = dVz