from .vehicle import Vehicle
from gym_lass.algos import LKS_PID


class CarLKS(Vehicle):
    def __init__(self, vehicle_id, init_state, lateral_controller: LKS_PID):
        super().__init__(vehicle_id, init_state)
        self._co_controller = lateral_controller

    def get_action(self, throttle):
        _, steer = self._co_controller.step(self.state)
        return [throttle, steer]
