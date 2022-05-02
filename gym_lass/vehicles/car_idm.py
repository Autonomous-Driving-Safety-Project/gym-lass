from .vehicle import Vehicle
from gym_lass.algos import IDM, LKS_PID

class CarIDM(Vehicle):
    def __init__(self, vehicle_id, init_state, idm_controller: IDM, lateral_controller: LKS_PID):
        super().__init__(vehicle_id, init_state)
        self._main_controller = idm_controller
        self._co_controller = lateral_controller

    def get_action(self, sensor_data):
        ideal_sensor = [v for v in sensor_data if v[0] == 'IdealSensor']
        # TODO: calculate lateral distance
        ideal_sensor_data = []
        for data in ideal_sensor:
            ideal_sensor_data += data[2]
        throttle, _ = self._main_controller.step(self.state, ideal_sensor_data)
        _, steer = self._co_controller.step(self.state)
        return [throttle, steer]
