from .vehicle import Vehicle
from gym_lass.algos import IDM, MOBIL
import sys


class CarMOBIL(Vehicle):
    def __init__(self, vehicle_id, init_state, longitudinal_controller: IDM, mobil_controller: MOBIL):
        super().__init__(vehicle_id, init_state)
        self._main_controller = longitudinal_controller
        self._co_controller = mobil_controller

    def get_action(self, sensor_data, full_state):
        ideal_sensor = [v for v in sensor_data if v[0] == 'IdealSensor']
        # TODO: calculate lateral distance
        ideal_sensor_data = []
        for data in ideal_sensor:
            ideal_sensor_data += data[2]
        throttle, _ = self._main_controller.step(self.state, ideal_sensor_data)

        road_sensor = [v for v in sensor_data if v[0] == 'RoadSensor']
        if road_sensor[0][1] == 0:
            return None # off-road

        _, steer = self._co_controller.step(self.state, road_sensor[0][2][0], full_state)

        return [throttle, steer]
