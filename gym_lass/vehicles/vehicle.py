from abc import abstractmethod

# class VehicleState:
#     def __init__(self, x, y, h, speed, s, t, lane_id, lane_offset, road_id, junction_id):
#         self.x = x
#         self.y = y
#         self.h = h
#         self.speed = speed
#         self.s = s
#         self.t = t
#         self.lane_id = lane_id
#         self.lane_offset = lane_offset
#         self.road_id = road_id
#         self.junction_id = junction_id


class Vehicle:
    def __init__(self, vehicle_id, init_state):
        self._id = vehicle_id
        self._main_controller = None
        self._co_controller = None
        self._state = init_state

    def update_state(self, state):
        self._state = state

    @abstractmethod
    def get_action(self, sensor_data):
        pass

    @property
    def id(self):
        return self._id

    @property
    def state(self):
        return self._state
