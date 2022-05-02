from .vehicle import Vehicle

class CarNaive(Vehicle):
    def __init__(self, vehicle_id, init_state):
        super().__init__(vehicle_id, init_state)

    def get_action(self, sensor_data):
        raise NotImplementedError("Action should be provided externally for a naive car.")