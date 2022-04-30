from abc import abstractmethod


class Algorithm:
    @abstractmethod
    def step(self, ego_state, **kargs):
        """get throttle and steer

        Parameters:
            ego_state (Lass.ObjectState): ego state

        Returns:
            throttle (float), steer (float): throttle and steer of ego
        """
        pass

    @property
    @abstractmethod
    def longitudinal(self):
        return False

    @property
    @abstractmethod
    def lateral(self):
        return False
