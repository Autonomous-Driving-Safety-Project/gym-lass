from abc import abstractmethod


class Algorithm:
    @abstractmethod
    def step(self, *args, **kwargs):
        """get throttle and steer

        Parameters:

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
