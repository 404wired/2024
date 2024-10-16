class Filter():
    """Implements the Alpha Beta Filter, 
    for more information please see this reference;
    https://en.wikipedia.org/wiki/Alpha_beta_filter
    """
    def __init__(self, dt=0.5, alpha=0.012, beta=0.0):
        self.dt = dt
        self.alpha = alpha
        self.beta = beta

        self.reset()

    def filter(self, xm):
        """Filters a signal.

        Args:
            xm (_type_): Signal you want to filter.

        Returns:
            _type_: A filtered signal.
        """
        xk = self.xk_1 + (self.vk_1 * self.dt)
        vk = self.vk_1

        rk = xm - xk

        xk += self.alpha * rk
        vk += (self.beta * rk) / self.dt

        self.xk_1 = xk
        self.vk_1 = vk

        return xk

    def reset(self):
        """Resets the memory of the filter.
        """
        self.xk_1 = 0.0
        self.vk_1 = 0.0
