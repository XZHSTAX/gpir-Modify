#!/usr/bin/env python3

from scipy.signal import butter, lfilter, lfilter_zi

class LowPassFilterStream:
    """
    A class to implement a real-time low-pass filter for streaming data.
    """
    def __init__(self, cutoff, fs, order=4):
        """
        Initializes the low-pass filter.

        Args:
            cutoff (float): The cutoff frequency of the filter.
            fs (float): The sampling frequency of the data.
            order (int, optional): The order of the filter. Defaults to 4.
        """
        self.b, self.a = butter(order, cutoff / (0.5 * fs), btype='low')
        self.zi = lfilter_zi(self.b, self.a)
        self.last_output = []

    def filter(self, x_new):
        """
        Filters a new data point.

        Args:
            x_new (float): The new data point to filter.

        Returns:
            float: The filtered data point.
        """
        y, self.zi = lfilter(self.b, self.a, [x_new], zi=self.zi)
        self.last_output.append(y[0])
        return y[0]