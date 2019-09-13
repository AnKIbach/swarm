#!/usr/bin/env python
import matplotlib.pyplot as plt

class plotter:
    def __init__(self):
        self.title = 'Klikkaboisa'
        self.x_label = 'clicks'
        self.y_label = 'speed'
        self.curve1 = 'speed out'
        self.curve2 = 'speed actual'
        self.curve3 = 'angle out'
        self.curve4 = 'angle actual'

    def present(self, speed_out, speed_actual, angle_out, angle_actual):
        plt.plot(speed_out, label = self.curve1)
        plt.plot(speed_actual, label = self.curve2)
        plt.plot(angle_out, label = self.curve3)
        plt.plot(angle_actual, label = self.curve4)
        plt.xlabel(self.x_label)
        plt.ylabel(self.y_label)
        plt.title(self.title)
        plt.legend()
        plt.show()

