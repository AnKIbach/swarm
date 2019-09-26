#!/usr/bin/env python
import matplotlib.pyplot as plt

class Plotter:
    def __init__(self):
        self.title = 'Klikkaboisa'
        self.x_label = 'clicks'
        self.y_label = 'speed'
        self.curveNames =['s actual',
                        's wanted',
                        's out',
                        'a actual'
                        'a wanted',
                        'a out']

    def present(self, *args):
        thingList = []
        for things in args:
            thingList.append(things)

        for idx,toList in enumerate(thingList):
            try:
                plt.plot(toList, label = self.curveNames[idx])
            except:
                pass

        plt.xlabel(self.x_label)
        plt.ylabel(self.y_label)
        plt.title(self.title)
        plt.legend()
        plt.show()

