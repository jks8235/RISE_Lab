#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Kyungsub Jee (RISE)

import numpy as np
import math
import random

def scaling(list_data):
    list_num = len(list_data)
    absoute_sum = 0
    # scalled_data = [0 for num in range(list_num)]
    for i in list_data:
        absoute_sum += abs(i)

    scalled_data = [float(list_data[num])/float(absoute_sum) for num in range(list_num)]

    return scalled_data
        

    # return float(list_data/absoute_sum)



class Walker():
    def __init__(self):
        self.Position = [0.0, 0.0, 0.0]
        self.Step_size = 0.01                       # unit is (m)

    def random_work(self, iter=3):
        for i in range(iter):

            random_ratio = [random.randrange(-100,101), random.randrange(-100,101), random.randrange(-100,101)]

            del_position = scaling(random_ratio)

            print(self.Position, del_position)


            self.Position = [round((i + j*self.Step_size),5) for i, j in zip(self.Position, del_position)]





if __name__ == "__main__":

    walker_1 = Walker()

    walker_1.random_work()