import numpy as np
import math
import pandas as pd

class ToF_Sensor:
    def __init__(self, distance, FOV=25, resolution=5):
        self.FOV = FOV
        self.resolution = resolution
        self.unit_angle = self.FOV/self.resolution
        self.distance = distance

        self.Map_size = 440
        self.Map = np.zeros([self.Map_size, self.Map_size])
        self.Map[:int(self.Map_size/2),:] = self.distance[0]
        self.Map[int(self.Map_size/2):,:] = self.distance[1]

        self.FOV_distance = np.zeros([self.resolution, self.resolution])
        self.FOV_distance_angle = np.zeros([self.resolution, self.resolution])
        self.FOV_distance_multiplier = np.zeros(self.FOV_distance.shape)
        self.Position = [0, 0, 0]
        self.mid_point = int(math.ceil(self.resolution/2.0))

        for i in range(0, self.resolution):
            for j in range(0, self.resolution):
                self.FOV_distance_angle[i,j] = self.unit_angle*(math.sqrt((i+1-self.mid_point)**2 + (j+1-self.mid_point)**2))

        for i in range(0, self.resolution):
            for j in range(0, self.resolution):
                self.FOV_distance_multiplier[i,j] = 1/math.cos(self.deg2rad(self.FOV_distance_angle[i,j]))

        off_set = np.mean(self.FOV_distance_multiplier) - 1
        self.FOV_distance_multiplier = self.FOV_distance_multiplier - off_set*np.ones(self.FOV_distance_multiplier.shape)

    def get_distance(self):
        distnace = np.mean(self.FOV_distance)
        return distnace

    def get_FOV_distance(self, position):
        d = min(self.distance) - position[2]

        for i in range(0, self.resolution):
            for j in range(0, self.resolution):
                self.FOV_distance[i,j] = self.Map[position[0]+int(math.ceil(d*math.tan(self.deg2rad(self.unit_angle*(i+1-self.mid_point))))), position[1]+int(math.ceil(d*math.tan(self.deg2rad(self.unit_angle*(j+1-self.mid_point)))))]
        
        self.FOV_distance += position[2]*np.ones(self.FOV_distance.shape)
        self.FOV_distance = self.FOV_distance*self.FOV_distance_multiplier
        return self.FOV_distance

    def deg2rad(self, deg):
        return deg/180.0*math.pi

    def set_start(self):
        r_1 = int(math.ceil(self.distance[0]*math.tan(self.deg2rad(self.FOV/2))))
        r_2 = int(math.ceil(self.distance[1]*math.tan(self.deg2rad(self.FOV/2))))

        x_s = r_1
        y_s = self.Map.shape[1]/2
        z_s = 0

        self.Move_range = [[200, self.Map_size-200], [200, self.Map_size-200], [max(d[0], d[1])-850, max(d[0], d[1])-50]]
        self.Position = [x_s, y_s, z_s]

if __name__ == "__main__":

    NN_input = []
    NN_output = []

    for d_1 in range(30, 850, 20):
        for d_2 in range(30, 850, 20):
            d = [d_1, d_2]

            sensor = ToF_Sensor(d)
            sensor.set_start()
    
            for x in range(sensor.Move_range[0][0], sensor.Move_range[0][1], 10):
                for y in range(sensor.Move_range[1][0], sensor.Move_range[1][1], 10):
                    for z in range(sensor.Move_range[2][0], sensor.Move_range[2][1], 20):
                        FOV = sensor.get_FOV_distance([x, y, z])
                        distance = sensor.get_distance()

                        temp_input = [x, y, z, distance]
                        temp_output = []

                        for i in range(0, sensor.resolution):
                            for j in range(0, sensor.resolution):
                                temp_output.append(FOV[j, sensor.resolution-i-1])

                        NN_input.append(temp_input)
                        NN_output.append(temp_output)

                        print(d_1, d_2, x, y, z)
                        # print(sensor.Move_range)
                        # print(FOV)

    data_input = pd.DataFrame(NN_input)
    data_output = pd.DataFrame(NN_output)

    # write csv
    write_path_input = '/home/jee/catkin_ws/src/RISE_Lab/Make_data_set/data/input.csv'
    write_path_output = '/home/jee/catkin_ws/src/RISE_Lab/Make_data_set/data/output.csv'
    data_input.to_csv(write_path_input, sep=',', header=None , index=None)
    data_output.to_csv(write_path_output, sep=',', header=None, index=None)
    print ('done')


