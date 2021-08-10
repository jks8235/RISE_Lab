import pandas as pd

Total_Dada = {'sensor_1_center':{'x':[0.1, 0.2], 'y':[0.1, 0.3], 'z':[0.4, 0.5], 'q_x':[0.2, 0.2], 'q_y':[0.2, 0.7], 'q_z':[0.3, 0.5], 'q_w':[0.1, 0.4], 'd':[0.8, 0.7]},
              'sensor_1_1':{'x':[0.1, 0.2], 'y':[0.1, 0.3], 'z':[0.4, 0.5], 'q_x':[0.2, 0.2], 'q_y':[0.2, 0.7], 'q_z':[0.3, 0.5], 'q_w':[0.1, 0.4], 'd':[0.8, 0.7]},
              'sensor_1_2':{'x':[0.1, 0.2], 'y':[0.1, 0.3], 'z':[0.4, 0.5], 'q_x':[0.2, 0.2], 'q_y':[0.2, 0.7], 'q_z':[0.3, 0.5], 'q_w':[0.1, 0.4], 'd':[0.8, 0.7]},
              'sensor_1_3':{'x':[0.1, 0.2], 'y':[0.1, 0.3], 'z':[0.4, 0.5], 'q_x':[0.2, 0.2], 'q_y':[0.2, 0.7], 'q_z':[0.3, 0.5], 'q_w':[0.1, 0.4], 'd':[0.8, 0.7]}}

Total_Dada['sensor_1_center']['x'].append(1)

print(Total_Dada['sensor_1_center']['x'])

# print(Total_Dada)
df_data = pd.DataFrame.from_dict(Total_Dada, orient='index')

print(df_data)

a = df_data.loc['sensor_1_center','q_w']

# print(type(a))

# def _save_data_as_csv(data, name):
#     path = '/home/jee/work_space/catkin_wk/src/RISE_Lab/Sensor_Learning/' + name + '.csv'
#     data.to_csv(path, sep=',', header=None, index=None)

# _save_data_as_csv(df_data, 'test')


# df = pd.DataFrame.from_dict(d, orient='index')
# print (df)
#         something1 something2 something3
# David X    string1    string2    string3
# David Y       str1       str2       str3
 