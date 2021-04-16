import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

input_data = pd.DataFrame(columns=range(5))
verify_data = pd.DataFrame(columns=range(25))
output_data = pd.DataFrame(columns=range(1))

for i in range(0, 4):
    edge_input_path = './data/edge_input_%d.csv'%(i+1)
    edge_verify_path = './data/edge_verify_%d.csv'%(i+1)
    edge_output_path = './data/edge_output_%d.csv'%(i+1)
    
    edge_temp_input = pd.read_csv(edge_input_path , sep=',' , header=None)
    edge_temp_verify = pd.read_csv(edge_verify_path , sep=',' , header=None)
    edge_temp_output = pd.read_csv(edge_output_path , sep=',' , header=None)
    
    input_data = np.concatenate([input_data, edge_temp_input], axis=0)
    verify_data = np.concatenate([verify_data, edge_temp_verify], axis=0)
    output_data = np.concatenate([output_data, edge_temp_output], axis=0)
    print(i)
print('edge data done')
    
for i in range(0, 4):
    conner_input_path = './data/conner_input_%d.csv'%(i+1)
    conner_verify_path = './data/conner_verify_%d.csv'%(i+1)
    conner_output_path = './data/conner_output_%d.csv'%(i+1)
    
    conner_temp_input = pd.read_csv(conner_input_path , sep=',' , header=None)
    conner_temp_verify = pd.read_csv(conner_verify_path , sep=',' , header=None)
    conner_temp_output = pd.read_csv(conner_output_path , sep=',' , header=None)
    
    input_data = np.concatenate([input_data, conner_temp_input], axis=0)
    verify_data = np.concatenate([verify_data, conner_temp_verify], axis=0)
    output_data = np.concatenate([output_data, conner_temp_output], axis=0)
    print(i)
print('conner data done')
    
#     print
input_data = pd.DataFrame(input_data)
verify_data = pd.DataFrame(verify_data)
output_data = pd.DataFrame(output_data)

print(input_data)
print(verify_data)
print(output_data)