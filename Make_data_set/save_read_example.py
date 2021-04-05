import pandas

# write csv
write_data = [1 2 3]
write_path = './ProcessedData/FeatureData'
write_data.to_csv(write_path, sep=',', header=None , index=None)

# read csv
save_path = './Data_process/inclined_%d.csv'%(i+1)
save_data = pd.read_csv(save_path , sep=',' , header=None)