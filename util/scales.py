import pandas as pd
import matplotlib.pyplot as plt
import math
import numpy as np

seq = "06"
ground_truth_filename = "/ext/data/kitti/odometry/dataset/poses/"+ seq + ".txt"

df = pd.read_csv("tx.txt", header=None)

df_truth = pd.read_csv(ground_truth_filename, sep=' ', header=None)
df_truth = df_truth[[3,7,11]]
df_truth.rename(columns={3:'x', 7:'y', 11:'z'}, inplace=True)
df_truth['translation'] = 1
for i in range(1, len(df_truth)):
    tx = math.pow(df_truth.iloc[i]['x'] - df_truth.iloc[i-1]['x'], 2) + math.pow(df_truth.iloc[i]['y'] - df_truth.iloc[i-1]['y'], 2) + math.pow(df_truth.iloc[i]['z'] - df_truth.iloc[i-1]['z'], 2) 
    df_truth.ix[i, 'translation'] = math.sqrt(tx)

# import pdb;pdb.set_trace()

ax =  plt.gca()
df_truth['translation'].plot(ax = ax)
# import pdb;pdb.set_trace()

df.plot.scatter(x = 0, y = 1, ax = ax, c ='r')
plt.yticks(np.arange(0, 1.8, 0.1))

plt.grid()
plt.show()


