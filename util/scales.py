import pandas as pd
import matplotlib.pyplot as plt
import math
import numpy as np
import sys

seq = sys.argv[1]
#ground_truth_filename = "/ext/data/kitti/odometry/dataset/poses/"+ seq + ".txt"
ground_truth_filename = "/home/cy/Documents/data/kitti/data_odometry_poses/"+ seq + ".txt"


df_truth = pd.read_csv(ground_truth_filename, sep=' ', header=None)
df_truth = df_truth[[3,7,11]]
df_truth.rename(columns={3:'x', 7:'y', 11:'z'}, inplace=True)
df_truth['translation'] = 1.
for i in range(1, len(df_truth)):
    tx = math.pow(df_truth.iloc[i]['x'] - df_truth.iloc[i-1]['x'], 2) + math.pow(df_truth.iloc[i]['y'] - df_truth.iloc[i-1]['y'], 2) + math.pow(df_truth.iloc[i]['z'] - df_truth.iloc[i-1]['z'], 2) 
    df_truth.at[i, 'translation'] = math.sqrt(tx)

import pdb; pdb.set_trace()

df = pd.read_csv("tx.txt", header=None)
df['avg'] = df.iloc[:,1].rolling(window=6).median()

ax =  plt.gca()
df_truth['translation'].plot(ax = ax)
# import pdb;pdb.set_trace()

df.plot.scatter(x = 0, y = 1, ax = ax, c ='r')
df.plot(x=0, y='avg', ax=ax)
plt.yticks(np.arange(0, 1.8, 0.1))
plt.ylim(0, 2.)
plt.title("Dingfu Scale Estimation of (Almost) Every Frame for Seq6 and Moving median of Window size 6")

plt.grid()
plt.show()


