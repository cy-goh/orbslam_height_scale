import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sb
import os
sb.set()


# seq = "00"
# seq = "04"
seq = "06"
# seq = "07"
orb_lcse_filename = "result/KeyFrameTrajectory-LCSE-" + seq + ".txt"
ground_truth_filename = "/media/sgp1053c/DATA/steven/kitti-dataset/poses/"+ seq + ".txt"

f, ax = plt.subplots()

df_orb_lcse = pd.read_csv(os.path.join(os.getcwd(),orb_lcse_filename), sep=' ', header=None)
df_orb_lcse = df_orb_lcse[[1,3]]
df_orb_lcse['Type'] = "ORB SLAM with LC & SE"
df_orb_lcse.rename(columns={1:"x", 3:"z"}, inplace=True)

df_truth = pd.read_csv(ground_truth_filename, sep=' ', header=None)
df_truth = df_truth[[3,11]]
df_truth["Type"] = "Ground Truth"
df_truth.rename(columns={3:"x", 11:"z"}, inplace=True)


df_combine = pd.concat([df_orb_lcse, df_truth])

sb.lineplot(x="x", y="z", hue="Type", data=df_combine, sort=False)

ax.legend()
plt.axis('equal')
plt.title("Sequence Number " + seq)
plt.show()


# time_filename = "/media/sgp1053c/DATA/steven/ORB_SLAM2/TimeTrack-" + seq + ".txt"
#
# df_time = pd.read_csv(time_filename, sep=' ', header=None, index_col=0)
# # print(df_time.describe())
# df_time.plot()
# plt.show()

# scale_filename = "/media/sgp1053c/DATA/steven/ORB_SLAM2/Scale-" + seq + ".txt"
#
# df_scale = pd.read_csv(scale_filename, sep=' ', header=None, index_col=0)
# # print(df_scale.describe())
# df_scale.plot()
# plt.show()

