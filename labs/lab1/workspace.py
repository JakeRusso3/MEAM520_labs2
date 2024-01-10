from lib.calculateFK import FK
from core.interfaces import ArmController

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fk = FK()

# the dictionary below contains the data returned by calling arm.joint_limits()
limits = [
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -1.7628, 'upper': 1.7628},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -3.0718, 'upper': -0.0698},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -0.0175, 'upper': 3.7525},
    {'lower': -2.8973, 'upper': 2.8973}
 ]

# TODO: create plot(s) which visualize the reachable workspace of the Panda arm,
# accounting for the joint limits.
#
# We've included some very basic plotting commands below, but you can find
# more functionality at https://matplotlib.org/stable/index.html

num_samples = 1000
end_positions = []
sample_angles = []
this_sample = []
for ii in num_samples:
    this_sample = []
    for jj in limits:
        this_sample.append(np.random.uniform(ii['lower'], ii['upper'], 1)
    this_joint, thisT0e = fk.forward(this_sample)
    end_positions.append(thisT0e[:3,3])

final_end_effector_positions = np.array(end_effector_positions)    
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# TODO: update this with real results
ax.scatter(1,1,1) # plot the point (1,1,1)

plt.show()
