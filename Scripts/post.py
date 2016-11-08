'''
Script for obtaining some control quality statistics:
dynamic and static errors.
'''

import numpy as np
import matplotlib.pyplot as plt
import sys
import csv

plt.ion()

# Import log file with telemetry data from CF
# Write log file name as parameter
file_name = str(sys.argv[1])
with open(file_name, 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    records_shape = map(int, reader.next())
    num_rec = records_shape[0]
    data_matrix = np.zeros(records_shape)
    field_names = reader.next()
    for rec in xrange(num_rec):
        data_matrix[rec,:] = map(float, reader.next())


time = data_matrix[:, field_names.index('time')] / 1000 #in ms
roll_set = data_matrix[:, field_names.index('roll_set')]
pitch_set = data_matrix[:, field_names.index('pitch_set')]
roll_cf = data_matrix[:, field_names.index('roll_cf')]
pitch_cf = data_matrix[:, field_names.index('pitch_cf')]
yaw_cf = data_matrix[:, field_names.index('yaw_cf')]
time_cf = data_matrix[:, field_names.index('time_cf')]#in 
marker_found = data_matrix[:, field_names.index('marker_found')]
thrust_set = data_matrix[:, field_names.index('thrust_set')]
is_pose_valid = data_matrix[:, field_names.index('is_pose_valid')]

x = data_matrix[:, field_names.index('x')]
y = data_matrix[:, field_names.index('y')]
z = data_matrix[:, field_names.index('z')]

x_set = data_matrix[:, field_names.index('x_set')]
y_set = data_matrix[:, field_names.index('y_set')]
z_set = data_matrix[:, field_names.index('z_set')]

# In case of absence observed values in log
try:
	x_obs = data_matrix[:, field_names.index('x_obs')]
	y_obs = data_matrix[:, field_names.index('y_obs')]
	z_obs = data_matrix[:, field_names.index('z_obs')]
except Exception:
	x_obs = x
	y_obs = y
	z_obs = z


#Synchronize time from PC with time from CF
#Find moment in time when cf is connected and send real data
for i in xrange(1, num_rec):
	if roll_cf[i] != 0 and roll_cf[i - 1] == 0:
		time1 = time[i]
		time_cf1 = time_cf[i]
		
time_diff = time_cf1 - time1

time_cf -= time_diff
mask = time_cf >= 0
time_cf = time_cf[mask]
roll_cf = roll_cf[mask]
pitch_cf = pitch_cf[mask]
yaw_cf = yaw_cf[mask]

# Remove Nans
x_set[np.isnan(x_set)] = 0
y_set[np.isnan(y_set)] = 0
z_set[np.isnan(z_set)] = 0

	
plt.figure()
ax1 = plt.subplot(2,1,1)
ax1.plot(time_cf, pitch_cf)
ax1.plot(time, pitch_set)
ax2 = plt.subplot(2,1,2, sharex=ax1)
ax2.plot(time, x)
ax2.plot(time, x_set)
ax2.plot(time, x_obs)

dx_set = np.diff(x_set)
idx = dx_set != 0
x_end_set = x_set[idx]
y_end_set = y_set[idx]
z_end_set = z_set[idx]
x_end_real = x_obs[idx]
z_end_real = z_obs[idx]
time_set = time[idx]


x_end_set = x_end_set[1:-1]
z_end_set = z_end_set[1:-1]
x_end_real = x_end_real[1:-1]
z_end_real = z_end_real[1:-1]
time_set = time_set[1:-1]

#plt.plot(time, x_set)
#plt.plot(time_set, x_end_set, 'o')
#plt.plot(time_set, x_end_real, 'o')
#plt.plot(time, x_obs)

#plt.figure()
#plt.plot(time, z_set)
#plt.plot(time_set, z_end_set, 'o')
#plt.plot(time_set, z_end_real, 'o')
#plt.plot(time, z_obs)

################# Statistics #################

#Calc. persentage of valid pose estimations
p_pose_valid = 100 * is_pose_valid.sum() / is_pose_valid.size
print "Persentage of valid pose estimations is:"
print p_pose_valid

# Calculate performance of feedback control

# Dynamic error
# Do not account first set (before launch)
idx_dyn = np.ones(idx.shape, dtype=np.bool)
for i in xrange(idx_dyn.shape[0]):
	if not idx[i]:
		idx_dyn[i] = False
	else:
		break

x_err2 = sum(pow(x_obs[idx_dyn] - x_set[idx_dyn], 2))
y_err2 = sum(pow(y_obs[idx_dyn] - y_set[idx_dyn], 2))
z_err2 = sum(pow(z_obs[idx_dyn] - z_set[idx_dyn], 2))

print "Integral of squared error in x"
print x_err2
print "Integral of squared error in y"
print y_err2
print "Integral of squared error in z"
print z_err2


# Static error
x_err_static = abs(x_end_set - x_end_real)
z_err_static = abs(z_end_set - z_end_real)

print "Max, mean and sum of static error in x"
print [x_err_static.max(), x_err_static.mean(), x_err_static.sum()]
print "Max, mean and sum of static error in z"
print [z_err_static.max(), z_err_static.mean(), z_err_static.sum()]
