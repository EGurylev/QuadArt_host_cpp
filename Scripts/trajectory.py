import numpy as np
import matplotlib.pyplot as plt
import sys
import cv2
import csv
from scipy.spatial.distance import pdist, squareform

#Dmitry Shintyakov shintyakov@gmail.com
from tsp_solver.greedy_numpy import solve_tsp

#### Approximate image by set of points

N_pts_flight = 50
N_flights = 10
N_pts = N_flights * N_pts_flight

x_range = 75.0 #cm

img = cv2.imread('linux.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#Vanish light regions
gray[gray > 125] = 255
#Make gray image negative
gray_neg = 255 - gray

x_size = gray_neg.shape[1]
y_size = gray_neg.shape[0]
#Reshape image into 1-D array 
arr_size = x_size * y_size
gray_r = gray_neg.reshape(arr_size, 1)
gray_r = gray_r.squeeze()
#Create array of probabilities proportional to gray level of each pixel
prob = gray_r / 255.0
prob /= prob.sum()#make them sum up to 1
#Choose N_pts pixels according to their probabilities
pts_p = np.random.choice(xrange(0, arr_size), size=N_pts, p=prob)
#Convert 1-D array to x and y coordinates of approximated image
x = pts_p % x_size
y = pts_p / x_size

points = np.vstack((x,y))

#Scale point coordinates into physical range
y_range = float(y_size) / float(x_size) * x_range
points = points.astype('float')
points[0] = points[0] * (x_range / x_size) - x_range / 2
points[1] = -(points[1] * (y_range / y_size) - y_range / 2)

#### Solve travelling salesman problem for optimal trajectory
# Make symmetric distance matrix from points

dist_matrix = squareform(pdist(points.T))
path = solve_tsp(dist_matrix)
#Reorder points according to optimal path
points = points.T[path]

#### Make trajectory and write it to file

dt = 0.01 #sec
time_for_point = 2 #sec
total_time = time_for_point * N_pts
num_rec = int(total_time / dt)


# Y coordinate is constant
y_t = 90 * np.ones((num_rec))

x_t = np.array([])
z_t = np.array([])
num_rec_pt = int(time_for_point / dt)

for point in points:
	x_i = point[0] * np.ones(num_rec_pt) 
	z_i = point[1] * np.ones(num_rec_pt)
	x_t = np.concatenate((x_t, x_i), axis=1)
	z_t = np.concatenate((z_t, z_i), axis=1)
	
time_t = np.linspace(0, time_for_point * N_pts_flight, N_pts_flight * num_rec_pt)

plt.plot(points.T[0], points.T[1], 'o')
plt.axis('equal')

# Form a data matrix in order to simplify writing into files
for n in xrange(N_flights):
	file_name = 'trajectory_' + str(n)
	keys = ['time', 'x', 'y', 'z']
	matrix_h = N_pts_flight * num_rec_pt
	matrix_w = len(keys)
	data_matrix = np.zeros((matrix_h, matrix_w))
	data_matrix[:, 0] = time_t
	data_matrix[:, 1] = x_t[num_rec_pt * n * N_pts_flight : num_rec_pt * (n + 1) * N_pts_flight]
	data_matrix[:, 2] = y_t[num_rec_pt * n * N_pts_flight : num_rec_pt * (n + 1) * N_pts_flight]
	data_matrix[:, 3] = z_t[num_rec_pt * n * N_pts_flight : num_rec_pt * (n + 1) * N_pts_flight]
	with open(file_name, 'wb') as csvfile:
		writer = csv.writer(csvfile, delimiter=' ')
		writer.writerow([matrix_h])# number of records for reader
		#writer.writerow(keys)# header row
		for log_slice in xrange(matrix_h):
		    writer.writerow(data_matrix[log_slice,:])
