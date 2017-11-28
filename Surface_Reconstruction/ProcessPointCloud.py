# from sympy import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time


class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def convert_log_string_to_points(log_string):
    start = time.time()
    log_points = log_string.split('float3')
    end = time.time()
    print_benchmark(start, end, "split")
    start = time.time()
    points = [add_point(p) for p in log_points]
    end = time.time()
    print_benchmark(start, end, "create points")
    return points


def add_point(log_point):
    log_point_list = log_point.split(', ')
    return Point(float(log_point_list[0][1:]), float(log_point_list[1]), float(log_point_list[2][:-1]))


def in_range(val, min_value, max_value):
    return min_value <= val <= max_value


def filter_points(points, x_min=-0.1, x_max=0.05, y_min=-0.2, y_max=0.1, z_min=-0.05, z_max=0.03):

    def in_range_all_dimension(p):
        return in_range(p.x, x_min, x_max) and in_range(p.y, y_min, y_max) and in_range(p.z, z_min, z_max)

    new_points = [p for p in points if in_range_all_dimension(p)]
    return new_points


def plot_points(points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    xs = [p.x for p in points]
    ys = [p.y for p in points]
    zs = [p.z for p in points]

    ax.plot(xs=xs, ys=ys, zs=zs, marker='.', ls='none')
    plt.show()


def write_to_xyz_file(points, file_name='data/bottlePoints.txt'):
    with open(file_name, 'w') as xyz:
        points_string = [add_point_string(p) for p in points]
        xyz.write('\n'.join(points_string))


def add_point_string(point):
    return str(point.x * 1000000000.) + ';' + str(point.y * 1000000000.) + ';' + str(point.z * 1000000000.)


def print_benchmark(start_time, end_time, op_string):
    print(op_string + " took " + str(((end_time - start_time))) + " seconds")


with open('data/bottleLogPoints2.txt', 'r') as file:

    start = time.time()
    data = file.read().replace('\n', '')
    end = time.time()
    print_benchmark(start, end, " read")

    points = convert_log_string_to_points(data)
    print(len(points))
    points = filter_points(points)
    print(len(points))
    start = time.time()
    write_to_xyz_file(points)
    end = time.time()
    print_benchmark(start, end, "write")
    plot_points(points)
