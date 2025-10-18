from math import radians
# start position = [a1, b1], end position = [a2, b2]
def compute_jacobian(start_position, end_position, delta_theta_1 = radians(5), delta_theta_2 = radians(5)):
    delta_x = end_position[0] - start_position[0]
    delta_y = end_position[1] - start_position[1]
    J = [[delta_x / delta_theta_1, delta_x / delta_theta_2],
         [delta_y / delta_theta_1, delta_y / delta_theta_2]]
    return J