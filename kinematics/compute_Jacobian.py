from math import radians
# start position = [a1, b1], end position = [a2, b2]
def compute_jacobian(start_position_1, end_position_1, start_position_2, end_position_2, delta_theta_1 = radians(5), delta_theta_2 = radians(5)):
     delta_x_1 = end_position_1[0] - start_position_1[0]
     delta_y_1 = end_position_1[1] - start_position_1[1]
     delta_x_2 = end_position_2[0] - start_position_2[0]
     delta_y_2 = end_position_2[1] - start_position_2[1]
     J = [[delta_x_1 / delta_theta_1, delta_x_2 / delta_theta_2],
         [delta_y_1 / delta_theta_1, delta_y_2 / delta_theta_2]]
     return J