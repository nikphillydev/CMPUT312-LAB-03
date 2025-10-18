from math import sqrt, sin, cos, acos, radians, degrees
from typing import List, Optional

def get_angle_between_two_lines(point1: List[float], point2: List[float], point3: List[float]) -> float:
    """Return the angle (deg) between the lines point1->point2 and point1->point3."""
    vector1 = tuple(x-y for x, y in zip(point1, point2))
    vector2 = tuple(x-y for x, y in zip(point1, point3))
    numerator = (vector1[0] * vector2[0] + vector1[1] * vector2[1])
    denominator = sqrt(vector1[0] * vector1[0] + vector1[1] * vector1[1]) * sqrt(vector2[0] * vector2[0] + vector2[1] * vector2[1])
    angle = acos(numerator / denominator)
    return degrees(angle)

def get_distance_between_two_points(point1, point2) -> float:
    """Return the distance between point1 (x1, y1) and point2 (x2, y2)."""
    x_term = (point2[0] - point1[0])**2
    y_term = (point2[1] - point1[1])**2
    return sqrt(x_term + y_term)

def compute_jacobian(lengths: List[float], angles: List[float]) -> List[List[float]]:
    """Compute and return the 2x2 Jacobian matrix of a 2D robot arm with link lengths (cm) and joint angles (radians)."""
    l1, l2 = lengths
    theta1, theta2 = angles[0], angles[1]
    row1 = [-l1*sin(theta1) - l2*sin(theta1+theta2), -l2*sin(theta1+theta2)]
    row2 = [ l1*cos(theta1) + l2*cos(theta1+theta2),  l2*cos(theta1+theta2)]
    return [row1, row2]

def invert_2x2_matrix(matrix: List[List[float]]) -> Optional[List[List[float]]]:
    """Compute and return the inverse of a 2x2 matrix, if it exists."""
    a, b = matrix[0]
    c, d = matrix[1]
    det = a*d - b*c
    if det == 0:
        print("ERROR: 2x2 matrix inverse does not exist! Returning None")
        return None
    inv = [[ d/det, -b/det],
           [-c/det,  a/det]]
    return inv

def lerp(v0: float, v1: float, t: float):
    """Linearly interpolate between v0 and v1 with parameter t [0, 1]"""
    return (1 - t) * v0 + t * v1

def matrix_multiplication(matrix1, matrix2):
    '''multiply two 2*2 matrix'''
    a1, b1 = matrix1[0]
    c1, d1 = matrix1[1]
    a2, b2 = matrix2[0]
    c2, d2 = matrix2[1]
    mul = [[a1 * a2 + b1 * c2, a1 * b2 + b1 * d2],
           [c1 * a2 + d1 * c2, c1 * b2 + d1 * d2]]
    return mul

def matrix_4_to_2_mul(matrix1, matrix2):
    a1, b1 = matrix1[0]
    c1, d1 = matrix1[1]
    a2 = matrix2[0]
    b2 = matrix2[1]
    mul = [a1 * a2 + b1 * b2, c1 * a2 + d1 * b2]
    return mul

def matrix_2_to_2_mul(matrix1, matrix2):
    a1 = matrix1[0]
    b1 = matrix1[1]
    a2 = matrix2[0]
    b2 = matrix2[1]
    mul = [[a1 * a2, a1 * b2], [b1 * a2, b1 * b2]]
    return mul

def matrix_transpose(matrix):
    '''transpose the matrix'''
    a, b = matrix[0]
    c, d = matrix[1]
    tran = [[a,c],[b,d]]
    return tran

def matrix_addition(matrix1, matrix2):
    '''add up two 2 * 2 matrix'''
    a1, b1 = matrix1[0]
    c1, d1 = matrix1[1]
    a2, b2 = matrix2[0]
    c2, d2 = matrix2[1]
    add = [[a1 + a2, b1 + b2], [c1 + c2, d1 + d2]]
    return add
def matrix_subtraction(matrix1, matrix2):
    '''subtract two 2 * 1 matrix'''
    a1= matrix1[0]
    b1 = matrix1[1]
    a2 = matrix2[0]
    b2= matrix2[1]
    sub = [a1 - a2, b1 - b2]
    return sub
def matrix_division(matrix, scalar):
    '''divide a matrix by a scalar'''
    a, b = matrix[0]
    c, d = matrix[1]
    div = [[a/scalar,b/scalar],[c/scalar,d/scalar]]
    return div

def matrix_2_to_2_addition(matrix1, matrix2):
    '''add two 2 * 1 matrix'''
    a1= matrix1[0]
    b1 = matrix1[1]
    a2 = matrix2[0]
    b2= matrix2[1]
    add = [a1 + a2, b1 + b2]
    return add

def clip(a:Tuple):
    return list(a[0:2])