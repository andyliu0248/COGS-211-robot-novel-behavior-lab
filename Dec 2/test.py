from math import sin, cos, tan, asin, acos, atan, pi, e, sqrt
def get_angle_contained(theta, current_pos, target_pos):
    assert len(current_pos) == len(target_pos)
    direction_vector = (-sin(theta), cos(theta))
    print(direction_vector)
    target_vector = (target_pos[0] - current_pos[0], target_pos[1] - current_pos[1])
    target_dot_direction = dot_product(direction_vector, target_vector)
    length_product = (vector_len(target_vector)*vector_len(direction_vector))
    cos_angle_contained = target_dot_direction / length_product
    #print("dotted:", target_dot_direction, "length product:", length_product, "cos:", cos_angle_contained)
    print(cos_angle_contained);
    angle_contained = acos(cos_angle_contained)
    return (angle_contained, angle_contained / (2*pi) * 360)



def dot_product(v1, v2):
    assert len(v1) == len(v2)
    return sum([v1[i]*v2[i] for i in range(len(v1))])

def vector_len(v):
    square_sum = sum([v[i]**2 for i in range(len(v))])
    return square_sum ** 0.5


def cross_product(a, b):
    a1, a2, a3, b1, b2, b3 = a[0], a[1], a[2], b[0], b[1], b[2]
    return (a2*b3 - a3*b2, a3*b1 - a1*b3, a1*b2 - a2*b1)

def is_to_the_left(v1, v2):
    return cross_product(v1, v2)[2] < 0

v = (-.53, -.85, 0)
w = (-147, 71, 0)
i = (1,0,0)
j = (0,1,0)
k=(0,0,1)
