import math as m

import numpy as np

endHeight = -120  # final height of the robot at the end of every demo programm


def square(half_side_length=25, robot_height=-100, n=2):
    """Calculates coordinates for a square
        `halfSideLength`: half length of the edge
        `robotHeight`: z-Coordinate for 2D model (have to be a negative value)
        `n`: Number of rotations
        `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
        linear moving
        """
    #  _______
    # |       |
    # |       |
    # |_______|
    #
    # | a |
    # a = halfSideLength
    pos_square = [[half_side_length, half_side_length, robot_height, 0, 0, 0, 'mov']]

    for _ in range(n):
        pos_square.append([half_side_length, half_side_length, robot_height, 0, 0, 0, 'lin'])
        pos_square.append([-half_side_length, half_side_length, robot_height, 0, 0, 0, 'lin'])
        pos_square.append([-half_side_length, -half_side_length, robot_height, 0, 0, 0, 'lin'])
        pos_square.append([half_side_length, -half_side_length, robot_height, 0, 0, 0, 'lin'])

    pos_square.append([half_side_length, half_side_length, robot_height, 0, 0, 0, 'lin'])
    pos_square.append([0, 0, endHeight, 0, 0, 0, 'mov'])

    return pos_square


def triangle(half_side_length=30, robot_height=-100, n=2):
    """Calculates coordinates for a samesided triangle
        `halfSideLength`: half sidelength of the triangle
        `robotHeight`: z-Coordinate for 2D model (have to be a negative value)
        `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
        linear moving
        """
    #     ^
    #    / \
    #   /   \
    #  /     \
    # /_______\
    #
    # | a |
    # a = halfSideLength

    h_half = (half_side_length * m.sqrt(3) / 2) / 2
    pos_triangle = []

    for _ in range(n):
        pos_triangle.append([-h_half, half_side_length, robot_height, 0, 0, 0, 'mov'])
        pos_triangle.append([-h_half, -half_side_length, robot_height, 0, 0, 0, 'lin'])
        pos_triangle.append([h_half, 0, robot_height, 0, 0, 0, 'lin'])

    pos_triangle.append([-h_half, half_side_length, robot_height, 0, 0, 0, 'lin'])
    pos_triangle.append([0, 0, endHeight, 0, 0, 0, 'mov'])
    return pos_triangle


def circle(radius=30, resolution=30, robot_height=-100, n=2, dir_circ=0):
    """Calculates coordinates for a 2D-circle
        `radius`: Radius of the circle
        `resolution`: Number of circlepoints
        `robotHeight`: z-Coordinate for 2D model (have to be a negative value)
        `n`: Number of rotations
        `dir`: Direction of the circle
        `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
        linear moving
        """

    t = np.linspace(0, n * 2 * m.pi, resolution * n)
    circle_pos = []
    for num in t:
        if dir_circ == 0:
            x = m.cos(num) * radius
            y = m.sin(num) * radius
        else:
            x = m.cos(num) * radius
            y = m.sin(num - m.pi) * radius

        circle_pos.append([x, y, robot_height, 0, 0, 0, 'mov'])

    circle_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])
    return circle_pos


def eight(radius=15, resolution=30, robot_height=-100, n=1):
    """Calculates coordinates for a 2D-eight
        `radius`: Radius of one of the two circles
        `resolution`: Number of circlepoints
        `robotHeight`: z-Coordinate for 2D model (have to be a negative value)
        `n`: Number of rotations
        `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
        linear moving
        """

    n = max(1, n)
    t = np.linspace(0, n * 2 * m.pi, resolution * n)
    eight_pos = []
    for num in t:
        x = -m.sin(num) * radius
        y = m.cos(num) * radius - radius
        eight_pos.append([x, y, robot_height, 0, 0, 0, 'mov'])

    eight_pos.append([x, y, robot_height, 0, 0, 0, 'mov'])

    for num in t:
        x = -m.sin(num) * radius
        y = -m.cos(num) * radius + radius
        eight_pos.append([x, y, robot_height, 0, 0, 0, 'mov'])

    eight_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])
    return eight_pos


def pyramide(half_side_length=30, robot_height=-110):
    """Calculates coordinates for a tetrahedron
        `halfSideLength`: half sidelength of the tetrahedron
        `robotHeight`: z-Coordinate for 2D Base (have to be a negative value)
        `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
        linear moving
        """

    h_base_half = (half_side_length * m.sqrt(3) / 2) / 2
    h_tetra = m.sqrt(6) * half_side_length / 3
    pyramide_pos = [

        [0, 0, robot_height + h_tetra, 0, 0, 0, 'mov'],
        [-h_base_half, -half_side_length, robot_height, 0, 0, 0, 'lin'],
        [h_base_half, 0, robot_height, 0, 0, 0, 'lin'],
        [0, 0, robot_height + h_tetra, 0, 0, 0, 'mov'],

        [h_base_half, 0, robot_height, 0, 0, 0, 'lin'],
        [-h_base_half, half_side_length, robot_height, 0, 0, 0, 'lin'],
        [0, 0, robot_height + h_tetra, 0, 0, 0, 'mov'],

        [-h_base_half, half_side_length, robot_height, 0, 0, 0, 'lin'],
        [-h_base_half, -half_side_length, robot_height, 0, 0, 0, 'lin'],
        [0, 0, robot_height + h_tetra, 0, 0, 0, 'mov'],

        [0, 0, endHeight, 0, 0, 0, 'mov']
    ]

    return pyramide_pos


def pick_place(distx=15, disty=15, mid_dist=20, default_height=-80, lin_height=20, robot_height=-110):
    """Calculates coordinates for a 3x2 palette
        `distx`: Distance between the palette places in x direction
        `disty`: Distance between the palette places in y direction
        `midDist`: Distance between mid and palette places 
        `defaulttHeight`: z-Coordinate for upper position of pick and place (have to be a negative value)
        `linHeight`: Linear distance to pick up/ place a piece 
        `robotHeight`: z-Coordinate for 2D Base (have to be a negative value)
        `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
        linear moving
        """
    pick_place_pos = []
    y_count = [0, 1]
    x_count = [-1, 0, 1]

    for numx in x_count:
        for numy in y_count:
            pick_place_pos.append(
                [numx * distx, numy * disty - mid_dist, robot_height + lin_height, 0, 0, 0, 'lin'])
            pick_place_pos.append(
                [numx * distx, numy * disty - mid_dist, robot_height, 0, 0, 0, 'mov'])
            pick_place_pos.append(
                [numx * distx, numy * disty - mid_dist, robot_height + lin_height, 0, 0, 0, 'lin'])

            pick_place_pos.append([numx * distx, 0, default_height, 0, 0, 0, 'mov'])

            pick_place_pos.append(
                [numx * distx, numy * disty + mid_dist, robot_height + lin_height, 0, 0, 0, 'lin'])
            pick_place_pos.append(
                [numx * distx, numy * disty + mid_dist, robot_height, 0, 0, 0, 'mov'])
            pick_place_pos.append(
                [numx * distx, numy * disty + mid_dist, robot_height + lin_height, 0, 0, 0, 'lin'])

            pick_place_pos.append([numx * distx, 0, default_height, 0, 0, 0, 'mov'])

    pick_place_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])
    return pick_place_pos


def rectangle_signal(flank_height=50, flank_width=15, robot_height=-100):
    """Calculates coordinates for rectangle Signal
    `flankHeight`: Flank height
    `flankWidth`: Flank width
    `robotHeight`: z-Coordinate for 2D Base (have to be a negative value)
    `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
    linear moving
    """
    rectangle_pos = [[flank_height / 2, -2.5 * flank_width, robot_height, 0, 0, 0, 'mov'],
                     [-flank_height / 2, -2.5 * flank_width, robot_height, 0, 0, 0, 'lin'],
                     [-flank_height / 2, -1.5 * flank_width, robot_height, 0, 0, 0, 'lin'],
                     [flank_height / 2, -1.5 * flank_width, robot_height, 0, 0, 0, 'lin'],
                     [flank_height / 2, -0.5 * flank_width, robot_height, 0, 0, 0, 'lin'],
                     [-flank_height / 2, -0.5 * flank_width, robot_height, 0, 0, 0, 'lin'],
                     [-flank_height / 2, 0.5 * flank_width, robot_height, 0, 0, 0, 'lin'],
                     [flank_height / 2, 0.5 * flank_width, robot_height, 0, 0, 0, 'lin'],
                     [flank_height / 2, 1.5 * flank_width, robot_height, 0, 0, 0, 'lin'],
                     [-flank_height / 2, 1.5 * flank_width, robot_height, 0, 0, 0, 'lin'],
                     [-flank_height / 2, 2.5 * flank_width, robot_height, 0, 0, 0, 'lin'],
                     [flank_height / 2, 2.5 * flank_width, robot_height, 0, 0, 0, 'mov'],
                     [0, 0, endHeight, 0, 0, 0, 'mov']]

    return rectangle_pos


def cylinder(down_circ=-120, up_circ=-80, radius=25, resolution=30):
    """Calculates coordinates for a cylinder
    `downCirc`: height of lower area of the cylinder
    `upCirc`: height of upper area of the cylinder
    `radius`: Radius of the cylinder
    `resolution`: Number of circlepoints
    `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
    linear moving
    """
    t = np.linspace(0, 2 * m.pi, resolution)
    cylinder_pos = []
    for num in t:
        x = -m.cos(num) * radius
        y = m.sin(num) * radius

        cylinder_pos.append([x, y, down_circ, 0, 0, 0, 'mov'])

    for num in t:
        x = -m.cos(num) * radius
        y = m.sin(num) * radius

        cylinder_pos.append([x, y, up_circ, 0, 0, 0, 'mov'])

    cylinder_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])
    return cylinder_pos


def cone(max_radius=25, resolution=30, n=5, robot_height=-130):
    """Calculates coordinates for a spiral
    `maxRadius`: Max radius of the spiral
    `resolution`: Number of circlepoints of one circle
    `n`:Numer of circles
    `robotHeight`: Start height of the spiral
    `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
    linear moving
    """

    t = np.linspace(0, n * 2 * m.pi, n * resolution)
    r = np.linspace(2, max_radius, n * resolution)
    spiral_pos = []

    for i, num in enumerate(t):
        x = -m.cos(num) * r[i]
        y = m.sin(num) * r[i]
        z = robot_height + 1.5 * r[i]
        spiral_pos.append([x, y, z, 0, 0, 0, 'mov'])

    spiral_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])

    return spiral_pos


def elaborated_curve(radius=20, resolution=28, robot_height=-100, distx=20, disty=20, lines=30):
    """Calculates coordinates for a 2D-Model
        `radius`: Radius of the circle
        `resolution`: Number of circlepoints, must be a multiple of 4
        `robotHeight`: z-Coordinate for 2D model (have to be a negative value)
        `distx`: x-distance between centerpoint of the circle and zero point of the coordinate system
        `disty`: y-distance between centerpoint of the circle and zero point of the coordinate system
        `lines`: Length of parallel lines
        `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
        linear moving
        """

    if resolution % 4 != 0:
        while resolution % 4 != 0:
            resolution += 1

    t = np.linspace(0, m.pi / 2, int(resolution / 4))
    elaborated_curve_pos = []
    for num in t:
        x = m.cos(num) * radius - distx
        y = -m.sin(num) * radius + disty
        elaborated_curve_pos.append([x, y, robot_height, 0, 0, 0, 'mov'])

    t = np.linspace(0, 2 * m.pi, resolution)
    for num in t:
        x = -m.sin(num) * radius - distx
        y = m.cos(num) * radius - disty
        elaborated_curve_pos.append([x, y, robot_height, 0, 0, 0, 'mov'])

    t = np.linspace(0, 3 * m.pi / 2, int(3 * resolution / 4))
    for num in t:
        x = -m.sin(num) * radius - distx
        y = -m.cos(num) * radius + disty
        elaborated_curve_pos.append([x, y, robot_height, 0, 0, 0, 'mov'])

    elaborated_curve_pos.append([lines, disty, robot_height, 0, 0, 0, 'lin'])

    z = np.linspace(0, m.pi, int(resolution / 2))
    for num in z:
        x = m.sin(num) * radius + lines
        y = m.cos(num) * radius
        elaborated_curve_pos.append([x, y, robot_height, 0, 0, 0, 'mov'])

    elaborated_curve_pos.append([0, -disty, robot_height, 0, 0, 0, 'lin'])

    elaborated_curve_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])
    return elaborated_curve_pos


if __name__ == '__main__':
    # Define return list values for demo sequences as this examples:
    # [x,y,z,a,b,c,'mov'] -> PTP
    # [x,y,z,a,b,c,'lin'] -> linear moving
    ans = square()
    print(ans)
