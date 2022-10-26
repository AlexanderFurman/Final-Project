from UprightTarget import UprightTarget
from geometry_msgs.msg import Point
from FallenTarget import FallenTarget

def main():
    cm = Point(5, 5, 10)
    point = Point(5, 6, 10)
    radius = 3

    target = UprightTarget(point, cm, radius)

    target.update_point_z(7.345)
    target.update_point_x(4)
    target.update_point_y(4)

    target.plot()

    # point1 = Point(1, 2, 3)
    # point2 = Point(3, 5, 14)
    # radius = 5
    # target = FallenTarget(point1, point2, radius)

    # target.plot()


if __name__ == '__main__':
    main()    

