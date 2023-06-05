from math import tan, atan2, sqrt
import numpy as np
from numpy import sin, cos, pi, linspace

p_lim = 10

P1X = -p_lim
P1Y = p_lim
P2X = p_lim
P2Y = p_lim
P3X = p_lim
P3Y = -p_lim
P4X = -p_lim
P4Y = -p_lim


class Robot:
    def __init__(self, x0=0, y0=0, length=2, heading=0,
                 sense_stand_dev=0.05,
                 move_stand_dev=1,
                 turn_stand_dev=0.01):
        self.x = x0
        self.y = y0
        self.length = length
        self.heading = heading
        self.steering = 0.1
        self.wheel_length = 0.5
        self.sense_stand_dev = sense_stand_dev
        self.move_stand_dev = move_stand_dev
        self.turn_stand_dev = turn_stand_dev

    def _calc_turn_radius(self, steering):
        tan_a = tan(steering)
        if tan_a != 0:
            return self.length / tan_a
        else:
            return None

    def _calc_turn_center(self, turn_radius):
        if turn_radius is not None:
            return (self.x - turn_radius * sin(self.heading),
                    self.y + turn_radius * cos(self.heading))
        else:
            return (None, None)

    def _calc_heading_after_move(self, turn_radius, distance):
        if turn_radius is not None:
            new_heading = self.heading + distance / turn_radius
            return new_heading
        else:
            return self.heading

    def _calc_position_after_move(self, turn_x, turn_y, turn_radius,
                                  new_heading):
        new_x = turn_x + turn_radius * sin(new_heading)
        new_y = turn_y - turn_radius * cos(new_heading)

        return (new_x, new_y)

    def move(self, distance, steering):
        self.steering = steering + np.random.normal(0, self.turn_stand_dev)
        # distance += np.random.normal(0, self.move_stand_dev)
        noise = np.random.normal(0, self.move_stand_dev)

        tr = self._calc_turn_radius(steering)
        if tr is not None:
            x_c, y_c = self._calc_turn_center(tr)
            new_heading = self._calc_heading_after_move(tr, distance + noise)
            self.x, self.y = self._calc_position_after_move(
                x_c, y_c, tr, new_heading)
            self.heading = new_heading
            # print('Robot moved to (', self.x, ',',
            #       self.y, ') heading ', self.heading)
        else:
            # straight line
            self.x = self.x + (distance + noise) * cos(self.heading)
            self.y = self.y + (distance + noise) * sin(self.heading)
            # print('Robot moved straight to (', self.x, ',',
            #       self.y, ') heading ', self.heading)

    def sense(self):
        mean = 0
        peleng1 = atan2(P1X - self.x, P1Y - self.y) + \
            np.random.normal(mean, self.sense_stand_dev)
        peleng2 = atan2(P2X - self.x, P2Y - self.y) + \
            np.random.normal(mean, self.sense_stand_dev)
        peleng3 = atan2(P3X - self.x, P3Y - self.y) + \
            np.random.normal(mean, self.sense_stand_dev)
        peleng4 = atan2(P4X - self.x, P4Y - self.y) + \
            np.random.normal(mean, self.sense_stand_dev)

        return (peleng1, peleng2, peleng3, peleng4)
