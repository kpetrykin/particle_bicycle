from matplotlib import pyplot as plt, patches as mpatches
import math
from math import tan, atan2
import numpy as np
from numpy import sin, cos, pi, linspace
import random
from Robot import Robot, p_lim


class Particle:
    def __init__(self, robot, weight):
        self.rob = robot
        self.weight = weight


class ParticleFilter:
    def __init__(self, robot, part_count):
        self._robot = robot
        self._part_count = part_count
        self.particles = []
        self._init_particles()
        self._normalize_weights()

    def _init_particles(self):
        for i in range(self._part_count):
            r = Robot(random.uniform(-p_lim, p_lim),
                      random.uniform(-p_lim, p_lim),
                      heading=random.uniform(-pi, pi))
            p = Particle(r, 1)
            self.particles.append(p)

        print(self.particles)

    def _normalize_weights(self):
        sum_weight = 0
        for _, p in enumerate(self.particles):
            sum_weight += p.weight

        for _, p in enumerate(self.particles):
            p.weight = p.weight / sum_weight


if __name__ == "__main__":
    r = Robot()
    pf = ParticleFilter(r, 5)
    pf._init_particles()
