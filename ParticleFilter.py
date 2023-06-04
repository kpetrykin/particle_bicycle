from matplotlib import pyplot as plt, patches as mpatches
import math
from math import tan, atan2, sqrt
import numpy as np
from numpy import sin, cos, pi, linspace
import random
from Robot import Robot, p_lim
import scipy.stats
from copy import deepcopy


class Particle:
    def __init__(self, robot, weight):
        self.rob = robot
        self.weight = weight
        self._cur_senses = None

    def _measurement_prob(self, reference_sensed):
        # print(f'reference_sensed: {reference_sensed}')
        sensed_probs = []
        for i, z_self in enumerate(self._cur_senses):
            # p_m = (1 / (sqrt(2 * pi * self.rob.sense_stand_dev**2))) * np.exp(
            #     -((z_self - reference_sensed[i])**2 / self.rob.sense_stand_dev**2))
            p = scipy.stats.norm(reference_sensed[i],
                                 self.rob.sense_stand_dev).pdf(z_self)

            # print(f'mine: {p_m} scipy: {p}')

            sensed_probs.append(p)

        return sensed_probs

    def sense(self):
        self._cur_senses = self.rob.sense()
        # print('cur_senses: ', self._cur_senses)

    def update_weight(self, reference_sensed):
        sensed_probs = self._measurement_prob(reference_sensed)
        # print(f'sensed_probs: {sensed_probs}')

        # print(f'old_w: {self.weight:.3f}')

        self.weight = sensed_probs[0] * sensed_probs[1] * \
            sensed_probs[2] * sensed_probs[3]

        # for _, p in enumerate(sensed_probs):
        #     self.weight *= p

        # print(f'new_w: {self.weight:.3f}')


class ParticleFilter:
    def __init__(self, robot, part_count):
        self._robot = robot
        self._part_count = part_count
        self.particles = []
        self._max_weigth = 0
        self._init_particles()
        self._normalize_weights()

    def _init_particles(self):
        for i in range(self._part_count):
            r = Robot(random.uniform(-p_lim, p_lim),
                      random.uniform(-p_lim, p_lim),
                      heading=random.uniform(-pi, pi))
            p = Particle(r, 1)
            self.particles.append(p)

    def _normalize_weights(self):
        sum_weight = 0
        for _, p in enumerate(self.particles):
            sum_weight += p.weight

        self._max_weigth = 0
        for _, p in enumerate(self.particles):
            p.weight = p.weight / sum_weight

            if p.weight > self._max_weigth:
                self._max_weigth = p.weight

    def particles_sense(self):
        for _, p in enumerate(self.particles):
            p.sense()

    def update_particles_weights(self, reference_sensed):
        for _, p in enumerate(self.particles):
            p.update_weight(reference_sensed)

        self._normalize_weights()

    def move_particles(self, dist, steer):
        for _, p in enumerate(self.particles):
            p.rob.move(dist, steer)

    def resample(self):
        new_particles = []

        index = random.randint(0, self._part_count)
        betta = 0
        for i in range(self._part_count):
            betta = betta + random.uniform(0, 2 * self._max_weigth)
            while betta > self.particles[index].weight:
                betta = betta - self.particles[index].weight
                index = (index + 1) % self._part_count
            new_particles.append(deepcopy(self.particles[index]))

        self.particles = new_particles
        self._normalize_weights()
        
    def estimate_pose(self):
        est_x = 0
        est_y = 0
        est_heading = 0
        
        for _, p in enumerate(self.particles):
            est_x += p.rob.x * p.weight
            est_y += p.rob.y * p.weight
            est_heading += p.rob.heading * p.weight
            
        return (est_x, est_y, est_heading)
        


if __name__ == "__main__":
    r = Robot()
    pf = ParticleFilter(r, 5)
    pf._init_particles()
