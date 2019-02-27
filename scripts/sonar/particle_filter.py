import math
import random
import numpy as np


class ParticleFilter:
    """
    Particle filter for sonar pinger localization
    """
    MOTION = 0.04  # random particle motion

    def __init__(self, init_pos, init_stdev, num_particles, sense_noise):
        """
        initializes the particle filter with a normal distribution
        :param init_pos: init position
        :param init_stdev: stdev for normal distribution
        :param num_particles: number of particles to create
        """
        self.particles = np.random.multivariate_normal(
            init_pos, [[init_stdev**2, 0], [0, init_stdev**2]], num_particles)
        self.weights = np.array(
            [1. / num_particles for _ in range(num_particles)])
        self.n = num_particles
        self.sense_noise = sense_noise

    def update_filter(self, measurement, robot_pos):
        """
        runs a filter iteration for the filter
        :param measurement: measurement from sensor
        :param robot_pos: robot x,y position
        """
        new_weights = []
        for p in self.particles:
            p = self.__move(p, self.MOTION)
            angle = self.__measurement(p, robot_pos)
            prob = self.__measurement_prob(angle, measurement,
                                           self.sense_noise)
            new_weights.append(prob)
        new_weights = np.array(new_weights)
        new_weights /= np.sum(new_weights)  # normalized weights
        self.weights = new_weights

        # if self.__neff() > self.n / 2:
        self.particles = self.__resample()

    def get_best_particle(self):
        """
        returns particle with the highest weight
        :return: particle
        """
        index = self.weights.argmax()
        return self.particles[index, :]

    def get_covariance(self):
        """
        returns the covariance matrix for the filter dist
        :return: covariance matrix (2x2)
        """
        x = self.particles[:, 0]
        y = self.particles[:, 1]
        X = np.stack((x, y), axis=0)
        return np.cov(X)

    def __neff(self):
        """
        calculates effective N
        number of particles meaningfully contributing to the prob dist
        :param weights: particle weights
        :return: effective N value
        """
        return 1. / np.sum(np.square(self.weights))

    def __resample(self):
        """
        resample particles with replacement. Resampling prob depends on particle weights (using wheel based method)
        :return: new list of resampled particles
        """
        p_resample = []
        w_max = max(self.weights)
        index = int(round(random.uniform(0, self.n - 1)))
        beta = 0
        for i in range(self.n):
            beta += random.uniform(0, 2 * w_max)
            while self.weights[index] < beta:
                beta -= self.weights[index]
                index = (index + 1) % self.n
            p_resample.append(self.particles[index, :])
        return np.array(p_resample)

    @staticmethod
    def __move(particle, motion):
        """
        move particle with a random gaussian
        :param particle: particle to move
        :param motion: stdev for gaussian
        :return: particle
        """
        particle[0] += random.gauss(0.0, motion)
        particle[1] += random.gauss(0.0, motion)
        return particle

    @staticmethod
    def __measurement(particle_pos, robot_pos):
        """
        measures the angle between the particle and the robot
        :param particle_pos: particle
        :param robot_pos: position of robot
        :return: angle in degrees
        """
        return np.rad2deg(
            math.atan2(particle_pos[1] - robot_pos[1],
                       particle_pos[0] - robot_pos[0]))

    @staticmethod
    def __measurement_prob(angle, measurement, noise):
        """
        returns the probability of seeing a measurement
        :param angle: angle of particle from robot
        :param measurement: measured angle
        :param noise: measurement noise
        :return: gaussian probability
        """
        return ParticleFilter.Gaussian(angle, noise, measurement)

    @staticmethod
    def Gaussian(mu, sigma, x):
        """
        calculate gaussian probability
        :param mu: mean
        :param sigma: stdev
        :param x: x
        :return: probability
        """
        return math.exp(-((mu - x)**2) /
                        (sigma**2) / 2.0) / math.sqrt(2.0 * math.pi *
                                                      (sigma**2))
