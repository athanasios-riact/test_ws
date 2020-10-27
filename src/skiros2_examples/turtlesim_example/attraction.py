import numpy as np
import argparse
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.animation import FuncAnimation
import math
import random
import copy
import time
import sys


class Plan():
    def __init__(self, start, goal, obstacles, w_attract=1.0, w_repel=0.4):
        """
        @brief      Constructs a new instance.

        @param      start      Tuple of the starting coordinates of the robot
        @param      goal       Tuple of the goal coordinates of the robot
        @param      obstacles  List of tuples of obstacle coordinates
        @param      w_attract  Weight of the attractor force
        @param      w_repel    Weight of the repeller force
        """

        self.start = np.array(start).astype(np.float32)
        self.goal = np.array(goal).astype(np.float32)
        self.curr_pose = copy.copy(self.start)
        self.w_attract = w_attract
        self.w_repel = w_repel
        self.f_attract_min = 1.0
        self.f_repel_max = 5.0
        self.obstacles = obstacles
        self.obs_t = []
        self.memory_x = []
        self.memory_y = []
        self.xs = []
        self.ys = []

        self._render()


    def _calculateEuclidean(self, p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


    def _calculateAttractionForces(self):
        return 1.0 + max(self.f_attract_min, self.w_attract * self._calculateEuclidean(self.curr_pose, self.goal))


    def _calculateRepulsiveForces(self):
        f_obs = []
        for i, obs in enumerate(self.obstacles):
            f_obs.append(self.w_repel * (1.0 / (1.0 + self._calculateEuclidean(self.curr_pose, obs)**4)))

        return max(f_obs)


    def update(self):
        if (self._calculateEuclidean(self.curr_pose, self.goal) > 0.1):
            f_rep_x = 0.0
            f_rep_y = 0.0
            # Calculate forces
            fx = 0.0
            fy = 0.0
            f_attract = self._calculateAttractionForces()
            f_repel = self._calculateRepulsiveForces()

            # For animation purposes
            self.memory_x.append(self.curr_pose[0])
            self.memory_y.append(self.curr_pose[1])

            # Calculate angle between robot and other objects(goal)
            goal_theta = math.atan2((self.goal[1] - self.curr_pose[1]), (self.goal[0] - self.curr_pose[0]))

            for i,obs_t in enumerate(self.obstacles):
                obs_t = math.atan2((self.obstacles[i][1] - self.curr_pose[1]),
                                    (self.obstacles[i][0] - self.curr_pose[0]))
                f_rep_x += f_repel * np.cos(obs_t)
                f_rep_y += f_repel * np.sin(obs_t)
                if obs_t < 0.0:
                    obs_t += 2*np.pi

            # Map angles to 0-360 degrees (in rad)
            if goal_theta < 0.0:
                goal_theta += 2*np.pi

            # Forces summation
            fx += f_attract * np.cos(goal_theta) - f_rep_x
            fy += f_attract * np.sin(goal_theta) - f_rep_y

            self._updatePose(fx, fy)

            return True

        return False

    def _updatePose(self, fx, fy):
        self.curr_pose += np.array([fx * 0.02, fy * 0.02])

    def _render(self):
        plt.scatter(self.goal[0], self.goal[1], color="blue", s=100)
        plt.scatter(self.curr_pose[0], self.curr_pose[1])
        plt.scatter(self.obstacles[0][0], self.obstacles[0][1], color="brown", s=100)
        plt.scatter(self.obstacles[1][0], self.obstacles[1][1], color="brown", s=100)
        plt.scatter(self.obstacles[2][0], self.obstacles[2][1], color="brown", s=100)
        plt.scatter(self.obstacles[3][0], self.obstacles[3][1], color="brown", s=100)
        plt.scatter(self.obstacles[4][0], self.obstacles[4][1], color="brown", s=100)

        plt.xlim(0, 11)
        plt.ylim(0, 11)
        # plt.annotate("turtle", self.curr_pose)
        plt.annotate("goal", self.goal)
        plt.annotate("obstacle", self.obstacles[0])


if __name__ == "__main__":
    P = Plan((1, 1), (10, 10), [(3.1, 4.0),(8.2,8.2),(6.0,5.0),(9.0,6.0),(4.7,8.0)], w_attract=0.5, w_repel=9.0)
    start_t = time.time()
    def animate(i):
        if P.update():
            P.xs.append(P.curr_pose[0])
            P.ys.append(P.curr_pose[1])
            plt.style.use("ggplot")
            plt.scatter(P.xs, P.ys)
            plt.plot(P.xs, P.ys, color="r")

    ani = FuncAnimation(plt.gcf(), animate, interval=10)
    plt.show()
    end = time.time()
    print("time elapsed")
    print(end - start_t)
