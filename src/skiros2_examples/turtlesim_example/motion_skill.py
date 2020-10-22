from skiros2_skill.core.skill import SkillDescription, SkillBase, Serial, ParallelFf, ParallelFs, Selector, SerialStar
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.conditions import ConditionProperty,ConditionRelation

import numpy as np
import argparse
import random
import copy
import time
import sys

#################################################################################
# motion_planning
#################################################################################
class Go_to_goal(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Goal",Element("skiros:Location"),ParamTypes.Required)
        # self.addParam("Obstacle",Element("skiros:Location"),ParamTypes.Required)
        self.addParam("f_attract_min", 1.0, ParamTypes.Required)
        self.addParam("f_repel_max", 5.0, ParamTypes.Required)
        self.addParam("w_attract", 0.5, ParamTypes.Required)
        self.addParam("w_repel", 9.0, ParamTypes.Required)


        # self.start = np.array(start).astype(np.float32)
        # self.goal = np.array(goal).astype(np.float32)
        # self.curr_pose = copy.copy(self.start)
        # self.w_attract = w_attract
        # self.w_repel = w_repel
        # self.f_attract_min = 1.0
        # self.f_repel_max = 5.0
        # self.obstacles = obstacles
        # self.obs_t = []


class go_to_goal(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Go_to_goal(), self.__class__.__name__)

    def _calculateEuclidean(self, p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


    def _calculateAttractionForces(self, distance, weight=0.5, threshold=1.0):
        return 1.0 + max(minimum, weight * distance)


    def _calculateRepulsiveForces(self, distance, weight=9.0, threshold=5.0):
        return -weight * min(maximum, (1.0 / (1.0 + distance**4)))

    def _calculateAngle(self, source, target):
        f = target - source
        return np.atan2(f[1], f[0])


    def _updatePose(self, fx, fy):
        curr_pose += np.array([fx * 0.02, fy * 0.02])

    # def onStart(self):
    #     # turtle = self.params["Turtle"].value.getProperty("turtlebot:TurtleName").value
    #     turtle = self.params["Turtle"].value
    #     curr_pose = np.array(turtle.getData(":Position"))[:2]
    #     target = self.params["Goal"].value
    #     goal = np.array(target.getData(":Position"))[:2]
    #     obs = self.params["Obstacle"].value
    #     obstacles = np.array(obs.getData(":Position"))[:2]
    #     return True

    def execute(self):

        goal_dist_threshold = 0.1
        obj_dist_threshold = 2.0

        force = np.array([0.0, 0.0])

        # get pose from param turtle
        turtle = self.params["Turtle"].value
        turtle_name = turtle.getProperty("turtlebot:TurtleName").value
        turtle_pose = np.array(turtle.getData(":Position")[:2])


        ###
        # Attractor
        ################################################################

        # get pose from param goal
        goal = self.params["Goal"].value
        goal_pose = np.array(goal.getData(":Position")[:2])

        # calculate attractor force
        #   calculate distance between turtle pose and goal pose
        dist_to_goal = self._calculateEuclidean(turtle_pose, goal_pose)

        # if turtle close to goal, success
        if dist_to_goal <= goal_dist_threshold:
            return self.success("{}: Close to goal".format(turtle_name))

        # calculate attractor force
        minimum = self.params["f_attract_min"].value
        weight = self.params["w_attract"].value
        f_attract = self._calculateAttractionForces(dist_to_goal, weight, minimum)

        # calculate angle between turtle and goal
        theta = self._calculateAngle(turtle_pose, goal_pose)
        if theta < 0.0:
            theta += 2*np.pi

        # calculate force on turtle
        force += f_attract * np.array(np.cos(theta), np.sin(theta))


        ###
        # Repulsor
        ################################################################

        # calculate repulsor force
        #   for each obstacle

        # only if skill gets obstacles as input
        # obstacles = self.params["Obstacle"].values

        obstacles = self.wmi.resolve_elements(Element("skiros:LargeBox"))
        for obj in obstancles:

            #   get pose of obstancle (either from params or directly from wm)
            obj_pose = np.array(obj.getData(":Position")[:2])

            #   calculate distance between turtle pose and obstacle pose
            dist_to_obj = self._calculateEuclidean(turtle_pose, obj_pose)

            #   if turtle not close to obj, success
            if dist_to_obj <= obj_dist_threshold:
                continue

            #   calculate repulsor force
            minimum = self.params["f_repel_max"].value
            weight = self.params["w_repel"].value

            # calculate forces
            f_repel = self._calculateRepulsiveForces(dist_to_obj, weight, minimum)

            # calculate angle between turtle and goal
            theta = self._calculateAngle(turtle_pose, obj_pose)
            if theta < 0.0:
                theta += 2*np.pi

            # calculate and sum force on turtle
            force += f_repel * np.array(np.cos(theta), np.sin(theta))





        # if (self._calculateEuclidean(self.curr_pose, self.goal) > 0.1):
        #     f_rep_x = 0.0
        #     f_rep_y = 0.0
        #     # Calculate forces
        #     fx = 0.0
        #     fy = 0.0
        #     f_attract = self._calculateAttractionForces()
        #     f_repel = self._calculateRepulsiveForces()

        #     # Calculate angle between robot and other objects
        #     goal_theta = np.atan2((self.goal[1] - self.curr_pose[1]), (self.goal[0] - self.curr_pose[0]))

        #     self.params["obs_t"] = np.atan2((self.obstacles[1] - self.curr_pose[1]),
        #                                 (self.obstacles[0] - self.curr_pose[0]))

        #     # for i,obs_t in enumerate(self.params["Obstacle"]):
        #     #     obs_t = np.atan2((self.obstacles[i][1] - curr_pose[1]),
        #     #                         (self.obstacles[i][0] - curr_pose[0]))

        #     # Map angles to 0-360 degrees (in rad)
        #     if goal_theta < 0.0:
        #         goal_theta += 2*np.pi

        #     if obs_t < 0.0:
        #         obs_t += 2*np.pi

        #     # Forces summation
        #     f_rep_x = f_repel * np.cos(obs_t)
        #     f_rep_y = f_repel * np.sin(obs_t)
        #     fx += f_attract * np.cos(goal_theta) - f_rep_x
        #     fy += f_attract * np.sin(goal_theta) - f_rep_y

            self._updatePose(fx, fy)

            return True

        return False


# ################################################################################
# #Move
# ################################################################################

# class Move(SkillDescription):
#     def createDescription(self):
#         self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
#         self.addParam("Distance", 0.0, ParamTypes.Required)
#         self.addParam("Angle", 0.0, ParamTypes.Required)
#         self.addParam("Duration", 1.0, ParamTypes.Optional)

# class move(SkillBase):
#     def createDescription(self):
#         self.setDescription(Move(), self.__class__.__name__)

#     def expand(self, skill):
#         t = self.params["Duration"].value
#         v = self.params["Distance"].value / t
#         w = self.params["Angle"].value / t
#         uid = id(self)
#         skill.setProcessor(ParallelFs())
#         skill(
#             self.skill("Monitor", "monitor"),
#             self.skill("Command", "command",
#                        specify={"Linear{}".format(uid): v, "Angular{}".format(uid): w},
#                        remap={"Linear": "Linear{}".format(uid), "Angular": "Angular{}".format(uid)}),
#             self.skill("Wait", "wait", specify={"Duration": t})
#         )

# class SpawnContainer(SkillDescription):
#     def createDescription(self):
#         #=======Params=========
#         self.addParam("Name", str, ParamTypes.Required)
#         self.addParam("X", 0.0, ParamTypes.Required)
#         self.addParam("Y", 0.0, ParamTypes.Required)
#         self.addParam("Rotation", 0.0, ParamTypes.Required)
#         self.addParam("Container", Element("skiros:Location"), ParamTypes.Required)

# class spawncontainer(SkillBase):
#     def createDescription(self):
#         self.setDescription(SpawnContainer(), self.__class__.__name__)

#     def expand(self, skill):
#         container = self.params["Container"].value
#         name = self.params["Name"].value
#         container.label = "Container:" + name
#         container.setProperty("Container:ContainerName", "/{}".format(name))
#         container.setData(":Position", [self.params["X"].value, self.params["Y"].value, 0.0])
#         container.setData(":OrientationEuler", [0.0, 0.0, np.np.radians(self.params["Rotation"].value)])
#         container.addRelation("skiros:Scene-0", "skiros:contain", "-1")
#         container = self._wmi.add_element(container)
#         self.params["Container"].value = container

#         return self.success("")

# #################################################################################
# # Spawning
# #################################################################################

# class SpawnRandom(SkillDescription):
#     def createDescription(self):
#         self.addParam("Name", str, ParamTypes.Required)
#         self.addParam("RangeX", [2.0, 8.0], ParamTypes.Optional)
#         self.addParam("RangeY", [2.0, 8.0], ParamTypes.Optional)
#         self.addParam("RangeR", [0.0, 360.0], ParamTypes.Optional)

# class spawn_random(SkillBase):
#     def createDescription(self):
#         self.setDescription(SpawnRandom(), self.__class__.__name__)

#     def expand(self, skill):
#         range_x = self.params["RangeX"].values
#         range_y = self.params["RangeY"].values
#         range_r = self.params["RangeR"].values
#         x = np.random.uniform(low=range_x[0], high=range_x[1])
#         y = np.random.uniform(low=range_y[0], high=range_y[1])
#         r = np.random.uniform(low=range_r[0], high=range_r[1])

#         skill.setProcessor(SerialStar())
#         skill(self.skill("Spawn", "spawn", specify={"X": x, "Y": y, "Rotation": r}))

