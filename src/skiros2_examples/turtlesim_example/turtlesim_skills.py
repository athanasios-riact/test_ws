from skiros2_skill.core.skill import SkillDescription, SkillBase, Serial, ParallelFf, ParallelFs, Selector, SerialStar
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.conditions import ConditionProperty,ConditionRelation#,ConditionAbstractRelation

import numpy as np

#################################################################################
# Spawning
#################################################################################

class SpawnRandom(SkillDescription):
    def createDescription(self):
        self.addParam("Name", str, ParamTypes.Required)
        self.addParam("RangeX", [2.0, 8.0], ParamTypes.Optional)
        self.addParam("RangeY", [2.0, 8.0], ParamTypes.Optional)
        self.addParam("RangeR", [0.0, 360.0], ParamTypes.Optional)

class spawn_random(SkillBase):
    def createDescription(self):
        self.setDescription(SpawnRandom(), self.__class__.__name__)

    def expand(self, skill):
        range_x = self.params["RangeX"].values
        range_y = self.params["RangeY"].values
        range_r = self.params["RangeR"].values
        x = np.random.uniform(low=range_x[0], high=range_x[1])
        y = np.random.uniform(low=range_y[0], high=range_y[1])
        r = np.random.uniform(low=range_r[0], high=range_r[1])

        skill.setProcessor(SerialStar())
        skill(self.skill("Spawn", "spawn", specify={"X": x, "Y": y, "Rotation": r}))






# #################################################################################
# # Patrol
# #################################################################################

# class Patrol(SkillDescription):
#     def createDescription(self):
#         self.addParam("Once", True, ParamTypes.Required)
#         self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)

# class patrol(SkillBase):
#     def createDescription(self):
#         self.setDescription(Patrol(), self.__class__.__name__)

#     def expand(self, skill):

#         path = [
#             self.skill("Move", "move", specify={"Distance": 3.0, "Angle": 0.0, "Duration": 2.0}),
#             self.skill("Move", "move", specify={"Distance": 0.5, "Angle": 90.0}),
#             self.skill("Move", "move", specify={"Distance": 3.0, "Angle": 0.0, "Duration": 2.0}),
#             self.skill("Move", "move", specify={"Distance": 0.5, "Angle": 90.0}),
#             self.skill("Move", "move", specify={"Distance": 3.0, "Angle": 0.0, "Duration": 2.0}),
#             self.skill("Move", "move", specify={"Distance": 0.5, "Angle": 90.0}),
#             self.skill("Move", "move", specify={"Distance": 3.0, "Angle": 0.0, "Duration": 2.0}),
#             self.skill("Move", "move", specify={"Distance": 0.5, "Angle": 90.0}),
#         ]

#         if self.params["Once"].value:
#             skill.setProcessor(SerialStar())
#             skill(*path)
#         else:
#             skill.setProcessor(ParallelFf())
#             skill(
#                 self.skill(SerialStar())(*path),
#                 self.skill("Wait", "wait", specify={"Duration": 10000.0})
#             )



# #################################################################################
# # Follow
# #################################################################################

# class Follow(SkillDescription):
#     def createDescription(self):
#         self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
#         self.addParam("Target", Element("sumo:Object"), ParamTypes.Required)

# class follow(SkillBase):
#     def createDescription(self):
#         self.setDescription(Follow(), self.__class__.__name__)

#     def expand(self, skill):
#         uid = id(self)
#         skill.setProcessor(ParallelFs())
#         skill(
#             self.skill("Monitor", "monitor"),
#             self.skill("PoseController", "pose_controller", specify={"MinVel": 2.0},
#                        remap={"Linear": "Linear{}".format(uid), "Angular": "Angular{}".format(uid)}),
#             self.skill("Command", "command",
#                        remap={"Linear": "Linear{}".format(uid), "Angular": "Angular{}".format(uid)}),
#             self.skill("Wait", "wait", specify={"Duration": 10000.0})
#         )



# #################################################################################
# # Orbit
# #################################################################################

# class Orbit(SkillDescription):
#     def createDescription(self):
#         self.addParam("Turtle1", Element("cora:Robot"), ParamTypes.Required)
#         self.addParam("Turtle2", Element("cora:Robot"), ParamTypes.Required)

# class orbit(SkillBase):
#     def createDescription(self):
#         self.setDescription(Orbit(), self.__class__.__name__)

#     def expand(self, skill):
#         skill.setProcessor(ParallelFs())
#         skill(
#             self.skill("Follow", "follow", remap={"Turtle": "Turtle1", "Target": "Turtle2"}),
#             self.skill("Follow", "follow", remap={"Turtle": "Turtle2", "Target": "Turtle1"}),
#             self.skill("Wait", "wait", specify={"Duration": 10000.0})
#         )



'''
#################################################################################
# move_solegga
#################################################################################
class move_solegga(SkillDescription):
   def createDescription(self):
        # parameters
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Distance", 0.0, ParamTypes.Required)
        self.addParam("Angle", 0.0, ParamTypes.Required)
        self.addParam("Duration", 0.0, ParamTypes.Required)

        # preconditions
        self.addPreCondition(ConditionProperty("right_pos","skiros:PositionX","Turtle",">", 2, True))
        # postconditions
        #self.addPostCondition("timeEqual", , "Duration", 3)


class move_solegga_turtle(SkillBase):
    def createDescription(self):
        self.setDescription(move_solegga(), self.__class__.__name__)

    def expand(self, skill):
        skill(
            self.skill("Move", "move", specify={"Distance": 0.5, "Angle": 90.0,"Duration": 3.0}),
            self.skill("Move", "move", specify={"Distance": 3.0, "Angle": 0.0, "Duration": 3.0}),
        )
        #return self.success("")

'''


#################################################################################
# turtle pick and place
#################################################################################
#
#################################################################################
# Move
#################################################################################

class Move(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Distance", 0.0, ParamTypes.Required)
        self.addParam("Angle", 0.0, ParamTypes.Required)
        self.addParam("Duration", 1.0, ParamTypes.Optional)

class move(SkillBase):
    def createDescription(self):
        self.setDescription(Move(), self.__class__.__name__)

    def expand(self, skill):
        t = self.params["Duration"].value
        v = self.params["Distance"].value / t
        w = self.params["Angle"].value / t
        uid = id(self)
        skill.setProcessor(ParallelFs())
        skill(
            self.skill("Monitor", "monitor"),
            self.skill("Command", "command",
                       specify={"Linear{}".format(uid): v, "Angular{}".format(uid): w},
                       remap={"Linear": "Linear{}".format(uid), "Angular": "Angular{}".format(uid)}),
            self.skill("Wait", "wait", specify={"Duration": t})
        )

class SpawnContainer(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Name", str, ParamTypes.Required)
        self.addParam("X", 0.0, ParamTypes.Required)
        self.addParam("Y", 0.0, ParamTypes.Required)
        self.addParam("Rotation", 0.0, ParamTypes.Required)
        self.addParam("Container", Element("skiros:Location"), ParamTypes.Required)

class spawncontainer(SkillBase):
    def createDescription(self):
        self.setDescription(SpawnContainer(), self.__class__.__name__)

    def expand(self, skill):
        container = self.params["Container"].value
        name = self.params["Name"].value
        container.label = "Container:" + name
        container.setProperty("Container:ContainerName", "/{}".format(name))
        container.setData(":Position", [self.params["X"].value, self.params["Y"].value, 0.0])
        container.setData(":OrientationEuler", [0.0, 0.0, np.math.radians(self.params["Rotation"].value)])
        container.addRelation("skiros:Scene-0", "skiros:contain", "-1")
        container = self._wmi.add_element(container)
        self.params["Container"].value = container


        #return self.success("Spawned container {}".format(name))
        return self.success("")

class Move_to_Container1(SkillDescription):
    def createDescription(self):
        #==========params========
        self.addParam("Turtle",Element("cora:Robot"),ParamTypes.Required)
        self.addParam("Container",Element("skiros:Location"),ParamTypes.Required)


class move_to_container1(SkillBase):
    def createDescription(self):
        self.setDescription(Move_to_Container1(), self.__class__.__name__)

    def expand(self, skill):
        turtle = self.params["Turtle"].value
        c1 = self.params["Container"].value
        turtle_position = turtle.getData(":Position")
        turtle_orientation = turtle.getData(":OrientationEuler")[2]
        #c1position = c1.getData(":Position")
        #c1_orientation = c1.getData(":OrientationEuler")[2]
        #
        skill.setProcessor(Serial())
        skill(
            self.skill("Move","move",specify={"Distance": 10.0, "Angle": 0.0, "Duration": 3.0}),
            self.skill("Move","move",specify={"Distance": 10.0, "Angle": 90.0, "Duration": 3.0}),
        )

class Pick(SkillDescription):
    def createDescription(self):
        #=======Parameters=========
        self.addParam("Turtle",Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Container",Element("skiros:Location"),ParamTypes.Required)
        self.addParam("Object",Element("skiros:Product"),ParamTypes.Required)

        #=======Preconditions=======
        self.addPreCondition(ConditionRelation("TurtleAt","skiros:at","Turtle","Container",True)) #checks that robot is in correct location
        #self.addPreCondition(ConditionAbstractRelation("ContainerForObject","skiros:partReference","Container","Object",True)) #checks if container can hold the object type
        self.addPreCondition(ConditionProperty("NotEmpty","skiros:ContainerState","Container","=","Empty",False)) #check that container is not empty


class pick(SkillBase):
    def createDescription(self):
        self.setDescription(Pick(), self.__class__.__name__)

    def expand(self, skill):
        self.setRelation("Turtle","skiros:contain","Object")
        skill(
            self.skill("WmMoveObject", "wm_move_object",
                       remap={"StartLocation": "Container", "TargetLocation": "Object"},
                       specify={"Relation": "skiros:contain"}),
        )

class Move_to_Container2(SkillDescription):
    def createDescription(self):
        #==========params========
        self.addParam("Turtle",Element("cora:Robot"),ParamTypes.Required)
        self.addParam("Container2",Element("skiros:Location"),ParamTypes.Required)


class move_to_container2(SkillBase):
    def createDescription(self):
        self.setDescription(Move_to_Container2(), self.__class__.__name__)

    def expand(self, skill):
        turtle = self.params["Turtle"].value
        c2 = self.params["Container2"].value
        turtle_position = turtle.getData(":Position")
        turtle_orientation = turtle.getData(":OrientationEuler")[2]
        #c2position = c2.getData(":Position")
        #c2_orientation = c2.getData(":OrientationEuler")[2]
        skill(
            self.skill("Move","move",specify={"Distance": 5, "Angle": 0.0, "Duration": 3.0}),
        )

class Place(SkillDescription):
    def createDescription(self):
        #=======Parameters=========
        self.addParam("Turtle",Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Container2",Element("skiros:Location"),ParamTypes.Required)
        self.addParam("Object",Element("skiros:Product"),ParamTypes.Required)

        #=======Preconditions=======
        self.addPreCondition(ConditionRelation("TurtleAt","skiros:at","Turtle","Container",True)) #checks that robot is in correct location
        #self.addPreCondition(ConditionAbstractRelation("ContainerForObject","skiros:partReference","Container","Object",True)) #checks if container can hold the object type
        self.addPreCondition(ConditionProperty("NotEmpty","skiros:ContainerState","Container","=","Empty",False)) #check that container is not empty


class place(SkillBase):
    def createDescription(self):
        self.setDescription(Place(), self.__class__.__name__)

    def expand(self, skill):
        self.setRelation("Container2","skiros:contain","Object")
        skill(
            self.skill("WmMoveObject", "wm_move_object",
                       remap={"StartLocation": "Turtle", "TargetLocation": "Container2"},
                       specify={"Relation": "skiros:contain"}),
        )

class Pick_and_Place(SkillDescription):
    def createDescription(self):
        #=======Parameters=========
        self.addParam("Turtle",Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Container",Element("skiros:Location"),ParamTypes.Required)
        self.addParam("Container2",Element("skiros:Location"),ParamTypes.Required)
        self.addParam("Object",Element("skiros:Product"),ParamTypes.Required)


class pick_and_place(SkillBase):
    def createDescription(self):
        self.setDescription(Pick_and_Place(), self.__class__.__name__)

    def expand(self, skill):
        skill(
            self.skill(Selector())(
                self.skill(SerialStar())(
                    self.skill("Move_to_Container1", "move_to_container1"),
                    self.skill("Pick","pick"),
                ),
                self.skill(SerialStar())(
                    self.skill("Move_to_Container2","move_to_container2"),
                    self.skill("Place","place"),
                ),
            ),
        )


#self.skill(Sequential())(self.display(msg), self.speak(msg))
# class move_solegga(SkillDescription):
#    def createDescription(self):
#         # parameters
#         self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
#         self.addParam("Distance", 0.0, ParamTypes.Required)
#         self.addParam("Angle", 0.0, ParamTypes.Required)
#         self.addParam("Duration", 0.0, ParamTypes.Required)

#         # preconditions
#         self.addPreCondition(ConditionProperty("right_pos","skiros:PositionX","Turtle",">", 2, True))
#         # postconditions
#         #self.addPostCondition("timeEqual", , "Duration", 3)


# class move_solegga_turtle(SkillBase):
#     def createDescription(self):
#         self.setDescription(move_solegga(), self.__class__.__name__)

#     def expand(self, skill):
#         skill(
#             self.skill("Move", "move", specify={"Distance": 0.5, "Angle": 90.0,"Duration": 3.0}),
#             self.skill("Move", "move", specify={"Distance": 3.0, "Angle": 0.0, "Duration": 3.0}),
#         )
#         #return self.success("")

