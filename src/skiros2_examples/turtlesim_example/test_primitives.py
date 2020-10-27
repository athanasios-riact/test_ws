import math
import numpy as np
import threading
from turtlesim.srv import Spawn as SpawnSrv
from geometry_msgs.msg import Twist
import turtlesim.msg as ts
import rospy
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
from skiros2_skill.core.skill import SkillDescription, ParamOptions
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.conditions import ConditionHasProperty, ConditionProperty, ConditionRelation


#################################################################################
# Spawn
#################################################################################

class SpawnTest(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Name", str, ParamTypes.Required)
        self.addParam("X", 0.0, ParamTypes.Required)
        self.addParam("Y", 0.0, ParamTypes.Required)
        self.addParam("OrientationDeg", 0.0, ParamTypes.Required)
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Optional)  # This is output
        # =======Pre-conditions=========

        # =======Post-conditions=========


class spawn_test(PrimitiveBase):
    def createDescription(self):
        self.setDescription(SpawnTest(), self.__class__.__name__)

    def onInit(self):
        """Called once when loading the primitive. If return False, the primitive is not loaded"""
        return True

    def onPreempt(self):
        """ Called when skill is requested to stop. """
        return self.success("Stopped")

    def onStart(self):
        """Called just before 1st execute"""
        name = self.params["Name"].value
        x = self.params["X"].value
        y = self.params["Y"].value
        print("Turtle {} will be spawned at ({}, {})".format(name, x, y))
        return True

    def execute(self):
        """ Main execution function """
        turtle = self.params["Turtle"].value
        name = self.params["Name"].value
        x = self.params["X"].value
        y = self.params["Y"].value
        theta_deg = self.params["OrientationDeg"].value
        if turtle.id == "":

            try:
                spawn_service = rospy.ServiceProxy("spawn", SpawnSrv)
                resp = spawn_service(x, y, np.deg2rad(theta_deg), name)
            except rospy.ServiceException as e:
                return self.fail("Failed to spawn turtle", -1)

            print("Turtle id: {}".format(turtle.id))

            turtle.label = "turtlebot:" + name
            turtle.setProperty("turtlebot:TurtleName", "/{}".format(name))
            turtle.setData(":Position", [x, y, 0.0])
            turtle.setData(":OrientationEuler", [0.0, 0.0, np.deg2rad(theta_deg)])
            turtle.addRelation("skiros:Scene-0", "skiros:contain", "-1")
            turtle = self._wmi.add_element(turtle)
            self.params["Turtle"].value = turtle

            return self.success("Spawned turtle {}".format(name))
        return self.success("Done")




#################################################################################
# Spawn Contrainer
#################################################################################

class SpawnContainer(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Name", str, ParamTypes.Required)
        self.addParam("X", 0.0, ParamTypes.Required)
        self.addParam("Y", 0.0, ParamTypes.Required)
        self.addParam("Container", Element("cora:Container"), ParamTypes.Optional)  # This is output
        # =======Pre-conditions=========

        # =======Post-conditions=========


class spawn_container(PrimitiveBase):
    def createDescription(self):
        self.setDescription(SpawnContainer(), self.__class__.__name__)

    def onInit(self):
        """Called once when loading the primitive. If return False, the primitive is not loaded"""
        return True

    def onPreempt(self):
        """ Called when skill is requested to stop. """
        return self.success("Stopped")

    def onStart(self):
        """Called just before 1st execute"""
        name = self.params["Name"].value
        x = self.params["X"].value
        y = self.params["Y"].value
        print("Container {} will be spawned at ({}, {})".format(name, x, y))
        return True

    def execute(self):
        """ Main execution function """
        Container = self.params["Turtle"].value
        name = self.params["Name"].value
        x = self.params["X"].value
        y = self.params["Y"].value
        theta_deg = self.params["OrientationDeg"].value
        if turtle.id == "":

            try:
                spawn_service = rospy.ServiceProxy("spawn", SpawnSrv)
                resp = spawn_service(x, y, np.deg2rad(theta_deg), name)
            except rospy.ServiceException as e:
                return self.fail("Failed to spawn turtle", -1)

            print("Turtle id: {}".format(turtle.id))

            turtle.label = "turtlebot:" + name
            turtle.setProperty("turtlebot:TurtleName", "/{}".format(name))
            turtle.setData(":Position", [x, y, 0.0])
            turtle.setData(":OrientationEuler", [0.0, 0.0, np.deg2rad(theta_deg)])
            turtle.addRelation("skiros:Scene-0", "skiros:contain", "-1")
            turtle = self._wmi.add_element(turtle)
            self.params["Turtle"].value = turtle

            return self.success("Spawned turtle {}".format(name))
        return self.success("Done")



#################################################################################
# Monitor
#################################################################################


class MonitorTest(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("TurtleInstance", Element("cora:Robot"), ParamTypes.Required)

class monitor_test(PrimitiveBase):
    def createDescription(self):
        self.setDescription(MonitorTest(), self.__class__.__name__)

    def onPreempt(self):
        return self.success("Preempted.")

    def onStart(self):
        name = self.params["TurtleInstance"].value.getProperty("turtlebot:TurtleName").value
        self._pose_sub = rospy.Subscriber(name + "/pose", ts.Pose, self._monitor)
        self._pose = None
        return True

    def onEnd(self):
        self._pose_sub.unregister()
        self._pose_sub = None
        return True

    def _monitor(self, msg):
        self._pose = [msg.x, msg.y, msg.theta]

    def execute(self):
        if self._pose is None:
            return self.step("No pose received")

        x,y,theta = self._pose

        turtle = self.params["TurtleInstance"].value
        turtle.setData(":Position", [x, y, 0.0])
        turtle.setData(":OrientationEuler", [0.0, 0.0, theta])
        self.params["TurtleInstance"].value = turtle

        return self.step("")

#################################################################################
# Command
#################################################################################

class CommandTest(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("TurtleInstance", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Linear", float, ParamTypes.Required)
        self.addParam("Angular", float, ParamTypes.Required)
        # =======Pre-conditions=========

        # =======Post-conditions=========


class command_test(PrimitiveBase):
    def createDescription(self):
        self.setDescription(CommandTest(), self.__class__.__name__)

    def _send_command(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = math.radians(angular)
        self.cmd_pub.publish(msg)

    def onInit(self):
        """Called once when loading the primitive. If return False, the primitive is not loaded"""
        return True

    def onPreempt(self):
        """ Called when skill is requested to stop. """
        return self.success("Stopped")

    def onStart(self):
        """Called just before 1st execute"""
        name = self.params["TurtleInstance"].value.getProperty("turtlebot:TurtleName").value
        print("Turtle name: {}".format(name))
        self.cmd_pub = rospy.Publisher(name + "/cmd_vel", Twist, queue_size=1)

        return True

    def onEnd(self):
        """Called just after last execute"""
        self._send_command(0,0)
        return True

    def execute(self):
        """ Main execution function """
        turtle = self.params["TurtleInstance"].value
        linear = self.params["Linear"].value
        angular = self.params["Angular"].value

        self._send_command(linear, angular)

        return self.step("{}: moving at [{} {}]".format(turtle, self.params["Linear"].value, self.params["Angular"].value))


#################################################################################
# PoseControllerALL
#################################################################################

class PoseControllerAllTest(SkillDescription):
    def createDescription(self):
        self.addParam("Catch", False, ParamTypes.Required)
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Target", Element("cora:Robot"), ParamTypes.Required)  # This one is sumo
        self.addParam("Mode", str, ParamTypes.Required)
        self.addParam("Linear", float, ParamTypes.Optional)
        self.addParam("Angular", float, ParamTypes.Optional)
        self.addParam("MinVel", float, ParamTypes.Optional)

        # =======Pre-conditions=========

        # =======Post-conditions=========


class pose_controller_all_test  (PrimitiveBase):
    def createDescription(self):
        self.setDescription(PoseControllerAllTest(), self.__class__.__name__)

    def onInit(self):
        """Called once when loading the primitive. If return False, the primitive is not loaded"""
        return True

    def onPreempt(self):
        """ Called when skill is requested to stop. """
        return self.success("Stopped")

    def onStart(self):
        """Called just before 1st execute"""
        return True

    def execute(self):
        """ Main execution function """
        turtle = self.params["Turtle"].value
        target = self.params["Target"].value

        turtle_vec = np.array(turtle.getData(":Position")[:2])
        target_vec = np.array(target.getData(":Position")[:2])
        diff_vec = target_vec - turtle_vec
        diff_mag = np.linalg.norm(diff_vec)

        turtle_rot = turtle.getData(":OrientationEuler")[2]
        target_rot = target.getData(":OrientationEuler")[2]
        orientation_diff = target_rot - turtle_rot

        if self.params["MinVel"].value is not None:
            diff_mag = max(diff_mag, self.params("MinVel").value)

        a = diff_vec / diff_mag
        b = np.array([math.cos(turtle_rot), math.sin(turtle_rot)])
        angle = math.acos(a.dot(b))


        if self.params["Mode"].value == "Orient":
            if math.degrees(angle) <= 0.5:
                return self.success("Turtle oriented towards target!")

            print("Difference: {}".format(math.degrees(angle)))
            v = 0.0
            w = math.degrees(angle) / 1.05

        elif self.params["Mode"].value == "Translate":
            if diff_mag <= 0.1:
                return self.success("Target reached!")

            print("Distance to target: {}".format(diff_mag))
            v = diff_mag / 2.0
            w = 0.0

        elif self.params["Mode"].value == "Pose":
            if self.params["Catch"] and diff_mag <= 0.01:
                return self.success("Target caught!")

            v = diff_mag / 4.0
            w = math.degrees(angle) / 2.0

        self.params["Linear"].value = v
        self.params["Angular"].value = w

        return self.step("")


#################################################################################
# Pick
#################################################################################

class PickObject(SkillDescription):
    """
    @brief Move an Object from StartLocation to TargetLocation in the world model
    """

    def createDescription(self):
        # =======Params=========
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Container", Element("skiros:Container"), ParamTypes.Required)
        self.addParam("Object", Element("skiros:Product"), ParamTypes.Required)
        self.addParam("Relation", "skiros:contain", ParamTypes.Required)
        # =======PreConditions=========
        # self.addPreCondition(self.getRelationCond("StartContainObj", "skiros:spatiallyRelated", "Container", "Object", True))
        # self.addPreCondition(ConditionRelation("TurtleAt", "skiros:at", "Turtle", "Container", True))

        # =======Post-conditions=========
        # self.addPostCondition(ConditionRelation("TurtleHasProduct", "skiros:contain", "Turtle", "Object", True))



class pick_object(PrimitiveBase):
    """
    Instant primitive

    Set Target-Contain-Object on the world model
    """

    def createDescription(self):
        self.setDescription(PickObject(), self.__class__.__name__)

    def onEnd(self):
        self.params["Container"].unset()
        return True

    def execute(self):
        start = self.params["Container"].value
        target = self.params["Turtle"].value if self.params["Turtle"].value.id else start
        objectt = self.params["Object"].value
        rel = objectt.getRelation(pred=self._wmi.get_sub_properties("skiros:spatiallyRelated"), obj="-1")

        rel["src"] = target.id
        rel["type"] = self.params["Relation"].value
        self._wmi.update_element_properties(start)
        self._wmi.update_element_properties(target)
        self.params["Object"].value = objectt
        return self.success("{} moved from {} to {}.".format(objectt.id, start.id, target.id))



#################################################################################
# Release
#################################################################################

class ReleaseObject(SkillDescription):
    """
    @brief Move an Object from StartLocation to TargetLocation in the world model
    """

    def createDescription(self):
        # =======Params=========
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Container", Element("skiros:Container"), ParamTypes.Required)
        self.addParam("Object", Element("skiros:Product"), ParamTypes.Required)
        self.addParam("Relation", "skiros:contain", ParamTypes.Required)
        # =======PreConditions=========
        # self.addPreCondition(self.getRelationCond("StartContainObj", "skiros:spatiallyRelated", "Container", "Object", True))
        # self.addPreCondition(ConditionRelation("TurtleAt", "skiros:at", "Turtle", "Container", True))

        # =======Post-conditions=========
        # self.addPostCondition(ConditionRelation("TurtleHasProduct", "skiros:contain", "Turtle", "Object", True))



class release_object(PrimitiveBase):
    """
    Instant primitive

    Set Target-Contain-Object on the world model
    """

    def createDescription(self):
        self.setDescription(ReleaseObject(), self.__class__.__name__)

    def onEnd(self):
        self.params["Container"].unset()
        return True

    def execute(self):
        target = self.params["Container"].value
        start = self.params["Turtle"].value if self.params["Turtle"].value.id else start
        objectt = self.params["Object"].value
        rel = objectt.getRelation(pred=self._wmi.get_sub_properties("skiros:spatiallyRelated"), obj="-1")

        rel["src"] = target.id
        rel["type"] = self.params["Relation"].value
        self._wmi.update_element_properties(start)
        self._wmi.update_element_properties(target)
        self.params["Object"].value = objectt
        return self.success("{} moved from {} to {}.".format(objectt.id, start.id, target.id))
