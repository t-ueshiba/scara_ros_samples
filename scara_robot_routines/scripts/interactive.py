#!/usr/bin/env python

import rospy
from geometry_msgs        import msg as gmsg
from tf                   import transformations as tfs
from math                 import radians
from scara_robot_routines import ScaraRobotRoutines

######################################################################
#  global functions                                                  #
######################################################################
def is_num(s):
    try:
        float(s)
    except ValueError:
        return False
    else:
        return True

######################################################################
#  class InteractiveRoutines                                         #
######################################################################
class InteractiveRoutines(ScaraRobotRoutines):
    def __init__(self):
        super(InteractiveRoutines, self).__init__()

        self._robot_name   = rospy.get_param('~robot_name',   'arm')
        self._gripper_name = rospy.get_param('~gripper_name', 'gripper')
        self._speed        = rospy.get_param('~speed',        0.1)

    def move(self, xyzrpy):
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.pose = gmsg.Pose(
            gmsg.Point(xyzrpy[0], xyzrpy[1], xyzrpy[2]),
            gmsg.Quaternion(
                *tfs.quaternion_from_euler(xyzrpy[3], xyzrpy[4], xyzrpy[5])))
        (success, _, current_pose) = self.go_to_pose_goal(
                                        self._robot_name, target_pose,
                                        self._speed, move_lin=True)
        return success

    def grasp(self):
        return self.go_to_named_pose(self._gripper_name, 'grasp')

    def release(self):
        return self.go_to_named_pose(self._gripper_name, 'release')

    def run(self):
        # Reset pose
        self.go_to_named_pose(self._robot_name, "ready")

        axis = 'Y'

        while not rospy.is_shutdown():
            current_pose = self.get_current_pose(self._robot_name)
            prompt = '{:>5}:{}>> ' \
                   .format(axis, self.format_pose(current_pose))

            key = raw_input(prompt)

            if key == 'q':
                break
            elif key == 'X':
                axis = 'X'
            elif key == 'Y':
                axis = 'Y'
            elif key == 'Z':
                axis = 'Z'
            elif key == 'R':
                axis = 'Roll'
            elif key == '+':
                offset = [0, 0, 0, 0, 0, 0]
                if axis == 'X':
                    offset[0] = 0.01
                elif axis == 'Y':
                    offset[1] = 0.01
                elif axis == 'Z':
                    offset[2] = 0.01
                else:
                    offset[3] = radians(10)
                self.move_relative(self._robot_name, offset, self._speed)
            elif key == '-':
                offset = [0, 0, 0, 0, 0, 0]
                if axis == 'X':
                    offset[0] = -0.01
                elif axis == 'Y':
                    offset[1] = -0.01
                elif axis == 'Z':
                    offset[2] = -0.01
                else:
                    offset[3] = radians(-10)
                self.move_relative(self._robot_name, offset, self._speed)
            elif is_num(key):
                xyzrpy = self.xyz_rpy(current_pose)
                if axis == 'X':
                    xyzrpy[0] = float(key)
                elif axis == 'Y':
                    xyzrpy[1] = float(key)
                elif axis == 'Z':
                    xyzrpy[2] = float(key)
                else:
                    xyzrpy[3] = radians(float(key))
                self.move(xyzrpy)
            elif key == 's':
                self.stop(self._robot_name)
            elif key == 'f':
                frame = raw_input("  frame? ")
                self.go_to_frame(self._robot_name, frame)
            elif key == 'h':
                self.go_to_named_pose(self._robot_name, "home")
            elif key == 'n':
                pose_name = raw_input("  pose name? ")
                try:
                    self.go_to_named_pose(self._robot_name, pose_name)
                except rospy.ROSException as e:
                    rospy.logerr('Unknown pose: %s' % e)
            elif key == 'g':
                self.grasp()
            elif key == 'r':
                self.release()

        # Reset pose
        self.go_to_named_pose(self._robot_name, "home")


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':

    rospy.init_node('interactive', anonymous=True)

    routines = InteractiveRoutines()
    routines.run()
