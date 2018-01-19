import math
import rospy
import actionlib
import tf
import tf2_geometry_msgs
from actionlib import GoalStatus
from hsrb_interface import geometry, mobile_base, settings, exceptions
from hsrb_interface.robot import Robot
from geometry_msgs.msg import PoseStamped, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal

class FastMove(object):

    def __init__(self, omni_base):
        self.omni_base = omni_base

        self.global_pose = PoseStamped()
        sub = rospy.Subscriber('/global_pose', PoseStamped, self.__global_pose_cb)


    def __global_pose_cb(self, pose):
        self.global_pose = pose



    def move(self, pose, timeout=0, ref_frame_id=None, angular_thresh=10.0, spatial_thresh=0.1):
        """
        Move to a pose in the map frame within some angle and linear epsilon
        :param pose:
        :param timeout:
        :param ref_frame_id:
        :param angular_thresh: degrees
        :param spatial_thresh: meters
        """
        mobile_base._validate_timeout(timeout)

        map_frame = settings.get_frame('map')

        goal_pose = PoseStamped()
        goal_pose.pose = geometry.tuples_to_pose(pose)

        # Get goal position relative to the map
        if ref_frame_id:
            goal_pose.header.frame_id = ref_frame_id
            transform = self.omni_base._tf2_buffer.lookup_transform(map_frame, ref_frame_id, rospy.Time(0), rospy.Duration(1))
            goal_pose = tf2_geometry_msgs.do_transform_pose(goal_pose, transform)

        goal_pose.header.frame_id = map_frame
        goal_pose.header.stamp = rospy.Time(0)

        goal = MoveBaseGoal()
        goal.target_pose = goal_pose

        ac = self.omni_base._action_client
        ac.send_goal(goal)

        end_time = rospy.Time.now() + rospy.Duration(timeout)
        try:
            # Wait for a few milliseconds and check again
            while not ac.wait_for_result(rospy.Duration(0.1)):
                angular_dist = self.__quaternion_dist(self.global_pose.pose.orientation, goal_pose.pose.orientation)
                linear_dist = self.__euclid_dist(self.global_pose.pose.position, goal_pose.pose.position)
                if angular_dist <= angular_thresh and linear_dist <= spatial_thresh:
                    ac.cancel_goal()
                elif timeout > 0 and rospy.Time.now() > end_time:
                    ac.cancel_goal()
                    raise RuntimeError("Fast move timed out!")

            final_state = ac.get_state()
            # FIXME: Not sure why the final state is sometimes GoalStatus.ACTIVE
            if final_state in [GoalStatus.PREEMPTED, GoalStatus.PREEMPTING, GoalStatus.PENDING, GoalStatus.ACTIVE]:
                rospy.loginfo("FastMove preempted motion. Final state {}:{}".format(final_state, ac.get_goal_status_text()))
                return
            elif ac.get_state() != actionlib.GoalStatus.SUCCEEDED:
                raise RuntimeError("Failed {}: {}".format(ac.get_state(), ac.get_goal_status_text()))

        except KeyboardInterrupt:
            ac.cancel_goal()


    """
    Go to a location (spatial + angle)
    """
    def go(self, x, y, yaw, timeout=0.0, relative=False, angular_thresh=10.0, spatial_thresh=0.1):
        mobile_base._validate_timeout(timeout)

        position = geometry.Vector3(x, y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        orientation = geometry.Quaternion(*quat)
        pose = (position, orientation)

        if relative:
            ref_frame_id = settings.get_frame('base')
        else:
            ref_frame_id = settings.get_frame('map')
        self.move(pose, timeout, ref_frame_id, angular_thresh)


    """
    Angle between two quaternions
    """
    def __quaternion_dist(self, q1, q2):
        ip = q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w
        rad = math.acos(2.*ip*ip - 1.)
        return rad * 180. / math.pi


    """
    Euclidean distance
    """
    def __euclid_dist(self, p1, p2):
        x = p1.x - p2.x
        y = p1.y - p2.y
        z = p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)


if __name__ == '__main__':
    robot = Robot()
    omni_base = robot.get('omni_base')
    fm = FastMove(omni_base)
    fm.move(geometry.pose(x=0., y=0., z=0., ek=0.52), ref_frame_id='/base_link')
    #omni_base.go(0., 0., 0.52, 100., relative=True)
    #omni_base.move(geometry.pose(x=0., y=0., z=0., ek=0.52), ref_frame_id='base_link')
