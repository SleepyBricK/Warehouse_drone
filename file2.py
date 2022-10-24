import rospy
import geometry_msgs.msg
import std_srvs.srv
import sys
import argparse


GOAL_POSE = {
    'pose_x': 5.0,
    'pose_y': 5.0,
    'pose_z': 0.0,
    'ang': 0.0,
    'ori_x': 0.0,
    'ori_y': 0.0,
    'ori_z': 0.0,
    'ori_w': 1.0
    }


def createParser ():
    parser = argparse.ArgumentParser()
    parser.add_argument ('-pose_x', default=0.0)
    parser.add_argument ('-pose_y', default=0.0)
    parser.add_argument ('-pose_z', default=0.0)
    parser.add_argument ('-ang', default=0.0)
    
    parser.add_argument ('-ori_x', default=0.0)
    parser.add_argument ('-ori_y', default=0.0)
    parser.add_argument ('-ori_z', default=0.0)
    parser.add_argument ('-ori_w', default=1.0)
    return parser.parse_args()

class NavigationStressTest(object):
    def __init__(self):
        self.event_in = None
        self.nav_goal = None
        self.r = rospy.Rate(10) # 10hz
        self.nav_pub = rospy.Publisher("/move_base_simple/goal", geometry_msgs.msg.PoseStamped, queue_size=100)
        args = createParser()
        GOAL_POSE['pose_x'] = float(args.pose_x)
        GOAL_POSE['pose_y'] = float(args.pose_y)
        GOAL_POSE['pose_z'] = float(args.pose_z)
        GOAL_POSE['ang'] = float(args.ori_x)
        GOAL_POSE['ori_x'] = float(args.ang)
        GOAL_POSE['ori_y'] = float(args.ori_y)
        GOAL_POSE['ori_z'] = float(args.ori_z)
        GOAL_POSE['ori_w'] = float(args.ori_w)
    def run(self):        
        #while not rospy.is_shutdown():
        for _ in range(5):
            self.publish_random_goal()
            self.r.sleep()
            

    def publish_random_goal(self):
        
        self.nav_goal = geometry_msgs.msg.PoseStamped()
        self.nav_goal.header.frame_id = 'world'
        self.nav_goal.pose.position.x = GOAL_POSE['pose_x']
        self.nav_goal.pose.position.y = GOAL_POSE['pose_y']
        self.nav_goal.pose.position.z = GOAL_POSE['pose_z']
        self.nav_goal.pose.orientation.x = GOAL_POSE['ori_x']
        self.nav_goal.pose.orientation.y = GOAL_POSE['ori_y']
        self.nav_goal.pose.orientation.z = GOAL_POSE['ori_z']
        self.nav_goal.pose.orientation.w = GOAL_POSE['ori_w']
        
        
        self.nav_pub.publish(self.nav_goal)
        rospy.loginfo("going to pose " )
       

def main():
    rospy.init_node("nav_goal_stresstest", anonymous=False)
    nav_stress_test = NavigationStressTest()
    nav_stress_test.run()

if __name__ == '__main__':
    main()
