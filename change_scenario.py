import rospy 
import rospkg 
from gazebo_msgs.msg import LinkState 
from gazebo_msgs.srv import SetLinkState,SpawnModel
from geometry_msgs.msg import Pose,Point,Quaternion


# Function to change the position of the box 
def change_pose():
    rospy.init_node('set_pose')

    state_msg = LinkState()
    state_msg.link_name = 'box'
    #Setting the required pose
    state_msg.pose.position.x = -2.0
    state_msg.pose.position.y = -1.0
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0

    rospy.wait_for_service('/gazebo/set_link_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
        #Publishing the message
        resp = set_state( state_msg )

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# This Function is to Spawn a barrier in the box place as model
def spawn_object():
    
    rospy.wait_for_service("/gazebo/spawn_urdf_model")

    try:
        
        spawner = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        #Loading the URDF file of the barrier
        spawner("box", open("/home/sparshith/catkin_ws/src/youbot_simulation/youbot_gazebo_worlds/urdf/barrier.urdf",'r').read(), "/rover", Pose(position= Point(2,0,0),orientation=Quaternion(0,0,0,0)),"wall_1")
       
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    try:
        #Calling the Functions to change the pose and place the object to the scenario
        change_pose()
        spawn_object()
    except rospy.ROSInterruptException:
        pass