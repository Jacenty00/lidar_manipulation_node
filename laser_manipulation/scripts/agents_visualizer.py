import pedsim_msgs.msg as agents
import rospy
import yaml
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty, EmptyResponse
from visualization_msgs.msg import Marker, MarkerArray


class AgentsVisualizer:
    def __init__(self):
        self.sub = rospy.Subscriber(
            "simulated_agents", agents.AgentStates, self.agents_callback
        )
        self.pub = rospy.Publisher("agents_marker", MarkerArray, queue_size=10)

    def agents_callback(self, msg):
        markers = []
        for agent in msg.agent_states:
            marker = Marker()
            marker.id = agent.id
            marker.ns = "agents"
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "map"
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.scale = Vector3(0.2, 0.2, 0.2)
            marker.color = ColorRGBA(0, 1, 0, 1)
            marker.pose = agent.pose
            markers.append(marker)
        self.pub.publish(MarkerArray(markers))


if __name__ == "__main__":
    rospy.init_node("agents_visualizer")
    AgentsVisualizer()
    while not rospy.is_shutdown():
        rospy.spin()
