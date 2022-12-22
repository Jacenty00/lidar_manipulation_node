import rospy
import pedsim_msgs.msg as agents
import numpy as np


class ArenaAgent:
    def __init__(self, starting_pos, waypoints, velocity):
        self.waypoints = np.array(starting_pos + waypoints)
        self.velocity = velocity
        self.target_waypoint_index = 1
        self.current_pos = np.array(starting_pos)
        self.current_direction = self.get_direction()

    def get_next_pos(self, time_delta):
        next_pos = (
            self.current_pos + time_delta * self.velocity * self.current_direction
        )
        self.current_pos = next_pos
        if (
            np.linalg.norm(
                self.waypoints[self.target_waypoint_index % len(self.waypoints)]
                - self.current_pos
            )
            < 0.1 * self.velocity
        ):
            self.target_waypoint_index = self.target_waypoint_index + 1 % len(
                self.waypoints
            )
            self.current_direction = self.get_direction()
        return next_pos

    def get_direction(self):
        direction = (
            self.waypoints[self.target_waypoint_index % len(self.waypoints)]
            - self.waypoints[(self.target_waypoint_index - 1) % len(self.waypoints)]
        )
        return direction / np.linalg.norm(direction)


class ArenaAgentsPublisher:
    def __init__(self):
        scenario_file_path = rospy.get_param("~scenario_json_path")

        scenario_file = self.read_scenario_file(scenario_file_path)
        pedsim_agents = scenario_file["pedsim_agents"]
        self.agents = []
        for agent in pedsim_agents:
            agent = ArenaAgent(
                agent["starting_pos"], agent["waypoints"], agent["velocity"]
            )
            self.agents.append(agent)

        self.starting_time = None
        self.pub = rospy.Publisher("arena_agents", agents.AgentStates, queue_size=1)
        self.pub_timer = rospy.Timer(rospy.Duration(0.1), self.pub_agents)

    def pub_agents(self, event):
        if self.starting_time is None:
            self.starting_time = rospy.Time.now()
        time_delta = rospy.Time.now() - self.starting_time
        agent_states = agents.AgentStates()
        for agent in self.agents:
            agent_state = agents.AgentState()
            new_pos = agent.get_next_pos(time_delta)
            agent_state.pose.position.x = new_pos[0]
            agent_state.pose.position.y = new_pos[1]
            agent_states.agent_states.append(agent_state)
        agent_states.header.stamp = rospy.Time.now()
        agent_states.header.frame_id = "map"
        self.pub.publish(agent_states)

    def read_scenario_file(self, scenario_file_path):
        with open(scenario_file_path, "r") as file:
            content = json.load(file)
            return content
