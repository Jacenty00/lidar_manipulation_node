#!/usr/bin/env python3
import json

import numpy as np
import pedsim_msgs.msg as agents
import rospy


class ArenaAgent:
    def __init__(self, starting_pos, waypoints, velocity, name):
        self.waypoints = np.array([starting_pos] + waypoints, dtype=object)
        self.velocity = velocity
        self.target_waypoint_index = 1
        self.current_pos = np.array(starting_pos)
        self.current_direction = self.get_direction()
        self.id = name

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
            <= 0.4
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
        for i, agent in enumerate(pedsim_agents):
            a = ArenaAgent(agent["pos"], agent["waypoints"], agent["vmax"], i)
            self.agents.append(a)

        self.starting_time = None
        self.pub = rospy.Publisher(
            "/simulated_agents", agents.AgentStates, queue_size=1
        )
        self.pub_timer = rospy.Timer(rospy.Duration(0.1), self.pub_agents)

    def pub_agents(self, event):
        if event.last_real is None:
            time_delta = 0.0
        else:
            time_delta = (event.current_real - event.last_real).to_sec()
        agent_states = agents.AgentStates()
        for agent in self.agents:
            agent_state = agents.AgentState()
            new_pos = agent.get_next_pos(time_delta)
            agent_state.id = agent.id
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


if __name__ == "__main__":
    rospy.init_node("arena_agents_publisher")
    arena_agents_publisher = ArenaAgentsPublisher()
    rospy.spin()
