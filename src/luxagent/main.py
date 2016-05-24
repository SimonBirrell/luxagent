# Setup and main loop.
# 
# This sets up the main program modules and then goes into an
# infinite loop where it interprets messages from the server.
#
# TODO: An elegant way to exit the loop and clean up, closing the
# socket.

from gevent import monkey; monkey.patch_all(aggressive=False)
import gevent
import sys
import logging
logging.basicConfig()

from agent_config import AgentConfig
from ros_communication import ros_comm 
from server_communication import websockets, manager_communication
from agent import Agent
from ros_communication.ros_graph_api import RosGraphApi
from ros_communication.ros_packages_api import RosPackagesApi
from ros_communication.ros_pubsub_api import RosPubSubApi

def main(agent_guid="undefined", agent_password="undefined"):
    # Read configuration from agent_config.py
    # This small file can be conveniently overwritten by 
    # agent_config_development.py or agent_config_production.py
    # depending on the environment.
    agent_config = AgentConfig()

    # Module for communicating with LuxManager API (Rails)
    manager = manager_communication.ManagerCommunication(agent_guid, agent_password)
    session_token = manager.logon()

    # Module for Communicating with the server
    server = websockets.ServerCommunication(agent_config.sockets_server())
    print "Server communication initialized ok"

    # Modules for communicating between main ROS API and ROS itself
    ros_graph_api = RosGraphApi()
    ros_packages_api = RosPackagesApi()
    ros_pubsub_api = RosPubSubApi()

    # Main ROS API is instantiated and modules injected.
    ros = ros_comm.RosCommunication(ros_graph_api, ros_packages_api, ros_pubsub_api)
    print "RosComm Initialized ok"
    
    # The Agent directs the communication between ROS and the server
    agent = Agent(server, ros, agent_guid=agent_guid, session_token=session_token)
    print "Agent created"

    # Start communication
    agent.start()
    print "Agent started"
    
    # Gevent is a cooperative multitasking library
    # This line spawns a separate task (not really a thread or a process)
    # That listens for incoming messages from the server
    gevent.spawn(server.interpet_messages_from_server)
    
    # The sleep line will call any pending cooperative tasks
    while True:
        gevent.sleep(0.1)
        sys.stdout.write('.')


    