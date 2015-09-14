import sys
import gevent

from ros_communication import ros_comm 
from server_communication import websockets
from agent import Agent
from ros_communication.ros_graph_api import RosGraphApi
from ros_communication.ros_packages_api import RosPackagesApi
from ros_communication.ros_pubsub_api import RosPubSubApi

def main():
    ros_graph_api = RosGraphApi()
    ros_packages_api = RosPackagesApi()
    ros_pubsub_api = RosPubSubApi()
    ros = ros_comm.RosCommunication(ros_graph_api, ros_packages_api, ros_pubsub_api)
    print "RosComm Initialized ok"
    
    server = websockets.ServerCommunication()
    print "Server communication initialized ok"

    agent = Agent(server, ros)
    print "Agent created"

    agent.start()
    print "Agent started"
    
    gevent.spawn(server.interpet_messages_from_server)
    
    while True:
        gevent.sleep(0.1)
        sys.stdout.write('.')

    