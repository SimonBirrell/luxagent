from gevent import monkey; monkey.patch_all()
import gevent

import sys
import time
import copy

import rospkg
import rosgraph.impl.graph
from rosgraph import masterapi
from rosgraph.impl import graph

#   RosCommunication() acts as a bridge between the formats used by Agent() and
#   the formats used by the modules that interact with ROS.
#   On construction, it receives instances of the classes that interact with ROS:
#       RosGraphApi() - which interacts with the commputational graph via the standard package rosgraph
#       RosPackagesApi() - which interacts with installed ROS packages
#       RosPubSubApi() - which handles subscriptions to ROS topics
#

class RosCommunication():
    # Register and open injected modules
    def __init__(self, ros_graph_api, ros_packages_api, ros_pubsub_api):
        self.observers = []
        self.ros_graph_api = ros_graph_api
        self.ros_graph_api.open(self)
        self.ros_packages_api = ros_packages_api
        self.ros_packages_api.open(self)
        self.ros_pubsub_api = ros_pubsub_api
        self.ros_pubsub_api.open(self)

    # Start subscribing to topics
    def start_pubsub(self):
        self.ros_pubsub_api.start()            
        
    ################# Calls from Agent ######################    
    # When a message arrives from the server.
    #

    # Return local part of the 'graph' of the machines in the ROS instance and the packages
    # installed.
    # TODO: Eventually we'll need slave agents on secondary machines that only return this
    # information. One master agent will deal with the ROS graph itself.
    # TODO: The robot name is hard-coded here. It should be moved off into some kind
    # of config file (that will probably also include authentication information)
    #
    def get_machine_graph(self):
        machine_graph = {
            'machine_details' : {
                # TODO: Hard-coded
                'human_name' : 'Pi Robot',
                'hostname' : self.get_machine_id(),
                'fqdn' : self.ros_graph_api.get_fully_qualified_domain_name(),
                'machine_type' : 'embedded'
            },
            'package_tree' : self.ros_packages_api.get_package_tree()
        }

        return machine_graph

    # The machine id is simply the bare hostname
    def get_machine_id(self):
        return "ros-workstation"
        return self.ros_graph_api.get_hostname()    

    # This will generally be the Agent wanting to get callbacks on ROS events
    #
    def register_observer(self, observer):
        self.observers.append(observer)    
        
    # Execute ROSRUN command from server  
    #  
    def rosrun(self, args):
        self.ros_packages_api.rosrun(args)
        
    # Only used in testing    
    #
    def rosrun_executed(self):
        return

    # Execute ROSLAUNCH command from server
    #
    def roslaunch(self, args):
        self.ros_packages_api.roslaunch(args)

    # Only used in testing    
    #
    def roslaunch_executed(self):
        return

    # Execute kill command from server
    #
    def kill(self, args):
        self.ros_packages_api.kill(args)

    # Only used in testing    
    #
    def kill_executed(self):
        return

    # We've received a topic message from the server. Send to ROS for publication.
    #
    def publish_topic_message(self, topic, topicMessage):
        self.ros_pubsub_api.publish_topic_message(topic, topicMessage)

    # Get ROS computational graph (list of nodes and topics) in format for transmitting
    # to server
    #
    def get_graph(self):
        #return self.ros_graph_api.convert_graph_to_agent_format(self.ros_graph_api.graph.nn_nodes, self.ros_graph_api.graph.nt_nodes, self.ros_graph_api.graph.nt_edges)
        return self.ros_graph_api.get_graph()
        
    ################# Calls from ROS modules ######################    
    # When a ROS or OS event occurs.
    #
     
    # Some new items have been added to the graph. If they contain any topics then
    # make sure we subscribe to their messages.
    # Then, send the updates to the observers (the server via the Agent class, basically).
    #    
    def event_graph_add(self, added_ros_graph): 
        self.ros_pubsub_api.subscribe_to_topics(added_ros_graph)
        self.ros_pubsub_api.schedule_topic_examinations(added_ros_graph)
        self.ros_pubsub_api.do_topic_examinations(added_ros_graph)
        if (len(added_ros_graph)>0):
            print "Added %s" % added_ros_graph
            self.register_event_with_observers('graphAdd', added_ros_graph)  
        
    # This passes on updated graph elements to observers.
    # TODO: This is messily trying to compensate for websocket message length limits. We
    # need a clean unified system for serialization, compression, chunking and error handling.
    # For the moment we're just dropping updates in a rather caveleir fashion. 
    #    
    def event_graph_upd(self, updated_ros_graph): 
        filtered_updated_ros_graph = []
        if (len(updated_ros_graph)>0):
            for i in range(len(updated_ros_graph)):
                value = updated_ros_graph[i]
                if (len(value)<5000):
                    filtered_updated_ros_graph.append(value)
                else:
                    print "Dropped update that was > 5000"
                    print i
                    print "length value %s" % len(value)
            if (len(updated_ros_graph[0])<1000):
                #self.register_event_with_observers('graphUpd', updated_ros_graph)  
                self.register_event_with_observers('graphUpd', filtered_updated_ros_graph)  
            else:
                print "******* TOO LONG ******"  
                print updated_ros_graph

    # Inform observers about items deleted from the graph
    #
    def event_graph_del(self, deleted_ros_graph):
        self.ros_pubsub_api.unsubscribe_from_topics(deleted_ros_graph)
        if (len(deleted_ros_graph)>0):
            print "Deleted %s" % deleted_ros_graph
            self.register_event_with_observers('graphDel', deleted_ros_graph)    

    # The package tree is a section of the graph that lists installed packages.
    # In the future, this will allow installation of new packages while the agent is running.
    # For the moment, this is only called once, so can send the tree as is
    # TODO: Cache the results and send the difference
    #
    def event_add_package_tree(self, package_tree):
        self.register_event_with_observers('machineGraphAdd', package_tree)        

    # A message has been received from a ROS topic. Send it on to observers as a graph update.
    #
    def event_message_from_topic(self, topic, message_type, message_count, message):
        #print "event_message_from_topic %s" % topic
        updated_ros_graph = self.ros_graph_api.ros_graph_from_topic_message(topic, message_type, message_count, message)
        self.event_graph_upd(updated_ros_graph)

    # ????
    def update_loop_actions(self, ros_graph):
        return self.ros_pubsub_api.update_loop_actions(ros_graph)    

    ################# Utility functions ################    
        
    def register_event_with_observers(self, event_type, event_body):
        for observer in self.observers:
            observer(event_type, event_body)
            
            