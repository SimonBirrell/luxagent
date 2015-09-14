import sys
import time
import copy

import gevent

import rospkg
import rosgraph.impl.graph
from rosgraph import masterapi
from rosgraph.impl import graph

#   RosCommunication() acts as a bridge between the formats used by Agent() and
#   the formats used by the modules that interact with ROS.
#   On construction, it receives instances of the classes that interact with ROS:
#       RosGraphApi() - which interacts with the commputational graph via rosgraph

class RosCommunication():
    def __init__(self, ros_graph_api, ros_packages_api, ros_pubsub_api):
        self.observers = []
        self.ros_graph_api = ros_graph_api
        self.ros_graph_api.open(self)
        self.ros_packages_api = ros_packages_api
        self.ros_packages_api.open(self)
        self.ros_pubsub_api = ros_pubsub_api
        self.ros_pubsub_api.open(self)
        
     ################# Calls from Agent ######################    

    def get_machine_graph(self):
        machine_graph = {
            'machine_details' : {
                'human_name' : 'Pi Robot',
                'hostname' : self.ros_graph_api.get_hostname(),
                'fqdn' : self.ros_graph_api.get_fully_qualified_domain_name(),
                'machine_type' : 'embedded'
            },
            'package_tree' : self.ros_packages_api.get_package_tree()
        }

        return machine_graph

    def get_machine_id(self):
        return self.ros_graph_api.get_hostname()    

    def register_observer(self, observer):
        self.observers.append(observer)    
        
    def rosrun(self, args):
        self.ros_packages_api.rosrun(args)
        
    def rosrun_executed(self):
        foo = 0

    def roslaunch(self, args):
        self.ros_packages_api.roslaunch(args)

    def roslaunch_executed(self):
        foo = 0

    def kill(self, args):
        self.ros_packages_api.kill(args)

    def kill_executed(self):
        foo = 0

    def publish_topic_message(self, topic, topicMessage):
        self.ros_pubsub_api.publish_topic_message(topic, topicMessage)

    def get_graph(self):
        return self.ros_graph_api.convert_graph_to_agent_format(self.ros_graph_api.graph.nn_nodes, self.ros_graph_api.graph.nt_nodes, self.ros_graph_api.graph.nt_edges)
        
    ################# Event Calls ######################    
        
    def event_graph_add(self, added_ros_graph): 
        self.ros_pubsub_api.subscribe_to_topics(added_ros_graph)
        self.ros_pubsub_api.schedule_topic_examinations(added_ros_graph)
        self.ros_pubsub_api.do_topic_examinations(added_ros_graph)
        if (len(added_ros_graph)>0):
            print "Added %s" % added_ros_graph
            self.register_event_with_agent('graphAdd', added_ros_graph)  
        
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
                #print "Updated %s" % updated_ros_graph
                #print "******************"
                #print "******************"
                #print updated_ros_graph
                #print "******************"
                #print "******************"
                #self.register_event_with_agent('graphUpd', updated_ros_graph)  
                self.register_event_with_agent('graphUpd', filtered_updated_ros_graph)  
            else:
                print "******* TOO LONG ******"  
                print updated_ros_graph

    def event_graph_del(self, deleted_ros_graph):
        self.ros_pubsub_api.unsubscribe_from_topics(deleted_ros_graph)
        if (len(deleted_ros_graph)>0):
            print "Deleted %s" % deleted_ros_graph
            self.register_event_with_agent('graphDel', deleted_ros_graph)    

    def event_add_package_tree(self, package_tree):
        # For the moment, this is only called once, so can send the tree as is
        # TODO: Cache the results and send the difference
        self.register_event_with_agent('machineGraphAdd', package_tree)        

    def event_message_from_topic(self, topic, message_type, message_count, message):
        #print "event_message_from_topic %s" % topic
        updated_ros_graph = self.ros_graph_api.ros_graph_from_topic_message(topic, message_type, message_count, message)
        #print updated_ros_graph
        self.event_graph_upd(updated_ros_graph)
        #gevent.sleep(1)
        #print

    def update_loop_actions(self, ros_graph):
        return self.ros_pubsub_api.update_loop_actions(ros_graph)    

    ################# Utility functions ################    
        
    def register_event_with_agent(self, event_type, event_body):
        for observer in self.observers:
            observer(event_type, event_body)
            
            