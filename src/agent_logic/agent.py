# The Agent class coordinates the communication between the ROS modules and the server.
#

#GRAPH_CHUNK_LIMIT = 20000
GRAPH_CHUNK_LIMIT = 200000
import sys
import json
import time
import gevent

class Agent():
    # Instantiate with:
    # - ServerCommunication (which handles websocket communication with the server)
    # - RosCommunication (which is an API to all the modules that deal with ROS)
    def __init__(self, server_comm, ros_comm):
        self.server_comm = server_comm
        self.ros_comm = ros_comm
        self.connected_to_server = False
        # Register to receive all server messages
        self.server_comm.register_observer(self.interpret_server_message)
        # Register to receive all ROS events
        self.ros_comm.register_observer(self.interpret_ros_event)

    # Start the communications flow
    # - Get machine ID from ROS API and send to server
    # - Start subscribing to ROS graph
    # - Add this host to "machine graph" (not a ROS concept) and send to server
    # TODO: This maybe should be a little less synchronous
    #
    def start(self):
        machine_id = self.ros_comm.get_machine_id()
        self.server_comm.open('ros_instance_base', 'org_id', machine_id, 'username', 'secret')
        self.ros_comm.start_pubsub()
        machine_graph = self.ros_comm.get_machine_graph()
        machine_graph_key = 'm ' + machine_id
        self.server_comm.send_to_server('graphAdd', [[machine_graph_key, machine_graph]])
        
    # Clean up on finish    
    def stop(self):
        self.server_comm.close()
        
    def is_connected_to_server(self):  
        return self.connected_to_server  
           
    # Callback for messages coming from the server
    #   mtype - type of command
    #   mbody - command payload
    # See LuxServer Readme for protocol 
    #    
    def interpret_server_message(self, mtype, mbody):
        print "Agent received %s with mbody %s" % (mtype, mbody)    
        if (mtype=='agentConnected'):
            print "Agent connected confirmed."
            self.connected_to_server = True
        elif (mtype=='subscribeGraph'):
            graph = self.ros_comm.get_graph()
            self.server_comm.send_to_server('graphAdd', graph)
        elif (mtype=='rosrun'):
            args = self.get_value(mbody, 'args')     
            self.ros_comm.rosrun(args)
        elif (mtype=='roslaunch'):
            args = self.get_value(mbody, 'args')     
            self.ros_comm.roslaunch(args)
        elif (mtype=='kill'):
            args = self.get_value(mbody, 'args')     
            self.ros_comm.kill(args)
        elif (mtype=='message'):
            topic = self.get_value(mbody, 'topic')     
            topicMessage = self.get_value(mbody, 'message')     
            self.ros_comm.publish_topic_message(topic, topicMessage)

    # mbody typically contains a JSON hash of parameters
    # This method siply extracts a value or returns a default
    # (usually and empty string)
    #        
    def get_value(self, mbody, key, default_value=""):
        if key in mbody:
            value = mbody[key]
        else:
            value = default_value
        return value

    # Callback for receiving events from the ROS and OS side of things
    # Long messages are split into chunks for sending via the websockets.
    # TODO: This chunking needs to be carefully integrated with compression.
    #
    def interpret_ros_event(self, event_type, event_body):
        if (event_type=='graphAdd'):
            print "ROS event %s with body %s" % (event_type, event_body)
            self.split_and_send_to_server(event_type, event_body)
        elif (event_type=='graphUpd'):
            self.split_and_send_to_server(event_type, event_body)
        elif (event_type=='graphDel'):
            print "ROS event %s with body %s" % (event_type, event_body)
            self.split_and_send_to_server(event_type, event_body)
        else:
            print "ROS event %s with body %s" % (event_type, event_body)
            raise NameError("Unhandled ROS event")   

    # Method that chunks an event body before sending via websockets.
    # Messages often consistent of arrays of graph items (e.g. graphAdd) so what this does is split
    # one addGraph message with 100 items into N addGraph messages, each with fewer graph items.
    # The cummulative length of the items in bytes should be below GRAPH_CHUNK_LIMIT.
    # TODO: THis works but needs rethinking to incorporate message-specific compression
    # Also, what are the fundamental limits of websockets in terms of message size?
    #
    def split_and_send_to_server(self, event_type, event_body):
            # TODO Get Binary working correctly
            binary = False
            index = 0
            number_graph_items = len(event_body)
            while (index <number_graph_items):
                graph_chunk = []
                cumulative_length = 0
                while (index <number_graph_items) and (cumulative_length < GRAPH_CHUNK_LIMIT):
                    next_length = len(event_body[index])
                    cumulative_length = cumulative_length + next_length
                    if (cumulative_length < GRAPH_CHUNK_LIMIT):
                        value = event_body[index]
                        value_as_string = json.dumps({'mtype': event_type, 'mbody': event_body[index]})
                        if (len(value_as_string) < GRAPH_CHUNK_LIMIT) or binary:
                            graph_chunk.append(event_body[index])
                        else:
                            # TODO move detection down to ws layer
                            print "split_and_send_to_server dropped a chunk %s" % event_type 
                        index = index + 1
                try:        
                    self.server_comm.send_to_server(event_type, graph_chunk, True) 
                except AttributeError:
                    print "*********ERROR IN ws.send**************"    
                    print event_type
                    print graph_chunk
                    print "*********ERROR IN ws.send**************"   
                    #time.sleep(30)
                    gevent.hub.get_hub().parent.throw(SystemExit()) 



         
        