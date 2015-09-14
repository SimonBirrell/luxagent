GRAPH_CHUNK_LIMIT = 20000

import json

class Agent():
    def __init__(self, server_comm, ros_comm):
        self.server_comm = server_comm
        self.ros_comm = ros_comm
        self.connected_to_server = False
        self.server_comm.register_observer(self.interpret_server_message)
        self.ros_comm.register_observer(self.interpret_ros_event)
        
    def start(self):
        machine_id = self.ros_comm.get_machine_id()
        self.server_comm.open('ros_instance_base', 'org_id', machine_id, 'username', 'secret')
        machine_graph = self.ros_comm.get_machine_graph()
        machine_graph_key = 'm ' + machine_id
        self.server_comm.send_to_server('graphAdd', [[machine_graph_key, machine_graph]])
        
    def stop(self):
        self.server_comm.close()
        
    def is_connected_to_server(self):  
        return self.connected_to_server  
           
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
            
    def get_value(self, mbody, key, default_value=""):
        if key in mbody:
            value = mbody[key]
        else:
            value = default_value
        return value

    def interpret_ros_event(self, event_type, event_body):
        if (event_type=='graphAdd'):
            print "ROS event %s with body %s" % (event_type, event_body)
            #graph = event_body
            #self.server_comm.send_to_server('graphAdd', graph)
            self.split_and_send_to_server(event_type, event_body)
        elif (event_type=='graphUpd'):
            #graph = event_body
            #self.server_comm.send_to_server('graphUpd', graph)
            self.split_and_send_to_server(event_type, event_body)
        elif (event_type=='graphDel'):
            print "ROS event %s with body %s" % (event_type, event_body)
            #graph = event_body
            #self.server_comm.send_to_server('graphDel', graph)
            self.split_and_send_to_server(event_type, event_body)
        else:
            print "ROS event %s with body %s" % (event_type, event_body)
            raise NameError("Unhandled ROS event")   

    def split_and_send_to_server(self, event_type, event_body):
            #graph = event_body
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
                        if (len(value_as_string) < GRAPH_CHUNK_LIMIT):
                            graph_chunk.append(event_body[index])
                        else:
                            print "split_and_send_to_server dropped a chunk %s" % event_type 
                        index = index + 1
                try:        
                    self.server_comm.send_to_server(event_type, graph_chunk)
                except AttributeError:
                    print "*********ERROR IN ws.send**************"    
                    print event_type
                    print graph_chunk
                    print "*********ERROR IN ws.send**************"    



    # RESTART
    # roscore
    # rosrun luxagent
    # roslaunch rbx1_bringup fake_turtlebot.launch
    # roslaunch rbx1_nav fake_move_base ...
    # Error on client side


        
        
        