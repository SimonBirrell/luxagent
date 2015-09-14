import sys 
import time
import copy

import gevent
import socket

try: #py3k
    import urllib.parse as urlparse
except ImportError:
    import urlparse

try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy    

import rospkg
import rosgraph.impl.graph
from rosgraph import masterapi
from rosgraph.impl import graph
from rostopic import get_topic_type

ID = '/rosnode'

#   RosGraphApi abstracts the interaction with the rosgraph module.
#   It calls back to RosCommunication() with parameters that are in the
#   rosgraph format.

class RosGraphApi(object):
        
    def open(self, ros_comm):    
        self.ros_comm = ros_comm
        if (self.check_master_ok()):
            gevent.spawn(self.launch_graph)
        self.graph = graph.Graph()     
        
    def check_master_ok(self):      
        master = masterapi.Master('rosgraph')
        try:
            master.getPid()
        except:
            raise NameError("ERROR: Unable to communicate with master!")
            return False
        print "Master OK"  
        return True
        
    def agent_graph_to_set_of_keys(self, agent_graph):
        keys = []
        for entry in agent_graph:
            keys.append(entry[0])
        return set(keys)        

    def launch_graph(self):
        print "Starting graph monitoring"
        old_nn_nodes = self.graph.nn_nodes        
        old_nt_nodes = self.graph.nt_nodes
        old_nt_edges = self.edge_list_to_agent_graph(self.graph.nt_all_edges)
        old_nt_edges_keys = self.agent_graph_to_set_of_keys(old_nt_edges)
        
        old_agent_graph = self.convert_graph_to_agent_format(self.graph.nn_nodes, self.graph.nt_nodes, self.graph.nt_all_edges)
        old_agent_graph_hash = self.agent_graph_to_hash(old_agent_graph) 
        old_agent_graph_keys =  self.agent_graph_to_set_of_keys(old_agent_graph)
        
        try:
            while 1:
                start= time.time()
                self.graph.update()
                end= time.time()
                print "Update took %s s" % (end - start)
                
                new_agent_graph = self.convert_graph_to_agent_format(self.graph.nn_nodes, self.graph.nt_nodes, self.graph.nt_all_edges)
                new_agent_graph_hash = self.agent_graph_to_hash(new_agent_graph) 
                new_agent_graph_keys =  self.agent_graph_to_set_of_keys(new_agent_graph)

                new_agent_graph = self.ros_comm.update_loop_actions(new_agent_graph)

                deleted_item_keys = old_agent_graph_keys - new_agent_graph_keys
                added_item_keys = new_agent_graph_keys - old_agent_graph_keys
                common_item_keys = old_agent_graph_keys.intersection(new_agent_graph_keys)

                added_graph = self.filter_graph_by_keys(new_agent_graph, added_item_keys)
                deleted_graph = self.filter_graph_by_keys(old_agent_graph, deleted_item_keys)
                updated_graph = self.graph_from_changed_values(common_item_keys, new_agent_graph_hash, old_agent_graph_hash)
            
                augmented_added_graph = self.augment_graph(added_graph)
                augmented_updated_graph = self.augment_graph(updated_graph)

                self.ros_comm.event_graph_add(augmented_added_graph)
                self.ros_comm.event_graph_del(deleted_graph)
                self.ros_comm.event_graph_upd(augmented_updated_graph)
                
                old_agent_graph = new_agent_graph
                old_agent_graph_hash = new_agent_graph_hash
                old_agent_graph_keys = new_agent_graph_keys

                #self.print_graph()

                gevent.sleep(1.0)
        except KeyboardInterrupt:
            print "Finishing graph monitoring"
            pass

    def graph_from_changed_values(self, common_item_keys, new_agent_graph_hash, old_agent_graph_hash):        
        updated_graph = []
        for key in common_item_keys:
            if (new_agent_graph_hash[key] != old_agent_graph_hash[key]):
               updated_graph.append([key, new_agent_graph_hash[key]])   
        return updated_graph;            

    def agent_graph_to_hash(self, agent_graph):
        agent_graph_hash = {}
        for entry in agent_graph:
            agent_graph_hash[entry[0]] = entry[1]
        return agent_graph_hash            
            
    def edge_keys(self, edge_list):
        return map(self.edge_to_key, edge_list.edges_by_start)
        
    def edge_to_key(self, edge):
        return edge.start + " | " + edge.end    
        
    def edge_list_to_agent_graph(self, edge_list):
        return self.convert_graph_to_agent_format([],[],edge_list)      
        
    def convert_graph_to_agent_format(self, nn_nodes, nt_nodes, nt_edges):
        graph_for_agent = []
        for node in nn_nodes:
            graph_for_agent.append([self._node_to_node_key(node), self._node_to_node_value(node)])
        for topic in nt_nodes:
            graph_for_agent.append([self._topic_to_topic_key(topic), self._topic_to_topic_value(topic)])
        for list_of_edges in nt_edges.edges_by_start.values():
            for edge in list_of_edges:
                graph_for_agent.append([self._edge_to_edge_key(edge), self._edge_to_edge_value(edge)])

        return graph_for_agent      

    def filter_graph_by_keys(self, graph, keys):
        filtered_graph = []
        for entry in graph:
            key = entry[0]
            value = entry[1]
            if key in keys:
                filtered_graph.append([key, value])
        return filtered_graph    

    def _node_to_node_key(self, node):
        return 'n ' + node

    def _node_key_to_node(self, node_key):
        return node_key[2:]    

    def _topic_to_topic_key(self, topic):
        return 't ' + topic

    def _edge_to_edge_key(self, edge):
        return 'e ' + edge.start + " | " + edge.end

    def _node_to_node_value(self, node):
        return {}    

    def _topic_to_topic_value(self, topic):
        return {}    

    def _edge_to_edge_value(self, edge):
        return {'label': edge.label}   
        
    def print_graph(self):
        if not self.graph.nn_nodes and not self.graph.srvs:
            print("empty")
        else:
            print('\n')
        if self.graph.nn_nodes:
            print('Nodes:')
        for n in self.graph.nn_nodes:
            prefix = n + '|'
            print("  '" + n + "' :")
            print('    Inbound:')
            for k in self.graph.nn_edges.edges_by_end.keys():
                if k.startswith(prefix):
                    for c in self.graph.nn_edges.edges_by_end[k]:
                        print('      ' + c.start)
            print('    Outbound:')
            for k in self.graph.nn_edges.edges_by_start.keys():
                if k.startswith(prefix):
                    for c in self.graph.nn_edges.edges_by_start[k]:
                        print('      ' + c.end)
        if self.graph.srvs:
            print('Services:')
            for s in self.graph.srvs:
                print('  ' + s)            

    def augment_graph(self, graph):
        augmented_graph = copy.deepcopy(graph)
        augmented_graph = self.add_machine_id_to_graph(augmented_graph)
        augmented_graph = self.add_pid_to_graph(augmented_graph)
        return augmented_graph     

    def add_machine_id_to_graph(self, graph):
        for item in graph:
            if is_a_node(item):
                self.add_machine_id_to_node_item(item)
        return graph    

    def add_machine_id_to_node_item(self, node_item):
        master = rosgraph.Master(ID)
        node_name = self._node_key_to_node(node_item[0])
        hostname = ""
        try:
            uri = master.lookupNode(node_name)
            hostname = urlparse.urlparse(uri).hostname
        except socket.error:
            raise ROSNodeIOException("Unable to communicate with master!")    
        except rosgraph.MasterError:
            # it's possible that the state changes as we are doing lookups. this is a soft-fail
            print "state change while querying node %s" % node_name
        node_item[1]['hostname'] = hostname
        return node_item                          

    def add_pid_to_graph(self, graph):
        for item in graph:
            if is_a_node(item):
                self.add_pid_to_node_item(item)
        return graph    

    def add_pid_to_node_item(self, node_item):
        node_name = self._node_key_to_node(node_item[0])
        master = rosgraph.Master(ID)
        node_api = get_api_uri(master, node_name)
        if not node_api:
            node_item[1]['pid'] = 0
            return node_item
            #raise "Unknown node %s" % node_name
        timeout = 3    
        socket.setdefaulttimeout(timeout)
        node = ServerProxy(node_api)
        pid = _succeed(node.getPid(ID))
        node_item[1]['pid'] = pid
        return node_item     

    def get_hostname(self):
        return socket.gethostname()

    def get_fully_qualified_domain_name(self):
        return socket.getfqdn()    

    def ros_graph_from_topic_message(self, topic, message_type, message_count, message):
        """
        Converts topic (name) and message (hash) to a 
        rosgraph containing the one topic and all data.
        """
        key = self._topic_to_topic_key(topic)
        value = {'type': message_type, 'count': message_count, 'message': message}
        topic_item = [key, value]
        return [topic_item]
                             
class RosGraph(object):
    def __init__(self, nn_nodes, nt_nodes, nt_edges):
        self.nn_nodes = nn_nodes
        self.nt_nodes = nt_nodes
        self.nt_edges = nt_edges

def is_a_node(item):
    key = item[0]
    return (key[0]=='n')

# need for calling node APIs
def _succeed(args):
    code, msg, val = args
    if code != 1:
        raise ROSNodeException("remote call failed: %s"%msg)
    return val

_caller_apis = {}
def get_api_uri(master, caller_id, skip_cache=False):
    """
    @param master: XMLRPC handle to ROS Master
    @type  master: rosgraph.Master
    @param caller_id: node name
    @type  caller_id: str
    @param skip_cache: flag to skip cached data and force to lookup node from master
    @type  skip_cache: bool
    @return: xmlrpc URI of caller_id
    @rtype: str
    @raise ROSNodeIOException: if unable to communicate with master
    """
    caller_api = _caller_apis.get(caller_id, None)
    if not caller_api or skip_cache:
        try:
            caller_api = master.lookupNode(caller_id)
            _caller_apis[caller_id] = caller_api
        except rosgraph.MasterError:
            return None
        except socket.error:
            raise ROSNodeIOException("Unable to communicate with master!")
    return caller_api




        