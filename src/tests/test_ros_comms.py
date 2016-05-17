# Test that when certain ROS events happen, that they get sent to the server.
#

import nose
from luxagent.ros_communication import ros_comm 
from luxagent.agent import Agent
from helpers import DummyManager

AGENT_FORMAT_TEST_GRAPH = [['foo','bar'], ['baz','boo']]

Server_received_mtype = False
Server_received_mbody = False

class TestRosComms():
    def setUp(self):
        global Server_received_mtype, Server_received_mbody
        Server_received_mtype = False
        Server_received_mbody = False

    # Dummy verions of ROS modules and websockets. We're really just testing the Agent layer.
    #
    def setupRosCommAndAgent(self):
        self.server_comm = DummyServerCommunication()
        self.ros_graph_api = DummyRosGraphApi()
        self.ros_packages_api = DummyRosPackagesApi() 
        self.ros_pubsub_api = DummyRosPubSubApi() 
        self.manager_communication = DummyManager()
        self.ros_comm = ros_comm.RosCommunication(self.ros_graph_api, self.ros_packages_api, self.ros_pubsub_api)
        self.agent = Agent(self.server_comm, self.ros_comm)
    
    # IF ROS adds a graph segment, does this get sent to the server?
    #
    def test_add_graph_from_ros(self):
        print "test_add_graph_from_ros"
        self.setupRosCommAndAgent()
        self.ros_comm.event_graph_add(self.test_graph())
        nose.tools.eq_(Server_received_mtype, 'graphAdd', "server was sent graphAdd") 
        nose.tools.eq_(Server_received_mbody, AGENT_FORMAT_TEST_GRAPH, "server was sent graph") 
        self.agent.start()
        self.agent.stop()
        print

    # IF ROS updates a graph segment, does this get sent to the server?
    #
    def test_upd_graph_from_ros(self):
        print "test_upd_graph_from_ros"
        self.setupRosCommAndAgent()
        self.ros_comm.event_graph_upd(self.test_graph())
        print "Server_received_mtype %s" % Server_received_mtype
        nose.tools.eq_(Server_received_mtype, 'graphUpd', "server was sent graphUpd") 
        nose.tools.eq_(Server_received_mbody, AGENT_FORMAT_TEST_GRAPH, "server was sent graph") 
        self.agent.start()
        self.agent.stop()
        print

    # IF ROS deletes a graph segment, does this get sent to the server?
    #
    def test_del_graph_from_ros(self):
        print "test_del_graph_from_ros"
        self.setupRosCommAndAgent()
        self.ros_comm.event_graph_del(self.test_graph())
        print "Server_received_mtype %s" % Server_received_mtype
        nose.tools.eq_(Server_received_mtype, 'graphDel', "server was sent graphDel") 
        nose.tools.eq_(Server_received_mbody, AGENT_FORMAT_TEST_GRAPH, "server was sent graph") 
        self.agent.start()
        self.agent.stop()
        print
        
    # Return a graph segment for testing
    #    
    def test_graph(self):
        return AGENT_FORMAT_TEST_GRAPH

# Mocks out ServerCommunication()
#
class DummyServerCommunication():
    def register_observer(self, observer):
        self.observer = observer
        
    def send_to_server(self, mtype, mbody, binary=False):
        global Server_received_mtype, Server_received_mbody
        Server_received_mtype = mtype
        Server_received_mbody = mbody
        
    def open(self, ros_instance_base, org_id, machine_id, username, secret):
        print "Connected to dummy server"

    def close(self):
        print "Disconnected from dummy server"
    
# Mocks out RosGraphApi()
#    
class DummyRosGraphApi(object):
    def open(self, ros_comm):    
        self.ros_comm = ros_comm
        self.graph = DummyGraph()

    def get_hostname(self):
        return "somehost"    
        
    def get_fully_qualified_domain_name(self):
        return "somehost"    
 
# Mocks out RosPackageApi()
#        
class DummyRosPackagesApi(object):
    def open(self, ros_comm):    
        self.ros_comm = ros_comm

    def get_package_tree(self):
        return {}    

# Mocks out RosPubSubApi()
#
class DummyRosPubSubApi(object):
    def open(self, ros_comm):    
        self.ros_comm = ros_comm

    def start(self):
        return    

    def get_package_tree(self):
        return {}

    def subscribe_to_topics(self, added_ros_graph):
        return    

    def unsubscribe_from_topics(self, deleted_ros_graph):
        return    

    def schedule_topic_examinations(self, added_ros_graph):
        return    

    def do_topic_examinations(self, added_ros_graph):
        return    

# Mocks out the ROS Graph() class
#
class DummyGraph(object):
    def __init__(self):
        self.nn_nodes = set([])        
        self.nt_nodes = set([])        
        self.nt_edges = DummyEdgeList()
    
class DummyEdgeList(object):
    def __init__(self):
        self.edges_by_start = {}
        
    
    
    
    
        
