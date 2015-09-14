import nose
from ros_communication import ros_comm 
from agent_logic.agent import Agent

AGENT_FORMAT_TEST_GRAPH = [['foo','bar'], ['baz','boo']]

Server_received_mtype = False
Server_received_mbody = False

class TestRosComms():
    def setUp(self):
        global Server_received_mtype, Server_received_mbody
        Server_received_mtype = False
        Server_received_mbody = False

    def setupRosCommAndAgent(self):
        self.server_comm = DummyServerCommunication()
        self.ros_graph_api = DummyRosGraphApi()
        self.ros_packages_api = DummyRosPackagesApi() 
        self.ros_pubsub_api = DummyRosPubSubApi() 
        self.ros_comm = ros_comm.RosCommunication(self.ros_graph_api, self.ros_packages_api, self.ros_pubsub_api)
        self.agent = Agent(self.server_comm, self.ros_comm)
    
    def test_add_graph_from_ros(self):
        print "test_add_graph_from_ros"
        self.setupRosCommAndAgent()
        self.ros_comm.event_graph_add(self.test_graph())
        nose.tools.eq_(Server_received_mtype, 'graphAdd', "server was sent graphAdd") 
        nose.tools.eq_(Server_received_mbody, AGENT_FORMAT_TEST_GRAPH, "server was sent graph") 
        self.agent.start()
        self.agent.stop()
        print

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
        
    def test_graph(self):
        return AGENT_FORMAT_TEST_GRAPH

class DummyServerCommunication():
    def register_observer(self, observer):
        self.observer = observer
        
    def send_to_server(self, mtype, mbody):
        global Server_received_mtype, Server_received_mbody
        Server_received_mtype = mtype
        Server_received_mbody = mbody
        
    def open(self, ros_instance_base, org_id, machine_id, username, secret):
        print "Connected to dummy server"

    def close(self):
        print "Disconnected from dummy server"
    
class DummyRosGraphApi(object):
    def open(self, ros_comm):    
        self.ros_comm = ros_comm
        self.graph = DummyGraph()

    def get_hostname(self):
        return "somehost"    
        
    def get_fully_qualified_domain_name(self):
        return "somehost"    
        
class DummyRosPackagesApi(object):
    def open(self, ros_comm):    
        self.ros_comm = ros_comm

    def get_package_tree(self):
        return {}    

class DummyRosPubSubApi(object):
    def open(self, ros_comm):    
        self.ros_comm = ros_comm

    def get_package_tree(self):
        return {}

    def subscribe_to_topics(self, added_ros_graph):
        foo = 0

    def unsubscribe_from_topics(self, deleted_ros_graph):
        foo = 0    

    def schedule_topic_examinations(self, added_ros_graph):
        foo = 0    

    def do_topic_examinations(self, added_ros_graph):
        foo = 0    

class DummyGraph(object):
    def __init__(self):
        self.nn_nodes = set([])        
        self.nt_nodes = set([])        
        self.nt_edges = DummyEdgeList()
    
class DummyEdgeList(object):
    def __init__(self):
        self.edges_by_start = {}
        
    
    
    
    
        
