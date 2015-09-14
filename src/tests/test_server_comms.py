from tests.helpers import TddHelp, DummyServerHandler 
import nose

Server_received_mtype = False
Server_received_mbody = False

class TestServerComms(TddHelp):
    def test_connection(self):
        self.run_generic(ServerSubscribeGraph, 'test_connection')
        nose.tools.ok_(self.agent.is_connected_to_server(), "server_received_agent_connect OK") 
        print    

    def test_bad_messages(self):
        self.run_generic(ServerBadMessages, 'test_bad_messages')
        print    

    def test_rosrun(self):
        self.run_generic(ServerRosRun, 'test_rosrun')
        nose.tools.ok_(self.ros_comm.rosrun_executed(), "Rosrun executed") 
        print    
        
    def test_roslaunch(self):
        self.run_generic(ServerRosLaunch, 'test_roslaunch')
        nose.tools.ok_(self.ros_comm.roslaunch_executed(), "Roslaunch executed") 
        print    

    def test_kill(self):
        self.run_generic(ServerKill, 'test_kill')
        nose.tools.ok_(self.ros_comm.kill_executed(), "Kill executed") 
        print    

    def test_topic_message(self):
        self.run_generic(ServerTopicMessage, 'test_topic_message')
        nose.tools.ok_(self.ros_comm.topic_message_published(), "Topic message published") 
        print
        
    def test_graphAdd(self):
        fake_graph = [['foo', 'bar'],['baz','boo']]
        dummy_server = self.run_generic_with_command(ServerInterpret, 'graphAdd', fake_graph, 'test_graphAdd')
        nose.tools.eq_(Server_received_mtype, 'graphAdd', 'graphAdd received') 
        nose.tools.eq_(Server_received_mbody, fake_graph, 'dummy graph received') 
        print                
        
    def test_graphUpd(self):
        fake_graph = [['foo', 'bar'],['baz','boo']]
        command = 'graphUpd'
        dummy_server = self.run_generic_with_command(ServerInterpret, command, fake_graph, 'test_' + command)
        nose.tools.eq_(Server_received_mtype, command, command + ' received') 
        nose.tools.eq_(Server_received_mbody, fake_graph, 'dummy graph received') 
        print                

    def test_graphDel(self):
        fake_graph = [['foo', 'bar'],['baz','boo']]
        command = 'graphDel'
        dummy_server = self.run_generic_with_command(ServerInterpret, command, fake_graph, 'test_' + command)
        nose.tools.eq_(Server_received_mtype, command, command + ' received') 
        nose.tools.eq_(Server_received_mbody, fake_graph, 'dummy graph received') 
        print      


class ServerSubscribeGraph(DummyServerHandler):
    def interpret_message(self, mtype, mbody):
        if (mtype=='agentConnect'):
            server_received_agent_connect = True
            self.send_message({'mtype': 'agentConnected'})
            self.send_message({'mtype': 'subscribeGraph'})
        elif (mtype=='graphAdd'):  
            graph = mbody
            print "Added Graph: ", str(mbody)
            for i in range(len(graph)):
                print "key: " + graph[i][0] + " value: " + graph[i][1]

class ServerRosRun(DummyServerHandler):
    def interpret_message(self, mtype, mbody):
        if (mtype=='agentConnect'):
            server_received_agent_connect = True
            self.send_message({'mtype': 'agentConnected'})
            self.send_message({'mtype': 'rosrun', 'mbody': {'args': 'vootie'}})

class ServerRosLaunch(DummyServerHandler):
    def interpret_message(self, mtype, mbody):
        if (mtype=='agentConnect'):
            server_received_agent_connect = True
            self.send_message({'mtype': 'agentConnected'})
            self.send_message({'mtype': 'roslaunch', 'mbody': {'args': 'vootie'}})

class ServerKill(DummyServerHandler):
    def interpret_message(self, mtype, mbody):
        if (mtype=='agentConnect'):
            server_received_agent_connect = True
            self.send_message({'mtype': 'agentConnected'})
            self.send_message({'mtype': 'kill', 'mbody': {'args': 'vootie'}})

class ServerTopicMessage(DummyServerHandler):
    def interpret_message(self, mtype, mbody):
        if (mtype=='agentConnect'):
            server_received_agent_connect = True
            self.send_message({'mtype': 'agentConnected'})
            self.send_message({'mtype': 'message', 'mbody': {'topic': 'vootie', 'message': {'foo':'bar'}}})

class ServerBadMessages(DummyServerHandler):
    def interpret_message(self, mtype, mbody):
        if (mtype=='agentConnect'):
            server_received_agent_connect = True
            self.send_message({'mtype': 'agentConnected'})
            self.send_message({})
            self.send_message('string')
            self.send_message('foomtypefoo')

class ServerInterpret(DummyServerHandler):
    def interpret_message(self, mtype, mbody):
        global Server_received_mtype
        global Server_received_mbody
        Server_received_mtype = mtype
        Server_received_mbody = mbody
        if (mtype=='agentConnect'):
            server_received_agent_connect = True
            self.send_message({'mtype': 'agentConnected'})
        elif (mtype=='graphAdd'):  
            graph = mbody
            print "Added Graph: ", str(mbody)
            
        
        
           
