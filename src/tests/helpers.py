import sys
if 'threading' in sys.modules:
    del sys.modules['threading']
    # See http://stackoverflow.com/questions/8774958/keyerror-in-module-threading-after-a-successful-py-test-run
from gevent import monkey; monkey.patch_all()
import gevent
from ws4py.websocket import WebSocket
from ws4py.websocket import EchoWebSocket
from ws4py.server.geventserver import WSGIServer
from ws4py.server.wsgiutils import WebSocketWSGIApplication
from ws4py.client.geventclient import WebSocketClient
from server_communication import websockets 
from agent_logic.agent import Agent
from time import sleep
import json
import nose

SERVER_HOST = '127.0.0.1'
SERVER_PORT = 8001

class TddHelp:
    def setUp(self):
        foo = 0
                     
    def tearDown(self):
        self.dummy_server.close()
    
    def run_generic(self, server_handler, title):
        print title
        self.dummy_server = DummyServer(server_handler)

        self.server_comm = websockets.ServerCommunication('ws://127.0.0.1:8001/')
        self.ros_comm = DummyRos()
        self.agent = Agent(self.server_comm, self.ros_comm)
        nose.tools.ok_(not self.agent.is_connected_to_server(), "server not connected yet") 

        self.dummy_server.serve_forever()

        self.agent.start()
        gevent.spawn(self.server_comm.interpet_messages_from_server)
        gevent.sleep(0.1)
        self.agent.stop()   

    def run_generic_with_command(self, server_handler, mtype, mbody, title):
        print title
        Server_received_mtype = False    
        Server_received_mbody = False
        self.dummy_server = DummyServer(server_handler)

        self.server_comm = websockets.ServerCommunication('ws://127.0.0.1:8001/')
        self.ros_comm = DummyRos()
        self.agent = Agent(self.server_comm, self.ros_comm)
        nose.tools.ok_(not self.agent.is_connected_to_server(), "server not connected yet") 

        self.dummy_server.serve_forever()

        self.agent.start()
        gevent.spawn(self.server_comm.interpet_messages_from_server)
        
        self.server_comm.send_to_server(mtype, mbody)
        
        gevent.sleep(0.1)
        self.agent.stop()   

class DummyServerHandler(WebSocket):
    def send_message(self, json_message):
        message = json.dumps(json_message)        
        self.send(message, False)     

    def received_message(self, message):
        jmessage = json.loads(message.data)
        mtype = jmessage['mtype']
        if 'mbody' in jmessage:
            mbody = jmessage['mbody']
        else:
            mbody = False
        self.interpret_message(mtype, mbody)            

################### Utility Functions #################################

class DummyServer():
    def __init__(self, handler):
        self.ws_server = WSGIServer((SERVER_HOST, SERVER_PORT), WebSocketWSGIApplication(handler_cls=handler))

    def serve_forever(self):
        print "Running dummy server"
        self.ws_server.start()    

    def close(self):
        self.ws_server.close()    

class DummyRos():
    def __init__(self):
        self.rosrun_received = False
        self.roslaunch_received = False
        self.kill_received = False
        self.topic_message_received = False

    def get_machine_graph(self):
        return {}    

    def register_observer(self, observer):
        self.observer = observer

    def rosrun(self, args):
        self.rosrun_received = True   

    def rosrun_executed(self):
        return self.rosrun_received

    def roslaunch(self, args):
        self.roslaunch_received = True   

    def roslaunch_executed(self):
        return self.roslaunch_received

    def kill(self, args):
        self.kill_received = True   

    def publish_topic_message(self, topic, topicMessage):
        self.topic_message_received = True    

    def topic_message_published(self):
        return self.topic_message_received    

    def kill_executed(self):
        return self.kill_received
        
    def get_graph(self):
        return [['foo', 'bar'],['baz','boo']]  

    def get_machine_id(self):
        return "machine"    





