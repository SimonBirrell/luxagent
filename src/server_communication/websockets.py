from gevent import monkey; monkey.patch_all()
import gevent
from ws4py.client.geventclient import WebSocketClient
import json
from time import sleep

SERVER_URL = 'ws://simonbirrell.local:8000/'
#SERVER_URL = 'ws://luxserver.herokuapp.com:8000/'

class ServerCommunication():
    def __init__(self, url=SERVER_URL):
        self.stay_listening = True
        self.ws = WebSocketClient(url, protocols=['http-only', 'chat'])
        self.observers = []
        self.agent_connection_confirmed = False
        self.connected = False

    def open(self, ros_instance_base, org_id, machine_id, username, secret):
        print "Connecting to server..."
        self.ws.connect()
        print "...connected to server."
        self.connected = True
        self.agent_connect(ros_instance_base, org_id, machine_id, username, secret)
        
    def register_observer(self, observer):
        self.observers.append(observer)    
        
    def close(self):
        self.stay_listening = False
        self.ws.close()
        
    def interpet_messages_from_server(self):
        while self.stay_listening:
            m = self.ws.receive()
            if m is not None:
                print "server -> agent: " + str(m)
                json_message = json.loads(str(m))
                mtype = self.get_value(json_message, 'mtype')    
                mbody = self.get_value(json_message, 'mbody', {})    
                for observer in self.observers:
                    observer(mtype, mbody)
            else:
                break
                
    def get_value(self, message, key, default_value=""):
        if (key in message) and (type(message) is dict):
            value = message[key]
        else:
            value = default_value
        return value         

    def agent_connect(self, ros_instance_base, org_id, machine_id, username, secret):
        print "Connecting agent: " +  ros_instance_base
        self.send_to_server('agentConnect', {'rosinstance': ros_instance_base, 'org': org_id, 'user': username, 'secret': secret, 'hostname' : machine_id})
        
    def send_to_server(self, mtype, mbody):
        message = json.dumps({'mtype': mtype, 'mbody': mbody})
        if (len(message)<40000):
            if (mtype!="graphUpd"):
                foo = 0
                #print "agent -> server: " + message
            if (self.connected):
                self.ws.send(message)
            else:
                print "Message to server dropped - no connection yet."
        else:
            print "=============================="
            print "Message too long - dropping!!!"        
            print "=============================="
            print mtype
            print mbody
            print mtype
            print "=============================="
          
            

        
        
        