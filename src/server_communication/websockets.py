# ServerCommunication() is a class that abstracts the websocket operations.
#

# Import the gevent coroutine libraries. These patch the system websocket library.
from gevent import monkey; monkey.patch_all()
import gevent

import sys
from ws4py.client.geventclient import WebSocketClient
from ws4py.exc import HandshakeError, WebSocketException

# Serialization and compression libraries
import json
import bson
#import lzw

from time import sleep
from threading import Lock

SIZE_OUTPUT_BUFFER = 100 

# Default URLS during development
#SERVER_URL = 'ws://simonbirrell.local:8000/'
#SERVER_URL = 'ws://luxserver.herokuapp.com:80/'
SERVER_URL = 'ws://simonbirrell.local:8080/'

class ServerCommunication():
    def __init__(self, url=SERVER_URL):
        self.url = url
        self.stay_listening = True
        self.observers = []
        self.agent_connection_confirmed = False
        self.connected = False
        self.output_buffer = []
        self.output_lock = Lock()

    # Open connection with the server
    #
    def open(self, ros_instance_base, org_id, machine_id, username, secret):
        print "Connecting to server..."
        connected = False
        while not self.connected:
            try:
                self.ws = WebSocketClient(self.url, protocols=['http-only', 'chat'])
                self.ws.connect()
                self.connected = True
            except: # WebSocketException:
                print "Websocket error communicating with %s" % self.url
                
        print "...connected to server."
        self.agent_connect(ros_instance_base, org_id, machine_id, username, secret)
        
    # Allow other objects to subscribe to messages from the server    
    #
    def register_observer(self, observer):
        self.observers.append(observer)    
        
    # Close the websocket.
    #    
    def close(self):
        self.stay_listening = False
        self.ws.close()
        
    # Loop to receive messages from server.
    # The gevent library will switch to other coroutines while idling in
    # ws.receive()
    #    
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
          
    # Extract mtype or mbody from a server message
    #            
    def get_value(self, message, key, default_value=""):
        if (key in message) and (type(message) is dict):
            value = message[key]
        else:
            value = default_value
        return value         

    # Tell the server that we're an agent who wants to connect.
    # TODO: Proper authentication
    #
    def agent_connect(self, ros_instance_base, org_id, machine_id, username, secret):
        print "Connecting agent: " +  ros_instance_base
        # TODO Human name hard-coded
        self.send_to_server('agentConnect', {'rosinstance': ros_instance_base, 'org': org_id, 'user': username, 'secret': secret, 'hostname' : machine_id, 'rosInstanceHuman' : 'Pi Robot'})
        
    # Send a (mtype, mbody) message to the server. Buffer message if the websocket isn't ready.   
    # Currently, this drops a message if it is so big that it is likely to crash the websocket.
    # TODO: better handling of large messages.
    #
    def send_to_server(self, mtype, mbody, binary=False):
        message = {'mtype': mtype, 'mbody': mbody}
        if binary:
            # This bizarre dance is because BSON seems to serialize embedded objects as binary data
            # The following will "normalize" the JSON array before trasnmitting
            message = json.dumps(message)
            message = json.loads(message)
            message = bson.dumps(message)

            if (self.connected):
                self.flush_buffer()
                self.safe_send(message, binary)
            else:
                self.add_to_output_buffer(message)
        else:
            message = json.dumps(message)
            if (len(message)<40000):
                if (mtype!="graphUpd"):
                    foo = 0
                    #print "agent -> server: " + message
                if (self.connected):
                    self.flush_buffer()
                    self.safe_send(message, binary)
                else:
                    self.add_to_output_buffer(message)
                    print "Message to server stored in output buffer - no connection yet."
            else:
                print "=============================="
                print "Message too long - dropping!!!"        
                print "=============================="
                print mtype
                print mbody
                print mtype
                print "=============================="

    # Save message for later transmission.
    #
    def add_to_output_buffer(self, message):
        if (len(self.output_buffer)>=SIZE_OUTPUT_BUFFER):
            print "================================"
            print "OUTPUT BUFFER OVERFLOW!"
            print "Dropping message..."
            print "================================"
            sys.exit()
        self.output_buffer.append(message)
          
    # Send buffered messages to the server.
    #      
    def flush_buffer(self):
        i = 1
        while (len(self.output_buffer)>0):
            print "Sending message %s from output buffer." % i
            message = self.output_buffer.pop(0)
            self.safe_send(message)
            i = i + 1

    # Send a message to the server, locking the websocket.
    #
    def safe_send(self, message, binary=False):
        self.output_lock.acquire()
        try:
            self.ws.send(message, binary)
        finally:    
            self.output_lock.release()




        
        
        