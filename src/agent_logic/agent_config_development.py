# Config file for running agent in development environment.
#
# I had some difficulty using localhost, so that's why I'm
# using a Bonjour-enabled URL for my local machine.
#

SERVER_URL = 'ws://simonbirrell.local:8080/'

class AgentConfig(object):

	def sockets_server(self):
		return SERVER_URL
