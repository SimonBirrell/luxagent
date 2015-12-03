# Config file for running agent in production environment.
#
# The server is assumed to be running on Heroku.
#
SERVER_URL = 'ws://luxserver.herokuapp.com:80/'

class AgentConfig(object):

	def sockets_server(self):
		return SERVER_URL
