#!/usr/bin/env python

# Module that deals with communication with LuxManager API
#

import requests
import json

class ManagerCommunication():

	def __init__(self, agent_guid, agent_password):
		self.agent_guid = agent_guid
		self.agent_password = agent_password

	def logon(self):
		print "Logging on with LuxManager"
		url = self.api_host() + "/api/v1/users/sign_in"
		parameters = { 'email': self.agent_guid, 'password': self.agent_password }
		head = { 'CONTENT_TYPE': 'application/json', 'ACCEPT': 'application/json' }
		ret = requests.post(url, params=parameters, headers=head)
		print "LuxManager status code", ret.status_code
		if ret.status_code != 201:
			raise ManagerError("Bad status code returned in logon()")
		self.sign_in_info = ret.json()
		return self.sign_in_info['user']['auth_token']

	def api_host(self):
		return "http://app.robotlux.com"	

class ManagerError(Exception):
	def __init__(self, value):
		self.value = value
	def __str__(self):
		return repr(self.value)	
