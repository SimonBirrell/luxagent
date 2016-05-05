#!/usr/bin/env python

import re
from config import get_config_value, set_config_value

def install(force_reconfig=False):
	agent_guid = get_config_value('agent_guid')
	print "agent_guid", agent_guid
	if force_reconfig or (agent_guid==None):
		print
		print "======================================================"
		print "Welcome to Luxagent configuration"
		print "Please follow the instructions to configure your agent"
		print "======================================================"
		print

		# Ask user for info
		print "1. First, we need to get some information from RobotLux"
		print
		username = ""
		password = ""
		while not valid_username(username):
			username = raw_input("RobotLux Username: ")
		while not valid_password(password):
			password = raw_input("RobotLux Password: ")
		print

		# Get org info
		print "Logging on to RobotLux manager..."
		print
		org_info = {'name': 'MyOrg', 'agents': ['foo', 'bar']}
		print "Org info", org_info
		print

		# Ask user for agent name
		existing_agents = org_info['agents']
		print "2. Please enter a unique name for the agent that describes the machine it is installed on."
		print "Please use only letters, numbers, underscores and dashes, with no spaces or other punctuation."
		print
		agent_name = ""
		while (agent_name in existing_agents) or not valid_name(agent_name):
			agent_name = raw_input("Agent name: ")
		print	
		print "Agent name " + agent_name + " is good."	
		print

		# Register agent
		print "Registering agent " + agent_name + " with RobotLux manager..."
		print
		agent_guid = 'foobaz'

		# Save agent_guid
		print "Saving Agent GUID", agent_guid
		set_config_value('agent_guid', agent_guid)

	return agent_guid	

def valid_name(name):
	if len(name)==0:
		return False
	# http://stackoverflow.com/questions/89909/how-do-i-verify-that-a-string-only-contains-letters-numbers-underscores-and-da	
	#return (re.match("^[A-Za-z0-9_-]*$", name) != None)
	return (re.match(r'[\w-]*$', name) != None)

def valid_username(name):
	return len(name)>0

def valid_password(name):
	return len(name)>0




