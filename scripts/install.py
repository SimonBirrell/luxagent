#!/usr/bin/env python

from config import get_config_value, set_config_value

def install(force_reconfig=True):
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
		# Get org info
		# Ask user for agent name
		# Register agent
		# Save agent_guid
		agent_guid = 'foobaz'
		print "Saving Agent GUID", agent_guid
		set_config_value('agent_guid', agent_guid)

	return agent_guid	