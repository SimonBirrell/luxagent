#!/usr/bin/env python

import re
import urllib2
import json
import getpass
from config import get_config_value, set_config_value

def do_request(req):
	success = False
	url = req.get_full_url()
	try:
		response_file = urllib2.urlopen(req)
	except urllib2.HTTPError as e:
		print "The server couldn't fulfill the request. (", e.code, ")"
		print e.read()
	except urllib2.URLError as e:
		print "We failed to reach the server", url
		print "Reason:", e.Reason
	else:		
		response = response_file.read()
		response_file.close()
		try:
			info = json.loads(response)
			return True, info
		except:
			print "Invalid result from", url
			print response
	return False, None

def sign_in():
	logged_in = False
	while (not logged_in):
		email = ""
		password = ""
		print
		while not valid_email(email):
			email = raw_input("Your RobotLux Email: ")
		while not valid_password(password):
			password = getpass.getpass("Your RobotLux Password: ")
			#password = raw_input("Your RobotLux Password: ")
		print

		# Get org info
		print "Logging in to RobotLux manager..."
		print
		sign_in_url = 'http://app.robotlux.com/api/v1/users/sign_in'
		values = {'email': email, 'password': password}
		data = json.dumps(values)
		req = urllib2.Request(sign_in_url, data, {'Content-Type': 'application/json', 'Accept': 'application/json'})
		logged_in, info = do_request(req)
	user_info = info['user']
	auth_token = user_info['auth_token']	
	org_id = user_info['org_id']
	return email, auth_token, org_id

def get_org_info(email, auth_token, org_id):
	get_org_info_url = 'http://app.robotlux.com/api/v1/orgs/' + str(org_id) + '/agents_info'
	req = urllib2.Request(get_org_info_url, headers = {'Content-Type': 'application/json', 'Accept': 'application/json', 'X-API-EMAIL': email, 'X-API-TOKEN': auth_token})
	success, info = do_request(req)
	return info['slug'], info['name'], info['agents']

def register_agent(email, auth_token, org_id, agent_slug):
	register_url = 'http://app.robotlux.com/api/v1/orgs/' + str(org_id) + '/agents'
	values = {'agent': {'slug': agent_slug}}
	data = json.dumps(values)
	req = urllib2.Request(register_url, data, {'Content-Type': 'application/json', 'Accept': 'application/json', 'X-API-EMAIL': email, 'X-API-TOKEN': auth_token})
	registered_ok, info = do_request(req)
	return registered_ok, info

def install(force_reconfig=False):
	agent_guid = get_config_value('agent_guid')
	agent_password = get_config_value('agent_password')
	print "agent_guid", agent_guid
	if force_reconfig or (agent_guid==None) or (agent_password==None):
		print
		print "======================================================"
		print "Welcome to Luxagent configuration"
		print "Please follow the instructions to configure your agent"
		print "======================================================"
		print

		# Ask user for info
		print "1. First, we need to get some information from RobotLux"
		print "Please give your user credentials so I can log on."
		email, auth_token, org_id = sign_in()
		org_slug, org_name, org_agents = get_org_info(email, auth_token, org_id)

		print "Welcome to", org_name, "!"
		existing_agent_slugs = map(lambda s: s['slug'], org_agents)
		existing_agent_slugs_list = ', '.join(map(str, existing_agent_slugs))
		if len(org_agents)>0:
			print "The following agents are already registered:", existing_agent_slugs_list
		else:
			print "This will be the first agent in the organization."
		print

		# Ask user for agent name
		print "2. Please enter a unique name for the agent. It should describe the machine it is installed on."
		print "Please use only letters, numbers, underscores and dashes, with no spaces or other punctuation."
		print
		agent_slug = ""
		while (agent_slug in existing_agent_slugs) or not valid_name(agent_slug):
			agent_slug = raw_input("Agent name: ")
		print	
		print "Agent name " + agent_slug + " is good."	
		print

		# Register agent
		print "Registering agent " + agent_slug + " with RobotLux manager..."
		print
		registered_ok, info = register_agent(email, auth_token, org_id, agent_slug)
		if not registered_ok:
			print "ERROR while registering agent. Please try again."
			print info
			sys.exit()
		print "Agent", agent_slug, "registered ok."	
		agent_guid = agent_slug + "@" + org_slug + ".orgs.robotlux.com"
		agent_password = info['password']
		print "Agent GUID", agent_guid
		print

		# Save agent_guid
		print "Saving Agent details to config.txt..."
		set_config_value('agent_guid', agent_guid)
		set_config_value('agent_password', agent_password)
		print
		print "Launching LuxAgent node..."
		print

	return agent_guid, agent_password	

def valid_name(name):
	if len(name)==0:
		return False
	# http://stackoverflow.com/questions/89909/how-do-i-verify-that-a-string-only-contains-letters-numbers-underscores-and-da	
	#return (re.match("^[A-Za-z0-9_-]*$", name) != None)
	return (re.match(r'[\w-]*$', name) != None)

def valid_email(name):
	return len(name)>0

def valid_password(name):
	return len(name)>0




