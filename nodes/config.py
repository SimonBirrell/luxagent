#!/usr/bin/env python

import os, sys
from subprocess import call, Popen

Config_dict = None

def get_config_value(key):
	create_config_file_if_necessary()
	return get_value(key)

def create_config_file_if_necessary():
	global Config_dict

	if Config_dict==None:
		script_dir, filename = os.path.split(sys.argv[0])
		config_dir = script_dir + "/../"
		config = "config.txt"
		config_default = "config-default.txt"
		config_pathname = config_dir + "/" + config
		config_default_pathname = config_dir + "/" + config_default
		config_exists = os.path.isfile(config_pathname)

		# Copy config-default.sh to config.sh if it doesn't already exist
		print "Checking for existence of", config_pathname, config_exists 
		if config_exists == True:
			print config_pathname + " already exists"
		else:	
			print "Copying " + config_default_pathname + " to " + config_pathname
			call(["cp", config_default_pathname, config_pathname])	

		Config_dict = {}
		with open(config_pathname, 'r') as f:
			for line in f:
				splitLine = line.split(':')
				if len(splitLine)>1:
					Config_dict[splitLine[0]] = splitLine[1]
		print "Config_dict"
		print Config_dict		

def get_value(key):
	print "Searching for key", key
	global Config_dict

	return Config_dict[key]
