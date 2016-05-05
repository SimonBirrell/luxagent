#!/usr/bin/env python

import os, sys
from subprocess import call, Popen
from tempfile import mkstemp
from shutil import move

Config_dict = None

def get_config_value(key):
	create_config_file_if_necessary()
	return get_value(key)

def set_config_value(key, value):
	create_config_file_if_necessary()
	return set_value(key, value)	

def create_config_file_if_necessary():
	global Config_dict

	if Config_dict==None:
		# script_dir, filename = os.path.split(sys.argv[0])
		# config_dir = script_dir + "/../"
		# config = "config.txt"
		# config_default = "config-default.txt"
		# config_pathname = config_dir + "/" + config
		# config_default_pathname = config_dir + "/" + config_default
		config_pathname, config_default_pathname = get_pathnames()
		config_exists = os.path.isfile(config_pathname)

		# Copy config-default.sh to config.sh if it doesn't already exist
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

def get_value(key):
	global Config_dict
	if key in Config_dict.keys():
		return Config_dict[key]

	return None

def set_value(key, value):
	print key, "=", value
	previous_value = get_value(key)
	if previous_value==None:
		append_to_file(key, value)
	else:
		replace_key_value(key, value)

def append_to_file(key, value):
	config_pathname, config_default_pathname = get_pathnames()
	with open(config_pathname, "a") as myfile:
		myfile.write(key + ":" + value)

def replace_key_value(key, value):
	config_pathname, config_default_pathname = get_pathnames()
	# Create temp file
	fh, abs_path = mkstemp()
	with open(abs_path,'w') as new_file:
		with open(config_pathname) as old_file:
			for line in old_file:
				splitLine = line.split(':')
				if len(splitLine)>1:
					if splitLine[0] == key:
						new_file.write(key + ":" + value)
					else:
						new_file.write(line)
	os.close(fh)
	# Remove original file
	os.remove(config_pathname)
	# Move new file
	move(abs_path, config_pathname)

def get_pathnames():
	script_dir, filename = os.path.split(sys.argv[0])
	config_dir = script_dir + "/.."
	config = "config.txt"
	config_default = "config-default.txt"
	config_pathname = config_dir + "/" + config
	config_default_pathname = config_dir + "/" + config_default
	return config_pathname, config_default_pathname

