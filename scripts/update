#!/usr/bin/env python

# This script updates to the latest version of the luxagent code
# on github. The branch used is 'distribution'.

import os
import sys
from subprocess import call, Popen

def get_base_dir():
	script_dir, filename = os.path.split(sys.argv[0])
	base_dir = script_dir + "/.."
	return base_dir	

def update():
	#script_dir, filename = os.path.split(sys.argv[0])
	#base_dir = script_dir + "/.."
	base_dir = get_base_dir()

	print "Update"
	call(['cat', 'src/luxagent/version.py'], cwd=base_dir)
	print "Configuring git"
	call(['git', 'config', 'user.email', 'foo@bar.com'], cwd=base_dir)
	print "git fetch --all"
	call(['git', 'fetch', '--all'], cwd=base_dir)
	print "git reset --hard origin/distribution"
	call(['git', 'reset', '--hard', 'origin/distribution'], cwd=base_dir)
	print "update complete"

# def get_next_script_from_args(args):
# 	# See if we need to pass on next script option to pull.sh
# 	next_script = None
# 	next_file_option = '-n'
# 	try:
# 		opts, args = getopt.getopt(args,"hn:")
# 	except getopt.GetoptError:
# 		print 'Error in parameters:'
# 		print 'update'
# 		print 'update -n <next script>'
# 		sys.exit(2)
# 	for opt, arg in opts:
# 		if opt == '-h':
# 			print 'Update Utility for Luxagent'
# 			print 'Options:'
# 			print '-h Print help'
# 			print next_file_option + ' <script> Next script to run.'
# 		elif opt == next_file_option:
# 			next_script = arg
# 	return next_script		

if __name__== '__main__':
	args = sys.argv[1:]
	base_dir = get_base_dir()
	#next_script = get_next_script_from_args(args)
	update()
	import luxagent
	print "Running version", luxagent.version()
	# if next_script != None:
	# 	print "Calling next script", next_script
	# 	call([next_script], cwd=base_dir)
	#luxagent.main()
