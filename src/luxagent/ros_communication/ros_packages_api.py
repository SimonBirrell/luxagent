# RosPackagesApi() is a class that interacts with installed packages on the host.
# There is no decent API for this in ROS 1.0, so some crawling of the file system is needed.
#

import os
import stat
import subprocess

import rospkg

class RosPackagesApi(object):
	# Open and inject the RosCommunication object for callbacks.
	#
	def open(self, ros_comm):
		print "Opening package API"
		self.ros_comm = ros_comm
		self.package_tree = {}
		# TODO: This is getting called before agent has registered observers
		# with Ros_Comm. May need rethink on startup sequence

	# Clean up
	def close(self):
		print "Closing package API"

	# Construct a graph of the packages installed on this machine.
	# A package may contain three types of executables:
	# 	Scripts: general executables from the /scripts folder
	# 	Nodes: executables from the /nodes folder. These launch a single node
	#	Launch Scripts: executable that launch some combination of nodes and roscore if required
	#
	#  {
	#		package1: {
	#					s: [script1, script2...],
	#					n: [node1, node2...],
	#					l: [launch script1, launch script2...]
	#				  }
	#		package2: ...
	#  }
	#
	def get_package_tree(self):
		self.rospack = rospkg.RosPack()
		package_list = self.rospack.list()
		self.package_tree = {}
		for package in package_list:
			self.root_directory = self.rospack.get_path(package)
			rosrun_scripts = self.get_rosrun_scripts(package)
			rosrun_nodes = self.get_rosrun_nodes(package)
			roslaunch_targets = self.get_roslaunch_targets(package)
			if (len(rosrun_scripts) + len(rosrun_nodes) + len(roslaunch_targets) > 0):
				self.package_tree[package] = {
					's' : rosrun_scripts,
					'n' : rosrun_nodes,
					'l' : roslaunch_targets
				}
		return self.package_tree

	# Executables in the /scripts or /rosbuild/scripts directories of the package.
	#
	def get_rosrun_scripts(self, package):
		scripts = self.get_executables_in_directory(self.root_directory + '/scripts') 
		scripts += self.get_executables_in_directory(self.root_directory + '/rosbuild/scripts')
		return scripts

	# Executables in the /nodes or /rosbuild/nodes directories of the package.
	#
	def get_rosrun_nodes(self, package):
		executables = self.get_executables_in_directory(self.root_directory + '/nodes')
		executables += self.get_executables_in_directory(self.root_directory + '/rosbuild/nodes')
		return executables

	# .launch files in the /launch folder.
	#
	def get_roslaunch_targets(self, package):
		launch_files = self.get_launch_files_in_directory(self.root_directory + '/launch')
		return launch_files

	# .launch files in a supplied directory.
	#
	def get_launch_files_in_directory(self, directory):
		if (not os.path.isdir(directory)):
			return []

		python_files = []
		for filename in os.listdir(directory):
			if (len(filename)>6):
				if (filename[-7:] == '.launch'):
					python_files.append(filename[:-7])

		return python_files					

	# Crawl the filesystem. Apparently ROS 2.0 will have an API for this.
	#
	def get_executables_in_directory(self, directory):
		if (not os.path.isdir(directory)):
			return []

		executable = stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH
		executable_list = []

		for filename in os.listdir(directory):
			pathname = directory + "/" + filename
			if os.path.isfile(pathname):
				st = os.stat(pathname)
				mode = st.st_mode
				if mode & executable:
					executable_list.append(filename)

		return executable_list

	# UNUSED. Candidate for deletion.
	#
	def get_python_scripts_in_directory(self, directory):
		if (not os.path.isdir(directory)):
			return []

		python_files = []
		for filename in os.listdir(directory):
			if (filename[-3:] == '.py'):
				python_files.append(filename)

		return python_files					

	# Called to execute rosrun <args>
	# Equivalent to launching from the shell.
	#	
	def rosrun(self, args):
		print "rosrun low level"
		args.insert(0, 'rosrun')
		print args
		p = subprocess.Popen(args)
		print "subprocess launched with PID"

	# Called to execute roslaunch <args>
	# Equivalent to launching from the shell.
	#	
	def roslaunch(self, args):
		print "rosrun low level"
		args.insert(0, 'roslaunch')
		args[2] += ".launch"
		print args
		p = subprocess.Popen(args)
		print "subprocess launched with PID"

	# Kill a process in execution. Right now the server supplies a PID value.
	# Would be more secure just to have the name of the process pass back and forth in the protocol.
	#	
	def kill(self, args):
		print "kill low level"
		args.insert(0, 'kill')
		args.insert(1, '-SIGINT')
		args[2] = str(args[2])
		print args
		p = subprocess.Popen(args)
		print "subprocess launched with PID"


