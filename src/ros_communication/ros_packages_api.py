import os
import stat
import subprocess

import rospkg

class RosPackagesApi(object):

	def open(self, ros_comm):
		print "Opening package API"
		self.ros_comm = ros_comm
		self.package_tree = {}
		# RESTART: This is getting called before agent has registered observers
		# with Ros_Comm. May need rethink on startup sequence

	def close(self):
		print "Closing package API"

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

	def get_rosrun_scripts(self, package):
		scripts = self.get_executables_in_directory(self.root_directory + '/scripts') 
		scripts += self.get_executables_in_directory(self.root_directory + '/rosbuild/scripts')
		return scripts

	def get_rosrun_nodes(self, package):
		executables = self.get_executables_in_directory(self.root_directory + '/nodes')
		executables += self.get_executables_in_directory(self.root_directory + '/rosbuild/nodes')
		return executables

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

	def get_python_scripts_in_directory(self, directory):
		if (not os.path.isdir(directory)):
			return []

		python_files = []
		for filename in os.listdir(directory):
			if (filename[-3:] == '.py'):
				python_files.append(filename)

		return python_files					

	def get_launch_files_in_directory(self, directory):
		if (not os.path.isdir(directory)):
			return []

		python_files = []
		for filename in os.listdir(directory):
			if (len(filename)>6):
				if (filename[-7:] == '.launch'):
					python_files.append(filename[:-7])

		return python_files					

	def get_roslaunch_targets(self, package):
		launch_files = self.get_launch_files_in_directory(self.root_directory + '/launch')
		return launch_files

	def rosrun(self, args):
		print "rosrun low level"
		args.insert(0, 'rosrun')
		print args
		p = subprocess.Popen(args)
		print "subprocess launched with PID"

	def roslaunch(self, args):
		print "rosrun low level"
		args.insert(0, 'roslaunch')
		args[2] += ".launch"
		print args
		p = subprocess.Popen(args)
		print "subprocess launched with PID"

	def kill(self, args):
		print "kill low level"
		args.insert(0, 'kill')
		args.insert(1, '-SIGINT')
		args[2] = str(args[2])
		print args
		p = subprocess.Popen(args)
		print "subprocess launched with PID"


