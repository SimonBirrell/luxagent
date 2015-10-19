LuxAgent
========
LuxAgent is a ROS node that is installed on a machine in the target ROS instance. It communicates with the ROS master and the local system and sends information to the server LuxServer for retransmitting to the appropriate browsers. It also receives commands and executes them, as well as topic messages that it publishes.

LuxAgent is implemented in Python using the Gevent library for cooperative multi-tasking.

Running
-------

From within a machine with ROS installed:

```
roslaunch luxagent luxagent.py
```

From a machine without ROS (won't do much):
```
cd src/luxagent
python luxagent.py
```

Tests
-----
```
cd src
nosetests --with-watch -s
```

Nosetests with watch require the following plugin to work:

pip install nose-watch

NOTE: Currently these can be run on a machine even if ROS isn't installed. This is because the code for libraries rosgraph and rosbridge have been included in the repo. These should be removed for cleanliness which will then mean you need to run the tests from a ROS-enabled machine.

Protocol
--------
The protocol used for server communication is defined in the README for LuxServer. It is sent (currently) in either JSON or BSON formats.

TODO
----
General Tidy up:
- The code for two standard modules ROSGRAPH and ROSBRIDGE are included in the repo because I haven't found a way of importing them reliably otherwise. This should be fixed and the code for these two modules removed.

New features:
- Add general compression to protocol. Have tried LZW but without success so far (apparent incompatibilities between JS and Python implementations).
- Add specific, efficient compression for images and streams of images.
- Add more robust authentication, configured on the client.
- Improve robustness, error trapping in sockets. Right now the Gevent library makes trapping
- errors tricky.
- Ideally, agents and browsers should be able to invisibly reconnect after server reboot.
- Send some machine-specific metrics, e.g. CPU usage, memory.

License
-------
(c) 2015 Robot Lux. All Rights Reserved.
