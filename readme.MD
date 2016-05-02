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

New features:
- Add general compression to protocol. Have tried LZW but without success so far (apparent incompatibilities between JS and Python implementations).
- Add specific, efficient compression for images and streams of images.
- Add more robust authentication, configured on the client.
- Improve robustness, error trapping in sockets. Right now the Gevent library makes trapping errors tricky.
- Ideally, agents and browsers should be able to invisibly reconnect after server reboot.
- Send some machine-specific metrics, e.g. CPU usage, memory.
- Get Python dependencies to auto-install via ROS mechanisms
- Automatic update of Agent software when a new version is published
- Pre-launch
- Check out some rosgraph error messages at startup

License
-------
(c) 2015 Robot Lux. All Rights Reserved.

[![Circle CI](https://circleci.com/gh/manacoa/luxagent.svg?style=svg&circle-token=812c7117d6a3f1d2d83ce1dd0994cf5a530cb9e8)](https://circleci.com/gh/manacoa/luxagent)

