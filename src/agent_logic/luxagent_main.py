from __future__ import print_function

import sys
import time

from rosgraph import roslogging
from rosgraph import masterapi

from rosgraph.impl import graph

def fullusage():
    print("""rosgraph is a command-line tool for debugging the ROS Computation Graph.

Usage:
\trosgraph
""")
    
def luxagent_main():
    if len(sys.argv) == 1:
        pass
    elif len(sys.argv) == 2 and (sys.argv[1] == '-h' or sys.argv[1] == '--help'):
        fullusage()
        return
    else:
        fullusage()
        sys.exit(-1)
    
    roslogging.configure_logging('rosgraph')

    # make sure master is available
    master = masterapi.Master('rosgraph')
    try:
        master.getPid()
    except:
        print("ERROR: Unable to communicate with master!", file=sys.stderr)
        return
        
    g = graph.Graph()
    try:
        while 1:
          g.update()

          if not g.nn_nodes and not g.srvs:
              print("empty")
          else:
              print('\n')
          if g.nn_nodes:
              print('Nodes:')
              for n in g.nn_nodes:
                  prefix = n + '|'
                  print('  ' + n + ' :')
                  print('    Inbound:')
                  for k in g.nn_edges.edges_by_end.keys():
                      if k.startswith(prefix):
                          for c in g.nn_edges.edges_by_end[k]:
                              print('      ' + c.start)
                  print('    Outbound:')
                  for k in g.nn_edges.edges_by_start.keys():
                      if k.startswith(prefix):
                          for c in g.nn_edges.edges_by_start[k]:
                              print('      ' + c.end)
          if g.srvs:
              print('Services:')
              for s in g.srvs:
                  print('  ' + s)

          time.sleep(1.0)
    except KeyboardInterrupt:
        pass

