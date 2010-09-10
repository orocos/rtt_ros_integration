#!/usr/bin/python
import sys

import roslib
roslib.load_manifest("ros_integration");

import roscpp
import roscpp.msg_gen 
from  roslib import packages,msgs 
 

NAME='create_boost_headers'

def create_boost_headers(argv, stdout, stderr):
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog [packages]", prog=NAME)
    (options, args) = parser.parse_args(argv)

    # get the file name
    if len(args) < 2:
        parser.error("you must specify at least package")
    pkgs = args[1:]
    for pkg in pkgs:
        pp = roslib.packages.get_pkg_dir(pkg);
        msgs = roslib.msgs.list_msg_types(pkg,False)
        for msg in msgs:
            roscpp.msg_gen.generate_boost_serialization(pp+'/msg/'+msg+'.msg')

if __name__ == "__main__":
    try:
        create_boost_headers(sys.argv, sys.stdout, sys.stderr)
    except Exception, e:
        print >> sys.stderr, e
        sys.exit(1)
