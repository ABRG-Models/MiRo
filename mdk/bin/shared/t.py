import sys

print sys.argv
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--robot_name", type=str)

pargs = parser.parse_args(sys.argv[1:])

print pargs

if pargs.robot_name is not None :
    print "success"
