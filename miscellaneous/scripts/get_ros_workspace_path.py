#!/usr/bin/python

import rospkg
import os
from glob import glob

import argparse
import sys

def GetWorkspacePath(package="", filename=""):

    if package == "" and filename == "":
        return ''

    # get the content of $ROS_WORKSPACE variable
    # and create an array out of it
    paths =  os.path.expandvars('$ROS_WORKSPACES')
    workspaces = paths.split()

    # iterate over all workspaces
    for single_workspace in workspaces:

        # get the full path to the workspace
        workspace_path = os.path.expanduser(single_workspace)

        # get all ros packages built in workspace's build directory
        paths = glob(workspace_path + "/build/*")

        # iterate over all the packages built in the workspace
        for package_path in paths:

            # test whether the package, to which "filename" belongs to, is in the workspace
            if package_path.endswith(package):

                # if it is, return path to its workspace
                return workspace_path

    return ''

def getOptions(args=sys.argv[1:]):

    parser = argparse.ArgumentParser(description="Parses command.")
    parser.add_argument("-p", "--package", help="A package name.")
    parser.add_argument("-f", "--file", help="A file name.")
    options = parser.parse_args(args)
    return options

options = getOptions(sys.argv[1:])

workspace_path = ""

if options.package:
    workspace_path = GetWorkspacePath(package=options.package)
elif options.file:
    workspace_path = GetWorkspacePath(filename=options.file)
else:
    exit(1)

print(workspace_path)

exit(0)
