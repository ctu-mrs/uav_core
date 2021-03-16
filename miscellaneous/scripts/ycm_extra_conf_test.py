#!/usr/bin/python3

from os.path import expanduser
HOME = expanduser("~")

exec(open(HOME+"/.ycm_extra_conf.py").read())

workspace = GetWorkspacePath(HOME+'/git/uav_core/ros_packages/mrs_uav_managers/src/control_manager.cpp')
print("workspace = " + workspace)

compile_params_folder = GetCompilationDatabaseFolder(HOME+'/git/uav_core/ros_packages/mrs_uav_managers/src/control_manager.cpp')
print("folder = " + compile_params_folder)
