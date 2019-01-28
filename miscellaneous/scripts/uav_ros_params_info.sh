#!/bin/bash
actual_dir=`dirname "$0"`
source $actual_dir/script_functions.sh
source $actual_dir/config.sh

echo -e "$header_begin""Your uav ros params are set to:""$header_end"
for variable in "${ros_uav_variables_to_show[@]}"    
do
    
    var=${variable}
    eval var_value=\$$var
    echo -e "- $header_begin""${variable} set to $green_color$var_value""$header_end"
done

#catkin config
extending_type=`catkin config | grep "Extending:" | awk '{print $2}'`
if [ "[env]" == "$extending_type" ];then
    actual_workspace=`catkin config | grep "Extending:" | awk '{print $3}' | awk -F/devel '{print $1}'`
else 
    actual_workspace=`catkin config | grep "Workspace:" | awk '{print $2}'`
fi

echo -e "- $header_begin""Your actual workspace is: ""$green_color""$actual_workspace""$header_end"