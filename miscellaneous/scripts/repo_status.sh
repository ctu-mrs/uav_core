#!/bin/bash

actual_dir=`dirname "$0"`
source $actual_dir/script_functions.sh
source $actual_dir/config.sh



echo -e "$header_begin""Your git repo status are:""$header_end"
for gitrepo in "${git_repos_to_check[@]}"    
do 
    #echo $gitrepo
    cd ~/git/$gitrepo
    current_branch=`git branch | grep \* | cut -d ' ' -f2`
    git fetch $current_branch &> /dev/null
    state=`git status | head -2 | tail -1`
    check_up_to_date="up-to-date"
    
    echo -e "- $header_begin""repository ""$green_color""$gitrepo""$default_color"" is on ""$green_color""$current_branch""$default_color"" branch""$header_end"
    if [[ $state = *"$check_up_to_date"* ]]; then
        echo -e "- $header_begin""repository ""$green_color""$gitrepo""$default_color"" status ""$green_color""$state""$default_color""$header_end"
    else
        echo -e "- $header_begin""repository ""$green_color""$gitrepo""$default_color"" status ""$red_color""$state""$default_color""$header_end"
    fi
    
done
