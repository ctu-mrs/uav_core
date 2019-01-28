#!/bin/bash
actual_dir=`dirname "$0"`
source $actual_dir/script_functions.sh
source $actual_dir/config.sh


current_arena_is=` ls -l $actual_world_location  | grep -Po '(?<=->\s)\w+' `
echo -e "$header_begin""Current world is set to ""$green_color$current_arena_is""$header_end"
