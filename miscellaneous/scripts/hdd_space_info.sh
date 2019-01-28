#!/bin/bash
actual_dir=`dirname "$0"`

source $actual_dir/script_functions.sh
source $actual_dir/config.sh

space_info=`df -BG --output=size,avail,target | grep "/$"`
total_size=`echo $space_info | awk '{print $1}'`
avaliable_size=`echo $space_info | awk '{print $2}'`
avail_num_only=`echo $avaliable_size | grep -o -E '[0-9]+'`

if (( $avail_num_only < $low_space_treshold_GB )); then
    echo -e "$header_begin""Your disk has only ""$red_color""$avaliable_size""$default_color"" available space out of $total_size.""$header_end"
else
    echo -e "$header_begin""Your disk has ""$green_color""$avaliable_size""$default_color"" available space out of $total_size.""$header_end"
fi

