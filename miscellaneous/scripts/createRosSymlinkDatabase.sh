#!/bin/bash

echo "Generating symlink database"

if [ ! -x "$(command -v ag)" ]; then
  echo "'ag' not installed"
  return 1
fi

SYMLINK_ARRAY_PATH="/tmp/symlink_array.sh"
symlink_list_tmp_file="/tmp/symlink_list_$RANDOM.txt"

IFS=' ' read -r -a WORKSPACES <<< "$ROS_WORKSPACES" # `

dirs=""

# for both
for ((j=0; j < ${#WORKSPACES[*]}; j++));
do

  # get me the absolute path
  eval workspace_path=${WORKSPACES[$j]}

  echo "running silver searcher on '$workspace_path'"

  workspace_files=`ag -f $workspace_path --nocolor -g ""`
  workspace_dirs=$(echo "$workspace_files" | sed -e 's:/[^/]*$::' | sort | uniq)

  dirs="$dirs $workspace_dirs"

done

# for all the symlinks that we found
for dir in `echo $dirs`
do

  # echo "Evaluating: $dir"

  # "original" = where the link is pointing to
  original=$(readlink "$dir")

  # if the "original" path is not empty
  if [[ ! -z "$original" ]];
  then

    # if the "original" path starts with "."
    # which means its a relative link
    if [[ "$original" == "."* ]] || [[ "$original" != *\/* ]]
    then
      #echo "original1 $original"
      # resolve the relative link
      temp="${dir%/*}/$original"
      original=`( builtin cd "$temp" && pwd )`
      #echo "original2 $original"
    fi

    # the linked path must not contain /git/
    if [[ $dir == *\/git\/* ]]
    then
      echo -e "\e[31mReject $dir\e[39m"
      continue
    else
      #echo "original final $original"
      echo -e "\e[32mAccept: $dir -> $original\e[39m"
    fi

    # put it into our output file
    echo "$dir, $original" >> "$symlink_list_tmp_file"
  fi
done

# delete duplicite lines in the file
cat "$symlink_list_tmp_file" | uniq > "$symlink_list_tmp_file.temp"
mv "$symlink_list_tmp_file.temp" "$symlink_list_tmp_file"

#######################################################################
# stage 2

# parse the csv file and extract file paths
i="1"

SYMLINK_LIST_PATHS1=()
SYMLINK_LIST_PATHS2=()

while IFS=, read -r path1 path2; do

  if [[ "$path1" == *ctop_planner* ]] || [[ "$path2" == *ctop_planner* ]]
  then
    continue
  fi

  SYMLINK_LIST_PATHS1[$i]=`eval echo "$path1"`
  SYMLINK_LIST_PATHS2[$i]=`eval echo "$path2"`

  # echo "${SYMLINK_LIST_PATHS1[$i]} -> ${SYMLINK_LIST_PATHS2[$i]}"

  i=$(expr $i + 1)
done < "$symlink_list_tmp_file"

rm $symlink_list_tmp_file

array_list_tmp_file="/tmp/symlink_array_$RANDOM.sh"
touch $array_list_tmp_file

echo "SYMLINK_LIST_PATHS1=(" > $array_list_tmp_file
for ((i=1; i < ${#SYMLINK_LIST_PATHS1[*]}+1; i++));
do
  echo "\"${SYMLINK_LIST_PATHS1[$i]}\" " >> $array_list_tmp_file
done
echo ")
" >> $array_list_tmp_file

echo "SYMLINK_LIST_PATHS2=(" >> $array_list_tmp_file
for ((i=1; i < ${#SYMLINK_LIST_PATHS2[*]}+1; i++));
do
  echo "\"${SYMLINK_LIST_PATHS2[$i]}\" " >> $array_list_tmp_file
done
echo ")" >> $array_list_tmp_file

mv "$array_list_tmp_file" "$SYMLINK_ARRAY_PATH"
