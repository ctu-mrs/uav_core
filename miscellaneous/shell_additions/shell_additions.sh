UAV_CORE_PATH=$GIT_PATH/uav_core/

# disable gitman caching
export GITMAN_CACHE_DISABLE=1

# #{ generating ctags

# use ctags to generate code tags

# generate projects' tags
if [ -z $TMUX ]; then

  if [ ! -e ~/tags ]; then
    ctagscmd="ctags --fields=+l -f ~/tags $CTAGS_SOURCE_DIR"
    $UAV_CORE_PATH/miscellaneous/scripts/detacher.sh "$ctagscmd"
  fi

  if [ ! -e ~/tags-once ]; then
    # generate `once generated tags`, e.g. ROS's tags
    if [ ! -e $(eval echo "$CTAGS_FILE_ONCE") ]; then
      ctagscmd="ctags --fields=+l -f $CTAGS_FILE_ONCE $CTAGS_ONCE_SOURCE_DIR"
      $UAV_CORE_PATH/miscellaneous/scripts/detacher.sh "$ctagscmd"
    fi
  fi

fi

# #}

# #{ killp()

# allows killing process with all its children
killp() {

  if [ $# -eq 0 ]; then
    echo "The command killp() needs an argument, but none was provided!"
    return
  else
    pes=$1
  fi

  for child in $(ps -o pid,ppid -ax | \
    awk "{ if ( \$2 == $pes ) { print \$1 }}")
    do
      # echo "Killing child process $child because ppid = $pes"
      killp $child
    done

# echo "killing $1"
kill -9 "$1" > /dev/null 2> /dev/null
}

# #}

# #{ sourceShellDotfile()

getRcFile() {

  case "$SHELL" in
    *bash*)
      RCFILE="$HOME/.bashrc"
      ;;
    *zsh*)
      RCFILE="$HOME/.zshrc"
      ;;
  esac

  echo "$RCFILE"
}

sourceShellDotfile() {

  RCFILE=$( getRcFile )

  source "$RCFILE"
}

# #}
alias sb="sourceShellDotfile"

# #{ cd()

SYMLINK_LIST_PATH="/tmp/symlink_list.txt"
SYMLINK_ARRAY_PATH="/tmp/symlink_array.sh"

# generate the symlink list
# if we are not in TMUX
if [ -z $TMUX ]; then

  # and the symlinklist does not exist
  if [ ! -e "$SYMLINK_LIST_PATH" ]; then

    # create the symlink list
    $UAV_CORE_PATH/miscellaneous/scripts/detacher.sh ~/.scripts/createRosSymlinkDatabase.sh
  fi
fi

# when the symlink list is generated, post-process it to create a shell file
# with the definition of the array
if [ ! -e "$SYMLINK_ARRAY_PATH" ] && [ -e "$SYMLINK_LIST_PATH" ]; then

  # parse the csv file and extract file paths
  i="1"
  while IFS=, read -r path1 path2; do

    if [[ "$path1" != *ctop_planner* ]] || [[ "$path2" != *ctop_planner* ]]
    then
      continue
    fi

    SYMLINK_LIST_PATHS1[$i]=`eval echo "$path1"`
    SYMLINK_LIST_PATHS2[$i]=`eval echo "$path2"`

    # echo "${SYMLINK_LIST_PATHS1[$i]} -> ${SYMLINK_LIST_PATHS2[$i]}"

    i=$(expr $i + 1)
  done < "$SYMLINK_LIST_PATH"

  echo "SYMLINK_LIST_PATHS1=(" > $SYMLINK_ARRAY_PATH
  i="0"
  for ((i=0; i < ${#SYMLINK_LIST_PATHS1[*]}; i++));
  do
    echo "\"${SYMLINK_LIST_PATHS1[$i]}\" " >> $SYMLINK_ARRAY_PATH
  done
  echo ")
  " >> $SYMLINK_ARRAY_PATH

  echo "SYMLINK_LIST_PATHS2=(" >> $SYMLINK_ARRAY_PATH
  i="0"
  for ((i=0; i < ${#SYMLINK_LIST_PATHS2[*]}; i++));
  do
    echo "\"${SYMLINK_LIST_PATHS2[$i]}\" " >> $SYMLINK_ARRAY_PATH
  done
  echo ")" >> $SYMLINK_ARRAY_PATH

fi

# if the array file exists, just source it
if [ -e "$SYMLINK_ARRAY_PATH" ]; then

  source $SYMLINK_ARRAY_PATH

fi

cd() {

  # if ag is missing, run normal "cd"
  if [ -z SYMLINK_LIST_PATHS1 ]; then

    builtin cd "$@"
    return

    # if we have ag, do the magic
  else

    builtin cd "$@"
    new_path=`pwd`

    # test original paths for prefix
    # echo ""
    j="1"
    for ((i=1; i < ${#SYMLINK_LIST_PATHS1[*]}+1; i++));
    do
      if [[ "$new_path" == *${SYMLINK_LIST_PATHS2[$i]}* ]]
      then
        # echo "found prefix: ${SYMLINK_LIST_PATHS1[$i]} -> ${SYMLINK_LIST_PATHS2[$i]} for $new_path"
        # echo substracted: ${new_path#*${SYMLINK_LIST_PATHS2[$i]}}
        repath[$j]="${SYMLINK_LIST_PATHS1[$i]}${new_path#*${SYMLINK_LIST_PATHS2[$i]}}"
        # echo new_path: ${repath[$j]}
        new_path=${repath[$j]}
        j=$(expr $j + 1)
        # echo ""
      fi
    done

    if [ "$j" -ge "2" ]
    then
      builtin cd "$new_path"
    fi
  fi
}

CURRENT_PATH=`pwd`
cd "$CURRENT_PATH"

# #}

## --------------------------------------------------------------
## |                             Git                            |
## --------------------------------------------------------------

# #{ git()

# upgrades the "git pull" to allow dotfiles profiling on linux-setup
# Other "git" features should not be changed
git() {

  case $* in pull*|checkout*|"reset --hard")

    # give me the path to root of the repo we are in
    ROOT_DIR=`git rev-parse --show-toplevel` 2> /dev/null

    if [[ "$?" == "0" ]]; then

      # if we are in the 'linux-setup' repo, use the Profile manager
      if [[ "$ROOT_DIR" == "$GIT_PATH/linux-setup" ]]; then

        PROFILE_MANAGER="$GIT_PATH/linux-setup/submodules/profile_manager/profile_manager.sh"

        bash -c "$PROFILE_MANAGER backup $GIT_PATH/linux-setup/appconfig/profile_manager/file_list.txt"

        command git "$@"

        if [[ "$?" == "0" ]]; then
          case $* in pull*|checkout*) # TODO: should only work for checkout of a branch
            echo "Syncing git submodules"
            command git submodule sync
            echo "Updating git submodules"
            command git submodule update --init --recursive

            if [ -e .gitman.yml ]; then
              if [[ ! $(git status .gitman.yml --porcelain) ]]; then # if .gitman.yml is unchanged
                echo "Updating gitman sub-repos"
                gitman install
              else
                echo -e "\e[31m.gitman.yml modified, not updating sub-repos\e[0m"
              fi
            fi
          esac
        fi

        if [[ "$?" == "0" ]]; then
          bash -c "$PROFILE_MANAGER deploy $GIT_PATH/linux-setup/appconfig/profile_manager/file_list.txt"
        fi

      else

        command git "$@"

        if [[ "$?" == "0" ]]; then
          case $* in pull*|checkout*) # TODO: should only work for checkout of a branch
            echo "Syncing git submodules"
            command git submodule sync
            echo "Updating git submodules"
            command git submodule update --init --recursive

            if [ -e .gitman.yml ]; then
              if [[ ! $(git status .gitman.yml --porcelain) ]]; then # if .gitman.yml is unchanged
                echo "Updating gitman sub-repos"
                gitman install
              else
                echo -e "\e[31m.gitman.yml modified, not updating sub-repos\e[0m"
              fi
            fi
          esac
        fi

      fi

    else

      command git "$@"

      if [[ "$?" == "0" ]]; then
        case $* in pull*|checkout*) # TODO: should only work for checkout of a branch
          echo "Syncing git submodules"
          command git submodule sync
          echo "Updating git submodules"
          command git submodule update --init --recursive

          if [ -e .gitman.yml ]; then
            if [[ ! $(git status .gitman.yml --porcelain) ]]; then # if .gitman.yml is unchanged
              echo "Updating gitman sub-repos"
              gitman install
            else
              echo -e "\e[31m.gitman.yml modified, not updating sub-repos\e[0m"
            fi
          fi
        esac
      fi
    fi
    ;;
  *)
    command git "$@"
    ;;

  esac
}

# #}

alias gs="git status"
alias gcmp="git checkout master; git pull"
alias flog="~/.scripts/git-forest.sh --all --date=relative --abbrev-commit --pretty=format:'%Cred%h%Creset -%C(yellow)%d%Creset %s %Cgreen(%cr) %C(bold blue)<%an>%Creset' --style=15"
alias glog="git log --graph --abbrev-commit --date=relative --pretty=format:'%Cred%h%Creset -%C(yellow)%d%Creset %s %Cgreen(%cr) %C(bold blue)<%an>%Creset'"

## --------------------------------------------------------------
## |                         ROS aliases                        |
## --------------------------------------------------------------

# #{ catkin()

catkin() {

  case $* in

    init*)

      # give me the path to root of the repo we are in
      ROOT_DIR=`git rev-parse --show-toplevel` 2> /dev/null

      command catkin "$@"
      command catkin config --profile debug --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS='-std=c++17 -march=native' -DCMAKE_C_FLAGS='-march=native'
      command catkin config --profile release --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS='-std=c++17 -march=native' -DCMAKE_C_FLAGS='-march=native'
      command catkin config --profile reldeb --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS='-std=c++17 -march=native' -DCMAKE_C_FLAGS='-march=native'

      command catkin profile set reldeb
      ;;

    build*|b|bt)

      PACKAGES=$(catkin list)
      if [ -z "$PACKAGES" ]; then
        echo "Cannot compile, not in a workspace"
      else
        command catkin "$@"
      fi

      ;;

    *)
      command catkin $@
      ;;

    esac
  }

# #}
alias cb="catkin build"

## --------------------------------------------------------------
## |                       waitFor* macros                      |
## --------------------------------------------------------------

# #{ waitForRos()

waitForRos() {
  until rostopic list > /dev/null 2>&1; do
    echo "waiting for ros"
    sleep 1;
  done
}

# #}

# #{ waitForSimulation()

waitForSimulation() {
  until timeout 3s rostopic echo /gazebo/model_states -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for simulation"
    sleep 1;
  done
  sleep 1;
}

# #}

# #{ waitForOdometry()

waitForOdometry() {
  until timeout 3s rostopic echo /$UAV_NAME/mavros/local_position/odom -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for odometry"
    sleep 1;
  done
}

# #}

# #{ waitForControlManager()

waitForControlManager() {
  until timeout 3s rostopic echo /$UAV_NAME/control_manager/diagnostics -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for control manager"
    sleep 1;
  done
}

# #}

# #{ waitForControl()

waitForControl() {
  until timeout 3s rostopic echo /$UAV_NAME/control_manager/diagnostics -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for control"
    sleep 1;
  done
  until timeout 3s rostopic echo /$UAV_NAME/odometry/odom_main -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for odom_main"
    sleep 1;
  done
}

# #}

# #{ waitForMpc()

waitForMpc() {
  until timeout 3s rostopic echo /$UAV_NAME/control_manager/diagnostics -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for control"
    sleep 1;
  done
  until timeout 3s rostopic echo /$UAV_NAME/odometry/odom_main -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for odom_main"
    sleep 1;
  done
}

# #}

# #{ waitForCompile()

waitForCompile() {
  while timeout 3s  ps aux | grep "catkin build" | grep -v grep > /dev/null 2>&1; do
    echo "waiting for compilation to complete"
    sleep 1;
  done
}

# #}
