# get the path to this script

PNAME=$( ps -p "$$" -o comm= )
SNAME=$( echo "$SHELL" | grep -Eo '[^/]+/?$' )

if [ "$SNAME" = "bash" ]; then
  MY_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
else
  MY_PATH=`dirname "$0"`
  MY_PATH=`( cd "$MY_PATH" && pwd )`
fi

UAV_CORE_PATH=$MY_PATH/../../

# disable gitman caching
export GITMAN_CACHE_DISABLE=1

# #{ CTAGS

# path possible ctags locations
# -R dir1 -R dir2 ...

# contains the list of user ROS workspaces
# used now for ctags generation and later for YCM (throught .ycm_extra_conf.py)
[ -z "$ROS_WORKSPACES" ] && export ROS_WORKSPACES="$ROS_WORKSPACE"

# if not defined, define it
[ -z "$CTAGS_SOURCE_DIR" ] && export CTAGS_SOURCE_DIR=""

# append location of ROS workspaces
for WS in $(echo $ROS_WORKSPACES); do
  export CTAGS_SOURCE_DIR="${CTAGS_SOURCE_DIR} -R $WS"
done

# path to source from which to generate `one-time generated ctags file`
# -R dir1 -R dir2 ...
[ -z "$CTAGS_ONCE_SOURCE_DIR" ] && export CTAGS_ONCE_SOURCE_DIR=""

# append the ros path
export CTAGS_ONCE_SOURCE_DIR="${CTAGS_ONCE_SOURCE_DIR} -R /opt/ros/melodic/include"

# the location of the `one-time generated ctags file`
export CTAGS_FILE_ONCE="~/tags-once"

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

SYMLINK_ARRAY_PATH="/tmp/symlink_array.sh"

# generate the symlink list
# if we are not in TMUX
if [ -z $TMUX ]; then

  # and the symlinklist does not exist
  if [ ! -f "$SYMLINK_ARRAY_PATH" ]; then

    # create the symlink list
    $UAV_CORE_PATH/miscellaneous/scripts/detacher.sh $UAV_CORE_PATH/miscellaneous/scripts/createRosSymlinkDatabase.sh
  fi
fi

# if the array file exists, just source it
[ -f "$SYMLINK_ARRAY_PATH" ] && source $SYMLINK_ARRAY_PATH

cd() {

  [ -z "$SYMLINK_LIST_PATHS1" ] && [ -f "$SYMLINK_ARRAY_PATH" ] && source $SYMLINK_ARRAY_PATH

  if [ -z "$SYMLINK_LIST_PATHS1" ]; then

    builtin cd "$@"
    return

    # if we have ag, do the magic
  else

    builtin cd "$@"
    new_path=`pwd`

    # test original paths for prefix

    case "$SHELL" in
      *bash*)
        fucking_shell_offset="0"
        ;;
      *zsh*)
        fucking_shell_offset="1"
        ;;
    esac

    # echo ""
    j="1"

    for ((i=$fucking_shell_offset; i < ${#SYMLINK_LIST_PATHS1[*]}+$fucking_shell_offset; i++));
    do

      dog=$( echo ${SYMLINK_LIST_PATHS2[$i]} | sed "s/\/$//" )

      if [[ $new_path == *$dog* ]]
      then

        # echo "found prefix: ${SYMLINK_LIST_PATHS1[$i]} -> $dog for $new_path"
        # echo substracted: ${new_path#*$dog}
        repath[$j]="${SYMLINK_LIST_PATHS1[$i]}${new_path#*$dog}"
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
## |                       MRS simulation                       |
## --------------------------------------------------------------

# smart way of creating alias for spawn_uav
spawn_uav() {
  rosrun mrs_simulation spawn $@
}

# #{ bash completion function definition
function _spawn_uav_bash_complete()
{
  local arg opts
  COMPREPLY=()
  arg="${COMP_WORDS[COMP_CWORD]}"
  opts=`rosrun mrs_simulation spawn --help | grep '  --' | awk '{print $1}'`
  COMPREPLY=( $(compgen -W "${opts}" -- ${arg}) )
}
# #}

# #{ zsh completion function definition
function _spawn_uav_zsh_complete()
{
  local opts
  reply=()
  opts=`rosrun mrs_simulation spawn --help | grep '  --' | awk '{print $1}'`
  reply=(${=opts})
}
# #}

# selection of specific function for different shells
case "$PNAME" in
  bash)
    complete -F "_spawn_uav_bash_complete" "spawn_uav"
    ;;
  zsh)
    compctl -K "_spawn_uav_zsh_complete" "spawn_uav"
    ;;
esac

## --------------------------------------------------------------
## |                             Git                            |
## --------------------------------------------------------------

# #{ git2https()

git2https() {

  old_remote=$(git remote get-url origin)
  echo Old remote: $old_remote

  new_remote=$(echo $old_remote | sed -r 's|.*git@(.+):(.+)|https://\1/\2|' | head -n 1)
  echo New remote: $new_remote

  if [ -n "$new_remote" ]; then
    git remote set-url origin "$new_remote"
  fi
}

# #}

# #{ git2ssh()

git2ssh() {

  old_remote=$(git remote get-url origin)
  echo Old remote: $old_remote

  new_remote=$(echo $old_remote | sed -r 's|https://([^/]+)/(.+)|git@\1:\2|' | head -n 1)
  echo New remote: $new_remote

  if [ -n "$new_remote" ]; then
    git remote set-url origin "$new_remote"
  fi
}

# #}

# #{ gitUpdateSubmodules()

gitUpdateSubmodules() {

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
}

# #}

# #{ git()

# upgrades the "git pull" to allow dotfiles profiling on linux-setup
# Other "git" features should not be changed
git() {

  case $* in

    push*)

      was_github=$(command git remote get-url origin | grep 'https://github.com' | wc -l)

      # change remote to ssh
      [ "$was_github" -ge 1 ] && git2ssh

      # run the original command
      command git "$@"
      ;;

    pull*|checkout*|"reset --hard")

      # give me the path to root of the repo we are in
      ROOT_DIR=`git rev-parse --show-toplevel` 2> /dev/null

      # we are in git repo subfolder
      if [[ "$?" == "0" ]]; then

        # if we are in the 'linux-setup' repo, use the Profile manager
        if [[ "$ROOT_DIR" == "$GIT_PATH/linux-setup" ]]; then

          PROFILE_MANAGER="$GIT_PATH/linux-setup/submodules/profile_manager/profile_manager.sh"

          bash -c "$PROFILE_MANAGER backup $GIT_PATH/linux-setup/appconfig/profile_manager/file_list.txt"

          command git "$@"

          if [[ "$?" == "0" ]]; then
            case $* in
              pull*|checkout*) # TODO: should only work for checkout of a branch
                gitUpdateSubmodules
                ;;
            esac
          fi

          if [[ "$?" == "0" ]]; then
            bash -c "$PROFILE_MANAGER deploy $GIT_PATH/linux-setup/appconfig/profile_manager/file_list.txt"
          fi

        # this is generic git repo subfolder
        else

          command git "$@"

          if [[ "$?" == "0" ]]; then
            case $* in
              pull*|checkout*) # TODO: should only work for checkout of a branch
                gitUpdateSubmodules
                ;;
            esac
          fi

        fi

      # we are not aware of being in a git subfolder
      else

        # lets run the command as it would originally would
        command git "$@"

        # and if it somehow succeeds, just update the submodules
        if [[ "$?" == "0" ]]; then
          case $* in
            pull*|checkout*) # TODO: should only work for checkout of a branch
              gitUpdateSubmodules
              ;;
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

      hostname=$( cat /etc/hostname )

      if [[ $hostname == uav* ]]; then
        memlimit="--mem-limit 50%"
      else
        memlimit=""
      fi

      PACKAGES=$(catkin list)
      if [ -z "$PACKAGES" ]; then
        echo "Cannot compile, probably not in a workspace (call catkin list, if the result is empty, build you workspace in its root first)."
      else
        if [ -z "$memlimit" ]; then
          command catkin "$@"
        else
          echo "Detected UAV PC, compiling with $memlimit"
          command catkin "$@" $memlimit
        fi
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

# #{ waitForOffboard()

waitForOffboard() {
  until timeout 1s rostopic echo /$UAV_NAME/control_manager/offboard_on -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for offboard mode"
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
