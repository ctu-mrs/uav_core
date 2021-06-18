# get the path to this script

PNAME=$( ps -p "$$" -o comm= )
SNAME=$( echo "$SHELL" | grep -Eo '[^/]+/?$' )

if [ "$SNAME" = "bash" ]; then
  MY_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
else
  MY_PATH=`dirname "$0"`
  MY_PATH=`( cd "$MY_PATH" && pwd )`
fi

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

UAV_CORE_PATH=$MY_PATH/../../

# disable gitman caching
export GITMAN_CACHE_DISABLE=1

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

      temp=${SYMLINK_LIST_PATHS2[$i]}

      if [[ $new_path == *$temp* ]]
      then

        # echo "found prefix: ${SYMLINK_LIST_PATHS1[$i]} -> $temp for $new_path"
        # echo substracted: ${new_path#*$temp}
        repath[$j]="${SYMLINK_LIST_PATHS1[$i]}${new_path#*$temp}"
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
## |                       SSH key swapper                      |
## --------------------------------------------------------------

# #{ sshkey()

sshkey() {

  # this script swaps the ssh key lines in .ssh/config for a given user name
  # and starts an ssh agent for that key

  if [ "$#" -eq "0" ]; then
    echo please supply a parameter: the ssh key file prefix
    exit
  fi

  SSH_KEY_NAME="$1"

  HOSTS=(
    'github.com'
    'mrs.felk.cvut.cz'
    'gitlab.fel.cvut.cz'
  )

  # get me vim, we will be using it alot to postprocess the generated json files
  if [ -x "$(whereis nvim | awk '{print $2}')" ]; then
    VIM_BIN="$(whereis nvim | awk '{print $2}')"
    HEADLESS="--headless"
  elif [ -x "$(whereis vim | awk '{print $2}')" ]; then
    VIM_BIN="$(whereis vim | awk '{print $2}')"
    HEADLESS=""
  fi

  case "$SHELL" in
    *bash*)
      fucking_shell_offset="0"
      ;;
    *zsh*)
      fucking_shell_offset="1"
      ;;
  esac

  for ((i=$fucking_shell_offset; i < ${#HOSTS[*]}+$fucking_shell_offset; i++));
  do

    HOST="${HOSTS[$i]}"

    echo "Updating .ssh/config for $HOST with $SSH_KEY_NAME"

    # comment out all keys in for the host
    $VIM_BIN $HEADLESS -nEs -c "delmarks!" -c "%g/^host $HOST/norm {ma}mb" -c "'a,'b g/^\s\+identityfile/norm I# " -c "delmarks!" -c "wqa" -- $HOME/.ssh/config

    # remove my own key
    $VIM_BIN $HEADLESS -nEs -c "delmarks!" -c "%g/^host $HOST/norm {ma}mb" -c "'a,'b g/^\s\+#\s\+identityfile.\+$SSH_KEY_NAME\s*/norm dd" -c "delmarks!" -c "wqa" -- $HOME/.ssh/config

    # add my own key
    $VIM_BIN $HEADLESS -nEs -c "delmarks!" -c "%g/^host $HOST/norm }kyypccidentityfile ~/.ssh/$SSH_KEY_NAME" -c "wqa" -- $HOME/.ssh/config # `

  done

  # copy the key from uav_core
  cp ~/git/uav_core/miscellaneous/dotssh/$SSH_KEY_NAME ~/.ssh/
  cp ~/git/uav_core/miscellaneous/dotssh/$SSH_KEY_NAME.pub ~/.ssh/

  # set the corret chmod to the keys
  chmod 0600 ~/.ssh/$SSH_KEY_NAME

  eval `ssh-agent`
  ssh-add ~/.ssh/$SSH_KEY_NAME
}

# #}

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
      command catkin config --profile debug --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_FLAGS='-std=c++17 -march=native' -DCMAKE_C_FLAGS='-march=native'
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

# #{ colcon()

colcon() {

  CURRENT_PATH=`pwd`

  case $* in

    init*)

      if [ ! -e "build/COLCON_IGNORE" ]; then # we are NOT at the workspace root
        command colcon build # this creates a new workspace
      fi

      ;;

    build*|b*)

      # go up the folder tree until we find the build/COLCON_IGNORE file or until we reach the root
      while [ ! -e "build/COLCON_IGNORE" ]; do
        cd ..
        if [[ `pwd` == "/" ]]; then
          # we reached the root and didn't find the build/COLCON_IGNORE file - that's a fail!
          echo "Cannot compile, probably not in a workspace (if you want to create a new workspace, call \"colcon init\" in its root first)".
          return 1
        fi
      done

      # if the flow got here, we found the build/COLCON_IGNORE file!
      # this is the folder we're looking for - call the actual colcon command here
      command colcon "$@" --symlink-install
      ret=$? # remember the return value of the colcon command
      cd "$CURRENT_PATH" # return to the path where this command was originaly called
      return $ret # return the original return value of the colcon command

      ;;

    clean*)

      if [ -e "build/COLCON_IGNORE" ]; then # we are at the workspace root
        rm -r build install log
        mkdir build
        cd build
        touch COLCON_IGNORE
      else
        while [ ! -e "build/COLCON_IGNORE" ]; do
          cd ..

          if [[ `pwd` == "/" ]]; then
            echo "Cannot clean, not in a workspace!"
            break
          elif [ -e "build/COLCON_IGNORE" ]; then
            rm -r build install log
            mkdir build
            cd build
            touch COLCON_IGNORE
            break
          fi
        done
      fi

      cd "$CURRENT_PATH" # return to the original folder where the command was called

      ;;

    *)
      command colcon $@
      ;;

  esac
}

# #}

# #{ cb()

cb() {

  # catkin?
  PACKAGES=$(catkin list)
  [ ! -z "$PACKAGES" ] && USE_CATKIN=1 || USE_CATKIN=0

  # colcon?
  CURRENT_PATH=`pwd`
  while [ ! -e "build/COLCON_IGNORE" ]; do
    cd ..

    if [[ `pwd` == "/" ]]; then
      break
    fi
  done
  [[ `pwd` == "/" ]] && USE_COLCON=0
  [ -e "build/COLCON_IGNORE" ] && USE_COLCON=1
  cd "$CURRENT_PATH"

  ret=1
  [[ $USE_CATKIN == "1" ]] && [[ $USE_COLCON == "0" ]] && ( catkin build; ret=$? )
  [[ $USE_CATKIN == "0" ]] && [[ $USE_COLCON == "1" ]] && ( colcon build; ret=$? )
  [[ $USE_CATKIN == "1" ]] && [[ $USE_COLCON == "1" ]] && ( colcon build; ret=$? )
  [[ $USE_CATKIN == "0" ]] && [[ $USE_COLCON == "0" ]] && echo "Cannot compile, not in a workspace"

  unset USE_CATKIN
  unset USE_COLCON
  return $ret
}

# #}

# catkin built [this/package] | less
# #{ cbl()

cbl () {

  if [ $# -eq 0 ]; then

    package="--this"

  else

    package="$1"
    workspace_path=$( python $UAV_CORE_PATH/miscellaneous/scripts/get_ros_workspace_path.py -p "$package" )
    cd "$workspace_path"

  fi

  command catkin build "$package" --force-color 2>&1 | less -r
}

# #}

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

# #{ waitForSimulation()

waitForSpawn() {
  until timeout 3s rostopic echo /mrs_drone_spawner/spawned -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for spawn"
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
  until timeout 3s rostopic echo /$UAV_NAME/control_manager/offboard_on -n 1 --noarr > /dev/null 2>&1; do
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

# #{ appendBag()

appendBag() {

  if [ "$#" -ne 1 ]; then
    echo ERROR: please supply one argument: the text that should be appended to the name of the folder with the latest rosbag file and logs
  else

    bag_adress=`readlink ~/bag_files/latest`

    if test -d "$bag_adress"; then

      appended_adress=$bag_adress$1
      mv $bag_adress $appended_adress
      ln -sf $appended_adress ~/bag_files/latest
			second_symlink_adress=$(sed 's|\(.*\)/.*|\1|' <<< $appended_adress)
      ln -sf $appended_adress $second_symlink_adress/latest

      echo Rosbag name appended: $appended_adress

    else
      echo ERROR: symlink ~/bag_files/latest does not point to a file! - $bag_adress
    fi
  fi
}

# #}
