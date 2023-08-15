#!/bin/bash

key_name="~/.ssh/id_rsa_drone_login"

input=(

  'alice'       'uav1'  '192.168.69.101'
  'bob'         'uav2'  '192.168.69.102'
  'carol'       'uav3'  '192.168.69.103'
  'diego'       'uav4'  '192.168.69.104'
  'eric'        'uav5'  '192.168.69.105'
  'frank'       'uav6'  '192.168.69.106'
  'grace'       'uav7'  '192.168.69.107'
  'heidi'       'uav8'  '192.168.69.108'
  'ivan'        'uav9'  '192.168.69.109'
  'judy'        'uav10' '192.168.69.110'
  'kyle'        'uav11' '192.168.69.111'
  'lemmy'       'uav12' '192.168.69.112'
  'max'         'uav13' '192.168.69.113'
  'nick'        'uav14' '192.168.69.114'
  'olivia'      'uav15' '192.168.69.115'
  'pat'         'uav16' '192.168.69.116'
  'quentin'     'uav17' '192.168.69.117'
  'robert'      'uav18' '192.168.69.118'
  'sophie'      'uav19' '192.168.69.119'
  'trudy'       'uav20' '192.168.69.120'
  'unique'      'uav21' '192.168.69.121'
  'vanna'       'uav22' '192.168.69.122'
  'walter'      'uav23' '192.168.69.123'
  'xena'        'uav24' '192.168.69.124'
  'uav25'       'uav25' '192.168.69.125'
  'uav26'       'uav26' '192.168.69.126'
  'uav27'       'uav27' '192.168.69.127'
  'uav28'       'uav28' '192.168.69.128'
  'uav29'       'uav29' '192.168.69.129'
  'uav30'       'uav30' '192.168.69.130'
  'uav31'       'uav31' '192.168.69.131'
  'uav32'       'uav32' '192.168.69.132'
  'uav33'       'uav33' '192.168.69.133'
  'uav34'       'uav34' '192.168.69.134'
  'uav35'       'uav35' '192.168.69.135'
  'uav36'       'uav36' '192.168.69.136'
  'uav37'       'uav37' '192.168.69.137'
  'uav38'       'uav38' '192.168.69.138'
  'uav39'       'uav39' '192.168.69.139'
  'uav40'       'uav40' '192.168.69.140'
  'uav41'       'uav41' '192.168.69.141'
  'uniform'     'uav42' '192.168.69.142'
  'uniform'     'uav43' '192.168.69.143'
  'whiskey'     'uav44' '192.168.69.144'
  'papa'        'uav45' '192.168.69.145'
  'oscar'       'uav46' '192.168.69.146'
  'quebec'      'uav47' '192.168.69.147'
  'romeo'       'uav49' '192.168.69.149'
  'sierra'      'uav50' '192.168.69.150'
  'tango'       'uav51' '192.168.69.151'
  'evzen'       'uav52' '192.168.69.152'
  'victor'      'uav53' '192.168.69.153'
  'rododendron' 'uav54' '192.168.69.154'
  'xray'        'uav55' '192.168.69.155'
  'zulu'        'uav56' '192.168.69.156'
  'alfa'        'uav60' '192.168.69.160'
  'bravo'       'uav61' '192.168.69.161'
  'charlie'     'uav62' '192.168.69.162'
  'delta'       'uav63' '192.168.69.163'
  'echo'        'uav64' '192.168.69.164'
  'foxtrot'     'uav65' '192.168.69.165'
  'golf'        'uav66' '192.168.69.166'
  'hotel'       'uav67' '192.168.69.167'
  'india'       'uav68' '192.168.69.168'
  'kilo'        'uav69' '192.168.69.169'
  'lima'        'uav70' '192.168.69.170'
  'mike'        'uav71' '192.168.69.171'
  'dofec'       'uav80' '192.168.69.180'
  'juliett'     'uav90' '192.168.69.190'
  'eagle'       'uav91' '192.168.69.191'
  'xavnx1'      'uav95' '192.168.69.195'
  'xavier'      'uav99' '192.168.69.199'

  # our laptops, remove if needed

  'klaxalk-local' 'klaxalk' '192.168.69.11'
  'dan-local'     'dan'     '192.168.69.42'
  'pavel'         'pavel'   '192.168.69.19'
  'parakh'         'parakh'   '192.168.69.81'
  'filip'         'filip'   '192.168.69.44'
)

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

VIM_BIN="$(whereis vim | awk '{print $2}')"
HEADLESS=""

for ((i=0; i < ${#input[*]}; i++));
do
  ((i%3==0)) && nato[$i/3]="${input[$i]}"
  ((i%3==1)) && hostname[$i/3]="${input[$i]}"
  ((i%3==2)) && ip[$i/3]="${input[$i]}"
done

# hostname=( "${hostname[@]}" "${nato[@]}" )
# ip=( "${ip[@]}" "${ip[@]}" )

hostname=( "${hostname[@]}" )
ip=( "${ip[@]}" )

my_hostname=$( cat /etc/hostname )

for ((i=0; i < ${#hostname[*]}; i++)); do

  echo ""
  echo "Processing ${hostname[i]}"

  num=`cat ~/.ssh/config | grep "host ${hostname[i]}" | wc -l`
  if [ "$num" -lt "1" ]; then

    echo Creating new entry in .ssh/config for ${hostname[i]} ${ip[i]}

  else

    echo Updating entry in .ssh/config for ${hostname[i]} ${ip[i]}

    $VIM_BIN $HEADLESS -Ens -c "set ignorecase" -c "%g/.*host ${hostname[i]}.*/norm dap" -c "wqa" -- "$HOME/.ssh/config"

  fi

  echo "host ${hostname[i]}
	hostname ${ip[i]}
	user mrs
	identityfile $key_name
" >> ~/.ssh/config

  # move the current entry to the bottom of the file
  $VIM_BIN $HEADLESS -Ens -c "set ignorecase" -c "%g/${ip[i]}\s.*/norm dapGp" -c "wqa" -- "$HOME/.ssh/config"

  num=`cat /etc/hosts | grep ".* ${hostname[i]}$" | wc -l`
  if [ "$num" -ge "1" ]; then

    # delete the old entry
    echo "deleting old entry in /etc/hosts"
    sudo $VIM_BIN $HEADLESS -Ens -c "set ignorecase" -c "%g/.*\s*${hostname[i]}$/norm dd" -c "wqa" -- "/etc/hosts"

  fi

  num=`cat /etc/hosts | grep "${ip[i]}\s" | wc -l`
  if [ "$num" -ge "1" ]; then

    # delete the old entry
    echo "deleting old entry in /etc/hosts"
    sudo $VIM_BIN $HEADLESS -Ens -c "set ignorecase" -c "%g/${ip[i]}\s/norm dd" -c "wqa" -- "/etc/hosts"

  fi

  # only do it if its another uav
  # this is neccessary for NimbroNetwork
  # !!! update 2023-04-20, this is no longer needed, Tomas
  # if [[ "$my_hostname" != "${hostname[i]}" ]]; then

    echo Updating entry in /etc/hosts for ${hostname[i]} ${ip[i]}

    sudo bash -c "echo ${ip[i]} ${hostname[i]} >> /etc/hosts"

    sudo $VIM_BIN $HEADLESS -Ens -c "set ignorecase" -c "%g/${ip[i]}\s/norm ddGp" -c "wqa" -- "/etc/hosts"
    sudo $VIM_BIN $HEADLESS -Ens -c "set ignorecase" -c "normal Go" -c "wqa" -- "/etc/hosts"

  # fi

done

echo ""
echo "deleting old entry of this PC in /etc/hosts"
sudo $VIM_BIN $HEADLESS -Ens -c "set ignorecase" -c "%g/127.0.1.1\s*${my_hostname}$/norm dd" -c "wqa" -- "/etc/hosts"

num=`cat ~/.ssh/config | grep "127.0.1.1 ${my_hostname}" | wc -l`
if [ "$num" -lt "1" ]; then

  echo ""
  echo Missing the 127.0.1.1 entry, creating it

  sudo $VIM_BIN $HEADLESS -Ens -c "set ignorecase" -c "norm ggo127.0.1.1 ${my_hostname}" -c "wqa" -- "/etc/hosts"

fi

sudo $VIM_BIN $HEADLESS -Ens -c "set ignorecase" -c "%g/^\\n$\\n$\\n/norm dd" -c "wqa" -- "/etc/hosts"
