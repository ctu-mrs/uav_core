#!/bin/bash

#location of the nas share is - https://nasmrs.felk.cvut.cz/index.php/apps/files/?dir=/shared/for_sharing/dotssh_drone&fileid=506475

mkdir -p ~/.ssh/
cd ~/.ssh/
echo `pwd`

function ask_yes_or_no() {
    read -p "$1 ([y]es or [N]o): "
    case $(echo $REPLY | tr '[A-Z]' '[a-z]') in
        y|yes|Y) echo "yes" ;;
        *)     echo "no" ;;
    esac
}

echo -n Password for downloading drone keys: 
read -s szPassword
response=$(curl -s --insecure -w "%{http_code}" -u WpSAIFfCqUds99l:$szPassword "https://nasmrs.felk.cvut.cz/public.php/webdav/id_rsa_drone_login" -o id_rsa_drone_login_tmp)

if [ "$response" = "200" ]; then
    echo -e "\nid_rsa_drone_login downloaded successfuly"
    mv -f id_rsa_drone_login_tmp id_rsa_drone_login
    chmod 400 id_rsa_drone_login
else
    echo -e "\ncan not download id_rsa_drone_login"
    rm -f id_rsa_drone_login_tmp
fi

response=$(curl -s --insecure -w "%{http_code}" -u WpSAIFfCqUds99l:$szPassword "https://nasmrs.felk.cvut.cz/public.php/webdav/id_rsa_drone_login.pub" -o id_rsa_drone_login.pub_tmp)
if [ "$response" = "200" ]; then
    echo "id_rsa_drone_login.pub downloaded successfuly"
    mv -f id_rsa_drone_login.pub_tmp id_rsa_drone_login.pub
    chmod 400 id_rsa_drone_login.pub
else
    echo "can not download id_rsa_drone_login.pub"
    rm -f id_rsa_drone_login.pub_tmp
fi


response=$(curl -s --insecure -w "%{http_code}" -u WpSAIFfCqUds99l:$szPassword "https://nasmrs.felk.cvut.cz/public.php/webdav/config" -o config_tmp)
if [ "$response" = "200" ]; then
    echo "config downloaded successfuly"
    if [[ -f "config" ]]; then
    	echo "config already exists"
	    if [[ "yes" == $(ask_yes_or_no "Do you want to replace it?") ]];then
	    	mv -f config_tmp config
	    	chmod 400 config
	    	echo "replaced"
	    else
	    	rm -f config_tmp
	    	echo "skipped"
		fi
	else
		mv -f config_tmp config
    	chmod 400 config
	fi
else
    echo "can not download config"
    rm -f config_tmp
fi


response=$(curl -s --insecure -w "%{http_code}" -u WpSAIFfCqUds99l:$szPassword "https://nasmrs.felk.cvut.cz/public.php/webdav/authorized_keys" -o authorized_keys_tmp)
if [ "$response" = "200" ]; then
    echo "authorized_keys downloaded successfuly"
    if [[ -f "authorized_keys" ]]; then
    	echo "authorized_keys already exists"
    	if [[ "yes" == $(ask_yes_or_no "Do you want to replace it?") ]];then
	    	mv -f authorized_keys_tmp authorized_keys
	    	chmod 400 authorized_keys
	    	echo "replaced"
	    else
	    	rm -f authorized_keys_tmp
	    	echo "skipped"
		fi
	else
		mv -f authorized_keys_tmp authorized_keys
    	chmod 400 authorized_keys
    fi
else
    echo "can not download authorized_keys"
    rm -f authorized_keys_tmp
fi