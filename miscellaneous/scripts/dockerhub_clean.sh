#!/bin/bash

set -e

UNAME="klaxalk"
UPASS=$( python -c "import keyring; print(keyring.get_password('dockerhub', 'personal'))" )
ORGANIZATIONNAME="ctumrs"
REPOSITORIES=(
  "mrs_uav_system"
  "mrs_uav_system_ls"
  "mrs_uav_system_modules"
  "mrs_uav_system_ls_modules"
)
DAYS="30"

# get token to be able to talk to Docker Hub
TOKEN=$(curl -s -H "Content-Type: application/json" -X POST -d '{"username": "'${UNAME}'", "password": "'${UPASS}'"}' https://hub.docker.com/v2/users/login/ | jq -r .token)

echo
echo "Identifying and deleting images which are older than $DAYS days in ${ORGANIZATIONNAME} docker hub account"

for ((i=0; i < ${#REPOSITORIES[*]}; i++)); do

  REPOSITORY=${REPOSITORIES[$i]}

  # get tags for repo
  echo
  echo "Looping Through ${REPOSITORY} repository in ${UNAME} account"
  IMAGE_TAGS=$(curl -s -H "Authorization: JWT ${TOKEN}" https://hub.docker.com/v2/repositories/${ORGANIZATIONNAME}/${REPOSITORY}/tags/ | jq -r '.results|.[]|.name' | grep _w) 

  echo "$REPOSITORY IMAGE TAGS:
$IMAGE_TAGS"

  # build a list of images from tags
  for j in ${IMAGE_TAGS}
  do
      echo
      # add last_updated_time
    updated_time=$(curl -s -H "Authorization: JWT ${TOKEN}" https://hub.docker.com/v2/repositories/${ORGANIZATIONNAME}/${REPOSITORY}/tags/${j}/?page_size=10000 | jq -r '.last_updated')
    echo $updated_time
    datetime=$updated_time
    timeago="$DAYS days ago"

    dtSec=$(date --date "$datetime" +'%s')
    taSec=$(date --date "$timeago" +'%s')

    echo "INFO: dtSec=$dtSec, taSec=$taSec"

    if [ $dtSec -lt $taSec ]; then
      echo "This image ${UNAME}/${REPOSITORY}:${j} is older than $DAYS days, deleting this image"
      ## Please uncomment below line to delete docker hub images of docker hub repositories
      curl -s  -X DELETE  -H "Authorization: JWT ${TOKEN}" https://hub.docker.com/v2/repositories/${ORGANIZATIONNAME}/${REPOSITORY}/tags/${j}/
    else
      echo "This image ${UNAME}/${REPOSITORY}:${j} is within $DAYS days time range, keep this image"
    fi
  done

done
