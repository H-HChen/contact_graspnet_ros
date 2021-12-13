
#!/usr/bin/env bash

BASH_OPTION=bash

IMG=iscilab/grasp_tm5:GPU
containerid=$(docker ps -qf "ancestor=${IMG}") && echo $containerid

xhost +

if [[ -n "$containerid" ]]
then
    docker exec -it \
        --privileged \
        -e DISPLAY=${DISPLAY} \
        -e LINES="$(tput lines)" \
        locobot \
        $BASH_OPTION
else
    docker start -i grasp_tm5
fi
