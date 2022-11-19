#! /bin/bash

echo "auto_copy watcher running"
trap 'echo -e "\nexit signal caught, shutting down..."; exit 0' SIGINT SIGTERM

dest_path=~/catkin_ws/src/final_project/scripts
if [[ $1 != '' ]];
then
    dest_path=$1
fi

if [[ ! -d "$dest_path" ]]
then
    echo "$dest_path does not exist on your filesystem."
    exit 1
fi

declare -A FILELIST

shopt -s nullglob
for f in *.py;
do
    cp $f $dest_path
    chmod +x ${dest_path}/${f}
    FILELIST[$f]=$(
        md5=($(md5sum $f))
        echo $md5
    )
done
shopt -u nullglob


while sleep 1; do
    for key in ${!FILELIST[@]}; do
        if [ -f "$key" ];
        then
            if [[ ${FILELIST[$key]} != $(
                        md5=($(md5sum $key))
                        echo $md5
            ) ]];
            then
                echo Hash changed for $key
                cp $key $dest_path
                chmod +x ${dest_path}/${key}
                FILELIST[$key]=$(
                    md5=($(md5sum $key))
                    echo $md5
                )
            fi
        else
            echo "Removed: $key"
            unset FILELIST[${key}]
            rm -f ${dest_path}/${key}
            rm -f ${dest_path}/${key}c
        fi
        
    done
    
    shopt -s nullglob
    for f in *.py;
    do
        if [[ ! -v FILELIST[$f] ]]; then
            echo "Found new: $f"
            
            cp $f $dest_path
            chmod +x ${dest_path}/${f}
            FILELIST[$f]=$(
                md5=($(md5sum $f))
                echo $md5
            )
        fi
    done
    shopt -u nullglob
done
