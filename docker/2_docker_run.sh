docker run -d \
            --name="lidarimucalib" \
            -e DISPLAY=$DISPLAY \
            --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
            --volume="$HOME/.ssh:/root/.ssh" \
            --volume="$HOME/Downloads:/home/Downloads" \
            "tseanliu/lidarimucalib:latest" \
            /bin/sh -c "sed -i "s/CMD_PROMPT_PREFIX=.*$/CMD_PROMPT_PREFIX=$2/" /root/.bashrc && while true; do sleep 10; done"
