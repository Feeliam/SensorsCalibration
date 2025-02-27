#!/bin/sh

xhost +local:root;

if (nvidia-smi|grep NVIDIA)
then
    # nvidia
    echo "NVIDIA GPU detected, initialization calibration container"
    docker run -it --privileged=true --net=host --gpus all \
      --env="NVIDIA_DRIVER_CAPABILITIES=all" \
      --env="DISPLAY" \
      --env="QT_X11_NO_MITSHM=1" \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       --volume="${PWD}:/share" \
       --volume="/home/synkrotron/share:/home" \  # 注意创建共享文件夹 这里的/home/synkrotron/share是你的本地路径，/home是容器内路径
       scllovewkf/opencalib:v1 /bin/bash  -c "cd /share;  /bin/bash;"
else

    echo "NVIDIA GPU NOT detected, initialization calibration container"
    docker run -it --privileged=true --net=host \
       --env="DISPLAY" \
       --env="QT_X11_NO_MITSHM=1" \
       --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       --volume="${PWD}:/share" \
       scllovewkf/opencalib:v1 /bin/bash  -c "cd /share;  /bin/bash;"
fi
