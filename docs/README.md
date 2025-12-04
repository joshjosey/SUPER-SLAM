

 docker run -it --rm -v $(pwd):/app -v /home/josh/datasets/KITTI/SUPER-SLAM:/data -e DISPLAY=$DISPLAY --net=host -v /tmp/.X11-unix:/tmp/.X11-unix super-slam:latest