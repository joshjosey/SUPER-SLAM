

docker run -it --rm \
    -v $(pwd):/app \
    -v /home/josh/kitti_data:/data \
    super-slam-img