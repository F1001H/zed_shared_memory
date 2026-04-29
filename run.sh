sudo docker rm zed_5090

sudo docker run --runtime=nvidia --gpus all -it     --net=host     --shm-size=12gb     -v /dev/shm:/dev/shm     --env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/local/cuda/lib64     --env NVIDIA_VISIBLE_DEVICES=all     --env NVIDIA_DRIVER_CAPABILITIES=all     --env LD_PRELOAD=""     --name zed_5090 --cpuset-cpus="5-10"     zed-5090-receiver