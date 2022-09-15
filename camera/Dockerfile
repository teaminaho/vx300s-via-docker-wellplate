FROM ros:noetic

RUN apt-get update && apt-get install -y ros-noetic-usb-cam && apt-get autoremove --purge -y && apt-get autoclean && rm -rf /var/cache/apt/*

WORKDIR /app
COPY docker-entrypoint.bash /app/

ENTRYPOINT "/app/docker-entrypoint.bash"
