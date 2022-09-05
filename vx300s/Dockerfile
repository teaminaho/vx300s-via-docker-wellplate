FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive

RUN sed -r -i 's|(archive\|security)\.ubuntu\.com/ubuntu|ftp.jaist.ac.jp/pub/Linux/ubuntu|' /etc/apt/sources.list && \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y lsb-release software-properties-common python3-pip && \
    apt-get autoremove --purge && \
    apt-get autoclean && \
    rm -rf /var/cache/apt/*

WORKDIR /app
COPY xsarm_amd64_install.sh /app/
RUN ./xsarm_amd64_install.sh

RUN pip3 install jupyterlab

COPY arm_apps/ /app/arm_apps/
COPY docker-entrypoint.sh arm_ctrl_setup.bash /app/

RUN /app/arm_ctrl_setup.bash

ENTRYPOINT [ "./docker-entrypoint.sh" ]
