FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive

RUN sed -r -i 's|(archive\|security)\.ubuntu\.com/ubuntu|ftp.jaist.ac.jp/pub/Linux/ubuntu|' /etc/apt/sources.list && \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y lsb-release software-properties-common python3-pip openssh-server && \
    apt-get autoremove --purge && \
    rm -rf /var/cache/apt/*

WORKDIR /app
COPY docker-entrypoint.sh xsarm_amd64_install.sh /app/
RUN ./xsarm_amd64_install.sh

CMD [ "./docker-entrypoint.sh" ]
