FROM ros:humble
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install requirements
ARG DEBIAN_FRONTEND=noninteractive
RUN apt update
RUN apt install -y libboost-all-dev libcairo2-dev python3-pip curl tar nano ros-humble-example-interfaces

# Add MOON
RUN mkdir -p /MOON
COPY . /MOON
RUN pip install --upgrade pip
RUN pip install /MOON

# Set up the entrypoint.
CMD /bin/bash
COPY .docker/entrypoint.sh /entrypoint.sh
RUN chmod +x entrypoint.sh
WORKDIR /MOON
ENTRYPOINT [ "/entrypoint.sh" ]
