FROM ros:jazzy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install requirements
ARG DEBIAN_FRONTEND=noninteractive
RUN apt update && \
    apt install -y python3-pip curl tar nano ros-jazzy-example-interfaces ros-jazzy-plotjuggler && \
    apt install -y python3-jinja2 python3-cookiecutter python3-jedi python3-prompt-toolkit
RUN pip3 install --break-system-packages websocket_client rospy_message_converter pyyaml reelay

# Add MOON
RUN mkdir -p /MOON
COPY . /MOON

# Set up the entrypoint.
CMD /bin/bash
COPY .docker/entrypoint.sh /entrypoint.sh
RUN chmod +x entrypoint.sh
WORKDIR /MOON
ENTRYPOINT [ "/entrypoint.sh" ]
