FROM ros:noetic

RUN apt-get update \
    && apt-get install -y python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY python_webots_bridge /tmp/python_webots_bridge_src
RUN pip install /tmp/python_webots_bridge_src/.
