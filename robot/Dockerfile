FROM --platform=linux/amd64 localhost:5000/webots_bridge

RUN apt-get update \
    && apt-get install -y libgl1 ros-noetic-cv-bridge \
    && rm -rf /var/lib/apt/lists/*
    
RUN pip install opencv_python