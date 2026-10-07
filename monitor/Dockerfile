FROM ros:jazzy
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-psutil python3-pip ros-jazzy-rmw-cyclonedds-cpp \
    && pip install --no-cache-dir --break-system-packages nvidia-ml-py \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY hw_monitor.py .
CMD ["python3", "hw_monitor.py"]
