import json
import socket
from pathlib import Path

import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

# Sensor names that count as "the CPU" / "the GPU", checked in order.
# Jetson: CPU-therm / GPU-therm. Pi: cpu-thermal. Desktop: coretemp / k10temp.
CPU_NAMES = ['cpu-therm', 'cpu-thermal', 'coretemp', 'k10temp', 'x86_pkg_temp']
GPU_NAMES = ['gpu-therm', 'gpu-thermal', 'amdgpu', 'nouveau']

# Jetson GPU load files report 0-1000 (1000 = 100%). First one that exists wins.
GPU_LOAD_PATHS = [
    '/sys/devices/platform/17000000.ga10b/load',
    '/sys/devices/platform/gpu.0/load',
    '/sys/class/devfreq/17000000.ga10b/device/load',
    '/sys/devices/platform/bus@0/17000000.gpu/load',
]


def read_thermal_zones():
    """Read /sys/class/thermal; skip unpopulated sensors (e.g. -256000)."""
    temps = {}
    for zone in sorted(Path('/sys/class/thermal').glob('thermal_zone*')):
        try:
            name = (zone / 'type').read_text().strip().lower()
            milli = int((zone / 'temp').read_text().strip())
        except (OSError, ValueError):
            continue
        if milli < -50000:  # sensor not connected
            continue
        temps[name] = round(milli / 1000.0, 1)
    return temps


def read_psutil_temps():
    try:
        return {
            name.lower(): round(entries[0].current, 1)
            for name, entries in psutil.sensors_temperatures().items()
            if entries
        }
    except (AttributeError, OSError):
        return {}


def read_gpu_percent():
    """Jetson GPU load as a percentage, or None if no load file is found."""
    for path in GPU_LOAD_PATHS:
        try:
            return round(int(Path(path).read_text().strip()) / 10.0, 1)
        except (OSError, ValueError):
            continue
    return None


def pick(temps, names):
    for n in names:
        if n in temps:
            return temps[n]
    return None


class HwMonitor(Node):
    def __init__(self):
        super().__init__('hw_monitor')
        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('host', socket.gethostname().replace('-', '_'))

        host = self.get_parameter('host').value
        rate = self.get_parameter('rate_hz').value

        self.pub = self.create_publisher(
            String, f'/{host}/stats', qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f'Publishing on /{host}/stats at {rate} Hz')

    def tick(self):
        temps = {**read_psutil_temps(), **read_thermal_zones()}
        data = {
            'cpu_percent': psutil.cpu_percent(),
            'ram_percent': psutil.virtual_memory().percent,
            'gpu_percent': read_gpu_percent(),
            'cpu_temp_c': pick(temps, CPU_NAMES),
            'gpu_temp_c': pick(temps, GPU_NAMES),
            'temps_c': temps,
        }
        self.pub.publish(String(data=json.dumps(data)))


def main():
    rclpy.init()
    node = HwMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
