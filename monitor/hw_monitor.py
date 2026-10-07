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


# ---------- temperature helpers ----------

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


def pick(temps, names):
    for n in names:
        if n in temps:
            return temps[n]
    return None


# ---------- GPU collectors ----------
# Each backend has available() and read(). read() returns a dict that may
# contain 'gpu_percent' and/or 'gpu_temp_c'.

class NvmlGpu:
    """Desktop NVIDIA GPU via NVML (needs NVIDIA Container Toolkit in Docker)."""
    name = 'nvml'

    def __init__(self):
        self.handle = None
        self.nv = None
        try:
            import pynvml
            pynvml.nvmlInit()
            self.nv = pynvml
            self.handle = pynvml.nvmlDeviceGetHandleByIndex(0)
        except Exception:
            pass  # no library, no driver, or Jetson (Tegra has no NVML)

    def available(self):
        return self.handle is not None

    def read(self):
        nv = self.nv
        try:
            return {
                'gpu_percent': nv.nvmlDeviceGetUtilizationRates(self.handle).gpu,
                'gpu_temp_c': nv.nvmlDeviceGetTemperature(
                    self.handle, nv.NVML_TEMPERATURE_GPU),
            }
        except Exception:
            return {'gpu_percent': None}


class JetsonGpu:
    """Jetson GPU load from sysfs. Temperature comes from the thermal zones."""
    name = 'jetson'

    def __init__(self):
        self.path = next((p for p in GPU_LOAD_PATHS if Path(p).exists()), None)

    def available(self):
        return self.path is not None

    def read(self):
        try:
            raw = int(Path(self.path).read_text().strip())
            return {'gpu_percent': round(raw / 10.0, 1)}
        except (OSError, ValueError):
            return {'gpu_percent': None}


class NoGpu:
    """Fallback for machines with no readable GPU (e.g. Raspberry Pi)."""
    name = 'none'

    def available(self):
        return True

    def read(self):
        return {'gpu_percent': None}


GPU_BACKENDS = {'nvml': NvmlGpu, 'jetson': JetsonGpu, 'none': NoGpu}


def choose_gpu(requested):
    if requested != 'auto':
        return GPU_BACKENDS.get(requested, NoGpu)()
    for cls in (NvmlGpu, JetsonGpu):
        backend = cls()
        if backend.available():
            return backend
    return NoGpu()


# ---------- ROS node ----------

class HwMonitor(Node):
    def __init__(self):
        super().__init__('hw_monitor')
        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('host', socket.gethostname().replace('-', '_'))
        self.declare_parameter('gpu_backend', 'auto')  # auto | nvml | jetson | none

        host = self.get_parameter('host').value
        rate = self.get_parameter('rate_hz').value
        self.gpu = choose_gpu(self.get_parameter('gpu_backend').value)

        self.pub = self.create_publisher(
            String, f'/{host}/stats', qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f'Publishing on /{host}/stats at {rate} Hz')
        self.get_logger().info(f'GPU backend: {self.gpu.name}')

    def tick(self):
        temps = {**read_psutil_temps(), **read_thermal_zones()}
        gpu = self.gpu.read()
        data = {
            'cpu_percent': psutil.cpu_percent(),
            'ram_percent': psutil.virtual_memory().percent,
            'gpu_percent': gpu.get('gpu_percent'),
            'cpu_temp_c': pick(temps, CPU_NAMES),
            'gpu_temp_c': gpu.get('gpu_temp_c', pick(temps, GPU_NAMES)),
            'gpu_backend': self.gpu.name,
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
