# RflyArena Docker Workflow

This directory contains the Docker-based runtime used by RflyArena to benchmark UAV controllers under standardized compute constraints. The goal is to emulate embedded onboard computers with Docker and cgroup limits so that PID/SO(3), NMPC, and RL controllers can be compared under reproducible deployment conditions.

## Purpose

The Docker workflow is designed for the deployment-oriented evaluation chain described in the paper:

- `PC-SITL`: unconstrained controller evaluation on a desktop machine
- `Edge-HITL`: controller execution inside a resource-constrained container while the simulator remains on the host
- `Real Flight`: hardware validation after the simulation-side ranking has been established

The Docker setup is mainly used for the `Edge-HITL` stage.

## Recommended Architecture

### Host runs the simulator, container runs only the controller

This is the recommended mode for deployment-faithful benchmarking.

```text
Host machine
├── RFlySim / PX4-SITL / MAVROS / roscore
├── rflysim_odom_node (frame conversion if needed)
└── benchmark_node (publishes /position_cmd)

Docker container
└── Controller only (PID/SO(3), NMPC, or RL)
    ├── subscribes: /position_cmd, /vio/odometry_enu
    └── publishes:  /mavros/setpoint_raw/attitude
```

Why this mode is preferred:

- only the controller is compute-constrained
- trajectory generation does not consume container resources
- the setup more closely matches real deployment, where planning and supervision may run offboard while the controller runs onboard

### Legacy architecture

The scripts also support a legacy mode in which trajectory publication, odometry conversion, and controller logic all run inside the container. This can be enabled with `--legacy`, but it is not the default because it is less faithful to the intended deployment setting.

## Directory Structure

```text
docker/
├── Dockerfile
├── docker-compose.yml
├── entrypoint.sh
├── configs/
│   └── ros_network.env
└── scripts/
    ├── build.sh
    ├── host_benchmark.sh
    ├── monitor_resources.sh
    ├── resource_monitor.py
    ├── run_benchmark.sh
    └── run_comparison.sh
```

## Quick Start

### 1. Build the image

From the repository root:

```bash
chmod +x docker/scripts/*.sh
./docker/scripts/build.sh
```

To tag a specific image version:

```bash
./docker/scripts/build.sh -t v1.0.0
```

### 2. Run a single benchmark

Examples:

```bash
./docker/scripts/run_benchmark.sh -p jetson_nano -c nmpc -t circle
./docker/scripts/run_benchmark.sh -p rpi4 -c pid -t figure8 -s 1.5
./docker/scripts/run_benchmark.sh -p jetson_nano -c rl -t ellipse -s 1.0
```

Interactive shell mode:

```bash
./docker/scripts/run_benchmark.sh -p jetson_nano -i
```

Legacy mode:

```bash
./docker/scripts/run_benchmark.sh -p jetson_nano -c nmpc -t circle --legacy
```

### 3. Publish host-side tasks only

If the controller container has already been launched manually, you can publish the benchmark task from the host:

```bash
./docker/scripts/host_benchmark.sh -t circle -s 1.0 -d 30
./docker/scripts/host_benchmark.sh --help
```

### 4. Run comparison batches

```bash
./docker/scripts/run_comparison.sh
./docker/scripts/run_comparison.sh --quick
```

Example custom sweep:

```bash
./docker/scripts/run_comparison.sh \
    --profiles jetson_nano,jetson_xavier \
    --controllers nmpc,pid \
    --trajectories circle,figure8
```

## Built-In Resource Profiles

The benchmark scripts provide several built-in profiles that approximate representative onboard computers.

| Profile | CPU limit | Memory limit | Intended target |
|---|---:|---:|---|
| `jetson_nano` | 1.0 core | 2 GB | Jetson Nano-class embedded computer |
| `jetson_xavier` | 2.0 cores | 4 GB | Jetson Xavier NX-class embedded computer |
| `rpi4` | 0.5 core | 1 GB | Raspberry Pi 4-class platform |
| `px4_fmu` | 0.1 core | 256 MB | FMU-scale extreme stress test |
| `unlimited` | none | none | unconstrained reference |

### Rough scaling rule

The original workspace used the following rule of thumb when mapping ARM platforms to an x86 host:

```text
x86-equivalent cores = ARM cores × ARM frequency (GHz) × IPC factor / 3.0
```

Typical IPC factors:

- Cortex-A57: about `0.5`
- Cortex-A72: about `0.55`

This is only a practical approximation for benchmarking, not a hardware-accurate performance model.

These limits are currently defined directly inside the shell scripts rather than loaded from external YAML profile files.

## Expected Controller Behavior

The exact numbers depend on the host machine, solver settings, and trajectory difficulty, but the intended qualitative behavior is:

- `PID/SO(3)`: lightweight and generally stable even under strong compute limits
- `NMPC`: sensitive to CPU budget, prediction horizon, and solver configuration
- `RL`: usually lightweight per step, but may still show latency tails depending on runtime and model size

In constrained settings, NMPC often requires a shorter horizon or reduced prediction window to remain feasible.

## Monitoring

### Host-side monitoring

```bash
./docker/scripts/monitor_resources.sh benchmark_jetson_nano
./docker/scripts/monitor_resources.sh -a
./docker/scripts/monitor_resources.sh -i 0.5 benchmark_jetson_nano
```

### Inside the container

If resource monitoring is enabled, the container writes runtime usage to a CSV file.

```bash
cat /tmp/resource_usage.csv
ps aux | grep resource_monitor
```

## Integration with RFlySim / PX4

### Recommended host-container workflow

1. Start the host-side ROS and simulator stack:

```bash
roscore
roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
rosrun nmpc_control rflysim_odom_node.py
```

2. Start the constrained benchmark:

```bash
./docker/scripts/run_benchmark.sh -p jetson_nano -c nmpc -t circle
```

In this mode:

- the host publishes `/position_cmd`
- the host exposes `/vio/odometry_enu`
- the container runs only the controller

### Docker Compose mode

If you prefer compose-based orchestration:

```bash
docker-compose up roscore benchmark_jetson_nano
```

## Advanced Usage

### Auto-tuning inside a constrained environment

```bash
./docker/scripts/run_benchmark.sh -p jetson_xavier -i
```

Inside the container:

```bash
python3 /root/catkin_ws/src/benchmark_utils/scripts/auto_tuner.py \
    --controller nmpc \
    --trajectory "circle,figure8" \
    --trials 20
```

### Batch plot generation

After a benchmark run:

```bash
python3 /root/catkin_ws/src/benchmark_utils/scripts/batch_publication_plots.py \
    --input-dir /root/catkin_ws/benchmark_results \
    --output-dir /root/catkin_ws/publication_plots
```

## Troubleshooting

### The container cannot reach ROS master

Check the network first:

```bash
docker exec benchmark_jetson_nano ping host.docker.internal
```

The workflow is designed around host networking, so make sure the container is started with the expected network configuration.

### NMPC times out under tight compute limits

Reduce solver complexity, for example:

```python
N = 20
Tf = 0.2
```

The exact file and parameter names depend on the controller implementation you are using.

### How do I verify that resource limits are active?

Inside the container:

```bash
cat /sys/fs/cgroup/cpu/cpu.cfs_quota_us
cat /sys/fs/cgroup/memory/memory.limit_in_bytes
```

You can also inspect the live container with `docker stats`.

## Citation

If you use this benchmark workflow in academic work, please cite the corresponding RflyArena paper.

## License

Refer to the repository-level license and any third-party dependency notices before redistribution.
