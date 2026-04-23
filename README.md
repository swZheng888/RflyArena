# RflyArena

RflyArena: A Deployment-Oriented Edge-Computing Benchmark Platform for UAV Control with Hardware-in-the-Loop Validation.

This repository is a sanitized export of the original development workspace. It keeps the source packages, Docker workflow, controller models, and launch/config files needed to build and run the benchmark, while excluding local Git history, build caches, logs, and benchmark result dumps.

## Repository Layout

- `src/benchmark_utils`: benchmark orchestration, logging, tuning, analysis, and report generation
- `src/nmpc_control`: NMPC controller package
- `src/Px4Ctrl`: PX4-based baseline controller package
- `src/quadrotor_msgs`: custom ROS messages
- `src/rl_control`: RL controller package and policy assets
- `src/trajectory_publisher`: reference trajectory publication and task generation
- `src/uav_utils`: shared UAV utility code
- `docker/`: containerized deployment and resource-constrained benchmark workflow
- `windows/rflysim`: Windows-side RflySim startup scripts and the DLL dynamics model used by CopterSim

## Requirements

- Ubuntu + ROS 1 catkin workspace environment
- `catkin build` or `catkin_make`
- MAVROS / PX4 runtime environment for controller experiments
- Python dependencies required by the Python-based controller and benchmark scripts

## Build

```bash
cd RflyArena
catkin build
source devel/setup.bash
```

If `catkin build` is unavailable:

```bash
catkin_make
source devel/setup.bash
```

## Controller Entry Points

Windows-side RflySim startup assets:

```bat
windows\rflysim\SITLRun.bat
windows\rflysim\HITLRun.bat
```

The accompanying `windows/rflysim/FX150_H_Model.dll` file is copied into `C:\PX4PSP\CopterSim\external\model\` by the batch scripts before launching CopterSim.

PX4 baseline controller in RflySim:

```bash
roslaunch Px4Ctrl rflysim_pid.launch
```

NMPC controller in RflySim:

```bash
roslaunch nmpc_control rflysim_nmpc.launch
```

RL controller in RflySim:

```bash
roslaunch rl_control rflysim_rl.launch
```

Real-flight controller entry points:

```bash
roslaunch nmpc_control real_exp.launch
roslaunch rl_control real_rl.launch
```

Reusable benchmark launch files:

```bash
roslaunch benchmark_utils benchmark.launch
roslaunch benchmark_utils host_benchmark.launch
roslaunch benchmark_utils real_benchmark.launch
roslaunch benchmark_utils real_benchmark_repeated.launch
roslaunch benchmark_utils comprehensive_benchmark.launch
```

Reference trajectory publisher:

```bash
roslaunch trajectory_publisher trajectory.launch
```

## Paper-Aligned Evaluation Workflow

The paper evaluates controllers through a three-stage chain: PC-SITL, Edge-HITL, and Real Flight. The commands below map each stage to the launch files and scripts included in this repository.

### 1. PC-SITL: controller evaluation in RflySim

Start the simulator and ROS bridge first. In a typical setup this means `roscore`, PX4/MAVROS, and the RflySim data bridge are already running before the controller is launched.

On Windows, the repository includes the original RflySim launchers used to start the simulator side:

```bat
windows\rflysim\SITLRun.bat
```

Start one controller:

```bash
roslaunch Px4Ctrl rflysim_pid.launch
# or
roslaunch nmpc_control rflysim_nmpc.launch
# or
roslaunch rl_control rflysim_rl.launch
```

Run the benchmark node separately so that the same task definitions can be reused across controllers:

```bash
roslaunch benchmark_utils benchmark.launch \
    task_type:=dynamic \
    odom_topic:=/vio/odometry_enu \
    coordinate_frame:=1 \
    trajectory_type:=circle \
    speed_level:=1.0
```

Useful variants:

```bash
roslaunch benchmark_utils benchmark.launch task_type:=hover
roslaunch benchmark_utils benchmark.launch task_type:=dynamic trajectory_type:=figure8 speed_level:=3.0
roslaunch benchmark_utils benchmark.launch task_type:=dynamic trajectory_type:=ellipse speed_level:=1.0
```

### 2. Edge-HITL: constrained onboard-compute evaluation with Docker

This is the deployment-oriented workflow used in the paper. The recommended architecture keeps RflySim, PX4-SITL, MAVROS, odometry conversion, and trajectory publication on the host, while the controller alone runs inside a CPU- and memory-constrained container.

Build the image:

```bash
cd docker
./scripts/build.sh
```

Host-side prerequisites:

```bash
roscore
roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
rosrun nmpc_control rflysim_odom_node.py
```

Run a constrained benchmark:

```bash
./scripts/run_benchmark.sh -p jetson_nano -c nmpc -t circle
./scripts/run_benchmark.sh -p rpi4 -c pid -t figure8 -s 1.5
./scripts/run_benchmark.sh -p jetson_nano -c rl -t ellipse -s 1.0
```

If the controller container is already running, publish the host-side benchmark task directly:

```bash
roslaunch benchmark_utils host_benchmark.launch \
    task_type:=dynamic \
    trajectory_type:=circle \
    speed_level:=1.0 \
    duration:=30
```

### 3. Real Flight: hardware validation and safety-gated execution

For real-flight experiments, start the controller node first, then launch the task sequencer used in the paper. The real benchmark node is intentionally start-gated so that takeoff and task execution are triggered manually.

Start one controller:

```bash
roslaunch nmpc_control real_exp.launch
# or
roslaunch rl_control real_rl.launch
```

Launch the repeated evaluation workflow you typically use:

```bash
roslaunch benchmark_utils real_benchmark_repeated.launch \
    controller_name:=nmpc \
    tasks_file:=$(rospack find benchmark_utils)/config/real_benchmark_tasks.yaml \
    odom_topic:=/vio/odometry_enu \
    repeats:=1 \
    run_analysis:=true \
    record_bag:=true
```

Manual task control:

```bash
rostopic pub /real_benchmark/start std_msgs/Empty "{}"
rostopic pub /real_benchmark/abort std_msgs/Empty "{}"
```

If rosbag logging is required during real-flight experiments:

```bash
roslaunch benchmark_utils real_benchmark_repeated.launch \
    controller_name:=nmpc \
    tasks_file:=$(rospack find benchmark_utils)/config/real_benchmark_tasks.yaml \
    repeats:=10 \
    record_bag:=true \
    odom_topic:=/vio/odometry_enu
```

### 4. Batch experiments and repeated runs

For automated sweeps over multiple trajectories or task configurations:

```bash
roslaunch benchmark_utils comprehensive_benchmark.launch
```

For Docker-based comparison batches under several compute profiles:

```bash
cd docker
./scripts/run_comparison.sh
./scripts/run_comparison.sh --quick
```

### 5. Automatic tuning with Optuna

The repository also includes the Optuna-based tuning workflow used to configure deployment-sensitive controller parameters before the main benchmark comparison.

Single-trajectory tuning example:

```bash
python3 src/benchmark_utils/scripts/auto_tuner.py \
    --controller nmpc \
    --trajectory circle \
    --trials 10
```

Multi-trajectory tuning example:

```bash
python3 src/benchmark_utils/scripts/auto_tuner.py \
    --controller nmpc \
    --trajectory "circle,figure8,ellipse" \
    --trials 20
```

The auto-tuner expects the ROS environment, controller topics, and benchmark pipeline to be available. In practice, it is typically used after the relevant simulation/controller stack has been brought up for PC-SITL evaluation.

Tuning outputs are written under `tuning_results/`, including the Optuna database and the best parameter summary for each run.

## Docker Workflow

Build the benchmark image:

```bash
cd docker
./scripts/build.sh
```

Run a benchmark under a resource profile:

```bash
./scripts/run_benchmark.sh -p jetson_nano -c nmpc -t circle
./scripts/run_benchmark.sh -p rpi4 -c pid -t figure8 -s 1.5
```

Run comparison batches:

```bash
./scripts/run_comparison.sh
./scripts/run_comparison.sh --quick
```

## Windows RflySim Assets

The `windows/rflysim/` directory contains the Windows-side launcher scripts used with the RflySim/CopterSim environment:

- `SITLRun.bat`: starts PX4 SITL-based multi-vehicle simulation in RflySim
- `HITLRun.bat`: starts PX4 HITL simulation and binds each vehicle to a Pixhawk COM port
- `FX150_H_Model.dll`: DLL-based dynamics model copied into CopterSim before launch

By default the scripts expect the simulator toolchain under `C:\PX4PSP`. If your local RflySim installation lives elsewhere, edit the `PSP_PATH` and `PSP_PATH_LINUX` variables at the top of the batch files before running them.

## What Was Removed From the Original Workspace

- local Git history and nested Git metadata
- catkin tool state and cache directories
- Docker runtime logs
- benchmark result dumps and generated CSV summaries
- Python bytecode caches and local preview artifacts
- local desktop shortcut / machine-specific files

## Notes Before Publishing

- Review repository-wide licensing before making the project public
- Check whether any model files, datasets, or third-party packages require additional attribution
- Replace or anonymize any machine-specific paths that may still appear in comments or scripts
