# [![Peng](https://raw.githubusercontent.com/makeecat/Peng/main/assets/Peng.svg)](https://github.com/makeecat/Peng)

[![License](https://img.shields.io/badge/license-MIT%2FApache-blue.svg)](https://github.com/makeecat/Peng#license)
[![Crates.io](https://img.shields.io/crates/v/peng_quad.svg)](https://crates.io/crates/peng_quad)
[![Downloads](https://img.shields.io/crates/d/peng_quad.svg)](https://crates.io/crates/peng_quad)
[![Docs](https://docs.rs/peng_quad/badge.svg)](https://docs.rs/peng_quad/latest/peng_quad/)
[![CI](https://github.com/makeecat/Peng/actions/workflows/CI.yml/badge.svg)](https://github.com/makeecat/Peng/actions/workflows/CI.yml)
[![dependency status](https://deps.rs/repo/github/makeecat/peng/status.svg)](https://deps.rs/repo/github/makeecat/peng)
[![Gitter](https://img.shields.io/gitter/room/peng/peng)](https://app.gitter.im/#/room/#peng:gitter.im)

## 🔍 Overview

Peng is a minimal quadrotor autonomy framework written in Rust that provides real-time dynamics simulation, trajectory planning, and control with modern visualization capabilities.

[![rerun demo](https://raw.githubusercontent.com/makeecat/Peng/main/assets/Peng_demo.gif)](https://rerun.io/viewer?url=https%3A%2F%2Fyangrobotics.com%2Ffiles%2Fpeng_v0.5.3_demo.rrd)

## 🎯 Key Features

- 🚁 **Real-time Simulation**
  - High-fidelity quadrotor dynamics with configurable parameters
  - IMU and depth sensor simulation
  - Optional RK4 integration for accurate dynamics
- 🎮 **Advanced Control**
  - PID control for position and attitude with tunable gains
  - Integral windup prevention
  - Support for different control frequencies
- 📍 **Rich Trajectory Planning**
  - Minimum jerk line trajectory planner
  - Lissajous curve planner
  - Circular trajectory planner
  - Obstacle avoidance planner
  - Waypoint navigation planner
  - Landing planner
- 📊 **Visualization & Debug**
  - Real-time 3D visualization via rerun.io
  - Depth map rendering
  - State telemetry logging
  - Configurable logging frequencies
- ⚡ **Performance**
  - Memory-safe and Efficient Rust implementation
  - Multi-threaded depth rendering

## 🚀 Getting Started

### Prerequisites

- [Rust](https://www.rust-lang.org/tools/install)
- [rerun-cli](https://rerun.io/docs/getting-started/installing-viewer)

### Installation from Crates.io

```bash
# Install rerun-cli (ensure version >= 0.19.0)
cargo install rerun-cli
cargo install peng_quad
peng_quad config/quad.yaml
```

### Installation from Source

```bash
# Install rerun-cli (ensure version >= 0.19.0)
cargo install rerun-cli
git clone https://github.com/makeecat/Peng.git
cd Peng
cargo run --release config/quad.yaml
```

## ⚙️ Configuration

- You can configure the simulation through config file, see [quad.yaml](config/quad.yaml) for example.
- Configure simulation parameters such as mass, inertia, and control gains.
- Configure control parameters such as PID gains.
- Configure trajectory planner parameters such as waypoints, obstacles, and trajectory type.
- Configure visualization parameters such as camera intrinsics and depth rendering.

### Running a SITL with Liftoff®: FPV Drone Racing

This fork of Peng for the Hybrid Systems Lab supports interfaces to external simulators, name Liftoff®: FPV Drone Racing.
Peng is a realistic FPV drone simualtion designed to give hobbyists and enthusiasts flight experienece that is transferrable
to flying a real vehicle. Liftoff provides a means for retrieving detailed state feedback from the simulation from commands
on a UDP port. With the addition of a CyberRC device, we can close the loop and fly in Liftoff with automatic control.

To run Peng with Liftoff, follow these steps:

1. Install Liftoff: FPV Drone Racing from [Steam](https://store.steampowered.com/app/410340/Liftoff_FPV_Drone_Racing/).

2. Configure Liftoff telemetry so that the controller can get feedback from the drone.
   See the guide [here](https://steamcommunity.com/sharedfiles/filedetails/?id=3160488434).
   Liftoff provides feedback from the internal drone simulation at roughly 100Hz, which is quite good for position control.
   The configuration values specified in this file influence what is included in the feedback packet we get from Liftoff.
   By extension, this also influences how we need to parse the binary payload.
   The binary parser is currently implemented to read data from this configuration:

   ```json
   {
     "EndPoint": "127.0.0.1:9001",
     "StreamFormat": [
       "Timestamp",
       "Position",
       "Attitude",
       "Gyro",
       "Input",
       "MotorRPM"
     ]
   }
   ```

   If you need to add or remove items from telemetry, you will also need to modify the LiftoffPacket struct
   in `src/liftoff_quadrotor.rs`. Remote endpoints can also be specified, with the caveat that you will need to configure your network and interfaces to support such a setup.

3. Attach a CyberRC device to the host PC that is running Peng. This device will be used to send control signals to Liftoff. The device should be connected to the host PC via USB. You will also need a USB to serial UART adapter to communicate with the device.

4. Run Peng with a configuration file set up to run with Liftoff.
   This configuration file will be quite similar to the stock configuration file used to run self contained simulations with Peng, with some minor additional configuration for the port to retrieve feedback from Liftoff, which serial interface the CyberrC is connnected to, and what baud rate to use to send serial commands to the device.

## 🔧 Troubleshooting

If you encountered any issue with the rerun:

1. Verify rerun-cli version matches rerun version in [Cargo.toml](https://github.com/makeecat/Peng/blob/main/Cargo.toml):

```bash
rerun --version
```

2. For Linux/WSL2 users, consult the [rerun troubleshooting](https://rerun.io/docs/getting-started/troubleshooting).

## 🗺️ Roadmap

- [ ] Wind field and environmental effects
- [ ] Motor dynamics simulation
- [ ] Multi-quadrotor simulation
- [ ] Model Predictive Control (MPC)

## 🤝 Contributing

We welcome contributions of all kinds! Please check out the [Contributing Guidelines](CONTRIBUTING.md) for more details.

## 📄 License

Peng is free, open source and permissively licensed!
Except where noted (below and/or in individual files), all code in this repository is dual-licensed under either:

- MIT License ([LICENSE-MIT](LICENSE-MIT) or [http://opensource.org/licenses/MIT](http://opensource.org/licenses/MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0))
  at your option.

This means you can select the license you prefer!

## 🐦 Why call it Peng?

Peng (鵬/鹏, péng), or Dapeng (大鵬), represents a mythical Chinese bird that transforms from a giant Kun (鯤/鲲) fish. This name reflects our framework's adaptability and transformative capabilities.

## 📝 Blog Posts

- [Peng #1: Minimal quadrotor pipeline in Rust](https://yangrobotics.com/peng-1-minimal-quadrotor-pipeline-in-rust)
- [Peng #2: Error Handling, Configuration System and Obstacle Avoidance Planner](https://yangrobotics.com/peng-2-error-handling-configuration-system-and-obstacle-avoidance-planner)
- [Peng #3: Optimization of Depth Rendering and RK4-based Dynamics Update](https://yangrobotics.com/peng-3-optimization-of-depth-rendering-and-rk4-based-dynamics-update)

## 📚 Citation

If you use this project in your research or work, please cite it as follows:

```bibtex
@software{peng_quad,
  author       = {Yang Zhou},
  title        = {Peng: A Minimal Quadrotor Autonomy Framework in Rust},
  year         = {2024},
  publisher    = {GitHub},
  journal      = {GitHub repository},
  howpublished = {\url{https://github.com/makeecat/peng}},
}
```
