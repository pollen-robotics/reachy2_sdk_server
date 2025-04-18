# Reachy2 SDK Server

This repository contains the ROS 2-based SDK server implementation for controlling and interfacing with **Reachy 2**, the humanoid robot developed by Pollen Robotics. It bridges between the gRPC communication layer and ROS 2 interfaces, providing a modular and extensible structure to manage Reachy's hardware components and software interactions.

## Overview

The server is organized as a ROS 2 Python package and exposes key functionalities via gRPC and ROS 2 messaging, enabling high-level control and integration of the Reachy platform.

### Key Components

- **`abstract_bridge_node.py`**  
  Base class to implement bridge nodes that map gRPC services to ROS 2 actions, topics, and services.

- **`components.py` & `parts.py`**  
  Describe and organize Reachy's hardware components (arms, head, etc.) and their corresponding software entities.

- **`conversion.py`**  
  Utilities for converting between gRPC and ROS 2 message formats.

- **`utils.py`**  
  Helper functions and shared utilities across components.

- **`grpc_server/`**  
  Contains the server implementation for gRPC, handling communication between external clients and the ROS 2 backend.

## Installation

Install as a standard ROS 2 Python package using `colcon`:

```bash
colcon build --packages-select reachy_sdk_server
source install/setup.bash
```

Dependencies are listed in `requirements.txt` and `pyproject.toml`.

## Usage

This package is meant to be launched as part of the complete Reachy2 system. It exposes gRPC endpoints that control robot components and forward commands to the underlying ROS 2 infrastructure.

## License

This repository is licensed under the Apache License 2.0. See `LICENSE` for details.
