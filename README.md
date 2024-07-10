# dua_app_management

Collection of software modules and source files for the management of applications and processes in the Distributed Unified Architecture.

## Contents

This package contains modules, both compiled and source-only, to be included in DUA-based projects. It is particularly aimed at the development of ROS 2 applications, with the aim of easing some common tasks and code writing that is often repeated, or involves tinkering with the OS to do simple things.

### ros2_app_manager

This library is made of two parts.

#### ROS2AppManager

Header-only library that implements a wrapper object for the usual `main` thread code of ROS 2 applications, *i.e.*, processes that run single nodes or collections thereof not as components.

The `main` thread of a ROS 2 app should mostly contain calls to the following methods of the `ROS2AppManager` object, which summarize the lifecycle of a ROS 2 process:

- Constructor, to initialize the process, configure I/O and create ROS 2 objects.
- `ROS2AppManager::run` to spin the executor (note that this does not return unless a signal is caught).
- `ROS2AppManager::shutdown` to shutdown the ROS 2 application by deleting its objects.

There are also getters to access the ROS 2 objects, such as the node, the executor and the context.

#### `dua_component_container_mt`

Revised version of the `rclcpp`'s `component_container_mt` container process, which makes use of the tools in this package to robustify the process and make it more flexible: it is entirely built around the `MultiThreadedExecutor` ROS 2 executor for maximum performance, and it traces signals and allows to use `SIGUSR1` and `SIGUSR2` to interrupt threads without terminating the process.

### ros2_signal_handler

POSIX-compliant signal handler module for ROS 2 applications.

**The main motivation behind this is that `rclcpp::init` installs its own signal handler, thus overriding any other termination procedure you might need.**

Can be used by any ROS 2 standalone application but also by component managers, and allows to define custom behaviours upon process termination.

This also adds support for initialization of custom ROS 2 contexts, uses `sigaction` to manage handlers. You can also provide a pointer to an `rclcpp::Executor` to be canceled when any of the traced signal is received, or not provide it and manage it inside one of your custom handlers.

This module is made of a single header only: `signal_handler.hpp`, so to access its functions you just need to include that.

All operations are performed by a `ROS2SignalHandler::SignalHandler` object; to use it, interface it with the following public methods:

- `SignalHandler::init` to initialize it;
- `SignalHandler::install` to install a new signal handler for a given signal;
- `SignalHandler::uninstall` to uninstall a previously-installed signal handler;
- `SignalHandler::ignore` to ignore a specific signal;
- `SignalHandler::fini` to reset the `SignalHandler` object and restore everything to default settings and behaviours.

Routines may throw `std` exceptions upon error.

Refer to code comments for specific API documentation.

## Requirements

Builds on ROS 2 Humble Hawksbill.

See [`package.xml`](package.xml) for more information.

---

## Copyright and License

Copyright 2024 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.
