/**
 * DUA multithreaded component container.
 *
 * Roberto Masocco <r.masocco@dotxautomation.com>
 *
 * July 10, 2024
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define DUA_COMPONENT_CONTAINER

#include <cstdlib>
#include <csignal>

#include <rclcpp/rclcpp.hpp>

#include <ros2_app_manager/ros2_app_manager.hpp>
#include <ros2_signal_handler/ros2_signal_handler.hpp>

#include <rclcpp_components/component_manager.hpp>

#define UNUSED(arg) (void)(arg)

using namespace dua_app_management;

int main(int argc, char ** argv)
{
  ROS2AppManager<rclcpp::executors::MultiThreadedExecutor> app_manager(
    argc,
    argv,
    "dua_component_container_mt");
  auto executor = app_manager.get_executor();

  SignalHandler & sig_handler = SignalHandler::get_global_signal_handler();
  sig_handler.init(
    app_manager.get_context(),
    "dua_component_container_mt_signal_handler",
    nullptr,
    [executor](int sig, std::string & logger_name) -> void {
      UNUSED(logger_name);
      if (sig == SIGINT || sig == SIGTERM || sig == SIGQUIT) {
        executor->cancel();
      }
    });
  sig_handler.install(SIGINT);
  sig_handler.install(SIGTERM);
  sig_handler.install(SIGQUIT);
  sig_handler.install(SIGPIPE);
  sig_handler.install(SIGUSR1);
  sig_handler.install(SIGUSR2);
  sig_handler.install(SIGCHLD);
  sig_handler.ignore(SIGHUP);

  app_manager.run();

  app_manager.shutdown();
  sig_handler.fini();

  exit(EXIT_SUCCESS);
}
