/**
 * Wrapper class for the common main thread operations of a ROS 2 application.
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

#ifndef DUA_APP_MANAGEMENT__ROS2_APP_MANAGER_HPP_
#define DUA_APP_MANAGEMENT__ROS2_APP_MANAGER_HPP_

#include <memory>
#include <stdexcept>
#include <stdio.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#ifdef DUA_COMPONENT_CONTAINER
#include <rclcpp_components/component_manager.hpp>
#endif

namespace dua_app_management
{

#ifndef DUA_COMPONENT_CONTAINER

/**
 * Wraps common ROS 2 main thread tasks.
 */
template<class ExecutorT, class NodeT>
class ROS2AppManager final
{
public:
  /**
   * @brief Constructor: initializes the ROS 2 app.
   *
   * @param argc Number of process command line arguments.
   * @param argv Process command line arguments.
   * @param module_name Application name.
   *
   * @throws RuntimeError if I/O buffering configuration fails.
   */
  ROS2AppManager(
    int argc,
    char ** argv,
    std::string && module_name = std::string("ros2_app_manager"))
  : logger_name_(module_name)
  {
    // Disable I/O buffering
    if (setvbuf(stdout, NULL, _IONBF, 0)) {
      RCLCPP_FATAL(
        rclcpp::get_logger(logger_name_),
        "Failed to disable I/O buffering");
      throw std::runtime_error("Failed to disable I/O buffering.");
    }

    // Initialize ROS 2 context
    context_ = rclcpp::contexts::get_global_default_context();
    rclcpp::InitOptions init_options = rclcpp::InitOptions();
    init_options.shutdown_on_signal = false;
    context_->init(argc, argv, init_options);

    // Initialize ROS 2 node
    rclcpp::NodeOptions node_opts = rclcpp::NodeOptions();
    node_opts.context(context_);
    node_ = std::make_shared<NodeT>(node_opts);

    // Initialize ROS 2 executor
    rclcpp::ExecutorOptions executor_opts = rclcpp::ExecutorOptions();
    executor_opts.context = context_;
    executor_ = std::make_shared<ExecutorT>(executor_opts);
    executor_->add_node(node_);

    // Set object state
    is_valid_ = true;

    RCLCPP_WARN(
      rclcpp::get_logger(logger_name_),
      "Process started (%d)",
      getpid());
  }

  /**
   * @brief Destructor: shuts down the ROS 2 context if object is still valid.
   */
  ~ROS2AppManager()
  {
    if (is_valid_) {
      shutdown();
    }
  }

  /**
   * @brief Runs the app, does not return until stopped by a signal.
   *
   * @throws RuntimeError if the app manager is not valid.
   */
  void run()
  {
    if (!is_valid_) {
      throw std::runtime_error("ROS 2 app manager is not valid.");
    }
    RCLCPP_INFO(
      rclcpp::get_logger(logger_name_),
      "Starting executor...");
    executor_->spin();
  }

  /**
   * @brief Shuts down the app, deleting ROS 2 structures.
   */
  void shutdown()
  {
    RCLCPP_WARN(
      rclcpp::get_logger(logger_name_),
      "Shutting down...");
    executor_->remove_node(node_);
    node_.reset();
    context_->shutdown("ROS2AppManager::shutdown");
    executor_.reset();
    is_valid_ = false;
  }

  /**
   * @brief Gets the ROS 2 context.
   *
   * @return ROS 2 context.
   */
  std::shared_ptr<rclcpp::Context> get_context() const
  {
    return context_;
  }

  /**
   * @brief Gets the ROS 2 executor.
   */
  std::shared_ptr<ExecutorT> get_executor() const
  {
    return executor_;
  }

  /**
   * @brief Gets the ROS 2 node.
   */
  std::shared_ptr<NodeT> get_node() const
  {
    return node_;
  }

private:
  /* ROS 2 context. */
  std::shared_ptr<rclcpp::Context> context_;

  /* ROS 2 executor. */
  std::shared_ptr<ExecutorT> executor_;

  /* ROS 2 node. */
  std::shared_ptr<NodeT> node_;

  /* Object state. */
  bool is_valid_ = false;

  /* Logger name. */
  std::string logger_name_;
};

#else

/**
 * Specialization for rclcpp_components::ComponentManager node.
 */
template<class ExecutorT>
class ROS2AppManager final
{
public:
  /**
   * @brief Constructor: initializes the ROS 2 component container app.
   *
   * @param argc Number of process command line arguments.
   * @param argv Process command line arguments.
   * @param module_name Application name.
   *
   * @throws RuntimeError if I/O buffering configuration fails.
   */
  ROS2AppManager(
    int argc,
    char ** argv,
    std::string && module_name = std::string("ros2_app_manager"))
  : logger_name_(module_name)
  {
    // Disable I/O buffering
    if (setvbuf(stdout, NULL, _IONBF, 0)) {
      RCLCPP_FATAL(
        rclcpp::get_logger(logger_name_),
        "Failed to disable I/O buffering");
      throw std::runtime_error("Failed to disable I/O buffering.");
    }

    // Initialize ROS 2 context
    context_ = rclcpp::contexts::get_global_default_context();
    rclcpp::InitOptions init_options = rclcpp::InitOptions();
    init_options.shutdown_on_signal = false;
    context_->init(argc, argv, init_options);

    // Initialize ROS 2 executor
    rclcpp::ExecutorOptions executor_opts = rclcpp::ExecutorOptions();
    executor_opts.context = context_;
    executor_ = std::make_shared<ExecutorT>(executor_opts);

    // Initialize ROS 2 node
    rclcpp::NodeOptions node_opts = rclcpp::NodeOptions();
    node_opts.context(context_);
    node_opts.start_parameter_services(false);
    node_opts.start_parameter_event_publisher(false);
    node_ = std::make_shared<rclcpp_components::ComponentManager>(
      executor_,
      "dua_component_manager",
      node_opts);
    executor_->add_node(node_);

    // Set object state
    is_valid_ = true;

    RCLCPP_WARN(
      rclcpp::get_logger(logger_name_),
      "Process started (%d)",
      getpid());
  }

  /**
   * @brief Destructor: shuts down the ROS 2 context if object is still valid.
   */
  ~ROS2AppManager()
  {
    if (is_valid_) {
      shutdown();
    }
  }

  /**
   * @brief Runs the app, does not return until stopped by a signal.
   *
   * @throws RuntimeError if the app manager is not valid.
   */
  void run()
  {
    if (!is_valid_) {
      throw std::runtime_error("ROS 2 app manager is not valid.");
    }
    RCLCPP_INFO(
      rclcpp::get_logger(logger_name_),
      "Starting executor...");
    executor_->spin();
  }

  /**
   * @brief Shuts down the app, deleting ROS 2 structures.
   */
  void shutdown()
  {
    RCLCPP_WARN(
      rclcpp::get_logger(logger_name_),
      "Shutting down...");
    executor_->remove_node(node_);
    node_.reset();
    context_->shutdown("ROS2AppManager::shutdown");
    executor_.reset();
    is_valid_ = false;
  }

  /**
   * @brief Gets the ROS 2 context.
   *
   * @return ROS 2 context.
   */
  std::shared_ptr<rclcpp::Context> get_context() const
  {
    return context_;
  }

  /**
   * @brief Gets the ROS 2 executor.
   */
  std::shared_ptr<ExecutorT> get_executor() const
  {
    return executor_;
  }

  /**
   * @brief Gets the ROS 2 node.
   */
  std::shared_ptr<rclcpp_components::ComponentManager> get_node() const
  {
    return node_;
  }

private:
  /* ROS 2 context. */
  std::shared_ptr<rclcpp::Context> context_;

  /* ROS 2 executor. */
  std::shared_ptr<ExecutorT> executor_;

  /* ROS 2 node. */
  std::shared_ptr<rclcpp_components::ComponentManager> node_;

  /* Object state. */
  bool is_valid_ = false;

  /* Logger name. */
  std::string logger_name_;
};

#endif // DUA_COMPONENT_CONTAINER

} // namespace dua_app_management

#endif // DUA_APP_MANAGEMENT__ROS2_APP_MANAGER_HPP_
