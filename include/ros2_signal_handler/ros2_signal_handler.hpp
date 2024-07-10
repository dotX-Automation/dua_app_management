/**
 * POSIX-compliant signal handler for ROS 2 processes.
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

#ifndef DUA_APP_MANAGEMENT__ROS2_SIGNAL_HANDLER_HPP_
#define DUA_APP_MANAGEMENT__ROS2_SIGNAL_HANDLER_HPP_

#include <thread>
#include <mutex>
#include <atomic>
#include <semaphore.h>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

namespace dua_app_management
{

/**
 * Global singleton signal handler object.
 */
class SignalHandler final
{
public:
  /**
   * @brief Returns the global singleton instance.
   *
   * @return Global signal handler object.
   */
  static SignalHandler & get_global_signal_handler()
  {
    static SignalHandler instance;
    return instance;
  }

  /**
   * @brief Initializes the signal handler data.
   *
   * @param context rclcpp::Context to operate on.
   * @param logger_name Name to be visualized in rclcpp log prints.
   * @param executor Executor to be terminated on signal.
   * @param deferred_handler Signal handler to be called alongside main thread.
   * @param system_handler System signal handler to be called while tracing.
   *
   * @throws InvalidArgument if context is null, or RuntimeError.
   */
  void init(
    std::shared_ptr<rclcpp::Context> context,
    std::string && logger_name = std::string("signal_handler"),
    rclcpp::Executor::SharedPtr executor = nullptr,
    std::function<void(int, std::string &)> && deferred_handler = nullptr,
    std::function<void(int, siginfo_t *, void *)> && system_handler = nullptr)
  {
    // Check if provided context is valid
    if (!context) {
      throw std::invalid_argument("SignalHandler::init: context cannot be null");
    }

    // Acquire installation lock
    std::scoped_lock<std::mutex> init_lock(install_lock_);

    // Check if signal handler is currently valid
    if (valid_) {
      throw std::runtime_error("SignalHandler::init: called on valid object");
    }

    // Initialize semaphores
    if (sem_init(&sig_prod_, 0, 0) || sem_init(&sig_cons_, 0, 1)) {
      std::perror("sem_init");
      throw std::runtime_error("SignalHandler::init: failed to initialize sempahores");
    }

    // Initialize pointers
    context_ = context;
    executor_ = executor;
    deferred_handler_ = deferred_handler;
    system_handler_ = system_handler;

    // Initialize rclcpp logger name
    logger_name_ = logger_name;

    // Initialize local signal value
    sig_ = 0;

    // Initialize validity flag (this also acts as a barrier, validating data above)
    valid_.store(true, std::memory_order_release);

    // Initialize handler thread
    handler_thread_ = std::thread(
      [this]() -> void {
        this->handler_thread_routine();
      });
  }

  /**
   * @brief Installs the signal handler for a given signal.
   *
   * @param int Signal code.
   *
   * @throws RuntimeError
   */
  void install(int sig)
  {
    // Prepare sigaction
    struct sigaction new_act;
    if (sigfillset(&(new_act.sa_mask))) {
      std::perror("sigfillset");
      throw std::runtime_error("SignalHandler::install: failed to set blocked signals mask");
    }
    new_act.sa_flags = SA_SIGINFO;
    new_act.sa_sigaction = &this->common_handler;

    // Acquire installation lock
    std::scoped_lock<std::mutex> install_lock(install_lock_);

    // Install handler
    if (sigaction(sig, &new_act, NULL)) {
      std::perror("sigaction");
      throw std::runtime_error(
              "SignalHandler::install: failed to install handler for signal (" +
              std::to_string(sig) + ")"
      );
    }

    // Add signal to installed list
    installed_.push_back(sig);

    RCLCPP_INFO(
      rclcpp::get_logger(logger_name_),
      "Installed handler for signal (%d)",
      sig
    );
  }

  /**
   * @brief Uninstalls the signal handler for a given signal.
   *
   * @param int Signal code.
   *
   * @throws RuntimeError
   */
  void uninstall(int sig)
  {
    uninstall_(sig);
  }

  /**
   * @brief Sets the "ignore" flag for a given signal.
   *
   * @param int Signal code.
   *
   * @throws RuntimeError
   */
  void ignore(int sig)
  {
    // Prepare sigaction
    struct sigaction new_act;
    if (sigemptyset(&(new_act.sa_mask))) {
      std::perror("sigfillset");
      throw std::runtime_error("SignalHandler::ignore: failed to set blocked signals mask");
    }
    new_act.sa_flags = 0;
    new_act.sa_handler = SIG_IGN;

    // Acquire installation lock
    std::scoped_lock<std::mutex> ignore_lock(install_lock_);

    // Install ignore handler
    if (sigaction(sig, &new_act, NULL)) {
      std::perror("sigaction");
      throw std::runtime_error(
              "SignalHandler::ignore: failed to install handler for signal (" +
              std::to_string(sig) + ")"
      );
    }

    // Add signal to installed list
    installed_.push_back(sig);

    RCLCPP_INFO(
      rclcpp::get_logger(logger_name_),
      "Ignoring signal (%d)",
      sig
    );
  }

  /**
   * @brief Reinitializes this signal handler.
   *
   * @throws RuntimeError
   */
  void fini()
  {
    // Acquire installation lock
    std::scoped_lock<std::mutex> fini_lock(install_lock_);

    // Check if signal handler is currently valid
    if (!valid_) {
      throw std::runtime_error("SignalHandler::fini: called on invalid object");
    }

    // Remove all installed handlers
    std::vector<int> installed_local = installed_;
    for (auto sig : installed_local) {
      uninstall_(sig, true);
    }

    // Reset validity flag
    valid_ = false;

    // Terminate and join handler thread
    sem_wait_(&sig_cons_);
    sig_ = -1;
    sem_post_(&sig_prod_);
    handler_thread_.join();

    // Erase pointers
    context_ = nullptr;
    executor_ = nullptr;
    deferred_handler_ = nullptr;
    system_handler_ = nullptr;

    // Erase logger name
    logger_name_ = std::string("");

    // Destroy semaphores
    if (sem_destroy(&sig_prod_) || sem_destroy(&sig_cons_)) {
      std::perror("sem_destroy");
      throw std::runtime_error("SignalHandler::fini: failed to destroy semaphores");
    }

    // Erase local signal value
    sig_ = 0;
  }

  /* Singleton: no copy constructor. */
  SignalHandler(SignalHandler &&) = delete;

  /* Singleton: no move assignment operator. */
  void operator=(SignalHandler &&) = delete;

private:
  /* Singleton: private constructor. */
  SignalHandler() {}

  /* Internal rclcpp logger name. */
  std::string logger_name_;

  /* Pointer to executor to stop on signal. */
  std::shared_ptr<rclcpp::Executor> executor_ = nullptr;

  /* Pointer to context to operate on. */
  std::shared_ptr<rclcpp::Context> context_ = nullptr;

  /* Deferred signal handler. */
  std::function<void(int, std::string &)> deferred_handler_ = nullptr;

  /* System signal handler. */
  inline static std::function<void(int, siginfo_t *, void *)> system_handler_ = nullptr;

  /* Internal handler status flag. */
  inline static std::atomic_bool valid_ = false;

  /* Signal handler object installation mutex. */
  inline static std::mutex install_lock_;

  /* Delivered signal, also used to notify termination. */
  inline static int sig_ = 0;

  /* Delivered signal PRODUCED semaphore. */
  inline static sem_t sig_prod_;

  /* Delivered signal CONSUMED semaphore. */
  inline static sem_t sig_cons_;

  /* List of traced signals. */
  std::vector<int> installed_{};

  /* Signal handler thread. */
  std::thread handler_thread_;

  /**
   * @brief Wraps the call to sem_post.
   *
   * @throws RuntimeError
   */
  static void sem_post_(sem_t * sem)
  {
    if (sem_post(sem)) {
      std::perror("sem_post");
      throw std::runtime_error("SignalHandler::sem_post_: failed to post semaphore");
    }
  }

  /**
   * @brief Wraps the call to sem_wait.
   *
   * @throws RuntimeError
   */
  static void sem_wait_(sem_t * sem)
  {
    if (sem_wait(sem)) {
      std::perror("sem_wait");
      throw std::runtime_error("SignalHandler::sem_wait_: failed to wait sempahore");
    }
  }

  /**
   * @brief Uninstalls the signal handler for a given signal.
   *
   * @param int Signal code.
   * @param has_lock Marks lock acquisition.
   *
   * @throws RuntimeError
   */
  void uninstall_(int sig, bool has_lock = false)
  {
    // Prepare sigaction
    struct sigaction new_act;
    if (sigemptyset(&(new_act.sa_mask))) {
      std::perror("sigfillset");
      throw std::runtime_error("SignalHandler::uninstall_: failed to set blocked signals mask");
    }
    new_act.sa_flags = 0;
    new_act.sa_handler = SIG_DFL;

    // Acquire installation lock
    if (!has_lock) {
      install_lock_.lock();
    }

    // Check if signal handler has been installed before
    if (std::find(installed_.begin(), installed_.end(), sig) == installed_.end()) {
      throw std::runtime_error("SignalHandler::uninstall_: signal handler wasn't installed");
    }

    // Install default handler
    if (sigaction(sig, &new_act, NULL)) {
      std::perror("sigaction");
      throw std::runtime_error(
              "SignalHandler::uninstall_: failed to install handler for signal (" +
              std::to_string(sig) + ")"
      );
    }

    // Remove signal from installed list
    installed_.erase(std::remove(installed_.begin(), installed_.end(), sig), installed_.end());

    RCLCPP_INFO(
      rclcpp::get_logger(logger_name_),
      "Uninstalled handler for signal (%d)",
      sig
    );

    // Release installation lock
    if (!has_lock) {
      install_lock_.unlock();
    }
  }

  /**
   * @brief Signal handler thread routine.
   */
  void handler_thread_routine()
  {
    int local_sig;
    RCLCPP_INFO(rclcpp::get_logger(logger_name_), "Signal handler started");

    while (true) {
      // Wait for external wakeup call
      sem_wait_(&sig_prod_);
      local_sig = sig_;
      sem_post_(&sig_cons_);

      // Check if termination is due
      if (local_sig == -1) {
        RCLCPP_INFO(rclcpp::get_logger(logger_name_), "Signal handler terminated");
        return;
      }

      RCLCPP_INFO(rclcpp::get_logger(logger_name_), "Got signal (%d)", local_sig);

      // Call custom handler, if present
      if (deferred_handler_) {
        deferred_handler_(local_sig, logger_name_);
      }

      // Stop executor, if given
      if (executor_) {
        executor_->cancel();
      }

      // Shut down context if requested
      if (context_->is_valid() && context_->get_init_options().shutdown_on_signal) {
        RCLCPP_WARN(rclcpp::get_logger(logger_name_), "Shutting down context");
        context_->shutdown("signal (" + std::to_string(sig_) + ")");
      }
    }
  }

  /**
   * @brief Common signal handler installed for all signals.
   *
   * @param sig Traced signal.
   * @param info Additional trace information.
   * @param ucontext Additional context information provided by the kernel.
   */
  static void common_handler(int sig, siginfo_t * info, void * ucontext)
  {
    // Acquire signal handler lock
    std::scoped_lock<std::mutex> common_handler_lock(install_lock_);

    // Call system handler, if present, bypassing the rest of the system
    if (system_handler_) {
      system_handler_(sig, info, ucontext);
    }

    // Check if signal handler is currently valid
    if (!valid_) {
      return;
    }

    // Notify handler thread
    sem_wait_(&sig_cons_);
    sig_ = sig;
    sem_post_(&sig_prod_);
  }
};

} // namespace dua_app_management

#endif // DUA_APP_MANAGEMENT__ROS2_SIGNAL_HANDLER_HPP_
