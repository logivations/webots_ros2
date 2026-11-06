// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// The driver as an rclcpp component: the extern controller of one Webots robot inside a component container, so
// that the robot's sensor data reaches the nodes composed next to it without leaving the process (no DDS
// serialization over loopback for point clouds and images).
//
// libController connects one robot per process, so a container holds at most one DriverComponent. The container
// spins only the component's own small node (its parameters). The WebotsNode is created on a thread of its own once
// Webots accepted the controller (wb_robot_init blocks until then) and is stepped and spun there exactly like in
// Driver.cpp: Webots waits for every synchronized controller in each step, so the step loop must not share an
// executor with the container's other nodes.
//
// Parameters of the component node:
//   controller_url (string): WEBOTS_CONTROLLER_URL of the robot, e.g. ipc://1234/my_robot (empty: from the
//                            environment of the container)
//   arguments (string[]): ROS arguments of the WebotsNode, as the driver executable gets them from
//                         WebotsController ("--ros-args", "--params-file", ..., "-r", ...); the controller_manager of
//                         webots_ros2_control passes them on to its controllers
//   nice (int): nice value of the step thread (0: unchanged); a negative value needs CAP_SYS_NICE
//   use_intra_process_comms (bool): intra-process publishing of the WebotsNode and its plugins

#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <webots/robot.h>
#include <webots/vehicle/driver.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <webots_ros2_driver/PythonPlugin.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace webots_ros2_driver {
  class DriverComponent {
  public:
    explicit DriverComponent(const rclcpp::NodeOptions &options) :
      mState(std::make_shared<State>()) {
      mState->node = std::make_shared<rclcpp::Node>("webots_driver", options);
      rclcpp::Node &node = *mState->node;
      mState->controllerUrl = node.declare_parameter<std::string>("controller_url", "");
      mState->arguments = node.declare_parameter<std::vector<std::string>>("arguments", std::vector<std::string>());
      mState->nice = node.declare_parameter<int>("nice", 0);
      mState->intraProcess = node.declare_parameter<bool>("use_intra_process_comms", false);
      mDone = mState->done.get_future();
      // the thread owns the state: it may outlive the component (see the destructor)
      mThread = std::thread(&DriverComponent::run, mState);
    }

    ~DriverComponent() {
      mState->stop = true;
      // wb_robot_init and wb_robot_step block while Webots does not step (paused, or not started yet): never hang
      // the unloading or the shutdown of the container on them
      if (mDone.wait_for(std::chrono::seconds(2)) == std::future_status::ready)
        mThread.join();
      else {
        RCLCPP_WARN(mState->node->get_logger(), "Webots does not step, leaving the controller thread behind.");
        mThread.detach();
      }
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
      return mState->node->get_node_base_interface();
    }

  private:
    struct State {
      rclcpp::Node::SharedPtr node;
      std::string controllerUrl;
      std::vector<std::string> arguments;
      int nice = 0;
      bool intraProcess = false;
      std::atomic<bool> stop{false};
      std::promise<void> done;
    };

    static void run(std::shared_ptr<State> state) {
      runSteps(*state);
      state->done.set_value();
    }

    static void runSteps(State &state) {
      const rclcpp::Logger logger = state.node->get_logger();
      const rclcpp::Context::SharedPtr context = state.node->get_node_base_interface()->get_context();

      if (state.nice != 0 && setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), state.nice) != 0)
        RCLCPP_WARN(logger, "Cannot set nice %d on the controller thread: %s", state.nice, std::strerror(errno));

      if (!state.controllerUrl.empty())
        setenv("WEBOTS_CONTROLLER_URL", state.controllerUrl.c_str(), 1);
      RCLCPP_INFO(logger, "Waiting for Webots to accept the controller (%s).",
                  state.controllerUrl.empty() ? "WEBOTS_CONTROLLER_URL" : state.controllerUrl.c_str());
      // as Driver.cpp: a robot that can be a driver (vehicle library), else a simple Supervisor
      if (wbu_driver_initialization_is_possible())
        wbu_driver_init();
      else
        wb_robot_init();
      if (state.stop)
        return;

      std::string robotName(wb_robot_get_name());
      for (char notAllowedChar : " -.)(")
        std::replace(robotName.begin(), robotName.end(), notAllowedChar, '_');

      // the arguments address this node only, not the container's global arguments
      rclcpp::NodeOptions nodeOptions;
      nodeOptions.context(context);
      nodeOptions.arguments(state.arguments);
      nodeOptions.use_global_arguments(false);
      nodeOptions.use_intra_process_comms(state.intraProcess);
      std::shared_ptr<WebotsNode> webotsNode = std::make_shared<WebotsNode>(robotName, nodeOptions);
      webotsNode->init();
      RCLCPP_INFO(webotsNode->get_logger(), "Controller successfully connected to robot in Webots simulation.");

      rclcpp::ExecutorOptions executorOptions;
      executorOptions.context = context;
      rclcpp::executors::SingleThreadedExecutor executor(executorOptions);
      executor.add_node(webotsNode);
      while (!state.stop && rclcpp::ok(context)) {
        if (webotsNode->step() == -1)
          break;
        const auto spinStart = std::chrono::steady_clock::now();
        executor.spin_some();
        webotsNode->addSpinTime(
          std::chrono::duration<double>(std::chrono::steady_clock::now() - spinStart).count());
      }

      bool isPython = false;
      for (std::shared_ptr<PluginInterface> plugin : webotsNode->mPlugins) {
        std::shared_ptr<PythonPlugin> pythonPlugin = std::dynamic_pointer_cast<PythonPlugin>(plugin);
        if (pythonPlugin) {
          pythonPlugin->stop();
          isPython = true;
          break;
        }
      }
      if (!isPython)
        wb_robot_cleanup();
      RCLCPP_INFO(webotsNode->get_logger(), "Controller successfully disconnected from robot in Webots simulation.");
    }

    std::shared_ptr<State> mState;
    std::future<void> mDone;
    std::thread mThread;
  };
}  // namespace webots_ros2_driver

RCLCPP_COMPONENTS_REGISTER_NODE(webots_ros2_driver::DriverComponent)
