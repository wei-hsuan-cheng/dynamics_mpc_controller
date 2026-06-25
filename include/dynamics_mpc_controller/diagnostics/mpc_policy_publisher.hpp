#ifndef DYNAMICS_MPC_CONTROLLER__DIAGNOSTICS__MPC_POLICY_PUBLISHER_HPP_
#define DYNAMICS_MPC_CONTROLLER__DIAGNOSTICS__MPC_POLICY_PUBLISHER_HPP_

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MrtObserver.h>
#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>
#include <ocs2_oc/oc_data/PerformanceIndex.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/publisher.hpp>

namespace dynamics_mpc_controller::diagnostics
{

class MpcPolicyPublisher final : public ocs2::MrtObserver
{
public:
  using Message = ocs2_msgs::msg::MpcFlattenedController;
  using Publisher = rclcpp::Publisher<Message>;

  MpcPolicyPublisher(
    Publisher::SharedPtr publisher,
    const ocs2::MPC_BASE& mpc,
    rclcpp::Logger logger,
    double publishRate);
  ~MpcPolicyPublisher() override;

  void modifyBufferedSolution(
    const ocs2::CommandData& command,
    ocs2::PrimalSolution& primalSolution) override;

private:
  using Clock = std::chrono::steady_clock;

  Message createMessage(
    const ocs2::CommandData& command,
    const ocs2::PrimalSolution& primalSolution,
    const ocs2::PerformanceIndex& performance) const;
  void publisherWorker();

  struct PolicyBuffer
  {
    ocs2::CommandData command;
    ocs2::PrimalSolution primal_solution;
    ocs2::PerformanceIndex performance;
  };

  Publisher::SharedPtr publisher_;
  const ocs2::MPC_BASE& mpc_;
  rclcpp::Logger logger_;
  Clock::duration publish_period_;
  Clock::time_point last_publish_time_{};
  bool first_publish_{true};

  std::mutex buffer_mutex_;
  std::condition_variable buffer_condition_;
  std::unique_ptr<PolicyBuffer> policy_buffer_;
  bool stop_worker_{false};
  std::thread publisher_thread_;
};

}  // namespace dynamics_mpc_controller::diagnostics

#endif  // DYNAMICS_MPC_CONTROLLER__DIAGNOSTICS__MPC_POLICY_PUBLISHER_HPP_
