#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "irobot_create_msgs/action/drive_distance.hpp"
#include "irobot_create_msgs/action/rotate_angle.hpp"
#include "irobot_create_msgs/srv/e_stop.hpp"
#include "irobot_create_msgs/action/dock.hpp"
#include "irobot_create_msgs/action/undock.hpp"

#include "custom_msg/action/actuator_move.hpp"
#include "custom_msg/action/actuator_dock.hpp"
#include "custom_msg/srv/stop.hpp"


class Actuator : public rclcpp::Node {
public:
    Actuator() : Node("actuator")
    {
        using namespace std::placeholders;

        driveDistanceClient = rclcpp_action::create_client<irobot_create_msgs::action::DriveDistance>(
            this, "drive_distance");

        rotateAngleClient = rclcpp_action::create_client<irobot_create_msgs::action::RotateAngle>(
            this, "rotate_angle");

        eStopClient = this->create_client<irobot_create_msgs::srv::EStop>("e_stop");

        // Dock client
        dockClient = rclcpp_action::create_client<irobot_create_msgs::action::Dock>(
            this,
            "dock"
        );

        // Undock client
        undockClient = rclcpp_action::create_client<irobot_create_msgs::action::Undock>(
            this,
            "undock"
        );

        // Servers
        movementServer = rclcpp_action::create_server<custom_msg::action::ActuatorMove>(
            this,
            "actuator_movement",
            std::bind(&Actuator::handle_movement_goal, this, _1, _2),
            std::bind(&Actuator::handle_movement_cancel, this, _1),
            std::bind(&Actuator::handle_movement_accepted, this, _1)
        );

        dockServer = rclcpp_action::create_server<custom_msg::action::ActuatorDock>(
            this,
            "actuator_dock",
            std::bind(&Actuator::handle_dock_goal, this, _1, _2),
            std::bind(&Actuator::handle_dock_cancel, this, _1),
            std::bind(&Actuator::handle_dock_accepted, this, _1)
        );

        stopServer = this->create_service<custom_msg::srv::Stop>(
            "actuator_power",
            std::bind(&Actuator::handle_stop_request, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    // --------------------------
    // Client & Server handles
    // --------------------------
    rclcpp_action::Client<irobot_create_msgs::action::DriveDistance>::SharedPtr driveDistanceClient;
    rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SharedPtr rotateAngleClient;
    rclcpp::Client<irobot_create_msgs::srv::EStop>::SharedPtr eStopClient;
    rclcpp_action::Client<irobot_create_msgs::action::Dock>::SharedPtr dockClient;
    rclcpp_action::Client<irobot_create_msgs::action::Undock>::SharedPtr undockClient;

    rclcpp_action::Server<custom_msg::action::ActuatorMove>::SharedPtr movementServer;
    rclcpp_action::Server<custom_msg::action::ActuatorDock>::SharedPtr dockServer;
    rclcpp::Service<custom_msg::srv::Stop>::SharedPtr stopServer;


    // --------------------------
    // Dock Action Server Callbacks
    // --------------------------
    rclcpp_action::GoalResponse handle_dock_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const custom_msg::action::ActuatorDock::Goal> goal)
    {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_dock_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorDock>>)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_dock_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorDock>> goal_handle)
    {
        std::thread(&Actuator::execute_dock, this, goal_handle).detach();
    }

    // ====================
    //     DOCK CALLBACK
    // ====================
    void execute_dock(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorDock>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Dock action started");

        const auto goal = goal_handle->get_goal();

        // Prepara result
        auto result   = std::make_shared<custom_msg::action::ActuatorDock::Result>();

        // ============================
        //        DOCK
        // ============================
        if (goal->type == "DOCK")
        {
            send_dock_goal(goal_handle);
            return;
        }

        // ============================
        //         UNDOCK
        // ============================
        if (goal->type == "UNDOCK")
        {
            send_undock_goal(goal_handle);
            return;
        }

        // ============================
        //  Tipo sconosciuto → errore
        // ============================
        result->status = false;
        RCLCPP_ERROR(this->get_logger(), "Unknown type: '%s'", goal->type.c_str());
    }


    // ====================
    //     Dock goal
    // ====================
    void send_dock_goal(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorDock>> actuator_goal)
    {
        using namespace std::placeholders;
        using DockClientGoalHandle = rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::Dock>;
        using DockClientFuture = std::shared_future<std::shared_ptr<DockClientGoalHandle>>;

        if (!dockClient->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Dock action server not available");

            auto result = std::make_shared<custom_msg::action::ActuatorDock::Result>();
            result->status = false;
            actuator_goal->abort(result);
            return;
        }

        irobot_create_msgs::action::Dock::Goal goal_msg;

        auto options = rclcpp_action::Client<irobot_create_msgs::action::Dock>::SendGoalOptions();

        // RESPONSE
        options.goal_response_callback =
            [this](DockClientFuture handle) {
                auto goal_handle = handle.get();
                if (!goal_handle)
                    RCLCPP_ERROR(this->get_logger(), "[IROBOT DOCK] Goal rejected");
                else
                    RCLCPP_INFO(this->get_logger(), "[IROBOT DOCK] Goal accepted");
            };

        // RESULT
        options.result_callback =
            [this, actuator_goal](const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::Dock>::WrappedResult & r)
            {
                auto result = std::make_shared<custom_msg::action::ActuatorDock::Result>();

                switch (r.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        result->status = true;
                        actuator_goal->succeed(result);
                        RCLCPP_INFO(this->get_logger(), "[IROBOT DOCK] SUCCESS");
                        return;

                    case rclcpp_action::ResultCode::ABORTED:
                        result->status = false;
                        actuator_goal->abort(result);
                        RCLCPP_ERROR(this->get_logger(), "[IROBOT DOCK] ABORTED");
                        return;

                    case rclcpp_action::ResultCode::CANCELED:
                        result->status = false;
                        actuator_goal->canceled(result);
                        RCLCPP_ERROR(this->get_logger(), "[IROBOT DOCK] CANCELED");
                        return;

                    default:
                        result->status = false;
                        actuator_goal->abort(result);
                        RCLCPP_ERROR(this->get_logger(), "[IROBOT DOCK] UNKNOWN RESULT");
                        return;
                }
            };

        dockClient->async_send_goal(goal_msg, options);
    }


    // ====================
    //     Undock goal
    // ====================
    void send_undock_goal(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorDock>> actuator_goal)
    {
        using namespace std::placeholders;
        using UndockClientGoalHandle = rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::Undock>;
        using UndockClientFuture = std::shared_future<std::shared_ptr<UndockClientGoalHandle>>;

        if (!undockClient->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Undock action server not available");

            auto result = std::make_shared<custom_msg::action::ActuatorDock::Result>();
            result->status = false;
            actuator_goal->abort(result);
            return;
        }

        irobot_create_msgs::action::Undock::Goal goal_msg;

        auto options = rclcpp_action::Client<irobot_create_msgs::action::Undock>::SendGoalOptions();

        // RESPONSE
        options.goal_response_callback =
            [this](UndockClientFuture handle) {
                auto goal_handle = handle.get();
                if (!goal_handle)
                    RCLCPP_ERROR(this->get_logger(), "[IROBOT UNDOCK] Goal rejected");
                else
                    RCLCPP_INFO(this->get_logger(), "[IROBOT UNDOCK] Goal accepted");
            };

        // RESULT
        options.result_callback =
            [this, actuator_goal](const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::Undock>::WrappedResult &r)
            {
                auto result = std::make_shared<custom_msg::action::ActuatorDock::Result>();

                switch (r.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        result->status = true;
                        actuator_goal->succeed(result);
                        RCLCPP_INFO(this->get_logger(), "[IROBOT UNDOCK] SUCCESS");
                        return;

                    case rclcpp_action::ResultCode::ABORTED:
                        result->status = false;
                        actuator_goal->abort(result);
                        RCLCPP_ERROR(this->get_logger(), "[IROBOT UNDOCK] ABORTED");
                        return;

                    case rclcpp_action::ResultCode::CANCELED:
                        result->status = false;
                        actuator_goal->canceled(result);
                        RCLCPP_ERROR(this->get_logger(), "[IROBOT UNDOCK] CANCELED");
                        return;

                    default:
                        result->status = false;
                        actuator_goal->abort(result);
                        RCLCPP_ERROR(this->get_logger(), "[IROBOT UNDOCK] UNKNOWN RESULT");
                        return;
                }
            };

        undockClient->async_send_goal(goal_msg, options);
    }


    // ======================
    //      Handle stop
    // ======================
    void handle_stop_request(
        const std::shared_ptr<custom_msg::srv::Stop::Request> request,
        std::shared_ptr<custom_msg::srv::Stop::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Stop request received: %s", request->stop ? "true" : "false");

        // Send stop request
        send_request(request->stop);

        // Risposta al client
        response->status = true;
    }

    //EStop request
    void send_request(bool stop) {
        // Waiting client
        while (!eStopClient->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting e_stop service...");
        }

        // Creating request
        auto request = std::make_shared<irobot_create_msgs::srv::EStop::Request>();
        request->e_stop_on = stop;

        // Sending request
        eStopClient->async_send_request(request,
                                    [this](rclcpp::Client<irobot_create_msgs::srv::EStop>::SharedFuture future_response)
            {
                auto response = future_response.get();
                RCLCPP_INFO(this->get_logger(), "Response: success=%s", response->success ? "true" : "false");
            }
        );
    };


    // Runtime state
    std::atomic<float> last_drive_remaining = std::numeric_limits<float>::infinity();
    std::atomic<float> last_angle_remaining = std::numeric_limits<float>::infinity();
    std::atomic<int>  drive_status = -1;
    std::atomic<int>  angle_status = -1;

    // --------------------------
    // Movement Action Server Callbacks
    // --------------------------
    rclcpp_action::GoalResponse handle_movement_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const custom_msg::action::ActuatorMove::Goal> goal)
    {
        if (goal->type != "DISTANCE" && goal->type != "ANGLE") {
            RCLCPP_ERROR(get_logger(), "Invalid goal type: %s", goal->type.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_movement_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorMove>>)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_movement_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorMove>> goal_handle)
    {
        std::thread(&Actuator::execute_movement, this, goal_handle).detach();
    }

    // --------------------------
    // EXECUTION LOGIC
    // --------------------------
    void execute_movement(const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorMove>> goal_handle)
    {
        const auto goal = goal_handle->get_goal();

        auto feedback = std::make_shared<custom_msg::action::ActuatorMove::Feedback>();
        auto result   = std::make_shared<custom_msg::action::ActuatorMove::Result>();

        reset_state();
        publish_initial_feedback(goal_handle, feedback);

        // ============================
        //        DISTANCE MODE
        // ============================
        if (goal->type == "DISTANCE")
        {
            if (!create3_begin_drive(goal->distance, goal->max_speed, result, goal_handle))
                return;

            monitor_progress_distance(goal_handle, goal, feedback, result);
            return;
        }

        // ============================
        //         ANGLE MODE
        // ============================
        if (goal->type == "ANGLE")
        {
            if (!create3_begin_angle(goal->distance, goal->max_speed, result, goal_handle))
                return;

            monitor_progress_angle(goal_handle, goal, feedback, result);
            return;
        }
    }

    // --------------------------
    // TOTEM: Reset State
    // --------------------------
    void reset_state()
    {
        drive_status = -1;
        angle_status = -1;
        last_drive_remaining = std::numeric_limits<float>::infinity();
        last_angle_remaining = std::numeric_limits<float>::infinity();
    }

    // --------------------------
    // Ensure feedback starts at 0%
    // --------------------------
    void publish_initial_feedback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorMove>> &goal_handle,
        std::shared_ptr<custom_msg::action::ActuatorMove::Feedback> &feedback)
    {
        feedback->percentage = 0.0f;
        goal_handle->publish_feedback(feedback);
    }

    // ---------------------------------------------
    // SEND DRIVE DISTANCE GOAL
    // ---------------------------------------------
    bool create3_begin_drive(float distance, float speed,
                     std::shared_ptr<custom_msg::action::ActuatorMove::Result> &result,
                     const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorMove>> &goal_handle)
    {
        if (distance == 0.0f) {
            result->status = true;
            goal_handle->succeed(result);
            return false;
        }

        if (!driveDistanceClient->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "DriveDistance server unreachable");
            result->status = false;
            goal_handle->abort(result);
            return false;
        }

        irobot_create_msgs::action::DriveDistance::Goal goal_msg;
        goal_msg.distance = distance;
        goal_msg.max_translation_speed = speed;

        auto options = rclcpp_action::Client<irobot_create_msgs::action::DriveDistance>::SendGoalOptions();

        options.feedback_callback =
            [this](auto, const std::shared_ptr<const irobot_create_msgs::action::DriveDistance::Feedback> fb)
        {
            last_drive_remaining = fb->remaining_travel_distance;
        };

        options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::DriveDistance>::WrappedResult & res)
        {
            drive_status = static_cast<int>(res.code);
        };

        driveDistanceClient->async_send_goal(goal_msg, options);
        return true;
    }

    // ---------------------------------------------
    // SEND ROTATE ANGLE GOAL
    // ---------------------------------------------
    bool create3_begin_angle(float angle, float speed,
        std::shared_ptr<custom_msg::action::ActuatorMove::Result> &result,
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorMove>> &goal_handle)
    {
        if (angle == 0.0f) {
            result->status = true;
            goal_handle->succeed(result);
            return false;
        }

        if (!rotateAngleClient->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "RotateAngle server unreachable");
            result->status = false;
            goal_handle->abort(result);
            return false;
        }

        irobot_create_msgs::action::RotateAngle::Goal goal_msg;
        //float increment = 0.10347f * static_cast<int>(angle / 1.5);
        goal_msg.angle = angle;
        goal_msg.max_rotation_speed = speed;

        auto options = rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SendGoalOptions();

        options.feedback_callback =
            [this](auto, const std::shared_ptr<const irobot_create_msgs::action::RotateAngle::Feedback> fb)
        {
            last_angle_remaining = fb->remaining_angle_travel;
        };

        options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::RotateAngle>::WrappedResult & res)
        {
            angle_status = static_cast<int>(res.code);
        };

        rotateAngleClient->async_send_goal(goal_msg, options);
        return true;
    }

    // ---------------------------------------------
    // DRIVE PROGRESS MONITOR
    // ---------------------------------------------
    void monitor_progress_distance(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorMove>> &goal_handle,
        const std::shared_ptr<const custom_msg::action::ActuatorMove::Goal> &goal,
        std::shared_ptr<custom_msg::action::ActuatorMove::Feedback> &feedback,
        std::shared_ptr<custom_msg::action::ActuatorMove::Result> &result)
    {
        rclcpp::Rate rate(20);
        auto start = this->get_clock()->now();

        while (rclcpp::ok())
        {
            // Check if goal was canceled
            if (goal_handle->is_canceling()) {
                result->status = false;
                goal_handle->canceled(result);
                return;
            }

            // Publish feedback if we have a valid value
            if (!std::isinf(last_drive_remaining)) {
                const float req = std::abs(goal->distance);
                const float rem = std::abs(last_drive_remaining.load());
                float perc = (req <= 1e-6f) ? 100.0f : (1.0f - (rem / req)) * 100.0f;
                feedback->percentage = std::clamp(perc, 0.0f, 100.0f);
                goal_handle->publish_feedback(feedback);
            }

            // Check if goal is done
            if (drive_status != -1)
                break;

            // Timeout check independent from feedback
            if ((this->get_clock()->now() - start).seconds() > 20.0) {
                RCLCPP_ERROR(get_logger(), "Drive distance TIMEOUT");
                result->status = false;
                goal_handle->abort(result);
                return;
            }

            rate.sleep();
        }

        // Publish final result
        int status = drive_status.load();
        if (status == static_cast<int>(rclcpp_action::ResultCode::SUCCEEDED)) {
            result->status = true;
            goal_handle->succeed(result);
        } else if (status == static_cast<int>(rclcpp_action::ResultCode::CANCELED)) {
            result->status = false;
            goal_handle->canceled(result);
        } else {
            result->status = false;
            goal_handle->abort(result);
        }
    }

    // ---------------------------------------------
    // ANGLE PROGRESS MONITOR
    // ---------------------------------------------
    void monitor_progress_angle(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorMove>> &goal_handle,
        const std::shared_ptr<const custom_msg::action::ActuatorMove::Goal> &goal,
        std::shared_ptr<custom_msg::action::ActuatorMove::Feedback> &feedback,
        std::shared_ptr<custom_msg::action::ActuatorMove::Result> &result)
    {
        rclcpp::Rate rate(20);
        auto start = this->get_clock()->now();

        while (rclcpp::ok())
        {
            // Check if goal was canceled
            if (goal_handle->is_canceling()) {
                result->status = false;
                goal_handle->canceled(result);
                return;
            }

            // Publish feedback if we have a valid value
            if (!std::isinf(last_angle_remaining)) {
                const float req = std::abs(goal->distance);
                const float rem = std::abs(last_angle_remaining.load());
                float perc = (req <= 1e-6f) ? 100.0f : (1.0f - (rem / req)) * 100.0f;
                feedback->percentage = std::clamp(perc, 0.0f, 100.0f);
                goal_handle->publish_feedback(feedback);
            }

            // Check if goal is done
            if (angle_status != -1)
                break;

            // Timeout check independent from feedback
            if ((this->get_clock()->now() - start).seconds() > 20.0) {
                RCLCPP_ERROR(get_logger(), "Drive angle TIMEOUT");
                result->status = false;
                goal_handle->abort(result);
                return;
            }

            rate.sleep();
        }

        // Publish final result
        int status = angle_status.load();
        if (status == static_cast<int>(rclcpp_action::ResultCode::SUCCEEDED)) {
            result->status = true;
            goal_handle->succeed(result);
        } else if (status == static_cast<int>(rclcpp_action::ResultCode::CANCELED)) {
            result->status = false;
            goal_handle->canceled(result);
        } else {
            result->status = false;
            goal_handle->abort(result);
        }
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Actuator>());
    rclcpp::shutdown();
    return 0;
}