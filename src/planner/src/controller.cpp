#include "rclcpp/rclcpp.hpp"
#include "irobot_create_msgs/msg/dock_status.hpp"
#include "custom_msg/action/actuator_dock.hpp"
#include "custom_msg/action/actuator_move.hpp"
#include "custom_msg/action/solve.hpp"
#include "custom_msg/msg/command.hpp"
#include "custom_msg/srv/stop.hpp"
#include "custom_msg/srv/pause.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


class Controller : public rclcpp::Node {
    public:
        Controller(): Node("planner") {
            //////////////////
            //  PARAMETERS
            //////////////////
            // Dichiarazione parametri con default
            this->declare_parameter<int>("global_heading", 0);
            this->declare_parameter<std::vector<int64_t>>("goal", {1, 1});
            this->declare_parameter<double>("cell_length", 1.0);
            this->declare_parameter<double>("rotation_speed", 0.5);
            this->declare_parameter<double>("movement_distance", 1.0);
            this->declare_parameter<double>("movement_speed", 1.0);
            this->declare_parameter<double>("angle", 0.5760);

            std::vector<int64_t> tmp_goal;

            this->get_parameter("global_heading", global_heading);
            this->get_parameter("goal", tmp_goal);
            goal[0] = static_cast<int>(tmp_goal[0]);
            goal[1] = static_cast<int>(tmp_goal[1]);
            this->get_parameter("cell_length", cell_length);
            this->get_parameter("rotation_speed", rotation_speed);
            this->get_parameter("movement_distance", movement_distance);
            this->get_parameter("movement_speed", movement_speed);
            this->get_parameter("angle", angle);

            // Dock subscription
            rclcpp::QoS dock_qos = rclcpp::QoS(1).best_effort().durability_volatile();
            dockSubscription = this->create_subscription<irobot_create_msgs::msg::DockStatus>(
                "dock_status",
                dock_qos,
                [this](irobot_create_msgs::msg::DockStatus::SharedPtr msg) {
                    this->dockSubCallback(msg);
                }
            );

            // Maze Solver Action Client
            mazeClient = rclcpp_action::create_client<custom_msg::action::Solve>(
                this,
                "maze_solve"
            );

            // Dock Actuator Client
            dockClient = rclcpp_action::create_client<custom_msg::action::ActuatorDock>(
                this,
                "actuator_dock"
            );

            // Movement Actuator Client
            movementClient = rclcpp_action::create_client<custom_msg::action::ActuatorMove>(
                this,
                "actuator_movement"
            );

            // Stop Actuator Client
            StopClient = this->create_client<custom_msg::srv::Stop>("actuator_power");

            // Stop Actuator Client
            PauseClient = this->create_client<custom_msg::srv::Pause>("actuator_pause");

            // Command subscription
            rclcpp::QoS command_qos = rclcpp::QoS(5).reliable().durability_volatile();
            commandSubscription = this->create_subscription<custom_msg::msg::Command>(
                "command",
                command_qos,
                [this](custom_msg::msg::Command::SharedPtr msg) {
                    this->commandSubCallback(msg);
                }
            );

            // Timer for decisional loop
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), std::bind(&Controller::main_loop, this)
            );
        }
    private:
        int global_heading;
        int goal[2];
        double cell_length;
        double rotation_speed;
        double movement_distance;
        double movement_speed;
        double angle;

        // Status variables
        bool is_docked = true;
        std::string command = "";
        int mode = 0;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        // Dock Subsciption
        rclcpp::Subscription<irobot_create_msgs::msg::DockStatus>::SharedPtr dockSubscription;
        void dockSubCallback(irobot_create_msgs::msg::DockStatus::SharedPtr msg) {
            is_docked = msg->is_docked;
        }

        // Maze Action client
        rclcpp_action::Client<custom_msg::action::Solve>::SharedPtr mazeClient;

        // Dock Actuator client
        rclcpp_action::Client<custom_msg::action::ActuatorDock>::SharedPtr dockClient;

        // Movement Actuator Client
        rclcpp_action::Client<custom_msg::action::ActuatorMove>::SharedPtr movementClient;

        // Stop Actuator Client
        rclcpp::Client<custom_msg::srv::Stop>::SharedPtr StopClient;

        // Pause Actuator Client
        rclcpp::Client<custom_msg::srv::Pause>::SharedPtr PauseClient;

        // Command Subsciption
        rclcpp::Subscription<custom_msg::msg::Command>::SharedPtr commandSubscription;
        void commandSubCallback(custom_msg::msg::Command::SharedPtr msg) {
            command = msg->command;
        }

        // Dock callback
        void send_actuator_dock_goal(const std::string &type)
        {
            using namespace std::placeholders;
            using DockGoalHandle = rclcpp_action::ClientGoalHandle<custom_msg::action::ActuatorDock>;
            using DockFuture = std::shared_future<std::shared_ptr<DockGoalHandle>>;

            // Controllo input
            if (type != "DOCK" && type != "UNDOCK") {
                RCLCPP_ERROR(this->get_logger(), "Invalid type sent to actuator_dock: %s", type.c_str());
                return;
            }

            // Attesa server
            if (!dockClient->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "ActuatorDock server not available");
                return;
            }

            // Creazione messaggio goal
            custom_msg::action::ActuatorDock::Goal goal_msg;
            goal_msg.type = type;

            // Opzioni callback
            rclcpp_action::Client<custom_msg::action::ActuatorDock>::SendGoalOptions options;

            // GOAL response
            options.goal_response_callback =
                [this, type](DockFuture handle)
                {
                    auto goal_handle = handle.get();
                    if (!goal_handle) {
                        RCLCPP_ERROR(this->get_logger(), "[%s] Goal rejected", type.c_str());
                    } else {
                        RCLCPP_INFO(this->get_logger(), "[%s] Goal accepted", type.c_str());
                    }
                };

            // FEEDBACK
            options.feedback_callback =
                [](auto, auto) {

                };

            // RISULTATO
            options.result_callback =
                [this, type](const rclcpp_action::ClientGoalHandle<custom_msg::action::ActuatorDock>::WrappedResult &result)
                {
                    switch (result.code) {

                        case rclcpp_action::ResultCode::SUCCEEDED:
                            if (result.result->status)
                                RCLCPP_INFO(this->get_logger(), "[%s] COMPLETED SUCCESSFULLY", type.c_str());
                            else
                                RCLCPP_WARN(this->get_logger(), "[%s] Completed but status=false", type.c_str());
                            return;

                        case rclcpp_action::ResultCode::ABORTED:
                            RCLCPP_ERROR(this->get_logger(), "[%s] ABORTED", type.c_str());
                            return;

                        case rclcpp_action::ResultCode::CANCELED:
                            RCLCPP_ERROR(this->get_logger(), "[%s] CANCELED", type.c_str());
                            return;

                        default:
                            RCLCPP_ERROR(this->get_logger(), "[%s] UNKNOWN result code", type.c_str());
                            return;
                    }
                };

            // Invio del goal
            dockClient->async_send_goal(goal_msg, options);
        }

        // Movement callback
        void send_actuator_movement_goal(const std::string &type, float distance, float max_vel)
        {
            using namespace std::placeholders;
            using MoveGoalHandle = rclcpp_action::ClientGoalHandle<custom_msg::action::ActuatorMove>;
            using MoveFuture = std::shared_future<std::shared_ptr<MoveGoalHandle>>;

            // Controllo input
            if (type != "DISTANCE" && type != "ANGLE") {
                RCLCPP_ERROR(this->get_logger(), "Invalid type sent to actuator_movement: %s", type.c_str());
                return;
            }

            // Attesa server
            if (!movementClient->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "ActuatorMovement server not available");
                return;
            }

            // Creazione messaggio goal
            custom_msg::action::ActuatorMove::Goal goal_msg;
            goal_msg.type = type;
            goal_msg.distance = distance;
            goal_msg.max_speed = max_vel;

            // Opzioni callback
            rclcpp_action::Client<custom_msg::action::ActuatorMove>::SendGoalOptions options;

            // GOAL response
            options.goal_response_callback =
                [this, type](MoveFuture handle)
                {
                    auto goal_handle = handle.get();
                    if (!goal_handle) {
                        RCLCPP_ERROR(this->get_logger(), "[%s] Goal rejected", type.c_str());
                    } else {
                        RCLCPP_INFO(this->get_logger(), "[%s] Goal accepted", type.c_str());
                    }
                };

            // FEEDBACK
            options.feedback_callback =
                [this, type](auto, const std::shared_ptr<const custom_msg::action::ActuatorMove::Feedback> feedback)
                {
                    // Aggiorna log o interfaccia con percentuale completamento
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 500,
                                        "[%s] Progress: %.2f%%", type.c_str(), feedback->percentage);
                };

            // RISULTATO
            options.result_callback =
                [this, type](const rclcpp_action::ClientGoalHandle<custom_msg::action::ActuatorMove>::WrappedResult &result)
                {
                    switch (result.code) {

                        case rclcpp_action::ResultCode::SUCCEEDED:
                            if (result.result->status)
                                RCLCPP_INFO(this->get_logger(), "[%s] COMPLETED SUCCESSFULLY", type.c_str());
                            else
                                RCLCPP_WARN(this->get_logger(), "[%s] Completed but status=false", type.c_str());
                            return;

                        case rclcpp_action::ResultCode::ABORTED:
                            RCLCPP_ERROR(this->get_logger(), "[%s] ABORTED", type.c_str());
                            return;

                        case rclcpp_action::ResultCode::CANCELED:
                            RCLCPP_ERROR(this->get_logger(), "[%s] CANCELED", type.c_str());
                            return;

                        default:
                            RCLCPP_ERROR(this->get_logger(), "[%s] UNKNOWN result code", type.c_str());
                            return;
                    }
                };

            // Invio del goal
            movementClient->async_send_goal(goal_msg, options);
        }

        // Maze solve callback
        void send_maze_solve_goal(const std::string &algorithm, int* end_position)
        {
            using namespace std::placeholders;
            using SolveGoalHandle = rclcpp_action::ClientGoalHandle<custom_msg::action::Solve>;
            using SolveFuture = std::shared_future<std::shared_ptr<SolveGoalHandle>>;

            // Controllo input
            if (algorithm != "PLEDGE" && algorithm != "TREMAUX") {
                RCLCPP_ERROR(this->get_logger(), "Invalid type sent to maze solver: %s", algorithm.c_str());
                return;
            }

            // Attesa server
            if (!mazeClient->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "ActuatorMovement server not available");
                return;
            }

            // Creazione messaggio goal
            custom_msg::action::Solve::Goal goal_msg;
            goal_msg.algorithm = algorithm;

            goal_msg.end_position.clear();
            goal_msg.end_position.push_back(end_position[0]);
            goal_msg.end_position.push_back(end_position[1]);

            // Opzioni callback
            rclcpp_action::Client<custom_msg::action::Solve>::SendGoalOptions options;

            // GOAL response
            options.goal_response_callback =
                [this, algorithm](SolveFuture handle)
                {
                    auto goal_handle = handle.get();
                    if (!goal_handle) {
                        RCLCPP_ERROR(this->get_logger(), "[%s] Goal rejected", algorithm.c_str());
                    } else {
                        RCLCPP_INFO(this->get_logger(), "[%s] Goal accepted", algorithm.c_str());
                    }
                };

            // FEEDBACK
            options.feedback_callback =
                [this, algorithm](auto, const std::shared_ptr<const custom_msg::action::Solve::Feedback> feedback)
                {
                    const auto &pos = feedback->current_position;

                    // Stampa posizione (ad esempio "x: 1, y: 2")
                    if (pos.size() >= 2) {
                        RCLCPP_INFO(this->get_logger(),
                                    "[%s] Current position: x=%d, y=%d",
                                    algorithm.c_str(),
                                    pos[0],
                                    pos[1]);
                    } else {
                        RCLCPP_WARN(this->get_logger(),
                                    "[%s] Current position vector too short!",
                                    algorithm.c_str());
                    }
                };

            // RISULTATO
            options.result_callback =
                [this, algorithm](const rclcpp_action::ClientGoalHandle<custom_msg::action::Solve>::WrappedResult &result)
                {
                    switch (result.code) {

                        case rclcpp_action::ResultCode::SUCCEEDED:
                            if (result.result->status)
                                RCLCPP_INFO(this->get_logger(), "[%s] COMPLETED SUCCESSFULLY", algorithm.c_str());
                            else
                                RCLCPP_WARN(this->get_logger(), "[%s] Completed but status=false", algorithm.c_str());
                            return;

                        case rclcpp_action::ResultCode::ABORTED:
                            RCLCPP_ERROR(this->get_logger(), "[%s] ABORTED", algorithm.c_str());
                            return;

                        case rclcpp_action::ResultCode::CANCELED:
                            RCLCPP_ERROR(this->get_logger(), "[%s] CANCELED", algorithm.c_str());
                            return;

                        default:
                            RCLCPP_ERROR(this->get_logger(), "[%s] UNKNOWN result code", algorithm.c_str());
                            return;
                    }
                };

            // Invio del goal
            mazeClient->async_send_goal(goal_msg, options);
        }

        // =======================
        // Send Stop / Start Command
        // =======================
        void send_actuator_stop(bool stop)
        {
            if (!StopClient->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_ERROR(this->get_logger(), "Stop service not available");
                return;
            }

            auto request = std::make_shared<custom_msg::srv::Stop::Request>();
            request->stop = stop;

            auto future_result = StopClient->async_send_request(request,
                [this, stop](rclcpp::Client<custom_msg::srv::Stop>::SharedFuture future) {
                    try {
                        auto response = future.get();
                        if (response->status) {
                            RCLCPP_INFO(this->get_logger(), "Actuator %s successfully", stop ? "stopped" : "started");
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Actuator %s failed", stop ? "stop" : "start");
                        }
                    } catch (const std::exception &e) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to call stop service: %s", e.what());
                    }
                }
            );
        }

        // =======================
        // Send Pause / Resume Command
        // =======================
        void send_actuator_pause(bool pause)
        {
            if (!PauseClient->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_ERROR(this->get_logger(), "Pause service not available");
                return;
            }

            auto request = std::make_shared<custom_msg::srv::Pause::Request>();
            request->pause = pause;

            auto future_result = PauseClient->async_send_request(request,
                [this, pause](rclcpp::Client<custom_msg::srv::Pause>::SharedFuture future) {
                    try {
                        auto response = future.get();
                        if (response->status) {
                            RCLCPP_INFO(this->get_logger(), "Actuator %s successfully", pause ? "paused" : "resumed");
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Actuator %s failed", pause ? "pause" : "resume");
                        }
                    } catch (const std::exception &e) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to call pause service: %s", e.what());
                    }
                }
            );
        }

        // Main Loop
        void main_loop() {
            if(command != "") {
                if(command == "DOCK") {
                    if(is_docked) {
                        RCLCPP_ERROR(this->get_logger(), "Already docked!");
                    } else {
                        send_actuator_dock_goal("DOCK");
                    }
                } else if(command == "UNDOCK") {
                    if(is_docked) {
                        send_actuator_dock_goal("UNDOCK");
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Already undocked!");
                    }
                } else if(command == "MODE A") {
                    this->mode = 0;
                } else if(command == "MODE B") {
                    this->mode = 1;
                } else if(command == "SOLVE") {
                    send_maze_solve_goal(mode == 0 ? "PLEDGE" : "TREMAUX", this->goal);
                } else if(command == "FORWARD") {
                    send_actuator_movement_goal("DISTANCE", this->movement_distance, this->movement_speed);
                } else if(command == "BACKWARD") {
                    send_actuator_movement_goal("DISTANCE", -this->movement_distance, this->movement_speed);
                } else if(command == "ROTATE LEFT") {
                    send_actuator_movement_goal("ANGLE", 1.5707963267948966, this->rotation_speed);
                } else if(command == "ROTATE RIGHT") {
                    send_actuator_movement_goal("ANGLE", -1.5707963267948966, this->rotation_speed);
                } else if(command == "START") {
                    send_actuator_stop(false);
                } else if(command == "STOP") {
                    send_actuator_stop(true);
                } else if(command == "PAUSE") {
                    send_actuator_pause(true);
                } else if(command == "RESUME") {
                    send_actuator_pause(false);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Command unrecognized!");
                }

                command = "";
            }
        }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}