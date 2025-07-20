// Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//
#include <ROS2SubscribeRobotCmdNodeDatabase.h>
#include <string>
#include <memory>

// Include the mm_msgs RobotCmd message
#include "mm_msgs/msg/robot_cmd.h"

// ROS includes for creating nodes, subscribers etc.
#include "rcl/rcl.h"
#include "rcl/subscription.h"
#include "rmw/qos_profiles.h"

// Helpers to explicit shorten names you know you will use
using omni::graph::core::Type;
using omni::graph::core::BaseDataType;

namespace omni {
namespace example {
namespace cpp {
namespace omnigraph_node_ros {

class ROS2SubscribeRobotCmdNode
{
public:
    static bool compute(ROS2SubscribeRobotCmdNodeDatabase& db)
    {
        auto& state = db.internalState<ROS2SubscribeRobotCmdNode>();

        // Check if inputs have changed and need to recreate subscriber
        std::string current_topic = db.inputs.topicName();
        std::string current_namespace = db.inputs.nodeNamespace();
        std::string current_qos_profile = db.inputs.qosProfile();
        int current_queue_size = db.inputs.queueSize();
        uint64_t current_context = db.inputs.context();

        bool need_recreate = false;
        if (state.sub_created) {
            if (current_topic != state.last_topic_name ||
                current_namespace != state.last_namespace ||
                current_qos_profile != state.last_qos_profile ||
                current_queue_size != state.last_queue_size ||
                current_context != state.last_context) {

                // Parameters changed, need to recreate subscriber
                need_recreate = true;
                printf("ROS2 parameters changed, recreating subscriber...\n");

                // Clean up existing subscriber
                rcl_ret_t rc = rcl_subscription_fini(&state.my_sub, &state.my_node);
                if (rc != RCL_RET_OK) {
                    printf("Failed to finalize subscriber: %d\n", rc);
                }

                rc = rcl_node_fini(&state.my_node);
                if (rc != RCL_RET_OK) {
                    printf("Failed to finalize node: %d\n", rc);
                }

                state.sub_created = false;
            }
        }

        if(!state.sub_created || need_recreate)
        {
            // Store current parameters
            state.last_topic_name = current_topic;
            state.last_namespace = current_namespace;
            state.last_qos_profile = current_qos_profile;
            state.last_queue_size = current_queue_size;
            state.last_context = current_context;

            if (!state.context_initialized) {
                state.context = rcl_get_zero_initialized_context();
                state.init_options = rcl_get_zero_initialized_init_options();
                state.allocator = rcl_get_default_allocator();
                rcl_ret_t rc;

                // create init_options
                rc = rcl_init_options_init(&state.init_options, state.allocator);
                if (rc != RCL_RET_OK)
                {
                    printf("Error rcl_init_options_init.\n");
                    return false;
                }

                // create context
                rc = rcl_init(0, nullptr, &state.init_options, &state.context);
                if (rc != RCL_RET_OK)
                {
                    printf("Error in rcl_init.\n");
                    return false;
                }

                state.context_initialized = true;
            }

            // create rcl_node with dynamic namespace
            state.my_node = rcl_get_zero_initialized_node();
            state.node_ops = rcl_node_get_default_options();

            std::string full_namespace = current_namespace.empty() ? "" : ("/" + current_namespace);
            std::string node_name = "robot_cmd_subscriber"; // Fixed node name

            rcl_ret_t rc = rcl_node_init(&state.my_node,
                                       node_name.c_str(),
                                       full_namespace.c_str(),
                                       &state.context,
                                       &state.node_ops);
            if (rc != RCL_RET_OK)
            {
                printf("Error in rcl_node_init\n");
                return false;
            }

            const rosidl_message_type_support_t * my_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(mm_msgs, msg, RobotCmd);

            state.sub_options = rcl_subscription_get_default_options();

            // Configure QoS based on qosProfile input parameter
            std::string qos_profile = current_qos_profile;

            if (qos_profile == "sensor_data") {
                state.sub_options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
                state.sub_options.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
            } else if (qos_profile == "parameter_events") {
                state.sub_options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
                state.sub_options.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
            } else if (qos_profile == "system_default") {
                state.sub_options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
                state.sub_options.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
            } else { // "default" or any other value
                state.sub_options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
                state.sub_options.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
            }

            state.sub_options.qos.depth = static_cast<size_t>(current_queue_size);

            // Initialize Subscriber with dynamic topic name
            rc = rcl_subscription_init(
                &state.my_sub,
                &state.my_node,
                my_type_support,
                current_topic.c_str(),
                &state.sub_options);
            if (RCL_RET_OK != rc)
            {
                printf("Error in rcl_subscription_init %s.\n", current_topic.c_str());
                return false;
            }

            // Node, subscriber was successfully created
            state.sub_created = true;
            state.message_received = false;

            printf("ROS2 Robot Command subscriber initialized:\n");
            printf("  Topic: %s\n", current_topic.c_str());
            printf("  Node: %s\n", node_name.c_str());
            printf("  Namespace: %s\n", full_namespace.c_str());
            printf("  Queue Size: %d\n", current_queue_size);
            printf("  QoS Profile: %s\n", qos_profile.c_str());
        }

        // Try to receive a message
        mm_msgs__msg__RobotCmd * ros_msg = mm_msgs__msg__RobotCmd__create();
        rmw_message_info_t message_info;

        rcl_ret_t rc = rcl_take(&state.my_sub, ros_msg, &message_info, nullptr);

        if (rc == RCL_RET_OK)
        {
            // Message received successfully
            state.message_received = true;

            // Debug: Print raw message values to verify data reception
            // printf("Raw ROS message values:\n");
            // printf("  yaw: %f\n", ros_msg->yaw);
            // printf("  chassis_cmd.linear: x=%f, y=%f, z=%f\n",
            //        ros_msg->chassis_cmd.linear.x, ros_msg->chassis_cmd.linear.y, ros_msg->chassis_cmd.linear.z);
            // printf("  chassis_cmd.angular: x=%f, y=%f, z=%f\n",
            //        ros_msg->chassis_cmd.angular.x, ros_msg->chassis_cmd.angular.y, ros_msg->chassis_cmd.angular.z);
            // printf("  gripper_cmd: %f\n", ros_msg->gripper_cmd);

            // Set outputs with received data - using direct assignment like official nodes
            db.outputs.yaw() = static_cast<double>(ros_msg->yaw);
            db.outputs.gripperCmd() = static_cast<double>(ros_msg->gripper_cmd);
            db.outputs.messageReceived() = true;
            db.outputs.timestamp() = static_cast<double>(ros_msg->header.stamp.sec) +
                                     static_cast<double>(ros_msg->header.stamp.nanosec) * 1e-9;

            // Set chassis linear velocity - get reference and assign directly
            auto& chassisLinearVel = db.outputs.chassisLinearVel();
            chassisLinearVel[0] = static_cast<double>(ros_msg->chassis_cmd.linear.x);
            chassisLinearVel[1] = static_cast<double>(ros_msg->chassis_cmd.linear.y);
            chassisLinearVel[2] = static_cast<double>(ros_msg->chassis_cmd.linear.z);

            // Set chassis angular velocity - get reference and assign directly
            auto& chassisAngularVel = db.outputs.chassisAngularVel();
            chassisAngularVel[0] = static_cast<double>(ros_msg->chassis_cmd.angular.x);
            chassisAngularVel[1] = static_cast<double>(ros_msg->chassis_cmd.angular.y);
            chassisAngularVel[2] = static_cast<double>(ros_msg->chassis_cmd.angular.z);

            // Trigger execution output to indicate new data is available
            db.outputs.execOut() = 1;

            // printf("Outputs set: yaw=%.2f, linear_vel=[%.2f,%.2f,%.2f], angular_vel=[%.2f,%.2f,%.2f], gripper=%.2f\n",
            //        static_cast<double>(ros_msg->yaw),
            //        chassisLinearVel[0], chassisLinearVel[1], chassisLinearVel[2],
            //        chassisAngularVel[0], chassisAngularVel[1], chassisAngularVel[2],
            //        static_cast<double>(ros_msg->gripper_cmd));
        }
        else if (rc == RCL_RET_SUBSCRIPTION_TAKE_FAILED)
        {
            // No message available
            db.outputs.messageReceived() = false;
        }
        else
        {
            // Error occurred
            printf("Error taking RobotCmd message: %d\n", rc);
            db.outputs.messageReceived() = false;
        }

        // Clean up the message
        mm_msgs__msg__RobotCmd__destroy(ros_msg);

        // Always return true to continue execution
        return true;
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = ROS2SubscribeRobotCmdNodeDatabase::sPerInstanceState<ROS2SubscribeRobotCmdNode>(nodeObj, instanceId);

        if (state.sub_created)
        {
            // Remove Subscriber
            rcl_ret_t rc = rcl_subscription_fini(&state.my_sub, &state.my_node);
            if (rc != RCL_RET_OK) {
                printf("Failed to finalize subscriber: %d\n", rc);
            }

            // Remove Node
            rc = rcl_node_fini(&state.my_node);
            if (rc != RCL_RET_OK) {
                printf("Failed to finalize node: %d\n", rc);
            }

            state.sub_created = false;
        }

        if (state.context_initialized) {
            // Finalize init options
            rcl_ret_t rc = rcl_init_options_fini(&state.init_options);
            if (rc != RCL_RET_OK) {
                printf("Failed to finalize init options: %d\n", rc);
            }

            // Shutdown context
            rc = rcl_shutdown(&state.context);
            if (rc != RCL_RET_OK) {
                printf("Failed to shutdown context: %d\n", rc);
            }

            state.context_initialized = false;
            printf("ROS2 Robot Command subscriber cleaned up\n");
        }
    }

private:
    rcl_subscription_t my_sub;
    rcl_node_t my_node;
    rcl_context_t context;
    rcl_node_options_t node_ops;
    rcl_init_options_t init_options;
    rcl_allocator_t allocator;
    rcl_subscription_options_t sub_options;
    bool sub_created {false};
    bool message_received {false};
    bool context_initialized {false};

    // Store last used parameters to detect changes
    std::string last_topic_name;
    std::string last_namespace;
    std::string last_qos_profile;
    int last_queue_size {10};
    uint64_t last_context {0};
};

// This macro provides the information necessary to OmniGraph that lets it automatically register and deregister
// your node type definition.
REGISTER_OGN_NODE()

} // omnigraph_node_ros
} // cpp
} // omni
} // example
