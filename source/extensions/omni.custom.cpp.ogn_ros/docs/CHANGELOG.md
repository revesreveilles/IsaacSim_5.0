(changelog_omni_custom_cpp_ogn_ros)=

# Changelog

## [1.0.3] 2025-7-25
### Added
- Complete implementation of ROS2SubscribeRobotCmdNode for mm_msgs/RobotCmd messages
- Support for arm trajectory commands with joint position, velocity, and effort extraction
- Comprehensive output ports for chassis velocity, arm commands, gripper, and timestamps
- Safe state clearing when no messages are received or errors occur
- Proper reset() method for simulation lifecycle management

### Updated
- Renamed plugin directory from omni.custom.sub.cpp.omnigraph_node_ros to omni.custom.cpp.ogn_ros
- Simplified namespace structure to match extension naming
- Updated premake5.lua configuration for correct build integration

## [1.0.3] 2025-7-10
### Add
- Custom ROS 2 OGN that subscribe custom msgs.

## [1.0.3] - 2025-01-14
### Fixed
- Custom ROS 2 OmniGraph nodes are now correctly allocated and deallocated.

## [1.0.2] - 2024-01-02
### Updated
- Build against Kit 105.2

## [1.0.1] - 2023-04-27
### Updated
- Build against Kit 105.0

## [1.0.0] - 2022-08-15
### Added
- Initial implementation.
