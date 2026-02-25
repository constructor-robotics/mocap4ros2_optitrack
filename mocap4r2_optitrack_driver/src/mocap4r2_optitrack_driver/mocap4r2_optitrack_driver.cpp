// Copyright 2021 Institute for Robotics and Intelligent Machines,
//                Georgia Institute of Technology
// Copyright 2024 Intelligent Robotics Lab
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
//
// Author: Christian Llanes <christian.llanes@gatech.edu>
// Author: David Vargas Frutos <david.vargas@urjc.es>
// Author: Francisco Martín <fmrico@urjc.es>

#include <string>
#include <vector>
#include <memory>

#include "mocap4r2_msgs/msg/marker.hpp"
#include "mocap4r2_msgs/msg/markers.hpp"

#include "mocap4r2_optitrack_driver/mocap4r2_optitrack_driver.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace mocap4r2_optitrack_driver
{

using std::placeholders::_1;
using std::placeholders::_2;

OptitrackDriverNode::OptitrackDriverNode()
: ControlledLifecycleNode(
    "mocap4r2_optitrack_driver_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
  if (!has_parameter("connection_type")) {declare_parameter<std::string>("connection_type", "Unicast");}
  if (!has_parameter("server_address")) {declare_parameter<std::string>("server_address", "000.000.000.000");}
  if (!has_parameter("local_address")) {declare_parameter<std::string>("local_address", "000.000.000.000");}
  if (!has_parameter("multicast_address")) {declare_parameter<std::string>("multicast_address", "000.000.000.000");}
  if (!has_parameter("server_command_port")) {declare_parameter<uint16_t>("server_command_port", 1510);}
  if (!has_parameter("server_data_port")) {declare_parameter<uint16_t>("server_data_port", 1511);}

  if (!has_parameter("qos_history_policy")) {declare_parameter<std::string>("qos_history_policy", "keep_last");}
  if (!has_parameter("qos_reliability_policy")) {declare_parameter<std::string>("qos_reliability_policy", "reliable");}
  if (!has_parameter("qos_depth")) {declare_parameter<int>("qos_depth", 1000);}

  if (!has_parameter("publish_tf")) {declare_parameter<bool>("publish_tf", false);}
  if (!has_parameter("publish_y_up_tf")) {declare_parameter<bool>("publish_y_up_tf", false);}
  if (!has_parameter("rb_parent_frame_name")) {declare_parameter<std::string>("rb_parent_frame_name", "optitrack");}
  if (!has_parameter("y_up_frame_name")) {declare_parameter<std::string>("y_up_frame_name", "map");}

  client = new NatNetClient();
  client->SetFrameReceivedCallback(process_frame_callback, this);
}

OptitrackDriverNode::~OptitrackDriverNode()
{
}

void OptitrackDriverNode::set_settings_optitrack()
{
  if (connection_type_ == "Multicast") {
    client_params.connectionType = ConnectionType::ConnectionType_Multicast;
    client_params.multicastAddress = multicast_address_.c_str();
  } else if (connection_type_ == "Unicast") {
    client_params.connectionType = ConnectionType::ConnectionType_Unicast;
  } else {
    RCLCPP_FATAL(get_logger(), "Unknown connection type -- options are Multicast, Unicast");
    rclcpp::shutdown();
  }

  client_params.serverAddress = server_address_.c_str();
  client_params.localAddress = local_address_.c_str();
  client_params.serverCommandPort = server_command_port_;
  client_params.serverDataPort = server_data_port_;
}

bool OptitrackDriverNode::stop_optitrack()
{
  RCLCPP_INFO(get_logger(), "Disconnecting from optitrack DataStream SDK");

  return true;
}

void
OptitrackDriverNode::control_start(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
}

void
OptitrackDriverNode::control_stop(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
}

void NATNET_CALLCONV process_frame_callback(sFrameOfMocapData * data, void * pUserData)
{
  static_cast<OptitrackDriverNode *>(pUserData)->process_frame(data);
}

std::chrono::nanoseconds OptitrackDriverNode::get_optitrack_system_latency(sFrameOfMocapData * data)
{
  const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

  if (bSystemLatencyAvailable) {
    const double clientLatencySec =
      client->SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp);
    const double clientLatencyMillisec = clientLatencySec * 1000.0;
    const double transitLatencyMillisec =
      client->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;

    const double largeLatencyThreshold = 100.0;
    if (clientLatencyMillisec >= largeLatencyThreshold) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 500,
        "Optitrack system latency >%.0f ms: [Transmission: %.0fms, Total: %.0fms]",
        largeLatencyThreshold, transitLatencyMillisec, clientLatencyMillisec);
    }

    return round<std::chrono::nanoseconds>(std::chrono::duration<float>{clientLatencySec});
  } else {
    RCLCPP_WARN_ONCE(get_logger(), "Optitrack's system latency not available");
    return std::chrono::nanoseconds::zero();
  }
}

void
OptitrackDriverNode::process_frame(sFrameOfMocapData * data)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  frame_number_++;
  rclcpp::Duration frame_delay = rclcpp::Duration(get_optitrack_system_latency(data));

  std::map<int, std::vector<mocap4r2_msgs::msg::Marker>> marker2rb;

  // Markers
  if (mocap4r2_markers_pub_->get_subscription_count() > 0) {
    mocap4r2_msgs::msg::Markers msg;
    msg.header.stamp = now() - frame_delay;
    msg.header.frame_id = "map";
    msg.frame_number = frame_number_;

    for (int i = 0; i < data->nLabeledMarkers; i++) {
      bool Unlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
      bool ActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);
      sMarker & marker_data = data->LabeledMarkers[i];
      int modelID, markerID;
      NatNet_DecodeID(marker_data.ID, &modelID, &markerID);

      mocap4r2_msgs::msg::Marker marker;
      marker.id_type = mocap4r2_msgs::msg::Marker::USE_INDEX;
      marker.marker_index = i;
      marker.translation.x = marker_data.x;
      marker.translation.y = marker_data.y;
      marker.translation.z = marker_data.z;
      if (ActiveMarker || Unlabeled) {
        msg.markers.push_back(marker);
      } else {
        marker2rb[modelID].push_back(marker);
      }
    }
    mocap4r2_markers_pub_->publish(msg);
  }

  if (mocap4r2_rigid_body_pub_->get_subscription_count() > 0) {
    mocap4r2_msgs::msg::RigidBodies msg_rb;
    msg_rb.header.stamp = now() - frame_delay;
    msg_rb.header.frame_id = "map";
    msg_rb.frame_number = frame_number_;

    for (int i = 0; i < data->nRigidBodies; i++) {
      mocap4r2_msgs::msg::RigidBody rb;

      rb.rigid_body_name = std::to_string(data->RigidBodies[i].ID);
      rb.pose.position.x = data->RigidBodies[i].x;
      rb.pose.position.y = data->RigidBodies[i].y;
      rb.pose.position.z = data->RigidBodies[i].z;
      rb.pose.orientation.x = data->RigidBodies[i].qx;
      rb.pose.orientation.y = data->RigidBodies[i].qy;
      rb.pose.orientation.z = data->RigidBodies[i].qz;
      rb.pose.orientation.w = data->RigidBodies[i].qw;
      rb.markers = marker2rb[data->RigidBodies[i].ID];

      msg_rb.rigidbodies.push_back(rb);
    }

    mocap4r2_rigid_body_pub_->publish(msg_rb);
  }

  if (publish_tf_ && activate_tf_) {
    rclcpp::Time stamp = now() - frame_delay;
    publish_tf_data(data, stamp);
  }
}

void
OptitrackDriverNode::update_rigid_body_id_map()
{
  id_rigid_body_map_.clear();
  rigid_body_id_map_.clear();
  for (int i = 0; i < data_descriptions->nDataDescriptions; ++i) {
    if (data_descriptions->arrDataDescriptions[i].type == Descriptor_RigidBody) {
      auto * rb = data_descriptions->arrDataDescriptions[i].Data.RigidBodyDescription;
      id_rigid_body_map_[rb->ID] = rb->szName;
      rigid_body_id_map_[rb->szName] = rb->ID;
      RCLCPP_INFO_STREAM(get_logger(), "Mapped rigid body: ID=" << rb->ID << " name='" << rb->szName << "'");
    }
  }
  RCLCPP_INFO_STREAM(get_logger(), "Total rigid bodies mapped: " << id_rigid_body_map_.size());
}

void
OptitrackDriverNode::get_rigid_bodies_from_params()
{
  tf_rigid_bodies_to_publish_.clear();
  const auto result = this->get_node_parameters_interface()->list_parameters({"rigid_bodies"}, 0);
  RCLCPP_INFO_STREAM(get_logger(), "list_parameters prefixes (" << result.prefixes.size() << "):");
  for (const auto & p : result.prefixes) {
    RCLCPP_INFO_STREAM(get_logger(), "  prefix: '" << p << "'");
  }
  RCLCPP_INFO_STREAM(get_logger(), "list_parameters names (" << result.names.size() << "):");
  for (const auto & n : result.names) {
    RCLCPP_INFO_STREAM(get_logger(), "  name: '" << n << "'");
  }
  for (const auto & prefix : result.prefixes) {
    std::string temp_name;
    if (!get_parameter<std::string>(prefix + ".name", temp_name)) {
      RCLCPP_WARN_STREAM(get_logger(), "No 'name' sub-parameter in: " << prefix);
      continue;
    }
    if (rigid_body_id_map_.count(temp_name) > 0) {
      tf_rigid_bodies_to_publish_.insert(temp_name);
      RCLCPP_INFO_STREAM(get_logger(), "TF publishing enabled for rigid body: '" << temp_name << "'");
    } else {
      RCLCPP_WARN_STREAM(
        get_logger(), "Rigid body '" << temp_name << "' not found on NatNet server.");
    }
  }
  RCLCPP_INFO_STREAM(get_logger(), "Total rigid bodies to publish TF for: " << tf_rigid_bodies_to_publish_.size());
}

void
OptitrackDriverNode::publish_tf_data(sFrameOfMocapData * data, rclcpp::Time stamp)
{
  for (int i = 0; i < data->nRigidBodies; i++) {
    int id = data->RigidBodies[i].ID;
    if (id_rigid_body_map_.count(id) == 0) {continue;}
    const auto & name = id_rigid_body_map_[id];
    if (tf_rigid_bodies_to_publish_.count(name) == 0) {continue;}

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = rb_parent_frame_name_;
    t.child_frame_id = name;
    t.transform.translation.x = data->RigidBodies[i].x;
    t.transform.translation.y = data->RigidBodies[i].y;
    t.transform.translation.z = data->RigidBodies[i].z;
    t.transform.rotation.x = data->RigidBodies[i].qx;
    t.transform.rotation.y = data->RigidBodies[i].qy;
    t.transform.rotation.z = data->RigidBodies[i].qz;
    t.transform.rotation.w = data->RigidBodies[i].qw;
    tf_broadcaster_->sendTransform(t);
  }
}

void
OptitrackDriverNode::make_static_transform()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = y_up_frame_name_;
  t.child_frame_id = rb_parent_frame_name_;
  t.transform.rotation.x = 0.5;
  t.transform.rotation.y = 0.5;
  t.transform.rotation.z = 0.5;
  t.transform.rotation.w = 0.5;
  tf_static_broadcaster_->sendTransform(t);
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


// The next Callbacks are used to manage behavior in the different states of the lifecycle node.
CallbackReturnT
OptitrackDriverNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  initParameters();

  rclcpp::QoS qos_profile(qos_depth_);
  if (qos_history_policy_ == "keep_all") {
    qos_profile.keep_all();
  } else {
    qos_profile.keep_last(qos_depth_);
  }
  if (qos_reliability_policy_ == "best_effort") {
    qos_profile.best_effort();
  } else {
    qos_profile.reliable();
  }

  mocap4r2_markers_pub_ = create_publisher<mocap4r2_msgs::msg::Markers>(
    "markers", qos_profile);
  mocap4r2_rigid_body_pub_ = create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "rigid_bodies", qos_profile);

  connect_optitrack();

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  if (data_descriptions) {
    update_rigid_body_id_map();
  }
  if (publish_y_up_tf_) {
    make_static_transform();
  }
  if (publish_tf_) {
    get_rigid_bodies_from_params();
  }

  RCLCPP_INFO(get_logger(), "Configured!\n");

  return ControlledLifecycleNode::on_configure(state);
}

CallbackReturnT
OptitrackDriverNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  mocap4r2_markers_pub_->on_activate();
  mocap4r2_rigid_body_pub_->on_activate();
  activate_tf_ = true;
  RCLCPP_INFO(get_logger(), "Activated!\n");

  return ControlledLifecycleNode::on_activate(state);
}

CallbackReturnT
OptitrackDriverNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  mocap4r2_markers_pub_->on_deactivate();
  mocap4r2_rigid_body_pub_->on_deactivate();
  activate_tf_ = false;
  RCLCPP_INFO(get_logger(), "Deactivated!\n");

  return ControlledLifecycleNode::on_deactivate(state);
}

CallbackReturnT
OptitrackDriverNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaned up!\n");

  if (disconnect_optitrack()) {
    return ControlledLifecycleNode::on_cleanup(state);
  } else {
    return CallbackReturnT::FAILURE;
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OptitrackDriverNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutted down!\n");

  if (disconnect_optitrack()) {
    return ControlledLifecycleNode::on_shutdown(state);
  } else {
    return CallbackReturnT::FAILURE;
  }
}

CallbackReturnT
OptitrackDriverNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  disconnect_optitrack();

  return ControlledLifecycleNode::on_error(state);
}

bool
OptitrackDriverNode::connect_optitrack()
{
  RCLCPP_INFO(
    get_logger(),
    "Trying to connect to Optitrack NatNET SDK at %s ...", server_address_.c_str());

  client->Disconnect();
  set_settings_optitrack();

  if (client->Connect(client_params) == ErrorCode::ErrorCode_OK) {
    RCLCPP_INFO(get_logger(), "... connected!");

    memset(&server_description, 0, sizeof(server_description));
    client->GetServerDescription(&server_description);
    if (!server_description.HostPresent) {
      RCLCPP_DEBUG(get_logger(), "Unable to connect to server. Host not present.");
      return false;
    }

    if (client->GetDataDescriptionList(&data_descriptions) != ErrorCode_OK || !data_descriptions) {
      RCLCPP_DEBUG(get_logger(), "[Client] Unable to retrieve Data Descriptions.\n");
    }

    RCLCPP_INFO(get_logger(), "\n[Client] Server application info:\n");
    RCLCPP_INFO(
      get_logger(), "Application: %s (ver. %d.%d.%d.%d)\n",
      server_description.szHostApp, server_description.HostAppVersion[0],
      server_description.HostAppVersion[1], server_description.HostAppVersion[2],
      server_description.HostAppVersion[3]);
    RCLCPP_INFO(
      get_logger(), "NatNet Version: %d.%d.%d.%d\n", server_description.NatNetVersion[0],
      server_description.NatNetVersion[1],
      server_description.NatNetVersion[2], server_description.NatNetVersion[3]);
    RCLCPP_INFO(get_logger(), "Client IP:%s\n", client_params.localAddress);
    RCLCPP_INFO(get_logger(), "Server IP:%s\n", client_params.serverAddress);
    RCLCPP_INFO(get_logger(), "Server Name:%s\n", server_description.szHostComputerName);

    void * pResult;
    int nBytes = 0;

    if (client->SendMessageAndWait("FrameRate", &pResult, &nBytes) == ErrorCode_OK) {
      float fRate = *(static_cast<float *>(pResult));
      RCLCPP_INFO(get_logger(), "Mocap Framerate : %3.2f\n", fRate);
    } else {
      RCLCPP_DEBUG(get_logger(), "Error getting frame rate.\n");
    }
  } else {
    RCLCPP_INFO(get_logger(), "... not connected :( ");
    return false;
  }

  return true;
}

bool
OptitrackDriverNode::disconnect_optitrack()
{
  void * response;
  int nBytes;
  if (client->SendMessageAndWait("Disconnect", &response, &nBytes) == ErrorCode_OK) {
    client->Disconnect();
    RCLCPP_INFO(get_logger(), "[Client] Disconnected");
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "[Client] Disconnect not successful..");
    return false;
  }
}

void
OptitrackDriverNode::initParameters()
{
  get_parameter<std::string>("connection_type", connection_type_);
  get_parameter<std::string>("server_address", server_address_);
  get_parameter<std::string>("local_address", local_address_);
  get_parameter<std::string>("multicast_address", multicast_address_);
  get_parameter<uint16_t>("server_command_port", server_command_port_);
  get_parameter<uint16_t>("server_data_port", server_data_port_);

  get_parameter<std::string>("qos_history_policy", qos_history_policy_);
  get_parameter<std::string>("qos_reliability_policy", qos_reliability_policy_);
  get_parameter<int>("qos_depth", qos_depth_);

  get_parameter<bool>("publish_tf", publish_tf_);
  get_parameter<bool>("publish_y_up_tf", publish_y_up_tf_);
  if (publish_tf_) {
    get_parameter<std::string>("rb_parent_frame_name", rb_parent_frame_name_);
  }
  if (publish_y_up_tf_) {
    get_parameter<std::string>("y_up_frame_name", y_up_frame_name_);
  }
}

}  // namespace mocap4r2_optitrack_driver
