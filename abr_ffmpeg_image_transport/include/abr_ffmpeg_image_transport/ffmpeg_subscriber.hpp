// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef FFMPEG_IMAGE_TRANSPORT__FFMPEG_SUBSCRIBER_HPP_
#define FFMPEG_IMAGE_TRANSPORT__FFMPEG_SUBSCRIBER_HPP_

#include <image_transport/simple_subscriber_plugin.hpp>
#include <std_msgs/msg/string.hpp>
#include "abr_ffmpeg_image_transport/ffmpeg_decoder.hpp"
#include "abr_ffmpeg_image_transport/types.hpp"
#include "abr_ffmpeg_image_transport/abr_component.h"
#include <map>

namespace abr_ffmpeg_image_transport
{
using FFMPEGSubscriberPlugin = image_transport::SimpleSubscriberPlugin<FFMPEGPacket>;

class FFMPEGSubscriber : public FFMPEGSubscriberPlugin
{
public:
  FFMPEGSubscriber();
  ~FFMPEGSubscriber();

  std::string getTransportName() const override { return "abr_ffmpeg"; }

protected:
  void internalCallback(const FFMPEGPacketConstPtr & msg, const Callback & user_cb) override;

#ifdef IMAGE_TRANSPORT_API_V1
  void subscribeImpl(
    rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
    rmw_qos_profile_t custom_qos) override;
#else
  void subscribeImpl(
    rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
    rmw_qos_profile_t custom_qos, rclcpp::SubscriptionOptions) override;
#endif


int desired_width;
int desired_height; 
int framerate;
rclcpp::Node * node_;

private:

  //Plugin stuff
  rclcpp::Logger logger_;
  

  //Codec stuff
  void frameReady(const ImageConstPtr & img, bool /*isKeyFrame*/) const;
  void initialize(rclcpp::Node * node);
  FFMPEGDecoder decoder_;
  std::string decoderType_;
  const Callback * userCallback_;

  //ABR algorithm
  AbrComponent abr_component_;
  bool allow_transmition_ = false;
  mutable std::vector<double> bitrate_ladder;

  // ABR communication channels
  rclcpp::Publisher<ABRInfoPacket>::SharedPtr abr_info_publisher_;
  rclcpp::Subscription<ABRInfoPacket>::SharedPtr abr_info_subscriber_;
  void abrInfoCallback(const ABRInfoPacket::SharedPtr msg);

};
}  // namespace abr_ffmpeg_image_transport
#endif  // FFMPEG_IMAGE_TRANSPORT__FFMPEG_SUBSCRIBER_HPP_
