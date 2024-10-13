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

#ifndef FFMPEG_IMAGE_TRANSPORT__FFMPEG_PUBLISHER_HPP_
#define FFMPEG_IMAGE_TRANSPORT__FFMPEG_PUBLISHER_HPP_

#include <image_transport/simple_publisher_plugin.hpp>
#include <memory>
#include <std_msgs/msg/string.hpp>
//#include <unordered_map>
//#include <string>
//#include <utility>

#include "abr_ffmpeg_image_transport/ffmpeg_encoder.hpp"
#include "abr_ffmpeg_image_transport/types.hpp"

namespace abr_ffmpeg_image_transport
{
using FFMPEGPublisherPlugin = image_transport::SimplePublisherPlugin<FFMPEGPacket>;

class FFMPEGPublisher : public FFMPEGPublisherPlugin
{
public:
  FFMPEGPublisher();
  ~FFMPEGPublisher() override;
  std::string getTransportName() const override { return "abr_ffmpeg"; }

protected:
#if defined(IMAGE_TRANSPORT_API_V1) || defined(IMAGE_TRANSPORT_API_V2)
  void advertiseImpl(
    rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos) override;
#else
  void advertiseImpl(
    rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos,
    rclcpp::PublisherOptions opt) override;
#endif
  void publish(const Image & message, const PublishFn & publish_fn) const override;


  bool init_ready = false;
  mutable bool ready_to_recive_video = false;

private:
  void packetReady(const FFMPEGPacketConstPtr & pkt) const;
  rmw_qos_profile_t initialize(rclcpp::Node * node, rmw_qos_profile_t custom_qos);
  // variables ---------
  rclcpp::Logger logger_;
  mutable const PublishFn * publishFunction_{NULL};
  mutable FFMPEGEncoder encoder_;
  mutable uint32_t frameCounter_{0};
  // ---------- configurable parameters
  int performanceInterval_{175};  // num frames between perf printouts
  bool measurePerformance_{false};

  // ABR communication channels
  rclcpp::Publisher<ABRInfoPacket>::SharedPtr abr_info_publisher_;
  rclcpp::Subscription<ABRInfoPacket>::SharedPtr abr_info_subscriber_;
  void abrInfoCallback(const ABRInfoPacket::SharedPtr msg);

  // Bitrate ladder
  mutable int width;
  mutable int height;
  mutable int forced_width;
  mutable int forced_height;
  mutable int selected_bitrate;
  rclcpp::Node* node_;

  mutable double framerate = 0.0;
  mutable rclcpp::Time framerate_ts;
  mutable std::vector<double> bitrate_ladder;
  mutable std::string bitrate_ladder_string = ""; 
  std::unordered_map<std::string, std::pair<int, int>> resolution_map_;

  mutable bool ladder_ready = false;

  mutable int framerate_sum = 0;     // Suma de los valores de framerate en el último segundo
  mutable int framerate_count = 0;   // Cantidad de valores en el último segundo
  mutable rclcpp::Time last_update_time = rclcpp::Time(0, 0, RCL_STEADY_TIME);  // Marca de tiempo del último segundo


  std::string identifyResolution(int width, int height) const;
  void filterInefficientConfigurations(int width, int height) const;
  void filterResolutionsByName(const std::string& resolution_name, bool preserve) const;
  //std::pair<int, int> findBestResolutionMatch(int inner_width, int inner_height, int inner_selected_bitrate) const;
  std::pair<int, int>  parseResolution(const std::string& res_str) const;
  void expectedBitrateLadder(int width, int height, int framerate) const;
  void filterBitrateLadder(double max_rate_mbps) const;

};
}  // namespace abr_ffmpeg_image_transport

#endif  // FFMPEG_IMAGE_TRANSPORT__FFMPEG_PUBLISHER_HPP_
