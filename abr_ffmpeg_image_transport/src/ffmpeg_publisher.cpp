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

#include "abr_ffmpeg_image_transport/ffmpeg_publisher.hpp"
#include "abr_ffmpeg_image_transport/safe_param.hpp"
#include <std_msgs/msg/string.hpp>
#include <map>
#include <filesystem>
#include <fstream>


using namespace std::placeholders;

namespace abr_ffmpeg_image_transport
{

/*
   _____ ____  _   _ ______ 
  / ____/ __ \| \ | |  ____|
 | |   | |  | |  \| | |__   
 | |   | |  | | . ` |  __|  
 | |___| |__| | |\  | |     
  \_____\____/|_| \_|_|     
                                          
*/

FFMPEGPublisher::FFMPEGPublisher() : logger_(rclcpp::get_logger("FFMPEGPublisher")) {
  framerate_ts = rclcpp::Time(0, 0, RCL_STEADY_TIME); 
}

FFMPEGPublisher::~FFMPEGPublisher() {}

void FFMPEGPublisher::packetReady(const FFMPEGPacketConstPtr & pkt) const { (*publishFunction_)(*pkt); }

#if defined(IMAGE_TRANSPORT_API_V1) || defined(IMAGE_TRANSPORT_API_V2)
void FFMPEGPublisher::advertiseImpl(
  rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos)
{
  auto qos = initialize(node, custom_qos);
  FFMPEGPublisherPlugin::advertiseImpl(node, base_topic, qos);
}
#else
void FFMPEGPublisher::advertiseImpl(
  rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions opt)
{
  auto qos = initialize(node, custom_qos);
  FFMPEGPublisherPlugin::advertiseImpl(node, base_topic, qos, opt);
}
#endif

/*
  _____ _   _ _____ _______ 
 |_   _| \ | |_   _|__   __|
   | | |  \| | | |    | |   
   | | | . ` | | |    | |   
  _| |_| |\  |_| |_   | |   
 |_____|_| \_|_____|  |_|   
                            
*/

rmw_qos_profile_t FFMPEGPublisher::initialize(rclcpp::Node * node, rmw_qos_profile_t custom_qos)
{

  /*
  Load json posible resolutions resolutions
  */
  std::filesystem::path source_dir = std::filesystem::path(__FILE__).parent_path().parent_path();
  std::filesystem::path file_path = source_dir / "resolution_configurations.json";
  std::ifstream file1(file_path);
  
  if (!file1.is_open()) {
      throw std::runtime_error("Could not open the resolution configuration file.");
  }
  nlohmann::json json_config;
  file1 >> json_config;

  for (const auto& [key, value] : json_config.items()) {
      int width = value["width"];
      int height = value["height"];
      resolution_map_[key] = {width, height};
  }


  /*
  Load json bitladder
  */

    file_path = source_dir / "bitrate_ladder.json";

    std::cout << "Path: " << file_path << "\n";
    std::ifstream file2(file_path);
    if (!file2.is_open()) {
        std::cerr << "Couldnt open json file" << std::endl;
    }
    json configurations;
    file2 >> configurations;
    file2.close();

    for (const auto& value : configurations) {
        bitrate_ladder.push_back(value.get<double>());
    }


  /*
  Initialize ABR communication and configure QoS for info channel
  */

  rclcpp::QoS qos_settings(10);  // Depth de 10
  qos_settings.reliability(rclcpp::ReliabilityPolicy::Reliable);  // Reliable
  qos_settings.durability(rclcpp::DurabilityPolicy::TransientLocal);  // Transient Local

  abr_info_publisher_ = node->create_publisher<ABRInfoPacket>("abr_info", qos_settings);
  abr_info_subscriber_ = node->create_subscription<ABRInfoPacket>(
    "abr_info", qos_settings,
    [this, node](const ABRInfoPacket::SharedPtr msg) {
      this->abrInfoCallback(msg);
    }
  );
  /*
  Change CODEC parameters:
  */

  const std::string ns = "abr_ffmpeg_image_transport.";
  measurePerformance_ = get_safe_param<bool>(node, ns + "measure_performance", false);
  encoder_.setMeasurePerformance(measurePerformance_);
  performanceInterval_ = get_safe_param<int>(node, ns + "performance_interval", 175);

 
  /*
  Change VIDEO QoS parameters:
  */
  int depth = 0; 
  rclcpp::QoS custom_qos_profile = rclcpp::QoS(rclcpp::KeepLast(depth));
  custom_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  custom_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);

  custom_qos = custom_qos_profile.get_rmw_qos_profile();

  node_ = node;
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  auto current_time = steady_clock.now();

  init_ready = true;


  return custom_qos;
}


std::string FFMPEGPublisher::identifyResolution(int width, int height) const {
  /**
 * @brief Identifies the closest resolution name based on the given width and height.
 *
 * This function first checks for an exact match of the specified `width` and `height` 
 * within `resolution_map_`. If no exact match is found, it then calculates the aspect 
 * ratio of the provided dimensions and finds the closest matching resolution based on 
 * the smallest difference in aspect ratio.
 *
 * @param width The width of the resolution to identify.
 * @param height The height of the resolution to identify.
 * @return A `std::string` representing the name of the closest matching resolution.
 */

  double best_diff = std::numeric_limits<double>::max();
  std::string best_res_name = "Unknown";

  for (const auto& [res_name, dims] : resolution_map_) {
      if (width == dims.first && height == dims.second) {
          return res_name;  
      }
  }

  for (const auto& [res_name, dims] : resolution_map_) {
      double aspect_ratio_diff = std::abs(static_cast<double>(width) / height - static_cast<double>(dims.first) / dims.second);
      if (aspect_ratio_diff < best_diff) {
          best_diff = aspect_ratio_diff;
          best_res_name = res_name;
      }
  }

  RCLCPP_INFO(logger_, "No exact match found, using closest resolution: %s (%d x %d)", 
              best_res_name.c_str(), resolution_map_.at(best_res_name).first, resolution_map_.at(best_res_name).second);
  return best_res_name;
}



std::pair<int, int> FFMPEGPublisher::parseResolution(const std::string& res_str) const {
    /**
     * @brief Parses a resolution string to retrieve its corresponding width and height.
     *
     * This function looks up a resolution string (e.g., "1080p") in the `resolution_map_` 
     * and returns the corresponding resolution dimensions as a pair of integers. If the 
     * resolution string is not found, an error is logged, and a runtime exception is thrown.
     *
     * @param res_str A string representing the desired resolution (e.g., "720p", "1080p").
     * @return A `std::pair<int, int>` containing the width and height corresponding to the resolution.
     * @throws std::runtime_error if the resolution string is not found in `resolution_map_`.
     */

    auto it = resolution_map_.find(res_str);
    if (it != resolution_map_.end()) {
        return it->second;
    } else {
        RCLCPP_ERROR(logger_, "@parseResolution: Unknown resolution string: %s", res_str.c_str());
        throw std::runtime_error("Unknown resolution string: " + res_str);
    }
}



void FFMPEGPublisher::expectedBitrateLadder(int width, int height, int framerate) const {

  /**
   * @brief Calculates and updates the expected bitrate ladder based on resolution and framerate.
   *
   * This function recalculates the bitrate ladder values using a specified resolution (`width` x `height`)
   * and `framerate`, combined with predefined compression ratios. The result is stored in an updated
   * `bitrate_ladder` vector, which is then used for adaptive bitrate control.
   *
   * @param width The width of the video resolution.
   * @param height The height of the video resolution.
   * @param framerate The target framerate of the video.
   */
  
    std::vector<double> updated_bitrate_ladder;
    updated_bitrate_ladder.reserve(bitrate_ladder.size());

    for (const auto& compression_ratio : bitrate_ladder) {

        double expected_bitrate = static_cast<double>(width) * height * framerate * compression_ratio;
        updated_bitrate_ladder.push_back(expected_bitrate);
        RCLCPP_INFO(logger_, "Calculated expected bitrate: %f with compression factor: %f", expected_bitrate, compression_ratio);
    }

    bitrate_ladder = std::move(updated_bitrate_ladder);
}


void FFMPEGPublisher::filterBitrateLadder(double max_rate_mbps) const {
    bitrate_ladder.erase(
        std::remove_if(bitrate_ladder.begin(), bitrate_ladder.end(), 
                       [max_rate_mbps](double bitrate) { return bitrate > max_rate_mbps; }),
        bitrate_ladder.end()
    );

    RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "Filtered bitrate ladder:");
    for (const auto& bitrate : bitrate_ladder) {
        RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "%f Mbps", bitrate);
    }
}




/*
 __      _______ _____  ______ ____      _____          _      _      ____          _____ _  __
 \ \    / /_   _|  __ \|  ____/ __ \    / ____|   /\   | |    | |    |  _ \   /\   / ____| |/ /
  \ \  / /  | | | |  | | |__ | |  | |  | |       /  \  | |    | |    | |_) | /  \ | |    | ' / 
   \ \/ /   | | | |  | |  __|| |  | |  | |      / /\ \ | |    | |    |  _ < / /\ \| |    |  <  
    \  /   _| |_| |__| | |___| |__| |  | |____ / ____ \| |____| |____| |_) / ____ \ |____| . \ 
     \/   |_____|_____/|______\____/    \_____/_/    \_\______|______|____/_/    \_\_____|_|\_\
                                                                                                                                                   
*/

void FFMPEGPublisher::publish(const Image & msg, const PublishFn & publish_fn) const
{
  /**
   * @brief Publishes an image message using the specified encoding parameters.
   *
   * This function manages the encoding process for video frames and adapts settings dynamically based on the content
   * of a configuration file. The encoding process will not begin until the system is ready to receive video. When ready,
   * the encoder is initialized if necessary, and encoding parameters such as bitrate, resolution, and framerate are 
   * dynamically adjusted based on the configuration file.
   *
   * @param msg The image message to be encoded and published.
   * @param publish_fn A function object for publishing the encoded packets.
   */


  // Check if the system is ready to receive video
  if (ready_to_recive_video){

    // Initialize encoder if it hasn't been initialized
    if (!encoder_.isInitialized()) {
      publishFunction_ = &publish_fn;
      if (!encoder_.initialize(
            forced_width, forced_height, [this](const FFMPEGPacketConstPtr & pkt) {
                this->packetReady(pkt);
            })) {
        RCLCPP_ERROR_STREAM(logger_, "cannot initialize encoder!");
        return;
      }
    }

    // Encode the image frame using the forced width and height
    encoder_.encodeImage(msg,forced_width,forced_height);

    if (measurePerformance_) {
      if (static_cast<int>(++frameCounter_) > performanceInterval_) {
        encoder_.printTimers(logger_.get_name());
        encoder_.resetTimers();
        frameCounter_ = 0;
      }
    }


  }else{ // System not ready, check for initialization tasks
      
      if (init_ready) {

      // Load configuration settings from 'config.json'
      std::filesystem::path source_dir = std::filesystem::path(__FILE__).parent_path().parent_path();
      std::filesystem::path file_path = source_dir / "config.json";
      std::ifstream file1(file_path);
      if (!file1.is_open()) {
        throw std::runtime_error("Could not open the resolution configuration file.");
      }
      nlohmann::json json_config;
      file1 >> json_config;
      file1.close();

      // Set framerate based on configuration file or calculate dynamically if not defined
      if (json_config.contains("target_framerate")) {
        if (json_config["target_framerate"].is_number()) {

          framerate = json_config["target_framerate"].get<int>();
          RCLCPP_INFO(logger_, "Using target framerate from config.json: %f fps", framerate);
        } else if (json_config["target_framerate"].is_string() && json_config["target_framerate"] == "") {

          // Calculate framerate dynamically
          RCLCPP_INFO(logger_, "Target framerate is empty, estimating dynamically.");
          static std::vector<int> framerate_values;
          rclcpp::Clock steady_clock(RCL_STEADY_TIME);
          auto current_time = steady_clock.now();

          double elapsed_time = (current_time - last_update_time).seconds();
          if (elapsed_time >= 1.0) {
            if (!framerate_values.empty()) {
              double harmonic_sum = 0.0;
              for (int fr : framerate_values) {
                harmonic_sum += 1.0 / static_cast<double>(fr);
              }

              double harmonic_mean_framerate = framerate_values.size() / harmonic_sum;
              RCLCPP_INFO(logger_, "Harmonic mean framerate over 1 second: %.2f fps", harmonic_mean_framerate);

              framerate = static_cast<int>(std::floor(harmonic_mean_framerate));
            }

            framerate_values.clear();
            last_update_time = current_time;
          }

          double time_diff = (current_time - framerate_ts).seconds();
          if (time_diff > 0.001) {
            int new_framerate = std::round(1.0 / time_diff);

            if (new_framerate > 1) {
              framerate_values.push_back(new_framerate);
              RCLCPP_INFO(logger_, "Calculated framerate: %d fps", new_framerate);
            } else {
              RCLCPP_WARN(logger_, "Invalid framerate: %d fps (too low)", new_framerate);
            }
          }

          framerate_ts = current_time;
        } else {
          throw std::runtime_error("Invalid type for 'target_framerate' in config.json");
        }
      } else {
        throw std::runtime_error("Missing 'target_framerate' in config.json");
      }


      // Set encoding parameters if the framerate is valid
      if (framerate >= 1.0){

        if (framerate > 120){
          framerate = 120; // Limit framerate to 120 fps
        }
        
        // Load encoder settings from the config file
        width = msg.width;
        height = msg.height;
        std::string resolution = identifyResolution(msg.width, msg.height);

        std::filesystem::path source_dir = std::filesystem::path(__FILE__).parent_path().parent_path();

        std::filesystem::path file_path = source_dir / "config.json";
        std::ifstream file1(file_path);
        if (!file1.is_open()) {
            throw std::runtime_error("Could not open the resolution configuration file.");
        }
        nlohmann::json json_config;
        file1 >> json_config;


        double max_rate_mbps = json_config.value("max_rate_mbps", 0.0);
        if (max_rate_mbps == 0.0 || (json_config["max_rate_mbps"].is_string() && json_config["max_rate_mbps"].get<std::string>().empty())) {
            max_rate_mbps = std::numeric_limits<double>::max(); // Set to a very high value if max_rate_mbps is 0 or empty string
        }
        filterBitrateLadder(max_rate_mbps);

        nlohmann::json json_bitrate_ladder;
        for (const auto& bitrate : bitrate_ladder) {
            json_bitrate_ladder.push_back(bitrate);
        }

        bitrate_ladder_string = json_bitrate_ladder.dump();
        ladder_ready = true;


        // Find the configuration with the lowest bitrate
        if (!bitrate_ladder.empty()) {
          selected_bitrate = *std::min_element(bitrate_ladder.begin(), bitrate_ladder.end()) *1e6;
            RCLCPP_INFO(logger_, "Selected bitrate (smallest in list): %d bps", selected_bitrate );
        } else {
            RCLCPP_ERROR(logger_, "bitrate_ladder is empty, cannot set selected_bitrate.");
        }

        //Load from config.json
        std::unordered_map<std::string, std::string> encoderParams;
        encoderParams["codecName"] = json_config.value("codecName", "libx264");
        encoderParams["preset"] = json_config.value("codec_preset", "fast");
        encoderParams["tune"] = "zerolatency";
        encoderParams["gopSize"] = std::to_string(json_config.value("gop_size", 3));
        encoderParams["frame_rate"] = std::to_string(framerate); //Changes both framerate and timebase
        encoderParams["target_bitrate"] = std::to_string(selected_bitrate);
        encoder_.setParameters(node_, encoderParams);

        file1.close();

        forced_width = width;
        forced_height = height;

        RCLCPP_INFO(logger_, "Best resolution match found, forced_width: %d, forced_height: %d", 
            forced_width, forced_height);

        ready_to_recive_video = true;
      }
    }

  }
}


/*
  _____ _   _ ______ ____      _____          _      _      ____          _____ _  __
 |_   _| \ | |  ____/ __ \    / ____|   /\   | |    | |    |  _ \   /\   / ____| |/ /
   | | |  \| | |__ | |  | |  | |       /  \  | |    | |    | |_) | /  \ | |    | ' / 
   | | | . ` |  __|| |  | |  | |      / /\ \ | |    | |    |  _ < / /\ \| |    |  <  
  _| |_| |\  | |   | |__| |  | |____ / ____ \| |____| |____| |_) / ____ \ |____| . \ 
 |_____|_| \_|_|    \____/    \_____/_/    \_\______|______|____/_/    \_\_____|_|\_\
                                                                                     
*/

void FFMPEGPublisher::abrInfoCallback(const ABRInfoPacket::SharedPtr msg) {
  /**
 * @brief Callback function for handling incoming ABRInfoPacket messages related to Adaptive Bitrate (ABR) requests.
 *
 * This callback function processes ABR requests received from the client. It handles different message types:
 * - Type 0 (HANDSHAKE): Responds with information about the available bitrate ladder, target width, height, and framerate.
 * - Type 1 (QUALITY_CHANGE): Adjusts the encoding quality based on the desired bitrate specified by the client.
 *
 * @param msg The incoming ABRInfoPacket message pointer.
 */

  if (msg->role == "client") {

    if (msg->msg_type == 0) {  // HANDSHAKE

      json json_msg = json::parse(msg->msg_json);
      
      if (ladder_ready) {  // Check if JSON is empty or matches your expected structure
        ABRInfoPacket init_msg;
        init_msg.role = "server";
        init_msg.msg_type = 0;

        json response_json = {
          {"available", true},
          {"bitrate_ladder", bitrate_ladder_string},
          {"desired_width", width},
          {"desired_height", height},
          {"framerate",framerate}

        };

        init_msg.msg_json = response_json.dump();
        abr_info_publisher_->publish(init_msg);

      }else{

        ABRInfoPacket init_msg;
        init_msg.role = "server";
        init_msg.msg_type = 0;

        json response_json = {
          {"available", false},
          {"bitrate_ladder", ""}
        };

        init_msg.msg_json = response_json.dump();
        abr_info_publisher_->publish(init_msg);
      }


    } else if (msg->msg_type == 1)  // QUALITY_CHANGE: Adjust codec parameters to match requested bitrate
      {

      RCLCPP_INFO(logger_, "Changing the quality");

      json json_msg = json::parse(msg->msg_json);

      if (!json_msg.empty()) {  

          double desired_bitrate = json_msg["desired_bitrate"].get<double>();
          auto it = std::lower_bound(bitrate_ladder.begin(), bitrate_ladder.end(), desired_bitrate);
          
          if (it == bitrate_ladder.end()) {
              RCLCPP_WARN(logger_, "Desired bitrate not found in ladder, unable to change quality.");
              return;
          }
          
          double selected_bitrate = *it; 
          RCLCPP_INFO(logger_, "New configuration ASKED: Bitrate: %f", selected_bitrate);

          ABRInfoPacket init_msg;
          init_msg.role = "server";
          init_msg.msg_type = 1;

          json response_json = {
              {"confirmed", true},
              {"selected_bitrate", selected_bitrate}
          };
          init_msg.msg_json = response_json.dump();

          //////////////////////////////////////////////////
          encoder_.closeCodec();
          encoder_.setBitRate(selected_bitrate * 1e6);  

          if (!encoder_.openCodec(forced_width, forced_height)) {
              RCLCPP_ERROR_STREAM(logger_, "Failed to reopen codec with new parameters!");
          }
          //////////////////////////////

          abr_info_publisher_->publish(init_msg);
      }
    }
  } 
}

}  // namespace abr_ffmpeg_image_transport
