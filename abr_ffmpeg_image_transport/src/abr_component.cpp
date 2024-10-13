#include "abr_ffmpeg_image_transport/abr_component.h"
#include <rclcpp/serialization.hpp> 
#include <rclcpp/serialized_message.hpp> 
#include <cmath>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>

AbrComponent::AbrComponent()
{// Constructor
    reconfigureBuffers(); 
}

AbrComponent::~AbrComponent()
{// Destructor
    bitrate_buffer_.clear();
    voter_buffer_.clear();
    latency_buffer_.clear();

    if (log_file_.is_open()) {
            log_file_.close();
        }
}

void AbrComponent::reset()
{// Destructor
    bitrate_buffer_.clear();
    voter_buffer_.clear();
    latency_buffer_.clear();

}


void AbrComponent::reconfigureBuffers() {

    bitrate_buffer_.clear();  
    bitrate_buffer_.resize(getBitrateBufferSize(),1.0);  

    voter_buffer_.clear();  
    voter_buffer_.resize(getVoterBufferSize(),1.0);  

    latency_buffer_.clear();
    latency_buffer_.resize(getBitrateBufferSize(),1.0);  

}

double AbrComponent::calculateHarmonicMean() {

    /**
     * @brief Calculates the harmonic mean of the current bitrate values stored in the buffer.
     *
     * The harmonic mean is useful for averaging rates, as it emphasizes smaller values.
     * This function calculates the harmonic mean of the bitrates in `bitrate_buffer_`. 
     * If the buffer is empty, it logs a warning and returns 0. If any bitrate in the 
     * buffer is zero or negative, it also logs a warning and returns `NaN` since the 
     * harmonic mean is undefined for non-positive values.
     *
     * @return The harmonic mean of the `bitrate_buffer_`. Returns 0 if the buffer is empty,
     *         or `NaN` if any value is non-positive.
     */

    if (bitrate_buffer_.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("AbrComponent"), "Bitrate buffer is empty. Returning 0.");
        return 0.0;  // Return 0 if buffer is empty
    }

    double sumOfInverses = 0.0; 
    for (double bitrate : bitrate_buffer_) {
        if (bitrate <= 0) {
            RCLCPP_WARN(rclcpp::get_logger("AbrComponent"), "All values in the vector must be positive.");
            return std::numeric_limits<double>::quiet_NaN(); 
        }
        sumOfInverses += 1.0 / bitrate;
    }

    return bitrate_buffer_.size() / sumOfInverses;
}

double AbrComponent::calculateJitter() {
    /**
     * @brief Calculates the jitter based on the differences between consecutive latency values.
     *
     * Jitter represents the average variation between consecutive latency measurements stored 
     * in `latency_buffer_`. This function calculates the absolute difference between each pair 
     * of consecutive latency values, sums them, and then divides by the number of intervals to 
     * obtain the average jitter. If the buffer contains fewer than two entries, the function 
     * logs a warning and returns 0, as it is insufficient to calculate jitter.
     *
     * @return The calculated jitter. If there are fewer than two entries in the buffer, returns 0.
     */

    if (latency_buffer_.size() < 2) {
        RCLCPP_WARN(rclcpp::get_logger("AbrComponent"), "Not enough data in latency buffer to calculate jitter.");
        return 0.0;
    }

    double total_jitter = 0.0;

    // Calculate the sum of absolute differences between consecutive latency values
    for (size_t i = 1; i < latency_buffer_.size(); ++i) {
        total_jitter += std::abs(latency_buffer_[i] - latency_buffer_[i - 1]);
    }

    // Average the total jitter by dividing by the number of intervals
    double jitter = total_jitter / (latency_buffer_.size() - 1);

    return jitter;
}


double AbrComponent::calculateMean() {
    /**
     * @brief Calculates the mean of the current bitrate values stored in the buffer.
     *
     * This function computes the average value of the bitrates in `bitrate_buffer_`. 
     * If the buffer is empty, it returns 0. Otherwise, it sums all the values and divides 
     * by the total number of entries in the buffer to find the mean.
     *
     * @return The mean value of the `bitrate_buffer_`. If the buffer is empty, returns 0.
     */

    if (bitrate_buffer_.empty()) {
        return 0.0; 
    }

    double sum = std::accumulate(bitrate_buffer_.begin(), bitrate_buffer_.end(), 0.0);

    return sum / bitrate_buffer_.size();
}


double AbrComponent::calculateMedian() {
    /**
     * @brief Calculates the median of the current bitrate values stored in the buffer.
     *
     * This function finds the median of the values in `bitrate_buffer_`, which represents 
     * recent bitrate measurements. If the buffer is empty, the function logs a warning 
     * and returns 0. Otherwise, it sorts the values in ascending order, then identifies 
     * the middle value (or the average of the two middle values if the buffer size is even)
     * to calculate the median.
     *
     * @return The median value of the `bitrate_buffer_`. If the buffer is empty, returns 0.
     */

    if (bitrate_buffer_.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("AbrComponent"), "Bitrate buffer is empty. Returning 0.");
        return 0.0;  // Return 0 if buffer is empty
    }

    // Convert deque to vector for sorting
    std::vector<double> sorted_buffer(bitrate_buffer_.begin(), bitrate_buffer_.end());
    std::sort(sorted_buffer.begin(), sorted_buffer.end());

    // Find the middle index
    size_t middle = sorted_buffer.size() / 2;
    double median = 0.0;

    // Calculate the median depending on whether the buffer size is even or odd
    if (sorted_buffer.size() % 2 == 0) {
        median = (sorted_buffer[middle - 1] + sorted_buffer[middle]) / 2.0;
    } else {
        median = sorted_buffer[middle];
    }

    return median;
}


void AbrComponent::analyzeFlow(const ffmpeg_image_transport_msgs::msg::FFMPEGPacket &msg , rclcpp::Node * node_)
{
    /**
     * @brief Analyzes the data flow for a given FFMPEG packet, updating bitrate and latency metrics.
     *
     * This function processes each incoming FFMPEG packet, calculating metrics such as latency, 
     * instant bitrate, harmonic mean, mean, median, and jitter, which are used to monitor the 
     * system’s streaming performance. It also updates internal buffers and, if necessary, 
     * activates emergency mode if latency exceeds a defined threshold. Additionally, the function 
     * logs information to a CSV file and publishes key metrics as a JSON message when enabled.
     *
     * @param msg The incoming FFMPEG packet containing video data.
     * @param node_ The ROS node used for logging and timing information.
     */


    // Get the current system time
    auto system_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    rclcpp::Time actual_time = system_clock->now();

    // Calculate latency since the previous packet timestamp
    rclcpp::Duration duration = actual_time - previous_timeStamp;
    double latency = duration.seconds();

    // Calculate frames per second based on latency
    measured_fps = 1/latency;

    // Check if latency exceeds the emergency threshold and activate emergency mode if needed
    if (latency > getEmergencyTresh()) {
        activateEmergencyMode();
        emergency_mode = true;
        RCLCPP_INFO(node_->get_logger(), "Emergency mode activated");
    } 

    if (allow_transmition_ && (actual_time > getDateRefresh())) {

        // Update latency buffer
        if (static_cast<int>(latency_buffer_.size()) == getBitrateBufferSize()) {
            latency_buffer_.erase(latency_buffer_.begin()); 
        }

        latency_buffer_.emplace_back(latency);

        emergency_mode = false;

        // Serializate msg and calculate size
        rclcpp::Serialization<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> serializer;
        rclcpp::SerializedMessage serialized_msg;
        serializer.serialize_message(&msg, &serialized_msg);
        size_t size = serialized_msg.size() * 8;  

        double instant_bitrate = static_cast<double>(size) / latency;
        
        // Limit the instant value
        double max_bitrate = 2e9;  // 2,000,000,000 bits per second (2 Gbps)
        if (instant_bitrate > max_bitrate) {
            instant_bitrate = max_bitrate;
        }

        ideal_expected_bitrate = static_cast<double>(size) * fps;
        if (ideal_expected_bitrate > max_bitrate) {
            ideal_expected_bitrate = max_bitrate;
        }

        // Manage bitrate buffer as a circular buffer
        if (static_cast<int>(bitrate_buffer_.size()) == getBitrateBufferSize()) {
            bitrate_buffer_.erase(bitrate_buffer_.begin()); 
        }

        bitrate_buffer_.emplace_back(instant_bitrate);

        bitrate_buffer_position += 1;

        if (bitrate_buffer_position >= getBitrateBufferSize()){ //NO sliding window for the moment

            bitrate_buffer_position = 0; 

            double hm = calculateHarmonicMean(); 
            double mean = calculateMean();
            double median = calculateMedian();
            double jitter = calculateJitter();

            // Limit values to no surpass variable type
            if (hm > max_bitrate) {
                hm = max_bitrate;
            }

            if (mean > mean) {
                mean = mean;
            }

            if (median > median) {
                median = median;
            }


            // Update the voter buffer based on the selected metric
            if (abr_predictor == "median"){
                voter_buffer_.push_back(median);
            }else if (abr_predictor == "hm")
            {
                voter_buffer_.push_back(hm);
            }else if (abr_predictor == "mean")
            {
                voter_buffer_.push_back(mean);
            }else if (instant_bitrate)
            {
                voter_buffer_.push_back(instant_bitrate);
            }else{
                voter_buffer_.push_back(median);
            }
            
            // Keep voter buffer size at 2
            if (voter_buffer_.size() > 2) {
                voter_buffer_.pop_front();
            }

            // Log data to CSV if enabled
            if (getCsv() && log_file_.is_open()) {

                auto it = std::find(bitrate_ladder.begin(), bitrate_ladder.end(), actual_bitrate);

                int stability_count = std::count(unestability_buffer_.begin(), unestability_buffer_.end(), true);

                log_file_ << std::fixed << std::setprecision(12)
                        << actual_time.seconds() << "," 
                        << size / 1e6 << ","             
                        << latency << ","                    
                        << instant_bitrate / 1e6 << ","        
                        << mean / 1e6 << ","
                        << hm / 1e6 << ","                  
                        << median / 1e6 << ","             
                        << ideal_expected_bitrate / 1e6 <<  ","
                        << *it << ","
                        << jitter << ","
                        << stability_count << ","
                        << measured_fps << 
                        "\n";      

                log_file_.flush();
            
            }

            if (republish_data_){
                // Publish on data topic
                ffmpeg_image_transport_msgs::msg::ABRInfoPacket abr_info_msg;
                abr_info_msg.role = "client";  
                abr_info_msg.msg_type = 4;     
                json msg_json = {
                    {"elapsed_time", (actual_time - getStartTime()).seconds()},
                    {"size", size / 1e6},            
                    {"latency", latency},                   
                    {"instant bitrate", instant_bitrate / 1e6},            
                    {"mean", mean/ 1e6},  
                    {"hm", hm / 1e6},                        
                    {"median", median / 1e6},              
                    {"jitter", jitter},
                    {"ideal_rate", ideal_expected_bitrate / 1e6}, 
                    
                };

                abr_info_msg.msg_json = msg_json.dump();
                publish_msg_(abr_info_msg);  
            }

            /*
            RCLCPP_INFO(
            rclcpp::get_logger("AbrComponent"),
            "Size: %zu bits, Latency: %.6f seconds, instant_bitrate: %.2f  Mbits/s, mean bitrate %.2f Mbits/s, Harmonic Mean: %.2f Mbit/s, Median: %.2f Mbit/s, jitter: %.2f  , ideal_flow: %.2f",
            size, latency, instant_bitrate/ 1e6 ,mean /1e6 , hm/ 1e6 ,median/1e6, jitter, ideal_expected_bitrate/ 1e6 );
            */

            // Call ABR logic
            abr_logic2();
            bitrate_buffer_.clear();

        }
    }
    // Update previous timestamp for next packet
    previous_timeStamp = actual_time;
}


void AbrComponent::abr_logic2(){
    /**
     * @brief Implements the adaptive bitrate logic by analyzing recent network stability and adjusting the bitrate accordingly.
     *
     * This function performs the core logic for adjusting the bitrate based on network stability metrics. 
     * It utilizes a voting mechanism between two consecutive time windows (voter_buffer_) to determine if the bitrate 
     * should be increased, decreased, or maintained. In cases of significant instability or a detected ripple condition, 
     * the function may lower the bitrate to ensure reliable streaming. If a change in bitrate is deemed necessary, 
     * a message is published to request an updated bitrate.
     */

    bool send_package = false;
    double desired_bitrate = 0;

    float compare_bitrate = ideal_expected_bitrate;
    int vote_sum = 1;

    // Variables to store the harmonic means of two time windows

    if (voter_buffer_.size() == 2) {
        double window1 = voter_buffer_[0];
        double window2 = voter_buffer_[1];

        // Calculate the difference between the two windows
        double difference = fabs(window1 - window2);

        //check if the difference exceeds the tolerance
        //double stability_threshold = 0.3;

        if (difference > compare_bitrate * stability_threshold) {
            RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "Communication unstable, pushing to inestability buffer.");

            updateUnestabilityBuffer(true);
 
        }else{
            
            //double similarity_threshold = 0.80;
            RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "Communication stable, action.");

            updateUnestabilityBuffer(false);

            int vote_window2 = (window2 >= similarity_threshold * compare_bitrate) ? 1 : 0;
            int vote_window1 = (window1 >= similarity_threshold * compare_bitrate) ? 1 : 0;
            vote_sum = vote_window1 + vote_window2;

        }

        bool strong_unestable_conection = std::count(unestability_buffer_.begin(), unestability_buffer_.end(), true) == 3;

        if (strong_unestable_conection){
            RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "Strong unstabilidty: Reducing");
            desired_bitrate=lower_bitrate();
            send_package = true;

        }else if (actual_bitrate > 1.15*ideal_expected_bitrate/1e6)
        {
            // If we are at settings well above the maximum, lower (order unnecessary settings above).
            RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "Requesting ladders over real max bitrate for this configuration. Keeping in this ladder");

        }else{
                        
            if (vote_sum == 0) {

                // Both values are below the threshold, reduce bitrate
                desired_bitrate=lower_bitrate();

                //ripple contidion
                bool ripple = false;
                if (previous_bitrate == desired_bitrate && previous_bitrate > actual_bitrate){
                    RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "Ripple");
                    ripple = true;
                    
                    ripple_order_count = ripple_order_count+1;
                    if (ripple_order_count >= ripple_order){
                        previous_bitrate = actual_bitrate;
                        ripple_order_count = 0;
                    }
                }

                if (desired_bitrate != actual_bitrate && ripple == false){
                    RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "Reducing bitrate");
                    send_package = true;
                }
            } else if (vote_sum == 2) {

                // Both values are above the threshold, increase bitrate
                desired_bitrate=increase_bitrate();

                bool ripple = false;
                if (previous_bitrate == desired_bitrate && previous_bitrate > actual_bitrate){
                    RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "Ripple");
                    ripple = true;

                    ripple_order_count = ripple_order_count+1;
                    if (ripple_order_count >= ripple_order){
                        previous_bitrate = actual_bitrate;
                        ripple_order_count = 0;
                    }
                }

                if (desired_bitrate != actual_bitrate && ripple == false){
                    RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "Increasing bitrate");
                    send_package = true;
                }

            } else {
                // Stable situation, maintain current bitrate
                RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "Stable performance, maintaining current bitrate.");
                previous_bitrate = actual_bitrate;
                ripple_order_count = 0;
            }
        }

        // Send bitrate update request if necessary
        if (send_package && allow_transmition_) {
            // Prepare the message for bitrate change
            ffmpeg_image_transport_msgs::msg::ABRInfoPacket abr_info_msg;
            abr_info_msg.role = "client";
            abr_info_msg.msg_type = 1;

            // JSON structure with desired bitrate
            json msg_json = {{"desired_bitrate", desired_bitrate}};
            std::string json_str = msg_json.dump();

            abr_info_msg.msg_json = json_str;

            // Publish message and prevent additional changes until confirmed
            publish_msg_(abr_info_msg);
            send_package = false;
            allow_transmition_ = false;
            voter_buffer_.clear();
        }

    }else{
        return;
    }
}


double AbrComponent::increase_bitrate() {
    /**
     * @brief Increases the current bitrate by moving up the bitrate ladder by a specified factor.
     *
     * This function finds the current `actual_bitrate` in the `bitrate_ladder` and attempts to 
     * move up by `k_factor` positions. If the `k_factor` causes the position to exceed the 
     * available higher bitrates, the function will return the current `actual_bitrate` itself, 
     * indicating that no higher bitrate is available. Otherwise, it will return the new, 
     * increased bitrate.
     *
     * @return The new (increased) bitrate. If no higher bitrate is available, returns the current bitrate.
     */

    // Find the position of `actual_bitrate` within the `bitrate_ladder`
    auto it = std::find(bitrate_ladder.begin(), bitrate_ladder.end(), actual_bitrate);
    if (it == bitrate_ladder.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("AbrComponent"), "Cannot find actual_bitrate (%f) in bitrate_ladder.", actual_bitrate);
        return actual_bitrate;  // Return the current bitrate if not found in `bitrate_ladder`
    }

    // Calculate the target position by advancing `k_factor` positions
    int index = std::distance(bitrate_ladder.begin(), it);
    int desired_index = index + k_factor;

    // Check if the desired position exceeds the ladder's size
    if (desired_index >= static_cast<int>(bitrate_ladder.size())) {
        RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "No higher bitrate configurations available.");
        return actual_bitrate;  // Retorna el bitrate actual si no es posible avanzar
    }

    // Retrieve the desired bitrate at the new position
    double desired_bitrate = bitrate_ladder[desired_index];

    // Log the new bitrate to which the increase occurred
    RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "New desired bitrate: %f", desired_bitrate);

    return desired_bitrate;
}


double AbrComponent::lower_bitrate() {
    /**
     * @brief Lowers the current bitrate by moving down the bitrate ladder by a specified factor.
     *
     * This function finds the closest bitrate in `bitrate_ladder` that is lower than `actual_bitrate`.
     * It then moves down `k_factor` positions in the ladder, or to the lowest available bitrate
     * if `k_factor` exceeds the available lower options. If the current `actual_bitrate` is already 
     * the lowest or no lower bitrate is available, the function will return `actual_bitrate` itself.
     *
     * @return The new (lowered) bitrate. If no lower bitrate is available, returns the current bitrate.
     */


    // Find the closest index in `bitrate_ladder` that is not greater than `actual_bitrate`
    auto it = std::lower_bound(bitrate_ladder.begin(), bitrate_ladder.end(), actual_bitrate);

    // Check if `it` is at the beginning or if `actual_bitrate` is less than the first entry in `bitrate_ladder`
    if (it == bitrate_ladder.begin()) {
        RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "No lower bitrate configurations available for actual_bitrate: %f", actual_bitrate);
        return actual_bitrate;  // Return current bitrate if no lower option is available
    }

    // Move back by `k_factor` positions or to the start of the vector if `k_factor` exceeds available positions
    int index = std::distance(bitrate_ladder.begin(), it);
    int desired_index = (index < k_factor) ? 0 : index - k_factor;  
    double desired_bitrate = bitrate_ladder[desired_index];

    // Log the adjustment made in bitrate
    RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "Lowered bitrate from %f to %f", actual_bitrate, desired_bitrate);

    return desired_bitrate;
}


void AbrComponent::updateUnestabilityBuffer(bool isUnstable) {
    /**
     * @brief Updates the unestability buffer to track the recent stability status of the connection.
     *
     * This function records the stability status in `unestability_buffer_` as a sliding window
     * of size 3. If the connection is considered unstable (`isUnstable` is true), the function 
     * pushes `true` to the buffer, indicating instability; otherwise, it pushes `false`, indicating 
     * stability. The buffer size is limited to 3, and older entries are removed as new ones are added.
     * Additionally, it provides a visual representation of the buffer status and logs the current 
     * count of unstable entries.
     *
     * @param isUnstable Boolean indicating whether the connection is currently unstable (`true`) or stable (`false`).
     */
        if (isUnstable) {
            
            unestability_buffer_.push_back(true);
            if (unestability_buffer_.size() > 3) {
                unestability_buffer_.pop_front();
            }
        } else {
            
            unestability_buffer_.push_back(false);
            if (unestability_buffer_.size() > 3) {
                unestability_buffer_.pop_front();
            }
        }
        
        std::string buffer_visual = "[";
    
        for (const auto& status : unestability_buffer_) {
            buffer_visual += (status ? "X" : ".") + std::string(" ");
        }
        
        buffer_visual = buffer_visual.substr(0, buffer_visual.size() - 1) + "]";

        int true_count = std::count(unestability_buffer_.begin(), unestability_buffer_.end(), true);
        RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "Unestability Buffer Status: %s (Total: %d)", buffer_visual.c_str(), true_count);
}



void AbrComponent::activateEmergencyMode() {

    /**
     * @brief Activates emergency mode by requesting the lowest available bitrate.
     *
     * This function is used to handle critical situations where a significant reduction 
     * in bitrate is necessary to maintain a stable connection. It identifies the lowest 
     * bitrate in the `bitrate_ladder` and sends a request for that bitrate via an 
     * `ABRInfoPacket` message. If the `bitrate_ladder` is empty, the function logs an error
     * and exits without sending a message.
     */

    // Check if `bitrate_ladder` is empty and log an error if so
    if (bitrate_ladder.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("AbrComponent"), "bitrate_ladder is empty. Cannot send emergency bitrate.");
        return;
    }

    // Find the lowest bitrate in the `bitrate_ladder`
    auto lowest_bitrate_it = std::min_element(bitrate_ladder.begin(), bitrate_ladder.end());
    double lowest_bitrate = *lowest_bitrate_it;

    // Log the lowest bitrate found
    RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "Lowest bitrate found: %f", lowest_bitrate);

    // Prepare an ABRInfoPacket to request the lowest bitrate
    ffmpeg_image_transport_msgs::msg::ABRInfoPacket abr_info_msg;
    abr_info_msg.role = "client";
    abr_info_msg.msg_type = 1;

    // Create JSON with the desired bitrate
    json msg_json = {{"desired_bitrate", lowest_bitrate}};
    abr_info_msg.msg_json = msg_json.dump();

    // Publish the emergency bitrate request message
    publish_msg_(abr_info_msg);

    RCLCPP_WARN(
        rclcpp::get_logger("AbrComponent"),
        "Emergency mode activated: requesting lowest bitrate %f Bit/s", lowest_bitrate
    );
}



void AbrComponent::getAndPrintSystemUsage() {
    float cpuUsage, memUsage;

    // CPU Usage
    std::ifstream statFile("/proc/stat");
    std::string line;
    std::getline(statFile, line); 

    std::istringstream ss(line);
    std::string cpu;
    long user, nice, system, idle;
    ss >> cpu >> user >> nice >> system >> idle;

    long totalIdle = idle;
    long totalNonIdle = user + nice + system;
    long total = totalIdle + totalNonIdle;

    static long prevTotal = total;
    static long prevIdle = totalIdle;

    long totalDelta = total - prevTotal;
    long idleDelta = totalIdle - prevIdle;

    prevTotal = total;
    prevIdle = totalIdle;

    cpuUsage = (totalDelta - idleDelta) * 100.0 / totalDelta;

    // Memory Usage
    std::ifstream memFile("/proc/meminfo");
    long totalMemory = 0, freeMemory = 0;
    while (std::getline(memFile, line)) {
        if (line.find("MemTotal:") == 0) {
            sscanf(line.c_str(), "MemTotal: %ld kB", &totalMemory);
        } else if (line.find("MemAvailable:") == 0) {
            sscanf(line.c_str(), "MemAvailable: %ld kB", &freeMemory);
            break;
        }
    }
    long usedMemory = totalMemory - freeMemory;
    memUsage = usedMemory * 100.0 / totalMemory;

    // Print CPU and memory usage
    std::cout << "CPU Usage: " << cpuUsage << "%, "
              << "Memory Usage: " << memUsage << "%" << std::endl;
}


void AbrComponent::initCsvFile(){
    /**
     * @brief Initializes a CSV file for logging ABR component data.
     *
     * If CSV logging is enabled, this function opens a new file named "abrcomponent_csv.csv" 
     * and writes a header row with the column names. The file will be used to log various 
     * parameters related to Adaptive Bitrate (ABR) performance, such as packet size, latency, 
     * and jitter, among others.
     */

    if (getCsv()) {
            // Open CSV file
            log_file_.open("abrcomponent_csv.csv");
            log_file_ << "time_s,size_Mb,latency_s,instant_Mb,mean_Mb,hm_Mb,median_Mb,maximun_Mb,active_conf,jitter,stability_buffer,fps\n"; // CSV size header
            log_file_.flush();
        }

}