#ifndef ABR_COMPONENT_H_
#define ABR_COMPONENT_H_

#include <rclcpp/rclcpp.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp> 
#include "ffmpeg_image_transport_msgs/msg/abr_info_packet.hpp"
#include <nlohmann/json.hpp>
#include <deque> 
#include <fstream>

using json = nlohmann::json;

class AbrComponent
{
public:
    AbrComponent();
    ~AbrComponent();

    void reset();

    // Método para recibir el mensaje, serializarlo y calcular su tamaño
    void analyzeFlow(const ffmpeg_image_transport_msgs::msg::FFMPEGPacket &msg, rclcpp::Node * node_);
    double calculateHarmonicMean(); 
    double calculateMean();
    double calculateMedian();
    double calculateJitter();
    std::pair<double, double> calculateLinearVoterSlope();
    double lower_bitrate();
    double increase_bitrate();
    void abr_logic1(float predicted_bitrate);
    void abr_logic2();

    //bitrate_ladder public variables
    std::vector<double> bitrate_ladder;
    double actual_bitrate = 0.0;
    double previous_bitrate = -1.0;
    std::function<void(const ffmpeg_image_transport_msgs::msg::ABRInfoPacket&)>  publish_msg_;
    bool allow_transmition_ = false;
    bool force_fixed_resolution = true; //readjust dimention to always fit the original desired one.

    rclcpp::Time date_to_refresh;

    void setKFactor(int value) { k_factor = value; }
    void setBitrateTimeWindow(double value) { bitrate_time_window = value; }
    void setBitrateBufferSize(int value) { bitrate_buffer_size = value; }
    void setVoterTimeWindow(double value) { bitrate_time_window = value; }
    void setVoterBufferSize(int value) { bitrate_buffer_size = value; }
    void setfps(int value) { fps = value; }
    void setStabilityTime(double value){stability_recover_time = value;}
    void setForceResolution(bool value){force_fixed_resolution = value;}
    void setDateRefhesh(rclcpp::Time value){date_to_refresh = value;}
    void setCsv(bool value){csv = value;}
    void setStartTime(rclcpp::Time value){start_time = value;}
    void setPreviousTimeStamp(rclcpp::Time value){previous_timeStamp = value;}
    void setStabilityThreshold(double value){stability_threshold = value;}
    void setSimilarityThreshold(double value){similarity_threshold = value;}
    void setEmergencyTresh(double value){ emergency_latency_threshold = value;}
    void setPredictor(std::string value){abr_predictor = value;}
    void setRepublishData(bool republish_data) { republish_data_ = republish_data; }
    void setRippleOrder(int value) { ripple_order = value; }


    int getKFactor() const { return k_factor; }
    double getBitrateTimeWindow() const { return bitrate_time_window; }
    int getBitrateBufferSize() const { return bitrate_buffer_size; }
    double getVoterTimeWindow() const { return bitrate_time_window; }
    int getVoterBufferSize() const { return bitrate_buffer_size; }
    int getfps() const { return fps; }
    int getStabilityTime(){ return stability_recover_time;}
    bool getForceResolution(){return  force_fixed_resolution;}
    rclcpp::Time getDateRefresh(){return date_to_refresh;}
    bool getCsv(){return csv;}
    rclcpp::Time getStartTime(){return start_time;}
    double getStabilityThreshold(){return stability_threshold; }
    double getSimilarityThreshold(){return similarity_threshold;}
    double getEmergencyTresh(){return emergency_latency_threshold;}
    std::string getPredictor(){return abr_predictor;}
    bool getRepublishData() const { return republish_data_; }
    int getRippleOrder() const { return ripple_order; }

    void reconfigureBuffers(); 
    void getAndPrintSystemUsage();
    void initCsvFile();
    void activateEmergencyMode();
    void updateUnestabilityBuffer(bool isUnstable);

    int bitrate_buffer_position = 0; 
    std::deque<double> bitrate_buffer_;  
    std::deque<double> latency_buffer_; 
    std::deque<double> voter_buffer_; 
    std::deque<bool> unestability_buffer_;

    std::ofstream log_file_;
    bool csv = false;
    rclcpp::Time start_time; 
    rclcpp::Time previous_timeStamp; 
    double measured_fps = 30;

private:

    //Values:
    int k_factor = 1;      //Number of steps that the algorithim moved up or down
    std::string abr_predictor = "hm";
    double stability_recover_time = 1.0;
    double stability_threshold = 0.3;
    double similarity_threshold = 0.95;

    double ideal_expected_bitrate = 100000000;

    double emergency_latency_threshold = 5.0;
    bool emergency_mode = false;

    // Vector para almacenar las últimas 5 latencias
    int fps = 30;

    // Fine grain (n+1 prediction)
    double bitrate_time_window = 0.5; //seconds of SLIDING time window --> buffer size in seconds
    int bitrate_buffer_size = std::max(1, static_cast<int>(std::ceil(fps * bitrate_time_window))); //harmomic mean Bufer size in samples.
    
    int ripple_order = 0;
    int ripple_order_count = 0;

    bool republish_data_ = false;



};

#endif // ABR_COMPONENT_H_
