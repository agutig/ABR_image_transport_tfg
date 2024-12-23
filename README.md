# ABR Image Transport Plugin (H264/AVC)

## Introduction

This project addresses the challenges of video transmission in ROS 2 by implementing an Adaptive Bit Rate (ABR) system. The goal is to enhance video streaming performance, ensuring optimal quality and stability in real-time scenarios, even under challenging network conditions.

This work is part of my Bachelor's Final Project and serves as a foundation for further improvements. Future updates will include additional features such as extended codec support and refined QoS mechanisms, as well as potential adaptations for other use cases within ROS 2 and real-time video applications.

Documentation (Thesis) [ESP]: (link pending of publication)

### ABR Algorithm

The ABR (Adaptive Bit Rate) system dynamically adjusts the bitrate and resolution of video streams based on network conditions. It monitors the available bandwidth and latency, selecting an appropriate bitrate from a pre-defined "bitrate ladder" to ensure smooth transmission. The system uses the harmonic mean of throughput measurements as a predictor for available network capacity, providing a robust mechanism to adapt video quality. When network conditions are stable, the system increases quality; during instability, it reduces the bitrate to avoid buffering or packet loss. This ensures real-time performance and optimal quality tailored to the current network environment.

### ABR Logic

The Adaptive Bitrate (ABR) algorithm implemented in this project is based on throughput estimation. It uses the harmonic mean as a predictor to dynamically select the most suitable bitrate from a predefined bitrate ladder. This approach ensures smooth video streaming by adapting to varying network conditions while minimizing buffering and packet loss.

### Encoder Fork

This project leverages a fork of the repository [h264_image_transport](https://github.com/clydemcqueen/h264_image_transport) for the encoder component. Go check it if you are interested in a non-ABR solution for FFMPEG codecs!

### Video Comparison

A comparison with Theora is available in this video: [Comparison with Theora](https://www.youtube.com/watch?v=8eLEeWR9lw8&t=22s). (Big bunny bucket FHD@30fps)

### Features

- **Adaptive Image Transport Plugin**: Dynamically adjusts bitrate and resolution based on network conditions.
- **H264 Encoding and Decoding**: Efficient codec integration.
- **Fully Configurable**: Allows fine-tuning through JSON parameters (config.json).
- **ROS 2 Humble Compatible**: Designed for seamless integration.

## Usage Guide

## Installation

1. Have preinstalled Image_transport:


2. Install the required dependencies:

   ```bash
   sudo apt update
   sudo apt install -y libavcodec-dev libavformat-dev libavutil-dev libswscale-dev
   ```

3. Clone this repository into your ROS 2 workspace:

   ```bash
   git clone https://github.com/agutig/ABR_image_transport_tfg.git
   cd ABR_image_transport_tfg
   ```

4. Build the project again:

   ```bash
   colcon build
   ```

5. Source the environment:

   ```bash
   source install/setup.bash
   ```

## Configuration

The system supports customization through a JSON configuration file. Below are the key parameters and their explanations:

- **gop_size**: Specifies the Group of Pictures size. A value of 0 means the default value is used by the codec.
- **codecName**: The codec used for encoding. Default is "libx264".
- **target_framerate**: Desired frame rate for the video stream. Example: 24 frames per second.
- **abr_k_factor**: A scaling factor for bitrate adaptation, controlling sensitivity to throughput changes.
- **abr_bitrate_time_window**: The time window (in seconds) for measuring throughput for bitrate estimation.
- **abr_stability_recover_time**: Time (in seconds) for the system to recover stability after abrupt network changes.
- **codec_preset**: Specifies the FFmpeg codec preset for balancing speed and compression. Default is "medium".
- **abr_stability_threshold**: Threshold value to determine when network conditions are stable.
- **abr_similarity_threshold**: Similarity threshold to compare current and previous throughput measurements.
- **abr_emergency_latency_threshold**: Emergency threshold for maximum allowed latency (in seconds).
- **predictor**: The throughput predictor used. Default is "hm" (harmonic mean).
- **ripple_order**: Ripple effect order used for fine-tuning bitrate transitions.
- **csv**: Boolean to enable or disable CSV logging for debugging purposes.
- **republish_data**: Boolean to indicate whether data should be republished.
- **max_rate_mbps**: Maximum bitrate allowed (in Mbps).

### Example configuration

```json
{
    "gop_size": 0,
    "codecName": "libx264",
    "target_framerate": 24,
    "abr_k_factor": 1,
    "abr_bitrate_time_window": 0.5,
    "abr_stability_recover_time": 1,
    "codec_preset": "medium",
    "abr_stability_threshold": 0.3,
    "abr_similarity_threshold": 0.95,
    "abr_emergency_latency_threshold": 0.5,
    "predictor": "hm",
    "ripple_order": 4,
    "csv": false,
    "republish_data": false,
    "max_rate_mbps": 15
}
```

## Future Work

- Optimize for specific robotic use cases.
- Add advanced QoS mechanisms and debugging tools (ML).
- Translate the project documentation (Bachelor's degree Thesis) into English.
- Extend support to additional video codecs.

In general terms, i will keep working on ABR for robotics.

## Author

**Álvaro Gutiérrez García**  
Bachelor's Final Project - Universidad Rey Juan Carlos (EIF)
Contact: https://www.linkedin.com/in/alvaro-gutierrez-garcia-/



