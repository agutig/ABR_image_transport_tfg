# ABR Image Transport Plugin (H264/AVC)

## Introduction

An  **Adaptive Bitrate (ABR) system** dynamically adjusts the bitrate and resolution of video streams in real-time based on network conditions. By monitoring the transmition, it selects the most suitable bitrate from a predefined **"bitrate ladder"** to ensure smooth and reliable transmission. This approach guarantees optimal video quality tailored to the current network environment.

This project addresses the challenges of video transmission in ROS 2 by implementing an ABR system for the image_transport package (ROS 2 Humble). The goal is to enhance video streaming performance, ensuring optimal quality and stability in real-time scenarios, even under challenging network conditions (Ex: radiofrequency). This makes it ideal for teleoperated robots.


This work is part of my Bachelor's Final Project (Thesis). Future updates will include additional features such as extended codec support and refined ABR mechanisms, as well as potential adaptations for other use cases within ROS 2 and real-time video applications. This repository keeps the initial code for the thesis proposal. For ongoing development and maintenance, please refer to the [ABR_image_transport](https://github.com/agutig/ABR_image_transport) repository.

Documentation (Thesis v1) [ESP]: [link](https://github.com/agutig/media_repository/blob/main/tfg/memoria.pdf) (official link pending of publication)


#### Key Features

- **Image transport Plugin**:  
  This plugin facilitates the efficient transmission of images and videos over ROS 2 topics. It leverages adaptive bitrate techniques to ensure optimal performance in real-time streaming scenarios, dynamically adjusting to varying network conditions. It is implemented as an **Image Transport Plugin** to maintain compatibility and adhere to the ROS 2 standard, allowing seamless integration with existing ROS 2 tools and workflows.

- **Dynamic Adaptation**:  
  Starts streaming at the lowest quality (lowest bitrate) and progressively increases the quality as network stability improves. During instability, it reduces the bitrate to avoid framerate penalties, maintaining real-time performance.

- **Throughput Estimation**:  
  Utilizes the harmonic mean of throughput measurements to predict available network capacity, ensuring robust quality adaptation.

- **H.264 Codec Integration**:  
  Delivers efficient video encoding and decoding for high-quality compression and playback.

- **High Framerate Support**: 
  Supports framerates up to 120 fps, enabling smooth streaming for high-performance applications.

- **Full Configurability**:  
  Supports fine-tuning of parameters through a customizable JSON configuration file (`config.json`).

- **Configurable Bitrate Ladder**: 
  The predefined bitrate ladder can be customized via a configuration file (`bitrate_ladder.json`).

- **ROS 2 Humble Compatibility**:  
  Designed for seamless integration with ROS 2 systems.

- **Non-Reliable Transmission Optimization**:  
  The system has been specifically optimized to operate in a **non-reliable** mode, which reduces latency and ensures real-time responsiveness. By prioritizing data flow over guaranteed delivery, this mode is particularly suited for scenarios where timely data transmission is critical, such as live video streaming and robotic control applications.

- **Bidirectional Communication**:  
  The system supports bidirectional communication through two interlinked plugins, enabling dynamic feedback between the publisher and subscriber. This ensures synchronized transmission adjustments and real-time responsiveness, particularly useful for interactive robotic and video streaming applications.


### Base Fork

This project leverages a fork of the repository [ffmpeg_image_transport](https://github.com/ros-misc-utilities/ffmpeg_image_transport) for the encoder component. Go check it if you are interested in a non-ABR solution for FFMPEG codecs!


### Video Comparison
A comparison with Theora is available in this video: [Comparison with Theora](https://www.youtube.com/watch?v=8eLEeWR9lw8). (Big bunny bucket FHD@30fps)

[![Watch the video](https://img.youtube.com/vi/8eLEeWR9lw8/0.jpg)](https://www.youtube.com/watch?v=8eLEeWR9lw8)



## Usage Guide

### Installation

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

### Configuration (config.json)

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
- **predictor**: The throughput predictor used. Default is "hm" (harmonic mean) but "mean", "median" can be selected.
- **ripple_order**: Ripple effect order used for fine-tuning bitrate transitions.
- **csv**: Boolean to enable or disable CSV logging for debugging purposes.
- **republish_data**: Boolean to indicate whether data should be republished.
- **max_rate_mbps**: Maximum bitrate allowed (in Mbps).

#### Example configuration

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
- Add advanced QoS mechanisms and debugging tools (ML/DL).
- Translate the project documentation (Bachelor's degree Thesis) into English.
- Extend support to additional video codecs.

This future work on ABR for robotics will be developed in the repository: [ABR_image_transport](https://github.com/agutig/ABR_image_transport_tfg).

## Author

**Álvaro Gutiérrez García**  
Contact: [Linkedin](https://www.linkedin.com/in/alvaro-gutierrez-garcia-/)

Bachelor's Final Project - Universidad Rey Juan Carlos (EIF)



