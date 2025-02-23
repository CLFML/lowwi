/*
*  Copyright 2024 (C) Jeroen Veen <ducroq> & Victor Hogeweij <Hoog-V>
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*  http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
*
* This file is part of the Lowwi library
*
* Author:         Victor Hogeweij <Hoog-v>
*
*/

#include <rclcpp/rclcpp.hpp>
#include <audio_async.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "lowwi/srv/get_audio.hpp"

class AudioNode : public rclcpp::Node
{
public:
    AudioNode() : Node("audio_node")
    {
        declare_node_parameters();
    }
    ~AudioNode(){
        audio_->pause();
    }

private:
    void declare_node_parameters(){
        this->declare_parameter("microphone_index", "-1");
        this->declare_parameter("audio_sample_rate","16000");
        this->declare_parameter("audio_buffer_size_msec","5000");
        this->declare_parameter("audio_service_name","/get_microphone_audio");

        mic_index_ = this->get_parameter("microphone_index").as_int();
        int sample_rate = this->get_parameter("audio_sample_rate").as_int();
        int audio_ms = this->get_parameter("audio_buffer_size_msec").as_int();

        audio_ = std::make_shared<audio_async>(audio_ms);
        audio_->init(mic_index_, sample_rate);
        audio_->resume();
        audio_->clear();
        
        // audio_service_ = create_service<audio_node_interface::srv::GetAudio>(
        //     this->get_parameter("audio_service_name").as_string(),
        //     std::bind(&AudioNode::audio_service_cb, this, std::placeholders::_1, std::placeholders::_2)
        // );
    }

    // void audio_service_cb(const std::shared_ptr<audio_node_interface::srv::GetAudio::Request> request,
    //                       std::shared_ptr<audio_node_interface::srv::GetAudio::Response> response)
    // {
    //     audio_samples_.clear();
    //     audio_->get(request->duration_ms, audio_samples_);
        
    //     response->audio_samples.data = audio_samples_;
    //     response->audio_samples.layout.dim.resize(1);
    //     response->audio_samples.layout.dim[0].label = "Samples";
    //     response->audio_samples.layout.dim[0].size = audio_samples_.size();
    // }

    int32_t mic_index_;
    std::shared_ptr<audio_async> audio_;
    rclcpp::Service<lowwi::srv::GetAudio>::SharedPtr audio_service_;
    std::vector<float> audio_samples_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AudioNode>());
    rclcpp::shutdown();
    return 0;
}
