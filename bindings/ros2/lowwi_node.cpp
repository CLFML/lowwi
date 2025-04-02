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
#include <lowwi.hpp>
#include <cstdint>
#include <csignal>
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include "audio_tools/msg/audio_data.hpp"
#include "audio_tools/msg/audio_data_stamped.hpp"
#include "audio_tools/msg/audio_info.hpp"

/**
 * @brief Callback that's called when wakeword is triggered 
 */
static void wakeword_callback(CLFML::LOWWI::Lowwi_ctx_t ctx, std::shared_ptr<void> arg)
{
    std::cout << "Hey! who said \"" << ctx.phrase << "\"?" << ", I am for " << ctx.confidence << " % certain, I heard something!\n"
              << std::endl;
}


class LowwiNode : public rclcpp::Node
{
public:
    LowwiNode() : Node("lowwi_node"), _ww_runtime()
    {
        RCLCPP_INFO(this->get_logger(), "Lowwi node: init\n");
        this->declare_parameter<std::vector<std::string>>("wakeword.phrases");
        this->declare_parameter<std::vector<std::string>>("wakeword.models");
        this->declare_parameter<std::vector<int64_t>>("wakeword.min_activations");
        
        _samples.reserve(50000);
        load_wakewords();
        subscription_ = this->create_subscription<audio_tools::msg::AudioDataStamped>(
            "/audio_stamped",
            _qos,
            std::bind(&LowwiNode::audio_cb, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", subscription_->get_topic_name());


    
    }


private:
void load_wakewords()
{
    auto phrases = this->get_parameter("wakeword.phrases").as_string_array();
    auto models = this->get_parameter("wakeword.models").as_string_array();
    auto min_activations = this->get_parameter("wakeword.min_activations").as_integer_array();

    size_t count = phrases.size();

    if (models.size() != count || min_activations.size() != count) {
        RCLCPP_ERROR(this->get_logger(), "Wakeword parameter arrays must all be the same size.");
        return;
    }

    for (size_t i = 0; i < count; ++i) {
        CLFML::LOWWI::Lowwi_word_t w;
        w.phrase = phrases[i];
        w.model_path = models[i];
        w.min_activations = static_cast<int>(min_activations[i]);
        w.cbfunc = wakeword_callback;

        _ww_runtime.add_wakeword(w);

        RCLCPP_INFO(this->get_logger(), "Added wakeword: '%s' → %s", w.phrase.c_str(), w.model_path.c_str());
    }
}

void convert_s16le_to_float(const std::vector<uint8_t>& byte_data)
{
    if (byte_data.size() % 2 != 0) {
        RCLCPP_WARN(this->get_logger(), "Audio data size not divisible by 2. Dropping frame.");
        return;
    }

    const size_t num_samples = byte_data.size() / 2;

    const int16_t* raw_samples = reinterpret_cast<const int16_t*>(byte_data.data());

    // Resize (not reserve) so we can write directly into _samples
    _samples.resize(num_samples);

    _samples.assign(raw_samples, raw_samples + num_samples);
}

void audio_cb(const audio_tools::msg::AudioDataStamped::SharedPtr msg)
{
    const auto& audio_data = msg->audio.data;

    const std::vector<uint8_t>& s16le_bytes = msg->audio.data;

    // Convert to float32 range [-1, 1]
    convert_s16le_to_float(s16le_bytes);
    _ww_runtime.run(_samples);
}
    rclcpp::Subscription<audio_tools::msg::AudioDataStamped>::SharedPtr subscription_;

    /* Configure the wakeword model */
    CLFML::LOWWI::Lowwi_word_t _ww;

    /* Declare the runtime as a class member */
    CLFML::LOWWI::Lowwi _ww_runtime;
    std::vector<float> _samples;
    rclcpp::SensorDataQoS _qos;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LowwiNode>());
    rclcpp::shutdown();
    return 0;
}