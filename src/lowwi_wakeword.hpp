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
 * Author:          Victor Hogeweij <Hoog-V>
 *
 */

#ifndef LOWWI_WAKEWORD_HPP
#define LOWWI_WAKEWORD_HPP
#include <cstdint>
#include <vector>
#include <onnxruntime_cxx_api.h>
#include <filesystem>

namespace CLFML::LOWWI
{
    /* Wakeword result struct returned by the detect function */
    struct wakeword_result {
        uint8_t detected;
        float confidence;
    };

    class WakeWord
    {
    public:
        /**
          * @brief Prevent this class from being used with copy constructor
          */
         WakeWord(const WakeWord&) = delete;
         
        /**
          * @brief Prevent this class from being used with assignment constructor
          */
         WakeWord& operator=(const WakeWord&) = delete;

        /**
         * @brief Create new wakeword
         * @param env Onnx environment to run the model in
         * @param session_options configuration settings for the session runtime
         * @param threshold The threshold determines whether model confidence is worth acting on (default = 0.5f)
         * @param min_acivations Number of activations the model should have to be considered detected
         *                       (Default = 5, but depends on how well the model is trained and how easy to detect)
         *                       (It's like a debouncing system)
         * @param refractory The negative feedback on activation, when activated this factor makes the debouncing work :)
         *                   Increasing it gives a higher negative bounty, thus dampening any further activations.
         *                   (Default = 20)
         * @param debug      Prints the confidence factor (even when word is not triggered) to console.
         */
        WakeWord(Ort::Env &env, 
                 Ort::SessionOptions &session_options, 
                 const std::filesystem::path model_path, 
                 const float threshold, 
                 const float min_activations, 
                 const int refractory, 
                 const uint8_t debug);

        /**
         * @brief Runs the wakeword model on the features
         * @param features Reference to vector containing the features prepared with embedding model
         * @return wakeword_result struct with average confidence factor and detected flag
         */
        wakeword_result detect(const std::vector<float> &features);

        ~WakeWord();

    private:
        Ort::Env &_env;
        Ort::SessionOptions &_session_options;
        std::unique_ptr<Ort::Session> _session;
        Ort::MemoryInfo _mem_info;
        Ort::AllocatorWithDefaultOptions _allocator;

        std::vector<float> _samples_to_process;
        
        uint8_t _debug = false;
        const std::filesystem::path _model_path;
        const float _threshold;
        const float _min_activations;
        const int _refractory;
    };
}

#endif /* LOWWI_WAKEWORD_HPP */