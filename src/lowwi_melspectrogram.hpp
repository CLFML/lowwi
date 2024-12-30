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

#ifndef LOWWI_MELSPECTROGRAM_HPP
#define LOWWI_MELSPECTROGRAM_HPP
#include <vector>
#include "onnxruntime_cxx_api.h"
#include <filesystem>

namespace CLFML::LOWWI
{

    class Melspectrogram
    {
    public:

        /**
         * @param env Reference to Onnx environment which the model will run in
         * @param session_options Reference to Onnx options which configure the session for this model
         */
        Melspectrogram(Ort::Env &env, Ort::SessionOptions &session_options);

        /**
         * @brief Convert audio samples to melspectrogram samples
         *        using Onnx Melspectrogram model
         * @param audio_samples Reference to vector which stores your audio samples
         * @return Reference to internal buffer which stores the calculated melspectrogram
         *         samples
         */
        std::vector<float> &convert(const std::vector<float> &audio_samples);

        ~Melspectrogram();

    private:
        /* Onnx variables */
        Ort::Env &_env;
        Ort::SessionOptions &_session_options;
        std::unique_ptr<Ort::Session> _session;
        Ort::MemoryInfo _mem_info;
        Ort::AllocatorWithDefaultOptions _allocator;

        std::vector<float> _samples_to_process;
        std::vector<float> _melspectrogram_out;

        const std::filesystem::path _melspectrogram_model_path = "models/melspectrogram.onnx";

    };

}

#endif /* LOWWI_MELSPECTROGRAM_HPP */