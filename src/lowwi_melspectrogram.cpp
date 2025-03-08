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

#include "lowwi_melspectrogram.hpp"

namespace CLFML::LOWWI
{
    Melspectrogram::Melspectrogram(Ort::Env &env, Ort::SessionOptions &session_options) : _env(env),
                                                                                          _session_options(session_options),
                                                                                          _mem_info(Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeCPU))
    {
        /*
         * Create new session for our Onnx model
         * Also load the model in :)
         */
        _session = std::make_unique<Ort::Session>(_env, _melspectrogram_model_path.c_str(), _session_options);
    }

    std::vector<float> &Melspectrogram::convert(const std::vector<float> &audio_samples)
    {
        _melspectrogram_out.resize(0);

        _samples_to_process.reserve(audio_samples.size() + _samples_to_process.size());

        /*
         * This could have been done without copying.
         * But copying was a bit safer compared to making the parameter non-const and changing the passed in array directly.
         * The performance implications are not that big, mostly arrays between 5000-10000 floats.
         * Which would theoretically take around 0.008 msec (without cache misses) on modern intel CPU with DDR4.
         * This is almost negligible.
         * But might be interesting to profile in the near future, when doing further optimizations.
         */
        _samples_to_process.insert(_samples_to_process.end(), audio_samples.begin(), audio_samples.end());

        size_t start_idx = 0;

        /* Constants */
        const size_t _melspectrogram_frame_size = 1280 * 4;
        const std::array<const char *, 1> _input_names{"input"};
        const std::array<int64_t, 2> _input_shape{1, (int64_t)1280 * 4};
        const std::array<const char *, 1> _output_names{"output"};

        /*
         * The model is flexible and will take n-amount of samples.
         * However do to the feature model performing better on chunks,
         * The buffer has to be chunked to small segments of 1000-3000 samples at a time
         * The default setting of 1280 samples (*4 for windowing?) seems to work quite well
         */
        while (start_idx + _melspectrogram_frame_size <= _samples_to_process.size())
        {
            /* Load in the samples in to our model inputs */
            auto input_tensor = Ort::Value::CreateTensor<float>(
                _mem_info,
                &_samples_to_process[start_idx],
                _melspectrogram_frame_size,
                _input_shape.data(),
                _input_shape.size());

            /* Run model inference and save the melspectrogram output */
            auto output_tensors = _session->Run(Ort::RunOptions{nullptr},
                                                _input_names.data(),
                                                &input_tensor,
                                                _input_names.size(),
                                                _output_names.data(),
                                                _output_names.size());
            /* Get the output tensor */
            const auto &mel_out = output_tensors.front();
            const auto mel_shape = mel_out.GetTensorTypeAndShapeInfo().GetShape();

            /* Get one dimensional representation of the melspectrogram data */
            const auto *mel_data = mel_out.GetTensorData<float>();

            /* Data is multidimensional, to get the complete number of points we need to do cross-product */
            size_t mel_count =  (mel_shape.at(2) * mel_shape.at(3));

            /* Reserve space beforehand, doing it dynamically in loop is stupid */
            _melspectrogram_out.reserve(_melspectrogram_out.size() + mel_count);

            /* Scale/normalize the melspectrogram to range required for Google embedding model
             * See the paper for this model here: https://arxiv.org/abs/2002.01322
             * Now values will be in range 1.0 to 6.0 dB instead of -10.0 to 40.0 dB 
             */
            std::transform(mel_data, mel_data + mel_count, std::back_inserter(_melspectrogram_out),
                           [](float val)
                           { return (val / 10.0f) + 2.0f; });

            start_idx += _melspectrogram_frame_size;
        }

        if (start_idx > 0)
        {
            _samples_to_process.erase(_samples_to_process.begin(), _samples_to_process.begin() + start_idx);
        }
        return _melspectrogram_out;
    }

    Melspectrogram::~Melspectrogram()
    {
        /* Release the Onnx runtime session */
        _session.release();
    }

}