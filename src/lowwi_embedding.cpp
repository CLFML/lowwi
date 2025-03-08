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

#include "lowwi_embedding.hpp"

namespace CLFML::LOWWI
{
    Embedding::Embedding(Ort::Env &env, Ort::SessionOptions &session_options) : _env(env),
                                                                                _session_options(session_options),
                                                                                _mem_info(Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeCPU))
    {
        _session = std::make_unique<Ort::Session>(_env, _embedding_model_path.c_str(), _session_options);
    }

    std::vector<float> &Embedding::convert(const std::vector<float> &mels_in)
    {
        /* Clear any old samples before proceeding */
        _embedding_out.resize(0);

        /* Reserve the space for the samples */
        _samples_to_process.reserve(mels_in.size()+_samples_to_process.size());

        /* Copy the melspectrogram samples to internal buffer */
        _samples_to_process.insert(_samples_to_process.end(), mels_in.begin(), mels_in.end());

        /*
         * At a 16 kHz sampling rate,
         * the total duration of 76 frames (with appropriate hop size) should match 775 ms.
         */
        const size_t _embedding_window_size = 76;
        const size_t _embedding_mels_per_frame = 32; // fixed by model
        const size_t _embedding_step_size = 8;       // 80 ms

        /* Calculate the amount of mel_frames to process */
        size_t mel_frames = _samples_to_process.size() / _embedding_mels_per_frame;
        size_t sample_idx = 0;
        while (mel_frames >= _embedding_window_size)
        {
            /* Model constants */
            const std::array<const char *, 1> _input_names{"input_1"};
            const std::array<const char *, 1> _output_names{"conv2d_19"};
            const std::array<int64_t, 4> _input_shape{1, (int64_t)76, 32, 1};
            
            /* Create input tensor from data & input shape */
            auto input_tensor = Ort::Value::CreateTensor<float>(
                _mem_info,
                &_samples_to_process.at(sample_idx),
                (_embedding_window_size * _embedding_mels_per_frame),
                _input_shape.data(),
                _input_shape.size());

            auto output_tensors = _session->Run(Ort::RunOptions{nullptr},
                                                _input_names.data(),
                                                &input_tensor,
                                                _input_names.size(),
                                                _output_names.data(),
                                                _output_names.size());

            /* Get the output tensor */
            const auto &emb_out = output_tensors.front();
            const auto emb_shape = emb_out.GetTensorTypeAndShapeInfo().GetShape();

            /* Get one dimensional representation of the embedding data */
            const float *emb_out_data = emb_out.GetTensorData<float>();

            /* Small gotcha! Reserving before copy saves so much time! */
            int64_t emb_out_count = emb_shape.at(3);

            /* Return early if we got a negative number, which would cause severe memory issues! */
            if(emb_out_count < 0) {
                return _embedding_out;
            }

            _embedding_out.reserve(_embedding_out.size() + emb_out_count);

            /* Copy the embedding data from output tensor to internal buffer */
            std::copy(emb_out_data, emb_out_data + emb_out_count, std::back_inserter(_embedding_out));

            /* Move buffer a step worth of samples (80 ms) */
            sample_idx += _embedding_mels_per_frame * _embedding_step_size;

            /* Calculate new melframes */
            mel_frames = (_samples_to_process.size() - sample_idx) / _embedding_mels_per_frame;
        }

        if (sample_idx > 0)
        {
            /* erase all processed samples */
            _samples_to_process.erase(_samples_to_process.begin(), _samples_to_process.begin() + sample_idx);
        }

        return _embedding_out;
    }

    Embedding::~Embedding()
    {
    }

}