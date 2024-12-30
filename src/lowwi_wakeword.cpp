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

#include "lowwi_wakeword.hpp"
#include <iostream>

namespace CLFML::LOWWI
{

    WakeWord::WakeWord(Ort::Env &env, Ort::SessionOptions &session_options, const char *model_path, const float threshold, const float min_activations, const float refractory, const uint8_t debug) : _env(env),
                                                                                                                                                                                                       _session_options(session_options),
                                                                                                                                                                                                       _mem_info(Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeCPU)),
                                                                                                                                                                                                       _threshold(threshold), _model_path(model_path), _debug(debug), _min_activations(min_activations), _refractory(refractory)
    {
        /*
         * Create new session for our Onnx model
         * Also load the model in :)
         */
        _session = std::make_unique<Ort::Session>(_env, _model_path, _session_options);
    }

    wakeword_result WakeWord::detect(const std::vector<float> &features)
    {
        wakeword_result res{0, 0};

        if (features.size() == 0)
        {
            return res;
        }
        
        /* Reserve the space for the samples */
        _samples_to_process.reserve(features.size()+_samples_to_process.size());

        /* Copy the melspectrogram samples to internal buffer */
        _samples_to_process.insert(_samples_to_process.end(), features.begin(), features.end());

        /* Copy features to internal buffer for further processing */
        std::copy(features.begin(), features.end(), std::back_inserter(_samples_to_process));

        /* Used for scoring the wakeword probability/confidence */
        int activation = 0;
        int num_of_triggers = 0;
        int sum_probability = 0;

        /* Window is fixed by model */
        const size_t _wakeword_feature_window = 16;
        const size_t _wakeword_samples_per_feature = 96;

        size_t sample_idx = 0;
        size_t num_features = _samples_to_process.size() / _wakeword_samples_per_feature;
        while (num_features >= _wakeword_feature_window)
        {
            /* Model constants */
            const std::array<const char *, 1> _input_names{"onnx::Flatten_0"};
            const std::array<const char *, 1> _output_names{"39"};
            const std::array<int64_t, 3> _input_shape{1, (int64_t)16, 96};

            /* Create input tensor from data & input shape */
            auto input_tensor = Ort::Value::CreateTensor<float>(
                _mem_info,
                &_samples_to_process.at(sample_idx),
                (_wakeword_samples_per_feature * _wakeword_feature_window),
                _input_shape.data(),
                _input_shape.size());

            auto output_tensors = _session->Run(Ort::RunOptions{nullptr},
                                                _input_names.data(),
                                                &input_tensor,
                                                1,
                                                _output_names.data(),
                                                1);

            /* Get the output tensor */
            const auto &ww_out = output_tensors.front();
            const auto ww_shape = ww_out.GetTensorTypeAndShapeInfo().GetShape();

            /* Get output data */
            const float *ww_out_data = ww_out.GetTensorData<float>();

            /* Raw pointer; always check NULL! */
            if (ww_out_data != NULL)
            {
                auto probability = ww_out_data[0];

                if (probability > _threshold)
                {
                    if (_debug)
                    {
                        std::cerr << _model_path << " " << probability << '\n';
                    }

                    // Increment activation and check trigger
                    if (++activation >= _min_activations)
                    {
                        res.detected = 1; // Trigger level reached
                        activation = -_refractory; // Reset activation with refractory period
                        sum_probability += float(probability*100);
                        num_of_triggers++;
                    }
                }
                else
                {
                    // Adjust activation towards 0
                    activation += (activation > 0) ? -1 : 1;
                }
            }

            /* Move buffer a step worth of samples */
            sample_idx += _wakeword_samples_per_feature;

            /* Calculate new num_features */
            num_features = (_samples_to_process.size() - sample_idx) / _wakeword_samples_per_feature;
        }

        /* Calculate the avg probability or zero if no wakeword triggers */
        res.confidence = (num_of_triggers > 0) ? (sum_probability/num_of_triggers) : 0;

        if (sample_idx > 0)
        {
            /* erase all processed features */
            _samples_to_process.erase(_samples_to_process.begin(), _samples_to_process.begin() + sample_idx);
        }

        return res;
    }

    WakeWord::~WakeWord()
    {
    }

}