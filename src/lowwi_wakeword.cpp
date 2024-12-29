#include "lowwi_wakeword.hpp"
#include <numeric>
#include <iostream>

namespace CLFML::LOWWI
{

    void WakeWord::init_model_runtime()
    {
        _session = std::make_unique<Ort::Session>(_env, _model_path, _session_options);
    }

    WakeWord::WakeWord(Ort::Env &env, Ort::SessionOptions &session_options, const char *model_path, const float threshold) : _env(env),
                                                                                          _session_options(session_options),
                                                                                          _mem_info(Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeCPU)),
                                                                                          _threshold(threshold), _model_path(model_path)
    {
        init_model_runtime();
    }
    uint8_t WakeWord::detect(const std::vector<float> &features)
    {
        // std::cout << "Got features: " << features.size() << " to process\n";
        if(features.size() == 0) {
            return 0;
        }
        size_t num_buffered_features = 0;
        int activation = 0;
        uint8_t res = 0;
        std::copy(features.begin(), features.end(), std::back_inserter(_samples_to_process));
        num_buffered_features = _samples_to_process.size() / 96;
        while (num_buffered_features >= 16) {
            _input_tensors.push_back(Ort::Value::CreateTensor<float>(
                _mem_info, _samples_to_process.data(), 96*16,
                _input_shape.data(), _input_shape.size()));
            _output_tensors = _session->Run(Ort::RunOptions{nullptr}, _input_names.data(),
                              _input_tensors.data(), 1, _output_names.data(), 1);
            
            const auto &ww_out = _output_tensors.front();
            const auto ww_out_info = ww_out.GetTensorTypeAndShapeInfo();
            const auto ww_out_shape = ww_out_info.GetShape();
            const float *ww_out_data = ww_out.GetTensorData<float>();
            size_t wwOutCount =
                std::accumulate(ww_out_shape.begin(), ww_out_shape.end(), 1, std::multiplies<>());

            for (size_t i = 0; i < wwOutCount; i++)
            {
                auto probability = ww_out_data[i];

                if (probability > _threshold)
                {
                    // Activated
                    activation++;
                    if (activation >= 5)
                    {
                        // Trigger level reached
                        {
                            std::cout <<"Det!\n";
                            res = 1;
                        }
                        activation = -20;
                    }
                }
                else
                {
                    // Back towards 0
                    if (activation > 0)
                    {
                        activation = std::max(0, activation - 1);
                    }
                    else
                    {
                        activation = std::min(0, activation + 1);
                    }
                }
            }

            // Remove 1 embedding
            _samples_to_process.erase(_samples_to_process.begin(),
                               _samples_to_process.begin() + (1 * 96));

            num_buffered_features = _samples_to_process.size() / 96;
            _output_tensors.clear();
            _input_tensors.clear();
        }
        return res;
    }
    
    WakeWord::~WakeWord()
    {
    }

}