#include "lowwi_melspectrogram.hpp"
#include <numeric>
#include <iostream>

namespace CLFML::LOWWI
{

    void Melspectrogram::init_model_runtime()
    {
        _session = std::make_unique<Ort::Session>(_env, _melspectrogram_model_path, _session_options);
    }

    Melspectrogram::Melspectrogram(Ort::Env &env, Ort::SessionOptions &session_options) : _env(env),
                                                                                          _session_options(session_options),
                                                                                          _mem_info(Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeCPU))
    {
        init_model_runtime();
    }

    std::vector<float> &Melspectrogram::convert(const std::vector<float> &audio_samples)
    {
        _melspectrogram_out.clear();
        std::copy(audio_samples.begin(), audio_samples.end(), std::back_inserter(_samples_to_process));
        while (_samples_to_process.size() >= _melspectrogram_frame_size)
        {
            _input_tensors.push_back(Ort::Value::CreateTensor<float>(
                _mem_info, _samples_to_process.data(), _melspectrogram_frame_size,
                _input_shape.data(), _input_shape.size()));
            _output_tensors = _session->Run(Ort::RunOptions{nullptr}, _input_names.data(),
                                            _input_tensors.data(), _input_names.size(),
                                            _output_names.data(), _output_names.size());
            const auto &mel_out = _output_tensors.front();
            const auto mel_info = mel_out.GetTensorTypeAndShapeInfo();
            const auto mel_shape = mel_info.GetShape();

            const float *mel_data = mel_out.GetTensorData<float>();
            size_t mel_count = std::accumulate(mel_shape.begin(), mel_shape.end(), 1, std::multiplies<>());
            for (size_t i = 0; i < mel_count; i++)
            {
                // Scale mels for Google speech embedding model
                _melspectrogram_out.push_back((mel_data[i] / 10.0f) + 2.0f);
            }

            _samples_to_process.erase(_samples_to_process.begin(),
                                      _samples_to_process.begin() + _melspectrogram_frame_size);
            _output_tensors.clear();
            _input_tensors.clear();
        }
        return _melspectrogram_out;
    }

    Melspectrogram::~Melspectrogram()
    {
    }

}