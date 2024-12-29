#include "lowwi_embedding.hpp"
#include <numeric>

namespace CLFML::LOWWI
{

    void Embedding::init_model_runtime()
    {
        _session = std::make_unique<Ort::Session>(_env, _embedding_model_path, _session_options);
    }

    Embedding::Embedding(Ort::Env &env, Ort::SessionOptions &session_options) : _env(env),
                                                                                _session_options(session_options),
                                                                                _mem_info(Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeCPU))
    {
        init_model_runtime();
    }

    std::vector<float> &Embedding::convert(std::vector<float> &mels_in)
    {
        _embedding_out.clear();
        size_t mel_frames = 0;
        std::copy(mels_in.begin(), mels_in.end(), std::back_inserter(_samples_to_process));
        mel_frames = _samples_to_process.size() / 32;
        while (mel_frames >= 76) {
            _input_tensors.push_back(Ort::Value::CreateTensor<float>(
                _mem_info, _samples_to_process.data(), 76*32,
                _input_shape.data(), _input_shape.size()));
            _output_tensors = _session->Run(Ort::RunOptions{nullptr}, _input_names.data(),
                                            _input_tensors.data(), _input_names.size(),
                                            _output_names.data(), _output_names.size());
            const auto &emb_out = _output_tensors.front();
            const auto emb_info = emb_out.GetTensorTypeAndShapeInfo();
            const auto emb_shape = emb_info.GetShape();
            const float *emb_out_data = emb_out.GetTensorData<float>();
            size_t emb_out_count = std::accumulate(emb_shape.begin(), emb_shape.end(), 1, std::multiplies<>());
            std::copy(emb_out_data, emb_out_data + emb_out_count, std::back_inserter(_embedding_out));
            _samples_to_process.erase(_samples_to_process.begin(), _samples_to_process.begin() + (256));
            mel_frames = _samples_to_process.size() / 32;
            _output_tensors.clear();
            _input_tensors.clear();
        }
        return _embedding_out;
    }

    Embedding::~Embedding()
    {
    }

}