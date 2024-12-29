#ifndef LOWWI_MELSPECTROGRAM_HPP
#define LOWWI_MELSPECTROGRAM_HPP
#include <vector>
#include "onnxruntime_cxx_api.h"

namespace CLFML::LOWWI {

class Melspectrogram
{
    public:
    Melspectrogram(Ort::Env &env, Ort::SessionOptions &session_options);
    std::vector<float> &convert(const std::vector<float> &audio_samples);
    ~Melspectrogram();

    private:
    void init_model_runtime();
    Ort::Env &_env;
    Ort::SessionOptions &_session_options;
    std::unique_ptr<Ort::Session> _session;
    Ort::MemoryInfo _mem_info;
    std::array<const char*, 1> _input_names{"input"};
    std::array<const char*, 1> _output_names{"output"};
    std::vector<Ort::Value> _input_tensors;
    std::vector<Ort::Value> _output_tensors;

    std::vector<float> _samples_to_process;
    std::vector<float> _melspectrogram_out;
    Ort::AllocatorWithDefaultOptions _allocator;

    const char *_melspectrogram_model_path = CLFML_LOWWI_MODEL_DIR"/melspectrogram.onnx";
    const size_t _melspectrogram_frame_size = 1280 * 4;
    const std::array<int64_t, 2> _input_shape{1, (int64_t)1280*4};
};

}

#endif /* LOWWI_MELSPECTROGRAM_HPP */