#ifndef LOWWI_WAKEWORD_HPP
#define LOWWI_WAKEWORD_HPP
#include <cstdint>
#include <vector>
#include <onnxruntime_cxx_api.h>

namespace CLFML::LOWWI
{

    class WakeWord
    {
    public:
        WakeWord(Ort::Env &env, Ort::SessionOptions &session_options, const char *model_path, const float threshold);
        uint8_t detect(const std::vector<float> &features);
        ~WakeWord();
    private:
        const char *_model_path;
        const float _threshold;

        void init_model_runtime();
        Ort::Env &_env;
        Ort::SessionOptions &_session_options;
        std::unique_ptr<Ort::Session> _session;
        Ort::MemoryInfo _mem_info;
        std::array<const char *, 1> _input_names{"onnx::Flatten_0"};
        std::array<const char *, 1> _output_names{"39"};
        std::vector<Ort::Value> _input_tensors;
        std::vector<Ort::Value> _output_tensors;

        std::vector<float> _samples_to_process;
        Ort::AllocatorWithDefaultOptions _allocator;
        const std::array<int64_t, 3> _input_shape{1, (int64_t)16, 96};
    };
}

#endif /* LOWWI_WAKEWORD_HPP */