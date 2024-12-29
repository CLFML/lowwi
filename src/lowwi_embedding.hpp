#ifndef LOWWI_EMBEDDING_HPP
#define LOWWI_EMBEDDING_HPP
#include <vector>
#include <onnxruntime_cxx_api.h>
namespace CLFML::LOWWI
{

    class Embedding
    {
    public:
        Embedding(Ort::Env &env, Ort::SessionOptions &session_options);
        std::vector<float> &convert(std::vector<float> &mels_in);
        ~Embedding();

    private:
        void init_model_runtime();

        Ort::Env &_env;
        Ort::SessionOptions &_session_options;
        std::unique_ptr<Ort::Session> _session;
        Ort::MemoryInfo _mem_info;
        std::array<const char *, 1> _input_names{"input_1"};
        std::array<const char *, 1> _output_names{"conv2d_19"};
        std::vector<Ort::Value> _input_tensors;
        std::vector<Ort::Value> _output_tensors;

        std::vector<float> _samples_to_process;
        std::vector<float> _embedding_out;
        Ort::AllocatorWithDefaultOptions _allocator;

        const char *_embedding_model_path = CLFML_LOWWI_MODEL_DIR "/embedding_model.onnx";
        const size_t _embedding_frame_size = 1280 * 4;
        const std::array<int64_t, 4> _input_shape{1, (int64_t)76, 32, 1};
    };
}
#endif /* LOWWI_EMBEDDING_HPP */