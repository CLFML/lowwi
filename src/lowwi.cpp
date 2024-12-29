#include "lowwi.hpp"
#include <iostream>

namespace CLFML::LOWWI
{
    Lowwi::Lowwi(const int mic_index)
    {
        _env = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "test");
        _env.DisableTelemetryEvents();
        _session_opt.SetIntraOpNumThreads(1);
        _session_opt.SetInterOpNumThreads(1);
        _mel = std::make_unique<Melspectrogram>(std::ref(_env), std::ref(_session_opt));
        _emb = std::make_unique<Embedding>(std::ref(_env), std::ref(_session_opt));
        _ww = std::make_unique<WakeWord>(std::ref(_env), std::ref(_session_opt), CLFML_LOWWI_MODEL_DIR"/wakeword.onnx", 0.5f);
    }

    void Lowwi::add_wakeword(const Lowwi_word_t &lowwi_word)
    {
    }
    
    void Lowwi::remove_wakeword(const std::string lowi_word_phrase)
    {
    }
    
    void Lowwi::detect(const std::vector<float> &audio_samples)
    {
        if(audio_samples.empty()) {
            return;
        }
        
        _raw_samples = audio_samples;
        _mel_samples = _mel->convert(std::ref(_raw_samples));
        _feature_samples = _emb->convert(std::ref(_mel_samples));
        if(_ww->detect(std::ref(_feature_samples))) {
            std::cout << "WakeWord detected!" << '\n';
        }        
        _mel_samples.clear();
        _feature_samples.clear();
        _raw_samples.clear();
    }

    Lowwi::~Lowwi()
    {
    }
}