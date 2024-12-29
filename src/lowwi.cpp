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
    }

    void Lowwi::add_wakeword(const Lowwi_word_t lowwi_word)
    {
        wakeword_t wakeword = {
            .ww_inst = std::make_unique<WakeWord>(std::ref(_env), std::ref(_session_opt), lowwi_word.model_path, 0.5f),
            .properties = lowwi_word,
        };
        _wakewords.push_back(std::move(wakeword));
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
        for (auto &ww : _wakewords) {
            if(ww.ww_inst->detect(_feature_samples)) {
                Lowwi_cb_t cb = {.phrase = ww.properties.phrase, .confidence = 1};
                ww.properties.cbfunc(cb, ww.properties.cb_arg);
            }
        }
        _mel_samples.clear();
        _feature_samples.clear();
        _raw_samples.clear();
    }

    Lowwi::~Lowwi()
    {
    }
}