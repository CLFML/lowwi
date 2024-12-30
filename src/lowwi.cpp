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

#include "lowwi.hpp"
#include <iostream>
#include <exception>

namespace CLFML::LOWWI
{
    Lowwi::Lowwi()
    {
        _env = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "LOWWI_Runtime");
        _env.DisableTelemetryEvents();
        _session_opt.SetIntraOpNumThreads(1);
        _session_opt.SetInterOpNumThreads(1);
        _mel = std::make_unique<Melspectrogram>(_env, _session_opt);
        _emb = std::make_unique<Embedding>(_env, _session_opt);
    }

    void Lowwi::add_wakeword(const Lowwi_word_t& lowwi_word)
    {
        if(!lowwi_word.cbfunc) {
            throw std::runtime_error("[LOWI]: ERROR! No callback function defined for lowwi_word!");
        }
        wakeword_t wakeword {
            std::make_unique<WakeWord>(_env, _session_opt, lowwi_word.model_path, lowwi_word.threshold, lowwi_word.min_activations, lowwi_word.refractory, lowwi_word.debug),
            lowwi_word,
        };
        _wakewords.push_back(std::move(wakeword));
    }
    
    void Lowwi::remove_wakeword(const char *model_path)
    {
    }
    
    void Lowwi::run(const std::vector<float> &audio_samples)
    {
        if(audio_samples.empty()) {
            return;
        }
        
        _mel_samples = _mel->convert(std::ref(audio_samples));
        _feature_samples = _emb->convert(std::ref(_mel_samples));
        for (auto &ww : _wakewords) {
            wakeword_result res = ww.ww_inst->detect(_feature_samples);
            if(res.detected) {
                Lowwi_ctx_t cb = {ww.properties.phrase, res.confidence};
                ww.properties.cbfunc(cb, ww.properties.cb_arg);
            }
        }
        _mel_samples.resize(0);
        _feature_samples.resize(0);
    }

    Lowwi::~Lowwi()
    {
    }
}