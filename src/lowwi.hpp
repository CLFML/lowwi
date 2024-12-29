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
 * This file is part of the Openwakeword.Cpp library
 *
 * Author:          Victor Hogeweij <Hoog-V>
 *
 */

#ifndef LOWWI_HPP
#define LOWWI_HPP
#include <string>
#include <lowwi_melspectrogram.hpp>
#include <lowwi_embedding.hpp>
#include <lowwi_wakeword.hpp>
#include <functional>
#include <any>

namespace CLFML::LOWWI
{
    typedef struct {
        const std::string phrase;
        const int confidence;
    }Lowwi_cb_t;

    typedef struct
    {
        std::string phrase;
        const char *model_path;
        std::function<void(Lowwi_cb_t, std::any)> cbfunc;
        std::any cb_arg;
    } Lowwi_word_t;

    class Lowwi
    {
    public:
        Lowwi(const int mic_index);
        void add_wakeword(const Lowwi_word_t lowwi_word);
        void remove_wakeword(const std::string lowi_word_phrase);
        void detect(const std::vector<float> &audio_samples);
        ~Lowwi();

    private:
        Ort::Env _env;
        Ort::SessionOptions _session_opt;
        std::vector<float> _raw_samples;
        std::vector<float> _mel_samples;
        std::vector<float> _feature_samples;
        
        typedef struct {
            std::unique_ptr<WakeWord> ww_inst;
            Lowwi_word_t properties;
        } wakeword_t;

        std::vector<wakeword_t> _wakewords;
        std::unique_ptr<Melspectrogram> _mel;
        std::unique_ptr<Embedding> _emb;
    };
} /* CLFML:::LOWWI */

#endif /* LOWWI_HPP */