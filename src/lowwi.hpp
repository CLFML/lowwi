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

namespace CLFML::LOWWI
{
    typedef struct
    {
        const std::string phrase;
        const std::string model_path;
        void *cb_func;
    } Lowwi_word_t;

    class Lowwi
    {
    public:
        Lowwi(const int mic_index);
        void add_wakeword(const Lowwi_word_t &lowwi_word);
        void remove_wakeword(const std::string lowi_word_phrase);
        void detect(const std::vector<float> &audio_samples);
        ~Lowwi();

    private:
        void melspect_thread();
        void embedding_thread();
        void lowwi_thread();
        Ort::Env _env;
        Ort::SessionOptions _session_opt;
        std::vector<float> _raw_samples;
        std::vector<float> _mel_samples;
        std::vector<float> _feature_samples;
        std::unique_ptr<Melspectrogram> _mel;
        std::unique_ptr<Embedding> _emb;
        std::unique_ptr<WakeWord> _ww;
    };
} /* CLFML:::LOWWI */

#endif /* LOWWI_HPP */