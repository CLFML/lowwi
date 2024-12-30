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
        const float confidence;
    }Lowwi_ctx_t;

    struct Lowwi_word_t
    {
        std::string phrase = "";
        std::filesystem::path model_path = std::filesystem::path("");
        std::function<void(Lowwi_ctx_t, std::shared_ptr<void>)> cbfunc = nullptr;
        std::shared_ptr<void> cb_arg = nullptr;
        int refractory = 20;
        float threshold = 0.5f;
        float min_activations = 5;
        uint8_t debug = false;

    } ;

    class Lowwi
    {
    public:
        Lowwi();

        /**
        * @brief Add new wakeword to detection runtime
        * @param lowwi_word Struct with the properties 
        *                   of the to be added wakeword
        */
        void add_wakeword(const Lowwi_word_t& lowwi_word);

        /**
         * @brief Remove wakeword from detection runtime
         * @param model_path Model path of the to be removed wakeword
         */
        void remove_wakeword(const char* model_path);

        /**
         * @brief Runs wakeword detection runtime on audio samples
         * @param audio_samples Audio samples to parse
         */
        void run(const std::vector<float> &audio_samples);

        ~Lowwi();

    private:
        /**
         * @brief Onnx runtime environment and session otpions
         *        The melspectrogram, feature and wakeword models
         *        all share the same environment and session options.
         */
        Ort::Env _env;
        Ort::SessionOptions _session_opt;

        /**
         * @brief Vectors for the melspectrogram & feature samples.
         */
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