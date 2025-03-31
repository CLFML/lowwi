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
#include <any>
#include <functional>
#include <lowwi_embedding.hpp>
#include <lowwi_melspectrogram.hpp>
#include <lowwi_wakeword.hpp>
#include <string>

namespace CLFML::LOWWI {
/**
 * @brief Lowwi context which gets passed in to the callback
 * @param phrase The model identifier of the model that was triggered
 * @param confidence The model confidence;
 *                   How certain was the model on the wakeword presence in
 * audio?
 */
typedef struct {
  const std::string phrase;
  const float confidence;
} Lowwi_ctx_t;

/**
 * @brief The user-provided struct which provides the classifier model settings
 * @param phrase The model identifier which get's passed into the callback
 * function when triggered.
 * @param model_path The classifier model path
 * @param cbfunc Function pointer to a callback function that get's called when
 * wakeword is triggered
 * @param cb_arg Additional function argument that get's passed in the callback
 * when wakeword is triggered (void pointer)
 * @param refractory The negative feedback on activation, when activated this
 * factor makes the debouncing work :) Increasing it gives a higher negative
 * bounty, thus dampening any further activations. (Default = 20)
 *
 * @param threshold The threshold determines whether model confidence is worth
 * acting on (default = 0.5f)
 * @param min_activations Number of activations the model should have to be
 * considered detected (Default = 5, but depends on how well the model is
 * trained and how easy to detect) (It's like a debouncing system)
 */
struct Lowwi_word_t {
  std::string phrase = "";
  std::filesystem::path model_path = std::filesystem::path("");
  std::function<void(Lowwi_ctx_t, std::shared_ptr<void>)> cbfunc = nullptr;
  std::shared_ptr<void> cb_arg = nullptr;
  int refractory = 20;
  float threshold = 0.5f;
  float min_activations = 5;
  uint8_t debug = false;
};

class Lowwi {
public:
  Lowwi();

  /**
   * @brief Add new wakeword to detection runtime
   * @param lowwi_word Struct with the properties
   *                   of the to be added wakeword
   */
  void add_wakeword(const Lowwi_word_t &lowwi_word);

  /**
   * @brief Remove wakeword from detection runtime
   * @param model_path Model path of the to be removed wakeword
   */
  void remove_wakeword(std::filesystem::path model_path);

  /**
   * @brief Runs wakeword detection runtime on audio samples
   * @param audio_samples Audio samples to parse
   */
  void run(const std::vector<float> &audio_samples);

  ~Lowwi();

private:
  /**
   * @brief Onnx runtime environment and session options
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
} // namespace CLFML::LOWWI

#endif /* LOWWI_HPP */