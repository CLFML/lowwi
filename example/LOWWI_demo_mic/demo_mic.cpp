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

#include <lowwi.hpp>
#include <audio_async.hpp>
#include <iostream>
#include <vector>
#include <cstring>
#include <cstdint>
#include <csignal>

#define AUDIO_DEFAULT_MIC -1
#define AUDIO_SAMPLE_RATE 16000


// Flag to signal when to exit
bool keepRunning= true;

// Signal handler for SIGINT
void signalHandler(int signum)
{
    keepRunning = false;
}

/**
 * @brief Callback that's called when wakeword is triggered 
 */
void wakeword_callback(CLFML::LOWWI::Lowwi_ctx_t ctx, std::shared_ptr<void> arg)
{
    std::cout << "Hey! who said \"" << ctx.phrase << "\"?" << ", I am for " << ctx.confidence << " % certain, I heard something!\n";

    auto audio = std::static_pointer_cast<audio_async>(arg);
    /* Clear the wakeword part from audio, this makes sure we don't process the same audio */
    audio->clear();
}

int main(int argc, char *argv[])
{
    // Register signal handler
    std::signal(SIGINT, signalHandler);
    /* Create new audio_async recorder with 5 seconds of buffering space*/
    std::shared_ptr<audio_async> audio = std::make_shared<audio_async>(5 * 1000);

    /* Create new Lowwi runtime */
    CLFML::LOWWI::Lowwi ww_runtime;

    /* Create new wakeword */
    CLFML::LOWWI::Lowwi_word_t ww;
    ww.cbfunc = wakeword_callback;
    ww.cb_arg = audio;
    ww.model_path = "wakeword.onnx";
    ww.phrase = "Hey Mycroft";
    
    /* Add wakeword to ww-runtime */
    ww_runtime.add_wakeword(ww);

    /* Create audio buffer and reserve some space */
    std::vector<float> audio_buffer;
    audio_buffer.reserve(2560);

    /* Init the audio on default microphone (-1) and with sample_rate of 16KHz */
    audio->init(AUDIO_DEFAULT_MIC, AUDIO_SAMPLE_RATE);
    /* Start audio capture */
    audio->resume();
    /* Clear any samples in queue */
    audio->clear();

    while (keepRunning)
    {
        /* Get 2-seconds of audio */
        audio->get(2000, audio_buffer);
        /* Run the wakeword models */
        ww_runtime.run(audio_buffer);
        /* Clear the audio buffer */
        audio_buffer.clear();
    }

    return 0;
}
