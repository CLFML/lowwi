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
#include <cstdint>
#include <csignal>
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>

// flag to signal when to exit
bool keep_running = true;

// Signal handler for SIGINT
void signal_handler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received. Exiting..." << std::endl;
    keep_running = false;
}

/**
 * @brief Callback that's called when wakeword is triggered 
 */
void wakeword_callback(CLFML::LOWWI::Lowwi_ctx_t ctx, std::shared_ptr<void> arg)
{
    std::cout << "Hey! who said \"" << ctx.phrase << "\"?" << ", I am for " << ctx.confidence << " % certain, I heard something!\n"
              << std::endl;
}

/**
 * @brief Loads in .wav files and stores them in float vector
 */
std::vector<float> load_audio_samples(const std::string &file_path)
{
    // Open the WAV file in binary mode
    std::ifstream file(file_path, std::ios::binary);
    if (!file)
    {
        throw std::runtime_error("Failed to open file: " + file_path);
    }

    // Read the WAV header (44 bytes)
    char header[44];
    file.read(header, 44);
    if (file.gcount() < 44)
    {
        throw std::runtime_error("Invalid WAV file: Header is too short");
    }

    // Validate file format
    if (header[0] != 'R' || header[1] != 'I' || header[2] != 'F' || header[3] != 'F' ||
        header[8] != 'W' || header[9] != 'A' || header[10] != 'V' || header[11] != 'E')
    {
        throw std::runtime_error("Invalid WAV file: Incorrect RIFF or WAVE header");
    }

    // Read audio data as 16-bit signed integers
    std::vector<float> samples;
    while (!file.eof())
    {
        int16_t sample;
        file.read(reinterpret_cast<char *>(&sample), sizeof(int16_t));
        if (file.gcount() == sizeof(int16_t))
        {
            samples.push_back(static_cast<float>(sample));
        }
    }

    return samples;
}

int main(int argc, char *argv[])
{
    // Register signal handler
    std::signal(SIGINT, signal_handler);

    /* Create lowwi instance */
    CLFML::LOWWI::Lowwi ww_runtime;

    /* Configure the wakeword model */
    CLFML::LOWWI::Lowwi_word_t ww;
    ww.phrase = "Hey Mycroft";
    ww.model_path = std::filesystem::path("models/example_wakewords/hey_mycroft.onnx");
    ww.cbfunc = wakeword_callback;

    /* Add the wakeword to the runtime */
    ww_runtime.add_wakeword(ww);

    // Load audio samples from file
    std::vector<float> audio_samples = load_audio_samples(CLFML_LOWWI_EXAMPLE_FRAGMENTS "/hey_mycroft_test.wav");
    std::cout << "Number of samples loaded: " << audio_samples.size() << std::endl;

    while (keep_running)
    {
        /* Fill audio buffer with empty samples */
        std::fill(audio_samples.begin(), audio_samples.end(), 0.0f);
        /* run */
        ww_runtime.run(audio_samples);
        std::cout << "trying empty data; should not activate!" << std::endl;

        /* Sleep for 100 ms */
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        /* Fill the audio buffer with .wav samples */
        audio_samples = load_audio_samples(CLFML_LOWWI_EXAMPLE_FRAGMENTS "/hey_mycroft_test.wav");
        /* run */
        ww_runtime.run(audio_samples);
        std::cout << "trying test data; should activate!" << std::endl;
        
        /* sleep for 2 seconds */
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    return 0;
}
