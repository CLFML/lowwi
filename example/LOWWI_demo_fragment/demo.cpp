#include <lowwi.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <cstdint>
#include <lowwi.hpp>
#include <atomic>
#include <thread>
#include <csignal>

// Atomic flag to signal when to exit
std::atomic<bool> keepRunning(true);

// Signal handler for SIGINT
void signalHandler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received. Exiting..." << std::endl;
    keepRunning = false;
}

// Audio callback function
void audioCallback(void *userdata, uint8_t *stream, int len)
{
    std::vector<uint8_t> *audioBuffer = static_cast<std::vector<uint8_t> *>(userdata);
    audioBuffer->insert(audioBuffer->end(), stream, stream + len);
}


std::vector<float> loadAudioSamples(const std::string &filePath) {
    // Open the WAV file in binary mode
    std::ifstream file(filePath, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Failed to open file: " + filePath);
    }

    // Read the WAV header (44 bytes)
    char header[44];
    file.read(header, 44);
    if (file.gcount() < 44) {
        throw std::runtime_error("Invalid WAV file: Header is too short");
    }

    // Validate file format
    if (header[0] != 'R' || header[1] != 'I' || header[2] != 'F' || header[3] != 'F' ||
        header[8] != 'W' || header[9] != 'A' || header[10] != 'V' || header[11] != 'E') {
        throw std::runtime_error("Invalid WAV file: Incorrect RIFF or WAVE header");
    }

    // Read audio data as 16-bit signed integers
    std::vector<float> samples;
    while (!file.eof()) {
        int16_t sample;
        file.read(reinterpret_cast<char *>(&sample), sizeof(int16_t));
        if (file.gcount() == sizeof(int16_t)) {
            samples.push_back(static_cast<float>(sample));
        }
    }


    return samples;
}

int main(int argc, char *argv[]) {
     // Register signal handler
    std::signal(SIGINT, signalHandler);

    CLFML::LOWWI::Lowwi detector(0);

    // Load audio samples from file
    std::vector<float> audioSamples = loadAudioSamples(CLFML_LOWWI_EXAMPLE_FRAGMENTS "/hey_mycroft_test.wav");
    std::cout << "Number of samples loaded: " << audioSamples.size() << std::endl;

    std::vector<float> chunked_audio;
    const int reserved_size = 2560; // 140Ms of audio
    chunked_audio.reserve(reserved_size);
   
    while(keepRunning) {
        std::cout << "trying empty data; should not activate!" << '\n';
        for(uint8_t i =0; i < (audioSamples.size()/reserved_size); i++) {
        std::fill(chunked_audio.begin(), chunked_audio.end(), 0);
        // Perform wake word detection
        detector.detect(chunked_audio);
        chunked_audio.clear();
        }
        std::cout << "trying test data; should activate!" << '\n';
        for(uint8_t i =0; i < (audioSamples.size()/reserved_size); i++) {
        std::copy(audioSamples.begin() + reserved_size*sizeof(float)*i, audioSamples.end() + sizeof(float)*reserved_size*(i+1), std::back_inserter(chunked_audio));
        // Perform wake word detection
        detector.detect(chunked_audio);
        chunked_audio.clear();
    }

     std::this_thread::sleep_for(std::chrono::milliseconds(5000));
     }
 
    return 0;
}
