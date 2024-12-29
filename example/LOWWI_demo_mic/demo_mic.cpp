#include <lowwi.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <cstdint>
#include <lowwi.hpp>
#include <SDL.h>
#include <atomic>
#include <thread>
#include <csignal>
#include <audio_async.hpp>
// Atomic flag to signal when to exit
std::atomic<bool> keepRunning(true);

// Signal handler for SIGINT
void signalHandler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received. Exiting..." << std::endl;
    keepRunning = false;
}

int main(int argc, char *argv[]) {
     // Register signal handler
    std::signal(SIGINT, signalHandler);

    CLFML::LOWWI::Lowwi detector(0);
    audio_async audio(5*1000);
    std::vector<float> audioBuffer;
    audioBuffer.reserve(2560);
    audio.init(-1, 16000);
    audio.resume();
    audio.clear();
    while (keepRunning)
    {
        audio.get(2000, audioBuffer);
        detector.detect(audioBuffer);
        audioBuffer.clear();
    }


    return 0;
}
