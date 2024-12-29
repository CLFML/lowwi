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

void wakeword_callback(CLFML::LOWWI::Lowwi_cb_t ctx, std::any arg) {

std::cout << "Hey! who said " << ctx.phrase << "?\n";
/* Stop the audio */
audio_async *audio_inst = std::any_cast<audio_async*>(arg); 
if(audio_inst != nullptr) {
    audio_inst->clear();
}
}

int main(int argc, char *argv[]) {
     // Register signal handler
    std::signal(SIGINT, signalHandler);
    audio_async audio(5*1000);
    CLFML::LOWWI::Lowwi detector(0);
    CLFML::LOWWI::Lowwi_word_t ww;
    ww.cbfunc = wakeword_callback;
    ww.cb_arg = &audio;
    ww.model_path = "wakeword.onnx";
    ww.phrase = "Hey Mycroft";
    detector.add_wakeword(ww);
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
