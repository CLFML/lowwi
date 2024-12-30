# Library implementation details

Here some brief details as how the library is implemented;

## Architecture

The whole runtime consists of three stages:

1. Melspectrogram; A pre-processing model that computes a melspectrogram of the input audio data. This library uses an ONNX implementation of Torch's melspectrogram function with fixed parameters which enables efficient performance across devices. This onnx model is provided by [dscripka](https://github.com/dscripka/openWakeWord).

2. Embedding/feature generation; A shared feature extraction backbone model that converts melspectrogram inputs into general-purpose speech audio embeddings. This model is provided by Google as a TFHub module under an Apache-2.0 license. For openWakeWord, this model was manually re-implemented to separate out different functionality and allow for more control of architecture modifications compared to a TFHub module. The model itself is series of relatively simple convolutional blocks, and gains its strong performance from extensive pre-training on large amounts of data. This model is the core component of openWakeWord, and enables the strong performance that is seen even when training on fully-synthetic data.

3. Feature Classification: A classification model that follows the shared (and frozen) feature extraction model. The structure of this classification model is arbitrary, but in practice a simple fully-connected network or 2 layer RNN works well.


## Code implementation

The general architecture of openwakeword is also reflected in to the code. Each class is one of the stages, and runs one model:

- Class: Melspectrogram -> Runs melspectrogram stage

- Class: Embedding -> Runs embedding stage

- Class: WakeWord -> Runs classifier stage

Yes, it is that simple! the **Lowwi** class itself contains some glue logic that controls the pipeline and calls the right callback functions when classifier model detects wakeword.

For example see the Lowwi->Run function:

```cpp
void Lowwi::run(const std::vector<float> &audio_samples)
{
    if(audio_samples.empty()) {
       return; /* No samples */
    }
    
    _mel_samples = _mel->convert(std::ref(audio_samples));
    _feature_samples = _emb->convert(std::ref(_mel_samples));
    /* Loop through the classifier models */
    for (auto &ww : _wakewords) {
        wakeword_result res = ww.ww_inst->detect(_feature_samples);
        /* Check if wakeword is triggered */
        if(res.detected) {
            /* 
             * Wakeword triggered!
             * Construct context variable
             * with the triggered wakeword phrase & confidence factor 
             */
            Lowwi_ctx_t cb = {ww.properties.phrase, res.confidence};
            /* Run callback function */
            ww.properties.cbfunc(cb, ww.properties.cb_arg);
        }
    }
    /* Clear all processed Melspectrogram & feature samples */
    _mel_samples.resize(0);
    _feature_samples.resize(0);
}   
```

It just runs the steps followed by looping through all registered classifier models. When a wakeword is detected (`res.detected`) it runs the callback function(`ww.properties.cbfunc()`).


## Design decisions

Design decisions that were made (some might change!):

- Everything was designed single threaded (as the biggest c++ alternative of this library, used so much threading it slowed down the code drastically).
  Although for small amount of wakewords < 10 it is faster, over > 10 it might be slower or as fast.

- ONNX is used instead of liteRT, even though both are supported by openwakeword project. ONNX has been proven to be faster for this model. The only thing it might hinder is using the Coral edge tpu which is liteRT only.
