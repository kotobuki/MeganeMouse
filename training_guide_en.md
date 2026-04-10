# Training and Tuning Audio Models with Edge Impulse Studio

This document collects notes from our experience building a practical audio classification model with [Edge Impulse Studio](https://studio.edgeimpulse.com/) that runs on microcontrollers such as the ESP32-S3. We put it together while adding a sound-based click feature to [MeganeMouse](README_en.md), and it captures the kind of hard-won lessons we wished we had known from the start.

The running example throughout is detecting sounds used for mouse button operations — a click for the left button and a long-press. That said, most of the ideas here should carry over to any project that needs to recognize short target sounds in a noisy environment.

## 1. Class Design

Machine learning classifiers do not say "I don't know." When they encounter a sound they have never seen during training, they simply pick whichever class looks closest and report it with full confidence. If you train only on your target sounds, unrelated sounds that happen to drift in will get forced into one of those target classes, often with surprisingly high confidence.

To work around this, we set up two "catch-all" classes for non-target sounds and build the model with four classes in total:

- `click` (target, brief): a short click for the left button (press → brief pause → release)
- `long_press` (target, brief): a brief sound for toggling the left button
- `impulsive_noise` (non-target, brief): claps, knocks on a desk, coughs, and similar short sounds
- `background_noise` (non-target, sustained): conversation, TV audio, air conditioner hum, and similar ambient sounds

We split the non-target sounds into "brief" and "sustained" categories because, during our experiments, we noticed that both `click` and `long_press` are brief impulsive sounds and tended to get confused with `impulsive_noise`. Giving each failure mode its own bucket, and collecting enough samples for each, noticeably sharpened the classifier's decisions.

## 2. Collecting Training Data

In our experience, the quality of the data mattered far more than any algorithmic tweak. The three points below are easy to overlook at the start, and the kind of thing you end up kicking yourself about later.

### 2.1. Record with the actual hardware

Always record your training data on the same hardware you plan to deploy to. In our case, that meant using an AtomS3R with the microphone attached, running `SoundSampleCollector.ino`.

It is tempting to grab a laptop or phone and collect data quickly, but this almost always backfires. Different microphones have different noise floors and frequency responses, and laptop and phone microphones typically apply their own noise reduction and signal processing under the hood. A model trained on that data can look perfect in Edge Impulse Studio and then completely fall apart the moment you flash it to the target device. Most of the "why doesn't this work on the device?" surprises we hit traced back to this.

### 2.2. Feed misclassifications back into training (Hard Negative Mining)

Once we started testing on the device, we kept running into confident misclassifications: conversation being recognized as `long_press` with 100% confidence, or a clap being recognized as `click` with 100% confidence. This happens because the model has never seen those sounds, so it picks whichever of its known classes looks closest.

The fix is straightforward but requires patience: go to the environment where the misclassification happened, record the offending sound (voices, claps, whatever it was), add it to `background_noise` or `impulsive_noise`, and retrain. Looping through this "catch the mistake, teach the mistake" process a few times was the fastest path we found to a model that actually worked in real conditions.

### 2.3. Vary the position of the sound within the sample (Shift samples)

In the real world, the target sound almost never lands neatly in the middle of the analysis window. When the model runs continuously on the device, the sound may appear near either edge of the window, get clipped, or sit at an unexpected offset. The model needs to handle all of that.

Edge Impulse Studio has a convenient option for this. When you use the "Split sample" feature to chop a longer recording into individual samples, you can enable a checkbox called "Shift samples." With this option on, each split places the target sound at a randomly offset position within the sample instead of centering it. The resulting variation in the training data makes the model much more forgiving of real-world timing.

It is easy to forget to enable this when you are batch-splitting a long recording, so make it part of your checklist.

## 3. Impulse Design (Preprocessing and Feature Extraction)

For a model that has to run on a microcontroller, the way you slice up and represent the audio (the DSP stage) has more impact on final performance than the classifier architecture itself. Here are the settings that tripped us up along the way.

### 3.1. Match the window size to your target sound

Edge Impulse Studio defaults the analysis window size to `1000 ms` (1 second). Our target sounds are only 50–120 ms long, so with a 1-second window the vast majority of each sample is silence, and the actual signal gets drowned out in the averaging.

After some experimentation, we landed on `250 ms` as a good compromise. At that length, the target sound fills roughly half the window, so its features stand out clearly. It also divides nicely when we later split each window into smaller slices for continuous inference.

Note that this "window" is the same one referred to later in Section 7, in the context of `EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW`.

### 3.2. Spectrogram parameters

For the feature extraction block, we ended up using a raw Spectrogram rather than MFCC or MFE.

MFCC and MFE are designed around the sensitivity curve of human hearing, which emphasizes the lower frequencies where speech lives. They are a great fit for voice recognition. But our target sounds, along with similar impulsive noises like claps, spread their energy across a wide range of frequencies, and we found that the flat, uniform resolution of a plain Spectrogram preserved their distinguishing features more reliably.

Here are the parameter values that worked well for us, balancing inference speed against accuracy:

- **Frame length (e.g., `0.016` s)**: the length of each "frame" sliced out of the waveform. At a 16,000 Hz sampling rate, 0.016 s gives exactly 256 samples per frame. Matching this to the FFT length (below) avoids losing information at the frame boundaries.
- **Frame stride (e.g., `0.008` s)**: how far to advance between frames. Setting this to half the frame length gives a 50% overlap between adjacent frames, which keeps short sounds (tens of milliseconds to 100 ms) from slipping through the cracks between frames.
- **FFT length (e.g., `256`)**: the frequency resolution. This must be at least as large as the number of samples per frame, and must be a power of two. You can go up to 512 if you have compute headroom, but keeping it at 256 significantly reduces the cost of the downstream classifier.

## 4. Building the Classifier

Next comes the neural network that treats each Spectrogram as an "image" and classifies it.

### 4.1. Network architecture

2D convolutional layers — the workhorse of image recognition — turned out to be a natural fit for spotting the vertical streaks in a Spectrogram that characterize short impulsive sounds like clicks.

Here is the architecture we settled on for the Window 250 ms / FFT 256 configuration described above:

```text
Input layer (3,870 features)
        ↓
Reshape layer (257 columns)
        ↓
2D conv / pool layer (8 filters, kernel size 3, 1 layer)
        ↓
2D conv / pool layer (16 filters, kernel size 3, 1 layer)
        ↓
Flatten layer
        ↓
Dense layer (32 neurons)
        ↓
Dropout (rate 0.5)
        ↓
Output layer (4 classes)
```

If the model struggles to distinguish between similar sounds, you can increase its representational capacity with changes like the following. These come at the cost of longer inference time and higher memory use, so you will need to weigh them against the resources available on your target device.

- Increase the number of filters in the convolutional layers (e.g., `8 → 16`, `16 → 32`)
- Increase the number of neurons in the Dense layer, or add another Dense layer

## 5. Splitting Training and Test Data

To get an honest measurement of whether added noise data is actually helping the model generalize, keep a portion of your data aside as a test set.

1. Upload all newly collected data to the Training set first.
2. Open the Dashboard from the left menu of Edge Impulse Studio.
3. Scroll down to the "Danger zone" section and click `Perform train/test split`.
4. The data is automatically redistributed into an 80% training / 20% test split.
5. Retrain the model, then run Model testing to check whether previously unseen noise samples are being rejected correctly.

If misclassifications remain after Model testing, download the offending samples, give them a listen, and feed them back into training following the approach in Section 2.2.

## 6. Final Tuning for On-Device Deployment

Even with a well-trained model, running it continuously on the device can produce the occasional single-frame misclassification, depending on exactly how the incoming audio lines up with the inference window. This section covers how we dealt with those.

### 6.1. Smoothing out spikes with consecutive-frame detection

We ran into cases where, for example, only the tail end of a hand clap happened to land inside a slice, and the model briefly classified it as a target sound. These single-frame misfires are tricky to suppress inside the model itself without also dulling legitimate detections, so we found it cleaner to handle them in the code that consumes the inference results.

The trick is to require the same class to clear the threshold for several consecutive frames (we use two) before treating it as a real event. This small bit of state kept stray misfires from leaking through and noticeably improved the overall reliability of the system.

The pseudocode below illustrates the idea. `THRESHOLD`, `run_inference()`, `is_noise()`, and the other helpers should be adapted to whatever model and framework you are using.

```cpp
// State variables (global or static)
int last_class_id = -1;
int consecutive_count = 0;
const int REQUIRED_FRAMES = 2;  // Number of consecutive detections required

void loop() {
  // Run inference on one slice and get the top class and its probability
  int current_class_id = run_inference();
  float probability = get_probability(current_class_id);

  if (probability >= THRESHOLD) {
    if (current_class_id == last_class_id) {
      consecutive_count++; // Same class as before: increment
    } else {
      last_class_id = current_class_id;
      consecutive_count = 1; // Different class: reset to 1
    }

    // Treat it as a real event only at the moment we hit the threshold count
    if (consecutive_count == REQUIRED_FRAMES) {
      if (!is_noise(current_class_id)) {
        trigger_action(current_class_id); // Fire the click action, etc.
      }
    }
  } else {
    // Below threshold: reset state entirely
    last_class_id = -1;
    consecutive_count = 0;
  }
}
```

## 7. Troubleshooting

A couple of errors we actually hit during development, and how we worked around them.

### `ValueError` right after changing DSP parameters

#### Error

After changing a DSP parameter like `FFT length` in the Spectrogram block, starting a training run can crash with something like this:

```text
ValueError: total size of new array must be unchanged, input_shape = [4883], output_shape = [75, 65, 1]
```

#### Cause

The total number of features coming out of the DSP stage no longer matches the grid size that the `Reshape layer` at the front of the classifier is trying to assemble. Changing the FFT length changes the number of frequency bins, which changes the total feature count — but the `Reshape layer` setting does not update automatically to match.

#### Fix

Open the `Reshape layer` in the Classifier view and update the `Columns` value to match the new configuration. Alternatively, choose "Reset to default architecture" from the menu in the top right to have Edge Impulse rebuild the architecture from scratch.

The number to put in `Columns` depends on whether you want to treat the time axis or the frequency axis as columns of the resulting image. For example, with `FFT length = 512`, `Window = 250 ms`, and `Frame stride = 0.016 s`:

- Frequency bins: `512 ÷ 2 + 1 = 257`
- Time frames: `250 ms ÷ 16 ms ≈ 19`

So `Columns` should be set to either `257` or `19`, depending on which axis you want along the columns. Either works; the choice is mostly a matter of preference and how you want the convolutions to see the data.

### Runtime error about `frame_length`

#### Error

To improve responsiveness, it helps to split each inference window into more slices so that inferences run more frequently. If you try to do this by raising `EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW` in `model_metadata.h` (for example, from `4` to `8`), you may see a runtime error like the following:

```text
ERR: frame_length (800) cannot be larger than signal's total length (500)
```

#### Cause

Increasing the slice count shortens the audio segment passed to each inference. Once that segment becomes shorter than the minimum frame length required by the DSP stage, the DSP can no longer process it and errors out.

#### Fix

Lower the DSP's minimum required frame length so that shorter slices still fit:

1. Open the DSP (Spectrogram / MFE) settings page in Edge Impulse Studio.
2. Under Parameters, reduce the `Frame length` value (for example, from `0.05` s to `0.02` s).
3. Regenerate features and retrain the model.
