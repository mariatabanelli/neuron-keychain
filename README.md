# Neuron keychain

A single simulated neuron living on a 1.28" round touch display (Waveshare ESP32-S3-Touch-LCD-1.28).
Touch it to stimulate it. Feed it a real night of EEG and watch it sleep.

## Parts of the project

- `lif_neuron.py` — the model (leaky integrate-and-fire), tap / hold / shake scenarios. Runs on a laptop; the same equations go on the device.
- `eeg_to_current.py` — data layer: public sleep EEG (PhysioNet Sleep-EDF) → band powers per 30 s → input current for the neuron, with optional OSC streaming.
- `firmware/` — ESP32-S3 Arduino sketch (to come).
- `design/` — Figma exports, design system for the 240 px ring (to come).
- `touchdesigner/` — population visual driven by the device's spikes (to come).

## Run

    pip install numpy matplotlib mne python-osc
    python lif_neuron.py
    python eeg_to_current.py --demo     # synthetic, no download
    python eeg_to_current.py            # real data, downloads ~50 MB from PhysioNet
