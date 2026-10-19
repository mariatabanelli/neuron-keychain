"""
Data layer: a real night of EEG -> band powers per 30 s epoch -> input current
for the neuron, replayed over OSC.

Run:
    python eeg_to_current.py              # downloads one night of Sleep-EDF (PhysioNet, ~50 MB)
    python eeg_to_current.py --demo       # synthetic EEG, no download, to test the pipeline
    python eeg_to_current.py --osc        # also stream current + stage over OSC to 127.0.0.1:9000

Needs: pip install mne numpy matplotlib python-osc
"""

import argparse
import time

import numpy as np
import matplotlib.pyplot as plt
import mne

BANDS = {"delta": (0.5, 4), "theta": (4, 8), "alpha": (8, 12), "beta": (12, 30)}
EPOCH_S = 30.0          # Sleep-EDF stages are scored in 30 s epochs
CHANNEL = "EEG Fpz-Cz"  # the frontal-central channel in Sleep-EDF


def load_sleep_edf():
    """One night from the Sleep-EDF expanded database (subject 0, night 1)."""
    from mne.datasets.sleep_physionet.age import fetch_data
    [(psg_path, hyp_path)] = fetch_data(subjects=[0], recording=[1], path="./data")
    raw = mne.io.read_raw_edf(psg_path, preload=True, verbose=False)
    annot = mne.read_annotations(hyp_path)
    raw.set_annotations(annot)
    raw.pick([CHANNEL])
    return raw


def load_demo():
    """Synthetic 2-hour recording with four fake 'stages' so the pipeline runs offline."""
    sfreq = 100.0
    rng = np.random.default_rng(0)
    stages = ["W", "1", "2", "3", "R"]
    dom_freq = {"W": 10, "1": 6, "2": 12, "3": 1.5, "R": 7}   # Hz, crude caricature
    n_epochs = 240
    sig, labels = [], []
    t = np.arange(int(EPOCH_S * sfreq)) / sfreq
    for i in range(n_epochs):
        s = stages[(i // 30) % len(stages)]
        x = 30e-6 * np.sin(2 * np.pi * dom_freq[s] * t) + 10e-6 * rng.standard_normal(t.size)
        sig.append(x)
        labels.append(s)
    data = np.concatenate(sig)[None, :]
    info = mne.create_info([CHANNEL], sfreq, ch_types="eeg")
    raw = mne.io.RawArray(data, info, verbose=False)
    onsets = np.arange(n_epochs) * EPOCH_S
    raw.set_annotations(mne.Annotations(onsets, [EPOCH_S] * n_epochs,
                                        [f"Sleep stage {s}" for s in labels]))
    return raw


def band_powers(raw):
    """Returns (stages, powers) with powers shaped (n_epochs, n_bands), in uV^2."""
    raw.filter(0.5, 30.0, verbose=False)
    # Sleep-EDF scores W, 1, 2, 3, 4, R, plus "?" (unscored) and "Movement time".
    # Keep the real stages only; fold 4 into 3 (the modern convention).
    wanted = {"Sleep stage W": 0, "Sleep stage 1": 1, "Sleep stage 2": 2,
              "Sleep stage 3": 3, "Sleep stage 4": 3, "Sleep stage R": 4}
    keep = [i for i, d in enumerate(raw.annotations.description) if d in wanted]
    raw.set_annotations(raw.annotations[keep])
    events, _ = mne.events_from_annotations(raw, event_id=wanted,
                                            chunk_duration=EPOCH_S, verbose=False)
    present = sorted(set(events[:, 2]))
    names = {0: "W", 1: "1", 2: "2", 3: "3", 4: "R"}
    event_id = {names[c]: c for c in present}
    epochs = mne.Epochs(raw, events, event_id, tmin=0.0, tmax=EPOCH_S - 1.0 / raw.info["sfreq"],
                        baseline=None, preload=True, verbose=False)
    psd = epochs.compute_psd(fmin=0.5, fmax=30.0, verbose=False)
    freqs = psd.freqs
    spectra = psd.get_data()[:, 0, :] * 1e12        # V^2/Hz -> uV^2/Hz
    powers = np.column_stack([
        spectra[:, (freqs >= lo) & (freqs < hi)].mean(axis=1) for lo, hi in BANDS.values()
    ])
    stages = [names[e] for e in epochs.events[:, 2]]
    return stages, powers


def to_current(powers):
    """
    Map band power to the neuron's input current (nA).
    Rule for v1: more fast activity (alpha + beta) relative to slow (delta) = more drive.
    Scaled to 0..4 nA so the LIF neuron goes from silent to a steady train.
    """
    delta, theta, alpha, beta = powers.T
    ratio = (alpha + beta) / (delta + theta + 1e-9)
    ratio = np.log1p(ratio)
    lo, hi = np.percentile(ratio, [5, 95])
    return np.clip((ratio - lo) / (hi - lo), 0, 1) * 4.0


def plot(stages, powers, current, out="eeg_to_current.png"):
    fig, axes = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    x = np.arange(len(stages)) * EPOCH_S / 3600.0
    order = ["W", "R", "1", "2", "3", "4"]
    y = [order.index(s) if s in order else -1 for s in stages]
    axes[0].step(x, y, where="post")
    axes[0].set_yticks(range(len(order)), order)
    axes[0].invert_yaxis()
    axes[0].set_ylabel("stage")
    for name, col in zip(BANDS, powers.T):
        axes[1].plot(x, col, lw=0.8, label=name)
    axes[1].set_yscale("log")
    axes[1].set_ylabel("power (uV^2/Hz)")
    axes[1].legend(ncol=4, fontsize=8)
    axes[2].plot(x, current, lw=1)
    axes[2].set_ylabel("input current (nA)")
    axes[2].set_xlabel("hours")
    plt.tight_layout()
    plt.savefig(out, dpi=150)
    print(f"saved {out}")


def stream_osc(stages, current, host="127.0.0.1", port=9000, speed=600.0):
    """Replay the night over OSC. speed=600 means 10 min of sleep per second."""
    from pythonosc.udp_client import SimpleUDPClient
    client = SimpleUDPClient(host, port)
    dt = EPOCH_S / speed
    for s, i in zip(stages, current):
        client.send_message("/neuron/current", float(i))
        client.send_message("/neuron/stage", s)
        time.sleep(dt)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true", help="synthetic data, no download")
    ap.add_argument("--osc", action="store_true", help="stream over OSC after plotting")
    args = ap.parse_args()

    raw = load_demo() if args.demo else load_sleep_edf()
    stages, powers = band_powers(raw)
    current = to_current(powers)
    print(f"{len(stages)} epochs, {len(stages) * EPOCH_S / 3600:.1f} h, "
          f"current range {current.min():.2f} to {current.max():.2f} nA")
    np.savetxt("eeg_current.csv", np.column_stack([stages, current]), fmt="%s", delimiter=",",
               header="stage,current_nA", comments="")
    plot(stages, powers, current)
    if args.osc:
        stream_osc(stages, current)
