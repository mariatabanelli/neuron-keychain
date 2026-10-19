"""
Leaky integrate-and-fire (LIF) neuron, the model that will live on the keychain.

Run:  python lif_neuron.py
Needs: numpy, matplotlib   (pip install numpy matplotlib)

The neuron is a leaky bucket. Input current fills it, the leak drains it,
and when the level reaches the threshold it "spikes" and empties.
"""

import numpy as np
import matplotlib.pyplot as plt

# ---- model parameters (these become the dials on the device) ----
V_REST = -70.0      # mV, where the neuron sits when left alone
V_THRESH = -55.0    # mV, firing threshold
V_RESET = -75.0     # mV, where it drops to after a spike
TAU_M = 20.0        # ms, membrane time constant (how fast the leak drains)
R_M = 10.0          # MOhm, membrane resistance (how strongly current moves V)
T_REF = 2.0         # ms, refractory period after a spike (ignores input)

DT = 0.1            # ms, simulation timestep
T_TOTAL = 500.0     # ms


def simulate(current_fn):
    """current_fn(t_ms) -> input current in nA. Returns time, V, spike times."""
    n = int(T_TOTAL / DT)
    t = np.arange(n) * DT
    v = np.full(n, V_REST)
    spikes = []
    ref_until = -1.0

    for i in range(1, n):
        if t[i] < ref_until:
            v[i] = V_RESET
            continue
        i_in = current_fn(t[i])
        # dV/dt = (-(V - V_rest) + R*I) / tau
        dv = (-(v[i - 1] - V_REST) + R_M * i_in) / TAU_M
        v[i] = v[i - 1] + dv * DT
        if v[i] >= V_THRESH:
            v[i] = 20.0          # draw a spike
            spikes.append(t[i])
            ref_until = t[i] + T_REF
    return t, v, spikes


# ---- three "interactions" the keychain will have ----
def tap(t):                       # one short press: a 5 ms pulse
    return 8.0 if 100 <= t < 105 else 0.0

def hold(t):                      # finger held down: sustained current
    return 2.0 if 100 <= t < 400 else 0.0

def hold_with_noise(t):           # held down while the device is shaken
    return (2.0 if 100 <= t < 400 else 0.0) + np.random.normal(0, 1.5)


if __name__ == "__main__":
    np.random.seed(1)
    cases = [("tap", tap), ("hold", hold), ("hold + shake", hold_with_noise)]

    fig, axes = plt.subplots(len(cases), 1, figsize=(9, 7), sharex=True)
    for ax, (name, fn) in zip(axes, cases):
        t, v, spikes = simulate(fn)
        ax.plot(t, v, lw=1)
        ax.axhline(V_THRESH, ls="--", lw=0.8, color="grey")
        ax.set_ylabel("V (mV)")
        ax.set_title(f"{name}: {len(spikes)} spikes")
    axes[-1].set_xlabel("time (ms)")
    plt.tight_layout()
    plt.savefig("lif_neuron.png", dpi=150)
    plt.show()
