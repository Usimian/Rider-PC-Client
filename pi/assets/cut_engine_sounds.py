#!/usr/bin/env python3
"""Cut the Rider's motorcycle sounds from real recordings (2026-10-20, Marc: "more realistic ... a kickstart
motorcycle sound, old school"; cut points for the Kawasaki his). All CC0 (public domain) from Freesound --
downloading needs a free login:
  370340 "Motorcycle cranck up" (Kawasaki Z1000 2008 fire-up, in a garage)  by esperpentium
                                                               https://freesound.org/people/esperpentium/sounds/370340/
  450034 "motorcycle dirt bike motocross start idle, rev, stop, kick start, ..."  by kyles
                                                               https://freesound.org/people/kyles/sounds/450034/
The originals live (gitignored) in data/sounds/; the cut clips are committed in pi/assets/ (the originals can't be
fetched without an account) and installed by pi/deploy_bridge.sh.

  engine_start.wav  Kawasaki: the crank and fire-up, 0-2.72 s          (the robot is up)
  engine_rev.wav    Kawasaki: a rev, 3.2-4.35 s                         (joystick mode)
  engine_stop.wav   dirt bike: a last blip, then the engine dies        (computer mode)

Output: 16 kHz mono 16-bit (as beep.wav), each clip scaled to the beep's loudness (RMS ~450 -- the amplifier is
loud), short fades at the cuts. Decoding/resampling by ffmpeg; needs numpy: run with the sim venv.
Run:  sim/.venv/bin/python pi/assets/cut_engine_sounds.py [data/sounds] [pi/assets]
"""
import os, sys, wave, subprocess
import numpy as np

KAWA = "370340__esperpentium__motorcycle-cranck-up.aiff"
KYLES = "450034__kyles__motorcycle-dirt-bike-motocross-start-idle-rev-stop-kick-start-false-start-drive-off-pull-away-pull-up-stop.wav"
RATE = 16000
TARGET_RMS = 450.0
XFADE_S = 0.08

# name: list of (file, t0, t1) segments in seconds, joined with crossfades; picked from envelope + spectrogram maps
CLIPS = {
    "engine_start": [(KAWA, 0.00, 2.72)],
    "engine_rev":   [(KAWA, 3.20, 4.35)],
    "engine_stop":  [(KYLES, 77.50, 81.00)],
}


def load(path):
    """-> mono float in [-1, 1] at RATE, any input format (ffmpeg decodes and resamples)."""
    pcm = subprocess.run(["ffmpeg", "-v", "error", "-i", path, "-ac", "1", "-ar", str(RATE), "-f", "s16le", "-"],
                         capture_output=True, check=True).stdout
    return np.frombuffer(pcm, np.int16) / 2 ** 15


def cut(src, segs):
    out = None
    for f, t0, t1 in segs:
        s = src[f][int(t0 * RATE):int(t1 * RATE)].copy()
        if out is None:
            out = s
        else:                                       # equal-power crossfade
            m = int(XFADE_S * RATE)
            a = np.cos(np.linspace(0, np.pi / 2, m)); b = np.sin(np.linspace(0, np.pi / 2, m))
            out = np.concatenate([out[:-m], out[-m:] * a + s[:m] * b, s[m:]])
    fi, fo = int(0.03 * RATE), int(0.15 * RATE)
    out[:fi] *= np.linspace(0, 1, fi); out[-fo:] *= np.linspace(1, 0, fo)
    out *= TARGET_RMS / 32767 / np.sqrt(np.mean(out ** 2))
    return np.clip(np.round(out * 32767), -32767, 32767).astype(np.int16)


if __name__ == "__main__":
    src_dir = sys.argv[1] if len(sys.argv) > 1 else "data/sounds"
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(os.path.abspath(__file__))
    src = {f: load(os.path.join(src_dir, f)) for f in {s[0] for segs in CLIPS.values() for s in segs}}
    for name, segs in CLIPS.items():
        y = cut(src, segs)
        with wave.open(os.path.join(out_dir, name + ".wav"), "wb") as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(RATE); w.writeframes(y.tobytes())
        print("%s.wav  %.2f s  peak %d" % (name, len(y) / RATE, np.abs(y).max()))
