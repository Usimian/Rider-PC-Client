#!/usr/bin/env python3
"""Cut the Rider's motorcycle sounds from real recordings (2026-10-20, Marc: "more realistic ... a kickstart
motorcycle sound, old school"; the cut points of the Kawasaki and the Rover are his). From Freesound -- downloading
needs a free login:
  370340 "Motorcycle cranck up" (Kawasaki Z1000 2008 fire-up, in a garage)  by esperpentium            CC0
                                                               https://freesound.org/people/esperpentium/sounds/370340/
  106553 "car engine revving up to 7000 rpm (Rover 216 GSi)"  by Tomlija                            CC BY 3.0
                                                               https://freesound.org/people/tomlija/sounds/106553/
         CC BY 3.0 (https://creativecommons.org/licenses/by/3.0/) requires this credit: "car engine revving up to
         7000 rpm (Rover 216 GSi)" by Tomlija, freesound.org, CC BY 3.0 -- cut and level-adjusted for engine_rev.wav.
The originals live (gitignored) in data/sounds/; the cut clips are committed in pi/assets/ (the originals can't be
fetched without an account) and installed by pi/deploy_bridge.sh.

  engine_start.wav  Kawasaki: the crank and fire-up, 0-2.72 s          (the robot is up)
  engine_rev.wav    Rover: a rev, 8.33-10.33 s                          (joystick mode)
Switching to computer mode (or losing the controller) is silent -- Marc's choice; the LCD and web page show it.

Output: 16 kHz mono 16-bit (as beep.wav), each clip scaled to RMS TARGET_RMS with a soft limiter on the peaks (the
speaker mixer is already at max, so loudness is set here), short fades at the cuts. Decoding/resampling by ffmpeg;
needs numpy: run with the sim venv.
Run:  sim/.venv/bin/python pi/assets/cut_engine_sounds.py [data/sounds] [pi/assets]
"""
import os, sys, wave, subprocess
import numpy as np

KAWA = "370340__esperpentium__motorcycle-cranck-up.aiff"
ROVER = "106553__tomlija__car-engine-revving-up-to-7000-rpm-rover-216-gsi.aiff"
RATE = 16000
TARGET_RMS = 2500.0          # ~15 dB above the beep (RMS ~450); at the beep's level the engine was too quiet
LIMIT = 30000.0              # soft limiter: peaks are squashed towards this (tanh), never clipped
XFADE_S = 0.08
GAIN_DB = {"engine_start": 4.0, "engine_rev": -4.0}   # per clip, on top of TARGET_RMS (set by ear on the robot)

# name: list of (file, t0, t1) segments in seconds, joined with crossfades; picked from envelope + spectrogram maps
CLIPS = {
    "engine_start": [(KAWA, 0.00, 2.72)],
    "engine_rev":   [(ROVER, 8.33, 10.33)],
}


def load(path):
    """-> mono float in [-1, 1] at RATE, any input format (ffmpeg decodes and resamples)."""
    pcm = subprocess.run(["ffmpeg", "-v", "error", "-i", path, "-ac", "1", "-ar", str(RATE), "-f", "s16le", "-"],
                         capture_output=True, check=True).stdout
    return np.frombuffer(pcm, np.int16) / 2 ** 15


def cut(src, segs, gain_db=0.0):
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
    out *= TARGET_RMS / 32767 / np.sqrt(np.mean(out ** 2)) * 10 ** (gain_db / 20)
    out = LIMIT * np.tanh(out * 32767 / LIMIT)
    return np.round(out).astype(np.int16)


if __name__ == "__main__":
    src_dir = sys.argv[1] if len(sys.argv) > 1 else "data/sounds"
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(os.path.abspath(__file__))
    src = {f: load(os.path.join(src_dir, f)) for f in {s[0] for segs in CLIPS.values() for s in segs}}
    for name, segs in CLIPS.items():
        y = cut(src, segs, GAIN_DB.get(name, 0.0))
        with wave.open(os.path.join(out_dir, name + ".wav"), "wb") as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(RATE); w.writeframes(y.tobytes())
        print("%s.wav  %.2f s  peak %d" % (name, len(y) / RATE, np.abs(y).max()))
