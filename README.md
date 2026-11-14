# Liminal Intent Pulsaret

VCV Rack 2 plugin with one module, **Pulsaret PD**: a polyphonic hybrid phase-distortion /
pulsaret oscillator with a formant-locked secondary carrier.

## Controls
- **Pitch**, **V/OCT**: 1 V/octave around C4.
- **Formant** (200–4000 Hz), **FORM** CV: 1 V/octave.
- **Dist** (phase-distortion knee), **Mix** (formant carrier mix), **Duty** (pulsaret duty): 0–10 V CV sweeps the full range.
- **Window**: Hann, Blackman-Harris, Tukey.
- **Oversample**: 1x, 2x, 4x (half-band decimation).
- **Sync**: a rising edge through 0 V restarts the waveform; unpatched, the oscillator runs free.
- **Out**: up to 16 channels (follows V/OCT), strictly within ±5 V.

## Building
From the MSYS2 MinGW 64-bit shell (Windows) or a normal shell (Linux/macOS), with the
[Rack SDK](https://vcvrack.com/manual/Building):

    make RACK_DIR=<path to Rack-SDK>
    make install RACK_DIR=<path to Rack-SDK>

## License
MIT — see LICENSE.txt.
