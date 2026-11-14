#pragma once
// PulsaretHostVoice.hpp — one voice of PulsaretPDCore for a host wrapper (SynthEdit, VCV Rack).
// No host SDK types, so the exact code the wrappers run can be tested natively.
//
// Adds two things on top of the core's per-voice path (processVoiceSample, with DC blocker):
//   Sync  : when the sync input goes from low to high, the waveform restarts at phase 0.
//           An input that stays low never resets, i.e. the voice runs free.
//   Level : the output is normalised and strictly limited to [-1, +1]. Wrappers scale it
//           (SynthEdit: x0.5 on the wire = +-5 V; Rack: x5 = +-5 V).
//           1. Scaling: the core's ceiling is set so that the loudest steady-state setting
//              found by search (60,000 random settings plus hill-climbing; all windows and
//              oversampling modes, 44.1/48/96 kHz, f0 5 Hz..7 kHz) peaks just under 1.0.
//              The DC blocker can raise a waveform's peak above the core's ceiling (it
//              removes the mean, and near its 10 Hz corner it also reacts to each pulse),
//              which is why scaling is needed. This is a measured maximum, not a proof.
//           2. Guarantee: a hard clamp at exactly +-1.0. It is idle in steady state and only
//              catches transients (parameter jumps, sync resets).
#include "PulsaretPDCore.hpp"

namespace pulsaret::host {

// Worst steady-state peak of processVoiceSample() relative to voiceCeiling (measured by
// tests/worst_peak.cpp): 1.6554 at f0 6.3 Hz, formant 207 Hz, d 0, mix 1, duty 0.88,
// Tukey, 2x, 48 kHz. For f0 >= 40 Hz the largest value found was 1.49.
inline constexpr float kWorstPeakPerCeiling = 1.6554f;
inline constexpr float kLevelMargin = 1.02f;                                   // 2 % below the worst case
inline constexpr float kStrictCeiling = 1.0f / (kWorstPeakPerCeiling * kLevelMargin);

class Voice {
public:
    void prepare(float sampleRate) noexcept {
        core_.prepare(sampleRate);                  // 10 Hz per-voice DC blocker (core default)
        params_.voiceCeiling = kStrictCeiling;
        syncHigh_ = false;
        clampedSamples_ = 0;
    }

    void setWindow(int index) noexcept {            // 0 Hann, 1 Blackman-Harris, 2 Tukey
        params_.windowShape = (index == 0) ? dsp::FormantWindow::Hann
                            : (index == 2) ? dsp::FormantWindow::Tukey
                                           : dsp::FormantWindow::BlackmanHarris;
    }
    void setOversampling(int index) noexcept {      // 0 = 1x, 1 = 2x, 2 = 4x
        oversample_ = (index == 0) ? dsp::OversampleFactor::None_1x
                    : (index == 1) ? dsp::OversampleFactor::Halfband_2x
                                   : dsp::OversampleFactor::Halfband_4x;
    }

    // fundamentalHz: any positive value (clamped to 0.01..20000). formantHz: core clamps 200..4000.
    // distortion: core clamps 0.02..0.98. mix: clamped 0..1 here. duty: core clamps 0.1..1.
    // Returns a sample in [-1, +1].
    float process(float fundamentalHz, float formantHz, float distortion, float mix, float duty,
                  bool syncHigh) noexcept {
        if (syncHigh != syncHigh_) {
            syncHigh_ = syncHigh;
            if (syncHigh) core_.setVoicePhaseOffset(0, 0.0f);   // rising edge: restart the waveform
        }
        params_.fundamentalHz  = finiteClamp(fundamentalHz, 0.01f, 20000.0f, 440.0f);
        params_.formantHz      = finiteClamp(formantHz, 1.0f, 20000.0f, 880.0f);
        params_.pdDistortion   = finiteClamp(distortion, 0.0f, 1.0f, 0.5f);
        params_.secondaryPdMix = finiteClamp(mix, 0.0f, 1.0f, 0.0f);
        params_.pulsaretDuty   = finiteClamp(duty, 0.0f, 1.0f, 1.0f);

        const float y = core_.processVoiceSample(0, params_, oversample_);
        if (y > 1.0f)   { ++clampedSamples_; return 1.0f; }
        if (y < -1.0f)  { ++clampedSamples_; return -1.0f; }
        if (!(y == y))  { ++clampedSamples_; return 0.0f; }     // never emit NaN
        return y;
    }

    // Diagnostics for tests: how many output samples the safety clamp has touched.
    unsigned long long clampedSamples() const noexcept { return clampedSamples_; }

private:
    static float finiteClamp(float v, float lo, float hi, float ifNaN) noexcept {
        if (!(v == v)) return ifNaN;
        return (v < lo) ? lo : (v > hi ? hi : v);
    }

    dsp::PulsaretPDCore<1> core_;
    dsp::VoiceParameters   params_{};
    dsp::OversampleFactor  oversample_{dsp::OversampleFactor::Halfband_4x};
    bool  syncHigh_{false};
    unsigned long long clampedSamples_{0};
};

} // namespace pulsaret::host
