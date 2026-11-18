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
//   Duty  : short pulsarets carry less energy (RMS falls roughly with sqrt(duty)) while their
//           peaks stay about the same, so short duty sounds quieter. A duty-dependent gain
//           restores part of that loss. It is capped per duty so that the compensated worst-case
//           peak at that duty stays within the global worst case above: the overall scaling,
//           and the level at duty >= 0.65, are unchanged.
//   Formant tracking (optional): the formant is transposed with the note, i.e. multiplied by
//           fundamental / referenceHz. At the reference pitch it equals the untracked formant.
#include "PulsaretPDCore.hpp"

namespace pulsaret::host {

// Worst steady-state peak of processVoiceSample() relative to voiceCeiling (measured by
// tests/worst_peak.cpp): 1.6554 at f0 6.3 Hz, formant 207 Hz, d 0, mix 1, duty 0.88,
// Tukey, 2x, 48 kHz. For f0 >= 40 Hz the largest value found was 1.49.
inline constexpr float kWorstPeakPerCeiling = 1.6554f;
inline constexpr float kLevelMargin = 1.02f;                                   // 2 % below the worst case
inline constexpr float kStrictCeiling = 1.0f / (kWorstPeakPerCeiling * kLevelMargin);

// Duty compensation gain, tabulated at duty 0.10, 0.15, ... 0.65 per window (0 Hann,
// 1 Blackman-Harris, 2 Tukey); 1.0 at and above 0.65; linear in between.
// Values: min(RMS(0.65) / RMS(duty), 0.98 * kWorstPeakPerCeiling / worstPeak(duty)), never below 1,
// measured by tests/duty_level.cpp and tests/duty_worst.cpp.
inline constexpr int kDutyPoints = 12;
inline constexpr float kDutyStart = 0.10f, kDutyStep = 0.05f;
inline constexpr float kDutyGain[3][kDutyPoints] = {
    {1.3784f, 1.2830f, 1.2571f, 1.2102f, 1.1928f, 1.1499f, 1.1305f, 1.0950f, 1.0881f, 1.0638f, 1.0174f, 1.0000f},
    {1.4423f, 1.3831f, 1.3040f, 1.2803f, 1.2302f, 1.1994f, 1.1713f, 1.1310f, 1.0592f, 1.0175f, 1.0000f, 1.0000f},
    {1.2092f, 1.1369f, 1.0921f, 1.0597f, 1.0405f, 1.0211f, 1.0211f, 1.0000f, 1.0000f, 1.0064f, 1.0009f, 1.0000f}
};

inline float dutyCompensation(float duty, int window) noexcept {
    const float* t = kDutyGain[(window < 0 || window > 2) ? 1 : window];
    if (!(duty > kDutyStart)) return t[0];
    const float x = (duty - kDutyStart) / kDutyStep;
    if (x >= (float)(kDutyPoints - 1)) return 1.0f;
    const int i = (int)x;
    const float f = x - (float)i;
    return t[i] + (t[i + 1] - t[i]) * f;
}

class Voice {
public:
    void prepare(float sampleRate) noexcept {
        core_.prepare(sampleRate);                  // 10 Hz per-voice DC blocker (core default)
        params_.voiceCeiling = kStrictCeiling;
        syncHigh_ = false;
        clampedSamples_ = 0;
    }

    void setWindow(int index) noexcept {            // 0 Hann, 1 Blackman-Harris, 2 Tukey
        windowIndex_ = (index == 0 || index == 2) ? index : 1;
        params_.windowShape = (index == 0) ? dsp::FormantWindow::Hann
                            : (index == 2) ? dsp::FormantWindow::Tukey
                                           : dsp::FormantWindow::BlackmanHarris;
    }
    void setOversampling(int index) noexcept {      // 0 = 1x, 1 = 2x, 2 = 4x
        oversample_ = (index == 0) ? dsp::OversampleFactor::None_1x
                    : (index == 1) ? dsp::OversampleFactor::Halfband_2x
                                   : dsp::OversampleFactor::Halfband_4x;
    }

    // Formant tracking: off = formantHz is absolute; on = formantHz is the formant at referenceHz
    // and moves with the note (SynthEdit: 440 Hz = 5 V; Rack: C4 = 0 V).
    void setFormantTracking(bool on, float referenceHz) noexcept {
        trackFormant_ = on;
        trackReferenceHz_ = (referenceHz > 1.0f) ? referenceHz : 440.0f;
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
        float fHz = formantHz;
        if (trackFormant_) fHz *= params_.fundamentalHz / trackReferenceHz_;
        params_.formantHz      = finiteClamp(fHz, 1.0f, 20000.0f, 880.0f);   // core clamps 200..4000
        params_.pdDistortion   = finiteClamp(distortion, 0.0f, 1.0f, 0.5f);
        params_.secondaryPdMix = finiteClamp(mix, 0.0f, 1.0f, 0.0f);
        params_.pulsaretDuty   = finiteClamp(duty, 0.0f, 1.0f, 1.0f);

        const float y = core_.processVoiceSample(0, params_, oversample_)
                      * dutyCompensation(params_.pulsaretDuty, windowIndex_);
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
    int   windowIndex_{1};
    bool  trackFormant_{false};
    float trackReferenceHz_{440.0f};
    unsigned long long clampedSamples_{0};
};

} // namespace pulsaret::host
