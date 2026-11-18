#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>

namespace pulsaret::dsp {

enum class FormantWindow : uint8_t {
    Hann = 0,
    BlackmanHarris = 1,
    Tukey = 2
};

enum class OversampleFactor : uint8_t {
    None_1x = 1,
    Halfband_2x = 2,
    Halfband_4x = 4
};

struct VoiceParameters {
    float fundamentalHz{220.00f};
    float formantHz{880.00f};
    float pdDistortion{0.280f};  // Inflection knee d in [0.02, 0.98]
    float secondaryPdMix{0.350f};
    float pulsaretDuty{0.650f};
    FormantWindow windowShape{FormantWindow::BlackmanHarris};
    float voiceCeiling{0.74f}; // Production safe: 0.74f (legacy diagnostic: 0.85f)
};

/**
 * First-Order High-Pass DC Blocker:
 * y[n] = x[n] - x[n-1] + R * y[n-1], where R = exp(-2*pi*fc / fs)
 */
class DCBlocker1P {
public:
    void reset() noexcept {
        x1_ = 0.0f;
        y1_ = 0.0f;
    }

    void setCutoff(float fcHz, float sampleRate) noexcept {
        const float sr = (sampleRate > 8000.0f) ? sampleRate : 48000.0f;
        const float fc = std::clamp(fcHz, 0.5f, 200.0f);
        coeffR_ = std::exp(-2.0f * std::numbers::pi_v<float> * fc / sr);
    }

    float process(float x) noexcept {
        const float y = x - x1_ + coeffR_ * y1_;
        x1_ = x;
        y1_ = y;
        return y;
    }

private:
    float coeffR_{0.99869f};
    float x1_{0.0f};
    float y1_{0.0f};
};

/**
 * Symmetric 15-Tap Halfband FIR Decimator Stage (2:1 Downsampler)
 * Zero even-indexed taps except center tap h[0] = 0.5. Center tap delay = 7 ticks.
 * Filter group delay, counted from the newest input sample: Stage 1 = 7/4 = 1.75 base samples,
 * Stage 2 = 7/2 = 3.5, cascade = 5.25. Delay relative to the 1x path (what alignment tests
 * measure) is smaller, because the newest subsample is already 0.5 (2x) or 0.75 (4x) base
 * samples later than the 1x sample: 2x = 3.0, 4x = 4.5 (fractional; integer search ties 4/5).
 * DC/passband gain is 0.995 per stage (taps sum to 0.995, not 1.0). Verified by alignment_tests.cpp.
 */
class HalfbandDecimator2x {
public:
    void reset() noexcept {
        state_.fill(0.0f);
        writeIdx_ = 0;
    }

    float process2To1(float in0, float in1) noexcept {
        pushSample(in0);
        pushSample(in1);

        float acc = 0.5f * readTap(7);
        acc += kTap1 * (readTap(6) + readTap(8));
        acc += kTap3 * (readTap(4) + readTap(10));
        acc += kTap5 * (readTap(2) + readTap(12));
        acc += kTap7 * (readTap(0) + readTap(14));
        return acc;
    }

private:
    static constexpr float kTap1 =  0.31328195f;
    static constexpr float kTap3 = -0.08976314f;
    static constexpr float kTap5 =  0.02901480f;
    static constexpr float kTap7 = -0.00503361f;

    std::array<float, 16> state_{};
    size_t writeIdx_{0};

    void pushSample(float x) noexcept {
        state_[writeIdx_ & 15] = x;
        ++writeIdx_;
    }

    float readTap(size_t delay) const noexcept {
        return state_[(writeIdx_ - 1 - delay) & 15];
    }
};

/**
 * Per-sample mix-bus limiter, zero latency (no lookahead).
 *  Stage 1: Feed-forward peak limiter.
 *           Detector: instant attack, exponential release (releaseMs governs detector decay).
 *           Gain: attack-smoothed on the way down; when target >= gain, gain tracks target directly,
 *                 making release rate fully governed by the 80ms detector decay envelope.
 *  Stage 2: Asymptotic safety ceiling clip with narrow knee at 0.944f (room = 1.0 - knee = 0.056).
 *           Applies a C1 tanh transition from 0.944f asymptotically approaching 1.0f (saturates
 *           within 0.168 above knee, i.e., input >= 1.11f reaches >= 0.9995f). This acts as a
 *           tight safety ceiling catching transient overshoot during attack smoothing without
 *           hard digital wrapping or discontinuity, guaranteeing |y| <= 1.0f.
 * Intended placement: integrated directly into PulsaretPDCore after the voice sum and N^-exponent headroom gain.
 */
class MixLimiter {
public:
    void prepare(float sampleRate,
                 float thresholdLin = 0.891f,   // about -1 dBFS
                 float attackMs     = 0.5f,
                 float releaseMs    = 80.0f,
                 float safetyKnee   = 0.944f) noexcept   // about -0.5 dBFS; >= 1.0 disables the clip
    {
        sr_ = (sampleRate > 8000.0f) ? sampleRate : 48000.0f;
        thr_ = std::clamp(thresholdLin, 0.05f, 1.0f);
        knee_ = safetyKnee;
        atkCoef_ = 1.0f - std::exp(-1.0f / (std::max(attackMs, 0.01f) * 0.001f * sr_));
        detDecay_ = std::exp(-1.0f / (std::max(releaseMs, 1.0f) * 0.001f * sr_));
        reset();
    }

    void reset() noexcept { env_ = 0.0f; gain_ = 1.0f; }

    float process(float x) noexcept {
        const float a = std::fabs(x);
        env_ = (a > env_) ? a : env_ * detDecay_;
        const float target = (env_ > thr_) ? (thr_ / env_) : 1.0f;
        gain_ = (target < gain_) ? gain_ + (target - gain_) * atkCoef_ : target;
        return softKnee(x * gain_);
    }

    float lastGain() const noexcept { return gain_; }

private:
    // Asymptotic safety ceiling clip: narrow knee at knee_ (default 0.944f, room = 0.056f)
    // catches attack-phase overshoot and smoothly compresses it below 1.0f via C1 tanh.
    float softKnee(float y) const noexcept {
        if (knee_ >= 1.0f) return y;
        const float ay = std::fabs(y);
        if (ay <= knee_) return y;
        const float room = 1.0f - knee_;
        const float s = knee_ + room * std::tanh((ay - knee_) / room);
        return (y < 0.0f) ? -s : s;
    }

    float sr_{48000.0f};
    float thr_{0.891f};
    float knee_{0.944f};
    float atkCoef_{0.04081054f};
    float detDecay_{0.99973962f};
    float env_{0.0f};
    float gain_{1.0f};
};

/**
 * Headroom gain for N active voices: N^-exponent.
 * Measured (worst-case settings, ceiling 0.74): exponent 0.8 leaves the limiter essentially idle
 * for N >= 8 and active on a few percent of samples at N = 4; 0.5 (1/sqrt N) keeps it working
 * 94-99% of the time; 1.0 (1/N) never engages it but is 2-5 dB quieter. The limiter, not this
 * gain, is what guarantees |y| <= 1.0.
 */
inline float polyHeadroomGain(size_t activeVoices, float exponent = 0.8f) noexcept {
    return std::pow(static_cast<float>(std::max<size_t>(activeVoices, 1)), -exponent);
}

enum class NormStrategy : uint8_t {
    ProductionLocked = 0, // 0.74f voice ceiling + N^-0.8 + 2-Stage MixLimiter
    PowerLawSweep    = 1, // N^-p parameterized headroom
    Linear           = 2, // 1/N strict theoretical bound (RMS penalty)
    AdaptiveAGC      = 3, // Lookahead dual-ballistics AGC
    EqualEnergy      = 4, // 1/sqrt(N)
    Unnormalized     = 5, // Raw sum baseline
};

/**
 * Production Stateful Lookahead Dual-Ballistics Dynamic Automatic Gain Control (C++20)
 */
template <size_t LookaheadSamples = 32>
class DynamicPolyAGC {
public:
    struct Config {
        float attackMs{0.5f};
        float releaseMs{50.0f};
        float targetCeiling{0.95f};
        bool  voiceAdaptation{true};
        float voiceExponent{0.22f};
        bool  crestAdaptation{true};
        float makeupGainDb{0.0f};
        float kneeWidthDb{2.0f};
    };

    void prepare(float sampleRate = 48000.0f, const Config& config = {}) noexcept
    {
        sampleRate_ = sampleRate > 8000.0f ? sampleRate : 48000.0f;
        config_ = config;
        config_.targetCeiling = std::clamp(config_.targetCeiling, 0.40f, 0.98f);
        const float dt = 1.0f / sampleRate_;
        const float tauAtt = std::max(0.0001f, config_.attackMs * 0.001f);
        const float tauRel = std::max(0.005f, config_.releaseMs * 0.001f);
        constexpr float tauRms = 0.035f; // 35ms leaky RMS
        attackCoef_ = 1.0f - std::exp(-dt / tauAtt);
        baseReleaseCoef_ = 1.0f - std::exp(-dt / tauRel);
        rmsCoef_ = 1.0f - std::exp(-dt / tauRms);
        reset();
    }

    void reset() noexcept {
        delayBuffer_.fill(0.0f);
        writePos_ = 0;
        peakEnvelope_ = 0.0f;
        rmsEnvelopeSq_ = 0.0f;
        gain_ = 1.0f;
    }

    float process(float x, size_t activeVoices = 1) noexcept {
        const float ax = std::fabs(x);

        // 1. Dual-ballistics detector
        const float xSq = x * x;
        rmsEnvelopeSq_ += (xSq - rmsEnvelopeSq_) * rmsCoef_;
        const float rms = std::sqrt(std::max(1e-8f, rmsEnvelopeSq_));

        if (ax > peakEnvelope_) {
            peakEnvelope_ = ax;
        } else {
            peakEnvelope_ += (ax - peakEnvelope_) * baseReleaseCoef_;
        }

        // 2. Real-time Crest Factor
        const float crestRatio = peakEnvelope_ / std::max(1e-4f, rms);
        const float crestDb = 20.0f * std::log10(std::max(1.0f, crestRatio));

        // 3. Dynamic Crest-Factor Adaptive Release
        float effectiveRelCoef = baseReleaseCoef_;
        if (config_.crestAdaptation) {
            const float crestScale = std::clamp(12.0f / std::max(2.0f, crestDb), 0.4f, 2.2f);
            const float effectiveRelMs = config_.releaseMs * crestScale;
            const float dt = 1.0f / sampleRate_;
            const float tauRel = std::max(0.005f, effectiveRelMs * 0.001f);
            effectiveRelCoef = 1.0f - std::exp(-dt / tauRel);
        }

        // 4. Voice-Density Headroom Adaptation
        float targetCeil = config_.targetCeiling;
        if (config_.voiceAdaptation && activeVoices > 1) {
            const float voiceHeadroom = std::pow(static_cast<float>(activeVoices), -config_.voiceExponent);
            targetCeil = std::clamp(config_.targetCeiling * voiceHeadroom * std::pow(2.0f, config_.voiceExponent), 0.35f, config_.targetCeiling);
        }

        // 5. Effective Envelope & Soft-Knee Gain
        const float effectiveEnv = std::max(peakEnvelope_, rms * 1.414f);
        float targetGain = 1.0f;
        if (effectiveEnv > 1e-5f) {
            const float envDb = 20.0f * std::log10(effectiveEnv);
            const float threshDb = 20.0f * std::log10(targetCeil);
            const float knee = std::max(0.1f, config_.kneeWidthDb);
            const float diff = envDb - threshDb;

            float grDb = 0.0f;
            if (diff > knee * 0.5f) {
                grDb = -diff;
            } else if (diff > -knee * 0.5f) {
                const float k = diff + knee * 0.5f;
                grDb = -(k * k) / (2.0f * knee);
            }

            const float makeupDb = std::max(0.0f, config_.makeupGainDb);
            const float totalGainDb = std::min(makeupDb, grDb + makeupDb);
            targetGain = std::pow(10.0f, totalGainDb * 0.05f);
        }

        // 6. Asymmetric gain smoothing
        if (targetGain < gain_) {
            gain_ += (targetGain - gain_) * attackCoef_;
        } else {
            gain_ += (targetGain - gain_) * effectiveRelCoef;
        }

        // 7. Lookahead delay line
        delayBuffer_[writePos_] = x;
        const size_t readPos = (writePos_ + delayBuffer_.size() - LookaheadSamples) % delayBuffer_.size();
        const float delayed = delayBuffer_[readPos];
        writePos_ = (writePos_ + 1) % delayBuffer_.size();

        // 8. Output scaling & C1 continuous soft-knee backstop
        const float scaled = delayed * gain_;
        const float as = std::fabs(scaled);
        if (as > 0.95f) {
            constexpr float kRoom = 0.05f;
            const float bound = 0.95f + kRoom * std::tanh((as - 0.95f) / kRoom);
            return (scaled < 0.0f) ? -bound : bound;
        }
        return scaled;
    }

    float lastGain() const noexcept { return gain_; }
    float peakEnvelope() const noexcept { return peakEnvelope_; }

private:
    Config config_{};
    float sampleRate_{48000.0f};
    float attackCoef_{0.0f};
    float baseReleaseCoef_{0.0f};
    float rmsCoef_{0.0f};
    float peakEnvelope_{0.0f};
    float rmsEnvelopeSq_{0.0f};
    float gain_{1.0f};
    size_t writePos_{0};
    std::array<float, LookaheadSamples * 2> delayBuffer_{};
};

template <size_t LookaheadSamples = 32>
using AdaptivePolyAGC = DynamicPolyAGC<LookaheadSamples>;

/**
 * Wrapperless Polyphonic PulsaretPDCore Engine with Integrated MixLimiter
 * Zero host dependencies — directly embeddable in VCV Rack or SynthEdit.
 * Supports per-voice raw decimated synthesis (processVoiceSampleRaw),
 * per-voice DC-blocked synthesis (processVoiceSample), and core-integrated
 * N^-exponent headroom scaling + 2-stage zero-latency limiting (limitMixSample / processVoiceMix).
 */
template <size_t MaxPolyphony = 16>
class PulsaretPDCore {
public:
    void prepare(float sampleRate,
                 float dcBlockHz    = 10.0f,
                 float thresholdLin = 0.891f,
                 float attackMs     = 0.5f,
                 float releaseMs    = 80.0f,
                 float safetyKnee   = 0.944f) noexcept
    {
        sampleRate_ = (sampleRate > 8000.0f) ? sampleRate : 48000.0f;
        dcBlockHz_ = dcBlockHz;
        mixLimiter_.prepare(sampleRate_, thresholdLin, attackMs, releaseMs, safetyKnee);
        dynamicAgc_.prepare(sampleRate_);
        // Chord-gain attack: which default is right depends on how new voices enter.
        //  - Voices entering at phase 0 (window fades them in): instant attack puts a gain
        //    step on the already-sounding voices (+10..+21 dB curvature spike = click);
        //    setChordGainBallistics(false, 15, 0.3) removes it with no limiter engagement.
        //  - Voices entering at arbitrary phase (free-running, abrupt): 0.3 ms lets the
        //    unscaled sum through long enough to trip the limiter (down to -8.9 dB for
        //    ~85 ms at 1->16 voices); instant attack keeps the limiter idle.
        // Instant is kept as the default because it never engages the limiter.
        setChordGainBallistics(true, 15.0f, 0.3f);
        reset();
    }

    void prepareLimiter(float thresholdLin = 0.891f,
                        float attackMs     = 0.5f,
                        float releaseMs    = 80.0f,
                        float safetyKnee   = 0.944f) noexcept
    {
        mixLimiter_.prepare(sampleRate_, thresholdLin, attackMs, releaseMs, safetyKnee);
    }

    void setDcBlockHz(float dcBlockHz) noexcept {
        dcBlockHz_ = dcBlockHz;
        for (auto& dcb : dcBlockers_) {
            dcb.setCutoff(dcBlockHz_, sampleRate_);
        }
        busDcBlocker_.setCutoff(dcBlockHz_, sampleRate_);
    }

    void reset() noexcept {
        primaryPhase_.fill(0.0f);
        for (auto& s1 : decimStage1_) s1.reset();
        for (auto& s2 : decimStage2_) s2.reset();
        for (auto& dcb : dcBlockers_) {
            dcb.reset();
            dcb.setCutoff(dcBlockHz_, sampleRate_);
        }
        busDcBlocker_.reset();
        busDcBlocker_.setCutoff(dcBlockHz_, sampleRate_);
        mixLimiter_.reset();
        dynamicAgc_.reset();
        smoothedChordGain_ = 1.0f;
        chordGainPrimed_ = false;   // first updateChordGain() after reset snaps to target
    }

    void setVoicePhaseOffset(size_t voiceIdx, float phase01) noexcept {
        if (voiceIdx < MaxPolyphony) {
            primaryPhase_[voiceIdx] = phase01 - std::floor(phase01);
        }
    }

    // Per-output-sample constants. Everything here depends only on the parameters and the
    // rate, so it is computed once per output sample instead of once per sub-sample.
    // Values are computed with exactly the same float operations as before, so the
    // sub-sample results are unchanged.
    struct SubParams {
        float dt;                 // phase increment per sub-sample
        float d, slope1, slope2;  // primary knee
        float formantRatio;       // formant / fundamental
        float d2, slope2a, slope2b;   // secondary knee: 0.5/d2 and 0.5/(1-d2)
        float duty;
        float mix, oneMinusMix;
        float ceiling;
        FormantWindow window;
    };

    static SubParams makeSubParams(const VoiceParameters& p, float oversampledRate) noexcept {
        // Formant range clamp (absolute Hz). Above ~4 kHz the secondary carrier's
        // aliasing rises steeply and independently of pitch (measured: -35 dB @4k,
        // -29 dB @5k, -23 dB @8k at mix 1, 4x). Applied after any chord pitch-ratio
        // scaling, so a transposed chord voice is also held at the ceiling.
        static constexpr float kFormantMinHz = 200.0f;
        static constexpr float kFormantMaxHz = 4000.0f;
        SubParams sp{};
        sp.dt = p.fundamentalHz / oversampledRate;
        sp.d = std::clamp(p.pdDistortion, 0.02f, 0.98f);
        sp.slope1 = 0.5f / sp.d;
        sp.slope2 = 0.5f / (1.0f - sp.d);
        const float formantHz = std::clamp(p.formantHz, kFormantMinHz, kFormantMaxHz);
        sp.formantRatio = formantHz / ((p.fundamentalHz > 1.0f) ? p.fundamentalHz : 1.0f);
        sp.d2 = 1.0f - sp.d * 0.65f;
        sp.slope2a = 0.5f / sp.d2;
        sp.slope2b = 0.5f / (1.0f - sp.d2);
        sp.duty = std::clamp(p.pulsaretDuty, 0.1f, 1.0f);
        sp.mix = p.secondaryPdMix;
        sp.oneMinusMix = 1.0f - p.secondaryPdMix;
        sp.ceiling = p.voiceCeiling;
        sp.window = p.windowShape;
        return sp;
    }

    float synthesizeSubSample(size_t voiceIdx, const SubParams& sp) noexcept {
        float phi1 = primaryPhase_[voiceIdx];
        float out = 0.0f;

        // Outside the pulsaret (phi1 >= duty) the window is exactly 0, so the carriers are
        // not evaluated at all: the result was 0 * carrier before as well.
        if (phi1 < sp.duty) {
            const float u = phi1 / sp.duty;
            const float env = evaluateWindow(u, sp.window);
            if (env != 0.0f) {
                constexpr float kTwoPi = 2.0f * std::numbers::pi_v<float>;
                float carrier = 0.0f;
                if (sp.oneMinusMix != 0.0f) {
                    const float warped1 = (phi1 < sp.d) ? (phi1 * sp.slope1) : (0.5f + (phi1 - sp.d) * sp.slope2);
                    carrier += sp.oneMinusMix * -std::cos(kTwoPi * warped1);
                }
                if (sp.mix != 0.0f) {
                    // x - floor(x) equals fmod(x, 1) exactly for x >= 0, and is cheaper.
                    const float x = phi1 * sp.formantRatio;
                    const float phi2 = x - std::floor(x);
                    const float warped2 = (phi2 < sp.d2) ? (phi2 * sp.slope2a) : (0.5f + (phi2 - sp.d2) * sp.slope2b);
                    carrier += sp.mix * std::sin(kTwoPi * warped2);
                }
                out = env * carrier;
            }
        }

        phi1 += sp.dt;
        if (phi1 >= 1.0f) phi1 -= 1.0f;
        primaryPhase_[voiceIdx] = phi1;

        return out * sp.ceiling;
    }

    // Kept for API compatibility: one sub-sample with parameters computed on the spot.
    float synthesizeSubSample(size_t voiceIdx, const VoiceParameters& p, float oversampledRate) noexcept {
        return synthesizeSubSample(voiceIdx, makeSubParams(p, oversampledRate));
    }

    float processVoiceSampleRaw(size_t voiceIdx, const VoiceParameters& p, OversampleFactor os) noexcept {
        if (os == OversampleFactor::None_1x) {
            return synthesizeSubSample(voiceIdx, makeSubParams(p, sampleRate_));
        }
        if (os == OversampleFactor::Halfband_2x) {
            const SubParams sp = makeSubParams(p, sampleRate_ * 2.0f);
            const float s0 = synthesizeSubSample(voiceIdx, sp);
            const float s1 = synthesizeSubSample(voiceIdx, sp);
            return decimStage1_[voiceIdx].process2To1(s0, s1);
        }
        const SubParams sp = makeSubParams(p, sampleRate_ * 4.0f);
        const float s0 = synthesizeSubSample(voiceIdx, sp);
        const float s1 = synthesizeSubSample(voiceIdx, sp);
        const float s2 = synthesizeSubSample(voiceIdx, sp);
        const float s3 = synthesizeSubSample(voiceIdx, sp);
        const float mid0 = decimStage1_[voiceIdx].process2To1(s0, s1);
        const float mid1 = decimStage1_[voiceIdx].process2To1(s2, s3);
        return decimStage2_[voiceIdx].process2To1(mid0, mid1);
    }

    float processVoiceSample(size_t voiceIdx, const VoiceParameters& p, OversampleFactor os) noexcept {
        const float raw = processVoiceSampleRaw(voiceIdx, p, os);
        return dcBlockers_[voiceIdx].process(raw);
    }

    // Calibration harmonic cluster chord ratios (16 voices) matching the calibration suite:
    static constexpr std::array<float, 16> kDefaultChordRatios = {
        1.0f,       // Voice 0: Root (1:1)
        1.259921f,  // Voice 1: Major 3rd
        1.4983071f, // Voice 2: Perfect 5th
        1.8877486f, // Voice 3: Major 7th
        2.2449241f, // Voice 4: Major 9th
        0.5f,       // Voice 5: Sub-Octave
        1.6817928f, // Voice 6: Major 6th
        2.6696797f, // Voice 7: 11th
        0.7491535f, // Voice 8: Lower 5th
        3.0f,       // Voice 9: Octave + 5th
        1.122462f,  // Voice 10: Major 2nd
        1.3348398f, // Voice 11: Perfect 4th
        2.0f,       // Voice 12: Octave
        2.5198421f, // Voice 13: Octave + 3rd
        0.8408964f, // Voice 14: Lower 6th
        3.3635857f  // Voice 15: Two Octaves + 6th
    };

    // Updates bus chord headroom gain with asymmetric ballistics:
    // - On chord-entry / voice addition (targetGain < current): instant or fast attack ensures
    //   immediate headroom is available, preventing the -9.4 dB transient limiter clamp.
    // - On voice release (targetGain > current): smooth release (tau ~ 15ms) prevents volume pumping.
    void updateChordGain(float targetGain) noexcept {
        if (!chordGainPrimed_) {
            // Cold start: no previous voice count to smooth from. Starting at 1.0 would
            // pass the unscaled N-voice sum for the attack time and trip the limiter.
            smoothedChordGain_ = targetGain;
            chordGainPrimed_ = true;
            return;
        }
        if (targetGain < smoothedChordGain_) {
            if (instantChordAttack_) {
                smoothedChordGain_ = targetGain;
            } else {
                smoothedChordGain_ += (targetGain - smoothedChordGain_) * chordGainAttackCoef_;
            }
        } else {
            smoothedChordGain_ += (targetGain - smoothedChordGain_) * chordGainReleaseCoef_;
        }
    }

    void snapChordGain(size_t activeVoices, float exponent = 0.8f) noexcept {
        smoothedChordGain_ = polyHeadroomGain(activeVoices, exponent);
        chordGainPrimed_ = true;
    }

    void setChordGainBallistics(bool instantAttack, float releaseMs = 15.0f, float attackMs = 0.2f) noexcept {
        instantChordAttack_ = instantAttack;
        const float dt = 1.0f / sampleRate_;
        chordGainAttackCoef_ = 1.0f - std::exp(-dt / std::max(0.0001f, attackMs * 0.001f));
        chordGainReleaseCoef_ = 1.0f - std::exp(-dt / std::max(0.001f, releaseMs * 0.001f));
    }

    // Bus normalization stage applying the chosen strategy
    float processBusSample(float sum, size_t nVoices, NormStrategy strategy, float exponent) noexcept {
        switch (strategy) {
            case NormStrategy::ProductionLocked:
                return limitMixSample(sum, nVoices, 0.8f, false);

            case NormStrategy::PowerLawSweep:
                return limitMixSample(sum, nVoices, exponent, false);

            case NormStrategy::Linear: {
                const float targetGain = 1.0f / static_cast<float>(nVoices);
                updateChordGain(targetGain);
                return sum * smoothedChordGain_;
            }

            case NormStrategy::AdaptiveAGC:
                return dynamicAgc_.process(sum, nVoices);

            case NormStrategy::EqualEnergy: {
                const float targetGain = 1.0f / std::sqrt(static_cast<float>(nVoices));
                updateChordGain(targetGain);
                return mixLimiter_.process(sum * smoothedChordGain_);
            }

            case NormStrategy::Unnormalized:
            default:
                return sum;
        }
    }

    // Core-integrated mix-bus stage: applies N^-exponent headroom gain with asymmetric smoothing, optional bus DC-blocker, and 2-stage MixLimiter
    float limitMixSample(float voiceSum, size_t activeVoices, float exponent = 0.8f, bool applyBusDcBlock = false) noexcept {
        const float cleanSum = applyBusDcBlock ? busDcBlocker_.process(voiceSum) : voiceSum;
        const float targetGain = polyHeadroomGain(activeVoices, exponent);
        updateChordGain(targetGain);
        const float scaled = cleanSum * smoothedChordGain_;
        return mixLimiter_.process(scaled);
    }

    // Unison mix processor (all voices share base fundamental frequency and parameters)
    float processVoiceMix(size_t activeVoices,
                          const VoiceParameters& baseParams,
                          OversampleFactor os,
                          NormStrategy strategy = NormStrategy::ProductionLocked,
                          float exponent = 0.8f) noexcept
    {
        const size_t nVoices = std::clamp<size_t>(activeVoices, 1, MaxPolyphony);
        float sum = 0.0f;
        for (size_t v = 0; v < nVoices; ++v) {
            sum += processVoiceSample(v, baseParams, os);
        }
        return processBusSample(sum, nVoices, strategy, exponent);
    }

    // Harmonic chord mix processor using pitch ratios (defaulting to kDefaultChordRatios)
    // formantTracksPitch = true : each voice's formant scales with its pitch ratio
    //                             (constant formant/f0 ratio -> same timbre transposed).
    // formantTracksPitch = false: every voice shares baseParams.formantHz
    //                             (fixed resonance -> vocal/body-like formant).
    float processChordMix(size_t activeVoices,
                          const VoiceParameters& baseParams,
                          OversampleFactor os,
                          NormStrategy strategy = NormStrategy::ProductionLocked,
                          float exponent = 0.8f,
                          const float* pitchRatios = nullptr,
                          bool formantTracksPitch = true) noexcept
    {
        const size_t nVoices = std::clamp<size_t>(activeVoices, 1, MaxPolyphony);
        const float* ratios = (pitchRatios != nullptr) ? pitchRatios : kDefaultChordRatios.data();
        float sum = 0.0f;
        for (size_t v = 0; v < nVoices; ++v) {
            VoiceParameters vp = baseParams;
            const float r = ratios[v];
            vp.fundamentalHz *= r;
            if (formantTracksPitch) vp.formantHz *= r;
            sum += processVoiceSample(v, vp, os);
        }
        return processBusSample(sum, nVoices, strategy, exponent);
    }

    // Per-voice parameter array mix processor for arbitrary polyphonic synthesis
    float processVoiceMix(const VoiceParameters* voiceParamsArray,
                          size_t activeVoices,
                          OversampleFactor os,
                          NormStrategy strategy = NormStrategy::ProductionLocked,
                          float exponent = 0.8f) noexcept
    {
        const size_t nVoices = std::clamp<size_t>(activeVoices, 1, MaxPolyphony);
        float sum = 0.0f;
        for (size_t v = 0; v < nVoices; ++v) {
            sum += processVoiceSample(v, voiceParamsArray[v], os);
        }
        return processBusSample(sum, nVoices, strategy, exponent);
    }

    // Direct access to the core's internal MixLimiter for pre-scaled signals
    // Precondition: For optimal limiter tracking, preScaledMix should be free of large DC offsets.
    float processMixSample(float preScaledMix, bool applyBusDcBlock = false) noexcept {
        const float clean = applyBusDcBlock ? busDcBlocker_.process(preScaledMix) : preScaledMix;
        return mixLimiter_.process(clean);
    }

    float lastLimiterGain() const noexcept {
        return mixLimiter_.lastGain();
    }

    float currentChordGain() const noexcept {
        return smoothedChordGain_;
    }

    MixLimiter& mixLimiter() noexcept { return mixLimiter_; }
    const MixLimiter& mixLimiter() const noexcept { return mixLimiter_; }

    DynamicPolyAGC<32>& dynamicAgc() noexcept { return dynamicAgc_; }
    const DynamicPolyAGC<32>& dynamicAgc() const noexcept { return dynamicAgc_; }

    DCBlocker1P& busDcBlocker() noexcept { return busDcBlocker_; }

private:
    float sampleRate_{48000.0f};
    float dcBlockHz_{10.0f};
    std::array<float, MaxPolyphony> primaryPhase_{};
    std::array<HalfbandDecimator2x, MaxPolyphony> decimStage1_{};
    std::array<HalfbandDecimator2x, MaxPolyphony> decimStage2_{};
    std::array<DCBlocker1P, MaxPolyphony> dcBlockers_{};
    DCBlocker1P busDcBlocker_{};
    MixLimiter mixLimiter_{};
    DynamicPolyAGC<32> dynamicAgc_{};

    // Asymmetric chord-entry gain ballistics
    float smoothedChordGain_{1.0f};
    bool  instantChordAttack_{true};    // Instant headroom adjustment on voice addition
    bool  chordGainPrimed_{false};      // false until the first gain update after reset()
    float chordGainAttackCoef_{1.0f};   // Fallback fast-attack coefficient
    float chordGainReleaseCoef_{0.00139f}; // tau ~ 15ms release recovery on voice note-off (1 - exp(-1 / (0.015 * 48000)))

    static float evaluateWindow(float u, FormantWindow shape) noexcept {
        if (u <= 0.0f || u >= 1.0f) return 0.0f;
        constexpr float kTwoPi = 2.0f * std::numbers::pi_v<float>;
        if (shape == FormantWindow::Hann) {
            return 0.5f * (1.0f - std::cos(kTwoPi * u));
        }
        if (shape == FormantWindow::BlackmanHarris) {
            const float w = kTwoPi * u;
            return 0.35875f - 0.48829f * std::cos(w)
                 + 0.14128f * std::cos(2.0f * w) - 0.01168f * std::cos(3.0f * w);
        }
        if (u < 0.25f) return 0.5f * (1.0f + std::cos(std::numbers::pi_v<float> * (4.0f * u - 1.0f)));
        if (u > 0.75f) return 0.5f * (1.0f + std::cos(std::numbers::pi_v<float> * (4.0f * u - 3.0f)));
        return 1.0f;
    }
};

} // namespace pulsaret::dsp