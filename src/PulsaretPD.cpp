// PulsaretPD.cpp — VCV Rack 2 module wrapping PulsaretPDCore.
//
// Polyphonic: one pulsaret::host::Voice per channel of the V/OCT input (1..16). Each channel
// gets its own output channel; summing is left to the patch (as usual for Rack oscillators),
// so the core's chord bus and limiter are not used.
//
// Voltage conventions (VCV standards):
//   V/OCT, FORMANT CV : 1 V per octave, added to the knob
//   DIST, MIX, DUTY CV: 0..10 V sweeps the full range, added to the knob
//   SYNC              : rising through 0 V restarts the waveform; unpatched = free-running
//   OUT               : strictly within +-5 V
#include "plugin.hpp"
#include "PulsaretHostVoice.hpp"
#include <cmath>

struct PulsaretPD : Module {
	enum ParamId {
		PITCH_PARAM,
		FORMANT_PARAM,
		DIST_PARAM,
		MIX_PARAM,
		DUTY_PARAM,
		WINDOW_PARAM,
		OVERSAMPLE_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		VOCT_INPUT,
		FORMANT_INPUT,
		DIST_INPUT,
		MIX_INPUT,
		DUTY_INPUT,
		SYNC_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		OUT_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		LIGHTS_LEN
	};

	static constexpr int kMaxChannels = 16;
	static constexpr float kFormantMinHz = 200.f;
	static constexpr float kFormantMaxHz = 4000.f;

	pulsaret::host::Voice voices[kMaxChannels];
	float preparedSampleRate = 0.f;
	int windowIndex = -1;
	int oversampleIndex = -1;

	PulsaretPD() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		// Same pitch knob convention as VCV Fundamental VCO: semitones around C4.
		configParam(PITCH_PARAM, -54.f, 54.f, 0.f, "Frequency", " Hz", dsp::FREQ_SEMITONE, dsp::FREQ_C4);
		// Stored as log2(Hz) so the knob is exponential; displayed in Hz.
		configParam(FORMANT_PARAM, std::log2(kFormantMinHz), std::log2(kFormantMaxHz), std::log2(880.f), "Formant", " Hz", 2.f, 1.f);
		configParam(DIST_PARAM, 0.02f, 0.98f, 0.28f, "Phase distortion knee (d)");
		configParam(MIX_PARAM, 0.f, 1.f, 0.35f, "Formant carrier mix", "%", 0.f, 100.f);
		configParam(DUTY_PARAM, 0.1f, 1.f, 0.65f, "Pulsaret duty", "%", 0.f, 100.f);
		configSwitch(WINDOW_PARAM, 0.f, 2.f, 1.f, "Window", {"Hann", "Blackman-Harris", "Tukey"});
		configSwitch(OVERSAMPLE_PARAM, 0.f, 2.f, 2.f, "Oversampling", {"1x", "2x", "4x"});

		configInput(VOCT_INPUT, "1V/octave pitch");
		configInput(FORMANT_INPUT, "Formant (1V/octave)");
		configInput(DIST_INPUT, "Phase distortion CV");
		configInput(MIX_INPUT, "Mix CV");
		configInput(DUTY_INPUT, "Duty CV");
		configInput(SYNC_INPUT, "Sync");
		configOutput(OUT_OUTPUT, "Audio");
	}

	void prepareVoices(float sampleRate) {
		for (int c = 0; c < kMaxChannels; c++) {
			voices[c].prepare(sampleRate);
			if (windowIndex >= 0) voices[c].setWindow(windowIndex);
			if (oversampleIndex >= 0) voices[c].setOversampling(oversampleIndex);
		}
		preparedSampleRate = sampleRate;
	}

	void onSampleRateChange(const SampleRateChangeEvent& e) override {
		prepareVoices(e.sampleRate);
	}

	void onReset() override {
		if (preparedSampleRate > 0.f) prepareVoices(preparedSampleRate);
	}

	void process(const ProcessArgs& args) override {
		if (args.sampleRate != preparedSampleRate) prepareVoices(args.sampleRate);

		const int w = (int) std::round(params[WINDOW_PARAM].getValue());
		if (w != windowIndex) {
			windowIndex = w;
			for (int c = 0; c < kMaxChannels; c++) voices[c].setWindow(w);
		}
		const int o = (int) std::round(params[OVERSAMPLE_PARAM].getValue());
		if (o != oversampleIndex) {
			oversampleIndex = o;
			for (int c = 0; c < kMaxChannels; c++) voices[c].setOversampling(o);
		}

		const int channels = std::max(1, inputs[VOCT_INPUT].getChannels());
		const float pitchKnob = params[PITCH_PARAM].getValue() / 12.f;      // octaves from C4
		const float formantKnob = params[FORMANT_PARAM].getValue();         // log2(Hz)
		const float distKnob = params[DIST_PARAM].getValue();
		const float mixKnob = params[MIX_PARAM].getValue();
		const float dutyKnob = params[DUTY_PARAM].getValue();

		for (int c = 0; c < channels; c++) {
			const float pitch = pitchKnob + inputs[VOCT_INPUT].getPolyVoltage(c);
			const float f0 = dsp::FREQ_C4 * std::exp2(clamp(pitch, -12.f, 12.f));
			const float formantHz = std::exp2(clamp(formantKnob + inputs[FORMANT_INPUT].getPolyVoltage(c), 0.f, 15.f));
			const float dist = distKnob + inputs[DIST_INPUT].getPolyVoltage(c) * 0.1f;
			const float mix = mixKnob + inputs[MIX_INPUT].getPolyVoltage(c) * 0.1f;
			const float duty = dutyKnob + inputs[DUTY_INPUT].getPolyVoltage(c) * 0.1f;
			const bool sync = inputs[SYNC_INPUT].getPolyVoltage(c) > 0.f;

			const float y = voices[c].process(f0, clamp(formantHz, kFormantMinHz, kFormantMaxHz), dist, mix, duty, sync);
			outputs[OUT_OUTPUT].setVoltage(5.f * y, c);
		}
		outputs[OUT_OUTPUT].setChannels(channels);
	}
};

// Text labels are drawn in code with Rack's UI font: Rack's SVG loader does not render
// <text> elements, and this keeps the dev panel editable without a vector editor.
struct PulsaretLabels : widget::Widget {
	struct Label { float xMm, yMm; const char* text; float size; };
	std::vector<Label> labels;

	void draw(const DrawArgs& args) override {
		std::shared_ptr<window::Font> font = APP->window->uiFont;
		if (!font) return;
		nvgFontFaceId(args.vg, font->handle);
		nvgFillColor(args.vg, nvgRGB(0xe6, 0xe9, 0xf2));
		nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
		for (const Label& l : labels) {
			nvgFontSize(args.vg, l.size);
			const math::Vec p = mm2px(math::Vec(l.xMm, l.yMm));
			nvgText(args.vg, p.x, p.y, l.text, NULL);
		}
	}
};

struct PulsaretPDWidget : ModuleWidget {
	PulsaretPDWidget(PulsaretPD* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/PulsaretPD.svg")));   // 10 HP = 50.8 mm

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		const float colL = 14.0f, colR = 36.8f;                       // two-column rows
		const float c1 = 9.4f, c2 = 25.4f, c3 = 41.4f;                // three-column rows
		const float j1 = 8.0f, j2 = 19.6f, j3 = 31.2f, j4 = 42.8f;    // four-column jack rows

		addParam(createParamCentered<RoundBigBlackKnob>(mm2px(Vec(colL, 27.0f)), module, PulsaretPD::PITCH_PARAM));
		addParam(createParamCentered<RoundBigBlackKnob>(mm2px(Vec(colR, 27.0f)), module, PulsaretPD::FORMANT_PARAM));

		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(c1, 51.0f)), module, PulsaretPD::DIST_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(c2, 51.0f)), module, PulsaretPD::MIX_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(c3, 51.0f)), module, PulsaretPD::DUTY_PARAM));

		addParam(createParamCentered<CKSSThree>(mm2px(Vec(colL, 72.0f)), module, PulsaretPD::WINDOW_PARAM));
		addParam(createParamCentered<CKSSThree>(mm2px(Vec(colR, 72.0f)), module, PulsaretPD::OVERSAMPLE_PARAM));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(j1, 95.0f)), module, PulsaretPD::VOCT_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(j2, 95.0f)), module, PulsaretPD::FORMANT_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(j3, 95.0f)), module, PulsaretPD::DIST_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(j4, 95.0f)), module, PulsaretPD::MIX_INPUT));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(j1, 112.0f)), module, PulsaretPD::DUTY_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(j2, 112.0f)), module, PulsaretPD::SYNC_INPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(j4, 112.0f)), module, PulsaretPD::OUT_OUTPUT));

		PulsaretLabels* text = new PulsaretLabels;
		text->box.size = box.size;
		text->labels = {
			{25.4f, 9.0f, "PULSARET PD", 13.f},
			{colL, 16.5f, "PITCH", 9.f}, {colR, 16.5f, "FORMANT", 9.f},
			{c1, 43.0f, "DIST", 9.f}, {c2, 43.0f, "MIX", 9.f}, {c3, 43.0f, "DUTY", 9.f},
			{colL, 62.5f, "WINDOW", 9.f}, {colR, 62.5f, "OVERSAMPLE", 9.f},
			{colL + 9.5f, 68.6f, "TUK", 7.f}, {colL + 9.5f, 72.0f, "B-H", 7.f}, {colL + 9.5f, 75.4f, "HANN", 7.f},
			{colR + 8.0f, 68.6f, "4x", 7.f}, {colR + 8.0f, 72.0f, "2x", 7.f}, {colR + 8.0f, 75.4f, "1x", 7.f},
			{j1, 88.0f, "V/OCT", 8.f}, {j2, 88.0f, "FORM", 8.f}, {j3, 88.0f, "DIST", 8.f}, {j4, 88.0f, "MIX", 8.f},
			{j1, 105.0f, "DUTY", 8.f}, {j2, 105.0f, "SYNC", 8.f}, {j4, 105.0f, "OUT", 8.f},
			{25.4f, 122.5f, "LIMINAL INTENT", 8.f},
		};
		addChild(text);
	}
};

Model* modelPulsaretPD = createModel<PulsaretPD, PulsaretPDWidget>("PulsaretPD");
