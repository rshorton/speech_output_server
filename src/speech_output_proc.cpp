/*
Copyright 2021 Scott Horton

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include <speechapi_cxx.h>

#include <pulse/simple.h>
#include <pulse/error.h>
#include <pulse/gccmacro.h>

#include <openssl/md5.h>

#include "rclcpp/rclcpp.hpp"

#include "speech_output_proc.h"

using namespace std;
using namespace Microsoft::CognitiveServices::Speech;
using namespace Microsoft::CognitiveServices::Speech::Audio;

using std::chrono::system_clock;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::duration_cast;

#define SAMPLE_RATE         16000       // input sampling rate (filters assume this rate)
#define SAMPLE_BITS         16          // 16 bits per sample is the max size for the PDM MIC input
#define NUM_CHANNELS		1			// 1 channel

#define WAV_HEADER_LEN		44

const std::string wavCacheDir = "./speech_wav_cache/";

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

class AudioOutput
{
public:
	AudioOutput():
		_handle(NULL)
	{
	}

	~AudioOutput()
	{
		Close();
	}

	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////

	int Open()
	{
		if (_handle) {
			return -1;
		}

	    pa_sample_spec ss;
	    ss.format = PA_SAMPLE_S16LE;
	    ss.rate = SAMPLE_RATE;
	    ss.channels = NUM_CHANNELS;

		int err = -1;

		if (!(_handle = pa_simple_new(NULL, "speech_output_server", PA_STREAM_PLAYBACK, NULL, "playback", &ss, NULL, NULL, &err))) {
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR; Can't open pulseaudio for output. %s", pa_strerror(err));
			return err;
		}
		return 0;
	}

	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////

	int Close()
	{
		if (_handle) {
			pa_simple_free(_handle);
			_handle = NULL;
		}
		return 0;
	}

	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////

	void WriteComplete()
	{
		if (_handle) {
			int error;
			pa_simple_drain(_handle, &error);
		}
	}

	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////

	int WriteBlock(uint8_t* buffer, size_t num_bytes)
	{
		int err = -1;
		if (!_handle) {
			return -1;
		}

		if (pa_simple_write(_handle, buffer, num_bytes, &err) < 0) {
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR; Can't write to PulseAudio device. %s\n", pa_strerror(err));
			return err;
		}
		return 0;
	}

private:
	pa_simple *_handle;
};

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

enum SpeechSynthesisStatus {
	SpeechSynthesisStatus_Ok = 0,
	SpeechSynthesisStatus_InProg,
	SpeechSynthesisStatus_Error,
	SpeechSynthesisStatus_Done
};

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

class SpeechSynthesizerWrapper
{
public:
	SpeechSynthesizerWrapper():
		_synthesizer(nullptr),
		_open(false)
	{
	}

	~SpeechSynthesizerWrapper()
	{
	}

	SpeechSynthesisStatus Open()
	{
		// Speech recognizer setup
		const char* env_key = std::getenv("MS_COGNTIVE_SUB_KEY");
		const char* env_region = std::getenv("MS_COGNTIVE_SUB_REGION");
		if (!env_key || !env_region) {
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: MS Cogntive env vars MS_COGNTIVE_SUB_KEY/MS_COGNTIVE_SUB_REGION not set");
			return SpeechSynthesisStatus_Error;
		}
		auto config = SpeechConfig::FromSubscription(env_key, env_region);

	    config->SetSpeechSynthesisOutputFormat(SpeechSynthesisOutputFormat::Riff16Khz16BitMonoPcm);

	    // Creates a speech synthesizer using the default speaker as audio output. The default spoken language is "en-us".
	    _synthesizer = SpeechSynthesizer::FromConfig(config, NULL);

		_open = true;
		return SpeechSynthesisStatus_Ok;
	}

	SpeechSynthesisStatus Close()
	{
		return SpeechSynthesisStatus_Ok;
	}

	bool isOpen()
	{
		return _open;
	}

public:
	std::shared_ptr<SpeechSynthesizer> _synthesizer;

private:
	bool _open;
};

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

SpeechOutputProc::SpeechOutputProc():
		_open(false),
		_run(false)
{
}

SpeechOutputProc::~SpeechOutputProc()
{
	Close();
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

SpeechProcStatus SpeechOutputProc::Open()
{
	if (_open) {
		return SpeechProcStatus_Error;
	}

	_audio_output = std::make_unique<AudioOutput>();

	int ret = _audio_output->Open();
	if (ret) {
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR; Could not open audio output device (%s)\n", pa_strerror(ret));
		_audio_output.reset();
        return SpeechProcStatus_Error;
	}
	return SpeechProcStatus_Ok;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

SpeechProcStatus SpeechOutputProc::Close()
{
	if (!_open) {
		return SpeechProcStatus_Error;
	}

	StopProcessing();

	if (_audio_output != nullptr) {
		_audio_output->Close();
		_audio_output.reset();
	}
	_open = false;
	return SpeechProcStatus_Ok;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void SpeechOutputProc::StopProcessing()
{
	if (_run) {
		_run = false;
		_proc_thread.join();
	}
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

SpeechProcStatus SpeechOutputProc::SpeakStart(std::string text)
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting speech synthesis");

	// If processing last command, tell thread to stop, and then
	// wait for it to finished.  If not running, then cleanup thread
	// used for previous command (if any)
	StopProcessing();

	if (text.length() == 0) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Requested text string is empty");
		return SpeechProcStatus_Ok;
	}

	_text_to_speak = text;

	const std::lock_guard<std::mutex> lock(_mutex);
	if (_synthesizer_wrapper == nullptr) {
		_synthesizer_wrapper = std::make_unique<SpeechSynthesizerWrapper>();
	}
	if (!_synthesizer_wrapper->isOpen()) {
		if (_synthesizer_wrapper->Open() != SpeechSynthesisStatus_Ok) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open speech synthesizer");
			_synthesizer_wrapper.reset();
			return SpeechProcStatus_Error;
		}
	}

	// Start thread to do the processing
	_run = true;
	_proc_thread = std::thread{std::bind(&SpeechOutputProc::Process, this)};

	return SpeechProcStatus_Ok;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

SpeechProcStatus SpeechOutputProc::SpeakStop()
{
	StopProcessing();
	return SpeechProcStatus_Ok;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void SpeechOutputProc::SetSpeakCB(std::function<void(SpeechProcStatus)> callback)
{
	const std::lock_guard<std::mutex> lock(_mutex);
	_speak_cb = callback;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void SpeechOutputProc::SetSmileCB(std::function<void(std::string)> callback)
{
	const std::lock_guard<std::mutex> lock(_mutex);
	_smile_cb = callback;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void SpeechOutputProc::SetSpeechActiveCB(std::function<void(bool)> callback)
{
	const std::lock_guard<std::mutex> lock(_mutex);
	_speech_active_cb = callback;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void SpeechOutputProc::Process()
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SpeechOutputProc::Process: %s", _text_to_speak.c_str());

	uint8_t* audio_samples = NULL;
	int audio_num_bytes = 0;
	// Locally allocated buffer
	char* buffer = NULL;
	int16_t* buffer2chan = NULL;

	// Calc md5 over text to use a filename for cached speech file
	unsigned char md5[MD5_DIGEST_LENGTH];
	MD5((const unsigned char*)_text_to_speak.data(), _text_to_speak.length(), md5);

	stringstream sstr;
    for(int i = 0; i < MD5_DIGEST_LENGTH; i++) {
    	sstr << std::hex << (unsigned int)md5[i];
    }
    //cout << "MD5: " << sstr.str() << std::endl;

    std::string fname = wavCacheDir;
	fname += sstr.str();
    fname += "_tts.wav";

    // Try to read cached speech
    ifstream file(fname, ios::in | ios::binary | ios::ate);
    if (file.is_open()) {
    	streampos size = file.tellg();
    	buffer = new char [size];
    	file.seekg(0, ios::beg);
    	file.read(buffer, size);
    	file.close();

    	audio_samples = (uint8_t*)buffer;
    	audio_num_bytes = (int)size;
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Read cached speech: %s, size %d", fname.c_str(), audio_num_bytes);
    }

	const std::lock_guard<std::mutex> lock(_mutex);

    // Synthesize if audio wasn't cached
    std::shared_ptr<SpeechSynthesisResult> result;
    std::shared_ptr<std::vector<uint8_t>> audio;

    if (_run) {
    	if (audio_num_bytes == 0) {
    		if (_synthesizer_wrapper != nullptr && _synthesizer_wrapper->_synthesizer != nullptr) {
    			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Running speech synthesis");

				result = _synthesizer_wrapper->_synthesizer->SpeakSsmlAsync(_text_to_speak).get();

				if (result->Reason == ResultReason::SynthesizingAudioCompleted)
				{
					audio = result->GetAudioData();
					RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speech synthesized, size: %lu", audio->size());

					if (audio->size() > 0) {
						audio_samples = audio->data();
						audio_num_bytes = audio->size();

						// Cache the audio to a file to avoid needing to convert this same text in the future
						auto stream = AudioDataStream::FromResult(result);
						stream->SaveToWavFileAsync(fname).get();
					} else {
						RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: Synthesized speech too small");
						_speak_cb(SpeechProcStatus_Error);
						return;
					}
				}
				else if (result->Reason == ResultReason::Canceled)
				{
					auto cancellation = SpeechSynthesisCancellationDetails::FromResult(result);
					RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: Synthesis cancelled");
					_speak_cb(SpeechProcStatus_Error);
					return;
				}
    		}
    	}

    	// Skip past WAV header
    	if (audio_num_bytes > WAV_HEADER_LEN) {
    		audio_num_bytes -= WAV_HEADER_LEN;
    		audio_samples += WAV_HEADER_LEN;
    	}

		// Chop off silence at the end to reduce delay between a spoken prompt
		// and the start of listening.
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Removing silence at the end of audio.");
		int16_t *pSamples = (int16_t*)audio_samples;
		int numSamples = audio_num_bytes/2;

		int offset;
		for (offset = numSamples - 1; offset >= 0; offset--) {
			if (abs(pSamples[offset]) > 0xa) {
				break;
			}
		}

		// Play the file...
		_speech_active_cb(true);

		_audio_output->WriteBlock(audio_samples, offset*2);
		_audio_output->WriteComplete();

		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "2");

		_speech_active_cb(false);

		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "3");

		if (buffer) {
			delete[] buffer;
		}
		if (buffer2chan) {
			delete[] buffer2chan;
		}

		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "4");

	}
	// Was cancelled if !_run
	_speak_cb(_run? SpeechProcStatus_Ok: SpeechProcStatus_Error);

}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

