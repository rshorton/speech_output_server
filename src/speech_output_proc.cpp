
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include <speechapi_cxx.h>

#include <alsa/asoundlib.h>
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

#define AUDIO_DEVICE 		"hw:0"
#define SAMPLE_RATE         16000       // input sampling rate (filters assume this rate)
#define SAMPLE_BITS         16          // 16 bits per sample is the max size for the PDM MIC input
#define NUM_CHANNELS		1

#define WAV_HEADER_LEN		44
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

class AudioOutput
{
public:
	AudioOutput():
		_handle(NULL),
		_period_size_frames(0)
	{
	}

	~AudioOutput()
	{
		Close();
	}

	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////

	int Open(const char* pDevice)
	{
		if (_handle) {
			return -1;
		}

		int err = -1;

		do {
			/* Open the PCM device in playback mode */
			if ((err = snd_pcm_open(&_handle, AUDIO_DEVICE, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR; Can't open [%s] PCM device. %s",
						AUDIO_DEVICE, snd_strerror(err));
				break;
			}

			snd_pcm_hw_params_t *params;

			/* Allocate parameters object and fill it with default values*/
			snd_pcm_hw_params_alloca(&params);

			snd_pcm_hw_params_any(_handle, params);

			/* Set parameters */
			if ((err = snd_pcm_hw_params_set_access(_handle, params,
							SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR; Can't set interleaved mode. %s", snd_strerror(err));
				break;
			}

			if ((err = snd_pcm_hw_params_set_format(_handle, params,
								SND_PCM_FORMAT_S16_LE)) < 0) {
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR; Can't set format. %s", snd_strerror(err));
				break;
			}

			if ((err = snd_pcm_hw_params_set_channels(_handle, params, NUM_CHANNELS)) < 0) {
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR; Can't set channels number. %s", snd_strerror(err));
				break;
			}

			unsigned int rate = SAMPLE_RATE;
			if ((err = snd_pcm_hw_params_set_rate_near(_handle, params, &rate, 0)) < 0) {
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR; Can't set rate. %s", snd_strerror(err));
				break;
			}

			/* Write parameters */
			if ((err = snd_pcm_hw_params(_handle, params)) < 0) {
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR; Can't set harware parameters. %s", snd_strerror(err));
				break;
			}

			snd_pcm_hw_params_get_period_size(params, &_period_size_frames, 0);
			return 0;

		} while(0);

		if (_handle) {
			snd_pcm_close(_handle);
			_handle = NULL;
		}
		return err;
	}

	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////

	int Close()
	{
		if (_handle) {
			snd_pcm_close(_handle);
			_handle = NULL;
		}
		return 0;
	}

	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////

	int GetWriteSize()
	{
		if (_period_size_frames == 0) {
			_period_size_frames = 1024;
		}
		return _period_size_frames * (NUM_CHANNELS*SAMPLE_BITS/8);
	}

	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////

	void WriteComplete()
	{
		if (_handle) {
			snd_pcm_drain(_handle);
		}
	}

	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////

	int WriteBlock(uint8_t* buffer, int num_bytes)
	{
		int err = -1;
		if (!_handle) {
			return -1;
		}

		int frames = num_bytes/(NUM_CHANNELS*SAMPLE_BITS/8);
		if ((err = snd_pcm_writei(_handle, buffer, frames)) == -EPIPE) {
			snd_pcm_prepare(_handle);
		} else if (err < 0) {
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR; Can't write to PCM device. %s\n", snd_strerror(err));
			return err;
		}
		return 0;
	}

private:
	snd_pcm_t *_handle;
	snd_pcm_uframes_t _period_size_frames;
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
		_run(false),
		_synthesizer_wrapper(NULL),
		_audio_output(nullptr)
{
}

SpeechOutputProc::~SpeechOutputProc()
{
	Close();
	if (_synthesizer_wrapper) {
		delete _synthesizer_wrapper;
	}
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

SpeechProcStatus SpeechOutputProc::Open()
{
	if (_open) {
		return SpeechProcStatus_Error;
	}

	_audio_output = new AudioOutput();

	int ret = _audio_output->Open(AUDIO_DEVICE);
	if (ret) {
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR; Could not open audio output device (%s)\n", snd_strerror(ret));
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

	if (_audio_output) {
		_audio_output->Close();
		delete _audio_output;
		_audio_output = NULL;
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

	_text_to_speak = text;

	const std::lock_guard<std::mutex> lock(_mutex);
	if (!_synthesizer_wrapper) {
		_synthesizer_wrapper = new SpeechSynthesizerWrapper();
	}
	if (!_synthesizer_wrapper->isOpen()) {
		if (_synthesizer_wrapper->Open() != SpeechSynthesisStatus_Ok) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open speech synthesizer");
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

void SpeechOutputProc::Process()
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SpeechOutputProc::Process: %s", _text_to_speak.c_str());

	uint8_t* audio_samples = NULL;
	int audio_num_bytes = 0;
	// Locally allocated buffer
	char* buffer = NULL;

	// Calc md5 over text to use a filename for cached speech file
	unsigned char md5[MD5_DIGEST_LENGTH];
	MD5((const unsigned char*)_text_to_speak.data(), _text_to_speak.length(), md5);

	stringstream sstr;
    for(int i = 0; i < MD5_DIGEST_LENGTH; i++) {
    	sstr << std::hex << (unsigned int)md5[i];
    }
    //cout << "MD5: " << sstr.str() << std::endl;

    std::string fname = sstr.str();
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
    	if (size > WAV_HEADER_LEN) {
    		audio_num_bytes = (int)size - WAV_HEADER_LEN;
    		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Read cached speech: %s, size %d", fname.c_str(), audio_num_bytes);
    	}
    }

	const std::lock_guard<std::mutex> lock(_mutex);

    // Synthesize if audio wasn't cached
    std::shared_ptr<SpeechSynthesisResult> result;
    std::shared_ptr<std::vector<uint8_t>> audio;

    if (_run) {
    	if (audio_num_bytes == 0) {
    		if (_synthesizer_wrapper && _synthesizer_wrapper->_synthesizer != nullptr) {
    			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Running speech synthesis");

				result = _synthesizer_wrapper->_synthesizer->SpeakTextAsync(_text_to_speak).get();

				if (result->Reason == ResultReason::SynthesizingAudioCompleted)
				{
					audio = result->GetAudioData();
					RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speech synthesized, size: %u", audio->size());

					if (audio->size() > 0) {
						audio_samples = audio->data();
						audio_num_bytes = audio->size();

						// Cache the audio
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
					RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: Synthesis cancelled, reason: %d", cancellation->Reason);
					_speak_cb(SpeechProcStatus_Error);
					return;
				}
    		}
    	}

		// Play the file
		int write_size = _audio_output->GetWriteSize();

		for (auto offset = 0; _run && offset < audio_num_bytes;) {
			int to_write = write_size;
			if (audio_num_bytes - offset < to_write) {
				to_write = audio_num_bytes - offset;
			}
			//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Writing audio sample: %d bytes, offset: %u",
			//		to_write, offset);

			if (_audio_output->WriteBlock(audio_samples + offset, to_write) < 0) {
				break;
			}
			offset += to_write;
		}
		_audio_output->WriteComplete();

		if (buffer) {
			delete[] buffer;
		}
	}
	// Was cancelled if !_run
	_speak_cb(_run? SpeechProcStatus_Ok: SpeechProcStatus_Error);

}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

