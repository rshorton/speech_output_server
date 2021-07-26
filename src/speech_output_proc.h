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

#include <functional>
#include <thread>
#include <mutex>
#include <string>
#include <map>

class SpeechSynthesizerWrapper;
class AudioOutput;

enum SpeechProcStatus {
	SpeechProcStatus_Ok = 0,
	SpeechProcStatus_Error = 1,
};

class SpeechOutputProc
{
public:
	SpeechOutputProc();
	~SpeechOutputProc();

	SpeechProcStatus Open();
	SpeechProcStatus Close();

	SpeechProcStatus SpeakStart(std::string text);
	SpeechProcStatus SpeakStop();
	void SetSpeakCB(std::function<void(SpeechProcStatus)> callback);
	void SetSmileCB(std::function<void(std::string)> callback);
	void SetSpeechActiveCB(std::function<void(bool)> callback);

protected:
	void Process();
	void StopProcessing();
	int FindCard(const std::string cardName);

private:
	bool _open;
	bool _run;
	std::function<void(std::string)> _ww_cb;
	std::function<void(SpeechProcStatus)> _speak_cb;
	std::function<void(std::string)>_smile_cb;
	std::function<void(bool)>_speech_active_cb;

	std::unique_ptr<SpeechSynthesizerWrapper> _synthesizer_wrapper;

	std::unique_ptr<AudioOutput> _audio_output;

	std::thread _proc_thread;
	std::mutex _mutex;

	std::string _text_to_speak;
};
