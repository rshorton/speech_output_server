
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

protected:
	void Process();
	void StopProcessing();

private:
	bool _open;
	bool _run;
	std::function<void(std::string)> _ww_cb;
	std::function<void(SpeechProcStatus)> _speak_cb;
	std::function<void(std::string)>_smile_cb;

	SpeechSynthesizerWrapper *_synthesizer_wrapper;

	AudioOutput *_audio_output;

	std::thread _proc_thread;
	std::mutex _mutex;

	std::string _text_to_speak;
};
