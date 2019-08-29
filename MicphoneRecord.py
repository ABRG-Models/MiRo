WAVE_OUTPUT_FILENAME = "output.wav"

import wave
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wav
import soundfile as sf
import SpeechRecognitionAndSentiment.Speech2Text as Speech2Text
import scipy.io.wavfile as wavfile


fs=44100
duration = 2  # seconds


def save_wave_file(filename, data):
    wf = wave.open(filename, 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(2)
    wf.setframerate(fs)
    wf.writeframes("".join(data))
    wf.close()


#record speech wave first

myrecording = sd.rec(duration * fs, samplerate=fs, channels=1,dtype='float64')


print("Recording Audio")
sd.wait()
print("Audio recording complete , Play Audio")
# sd.play(myrecording, fs)
# sd.wait()
# print("Play Audio Complete")

#save the wave
sf.write(WAVE_OUTPUT_FILENAME, myrecording, fs)
print(myrecording.shape)
print(myrecording)
wavfile.write("output3.wav",fs,myrecording)
# save_wave_file("output3.wav", data)

#recognize the wave
S2T = Speech2Text.SpeechToText(-1) #use -1 for the microphone index on Miro as we have no direct stream access that Google would recognise for Miro's microphones - therefore you cannot use the getTextfromSpeech call on Miro for this reason

print("Sentiment from audio files")
print("**************************")
print("Sentiment for 'Good Miro' audio(output.wav): " + str(S2T.getSentiment(WAVE_OUTPUT_FILENAME)))