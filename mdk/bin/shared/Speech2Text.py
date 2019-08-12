import speech_recognition as sr
from nltk.sentiment import vader as v

class SpeechToText:

	def __init__(self, microphoneIndex):
		self.r = sr.Recognizer()
		self.analyzer = v.SentimentIntensityAnalyzer()
		if (microphoneIndex >=0):
			self.mic = sr.Microphone(device_index=microphoneIndex)

	def getTextfromSpeech(self, processAmbient):
		with self.mic as source:
			if (processAmbient):
				self.r.adjust_for_ambient_noise(source)
			audio = self.r.listen(source)
		try:
			questionText = self.r.recognize_google(audio)  
		except sr.UnknownValueError:
			print("Google Speech Recognition could not understand audio")
			questionText= None  
		except sr.RequestError as e:
			print("Could not request results from Google Speech Recognition service; {0}".format(e))  
			questionText=None       
		except:
			questionText=None
		return questionText

	def getTextfromFile(self, audioFile):
		with sr.AudioFile(audioFile) as source:
			audio = self.r.record(source)  # read the entire audio file
		
		try:
			questionText = self.r.recognize_google(audio)  
		except sr.UnknownValueError:
			print("Google Speech Recognition could not understand audio")
			questionText= None  
		except sr.RequestError as e:
			print("Could not request results from Google Speech Recognition service; {0}".format(e))  
			questionText=None       
		except:
			questionText=None  		
		return questionText

	def sentiment_analyzer_scores(self, sentence):
		score = self.analyzer.polarity_scores(sentence)
		return score
		
	def getSentiment(self, audiofile):
		text = self.getTextfromFile(audiofile)
		# print ("recognised speech: " + text)
		if (text is not None):
			print("recognised speech: " + text)
			result = self.sentiment_analyzer_scores(text)
			return result.get('compound', 0.0)
		else:
			return None
			
	
