#!/usr/bin/python2.7
import rospy
import speech_recognition as sr

r = sr.Recognizer()


with sr.Microphone() as source:
	while True:
		print('speak Anything:')
		audio = r.listen(source)

		try:
			text = r.recognize_google(audio)
			print('said : {}'.format(text))
			if text == "stop":
				break;
		except:
			print('voice not recognized')

