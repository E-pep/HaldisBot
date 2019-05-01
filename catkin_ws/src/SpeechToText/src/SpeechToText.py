#!/usr/bin/python2.7
import rospy
import speech_recognition as sr
from std_msgs.msg import Float32MultiArray


def speech_recog():
	
	pub = rospy.Publisher('/speechToText', Float32MultiArray, queue_size=4)
	rospy.init_node('SpeechToText', anonymous=True)
	r = sr.Recognizer()	
	while not rospy.is_shutdown():

		with sr.Microphone() as source:
			print('speak Anything:')
			audio = r.listen(source)
			try:
				text = r.recognize_google(audio)
				print('said : {}'.format(text))
					
				if text.lower() == "lemonade":
					to_publish = Float32MultiArray(data = [0])
					pub.publish(to_publish)
				elif text.lower() == "coke":
					to_publish = Float32MultiArray(data = [1])
					pub.publish(to_publish)
				elif text.lower() == "water" or text.lower() == "raptor":
					to_publish = Float32MultiArray(data = [2])
					pub.publish(to_publish)
			except:
				print('voice not recognized')





if __name__ == '__main__':
     try:
         speech_recog()
     except rospy.ROSInterruptException:
         pass

