#!/usr/bin/env python
import rospy, pykeyboard
import subprocess
from std_msgs.msg import String, Empty
from vlc.srv import Play, Pause, Forward10, Back10, MuteToggle, FullscreenToggle, StartVideo
from vlc.srv import StartVideoResponse
from vlc.msg import PlayerState

class VLCController(object):
	def __init__(self):
		self._paused = False
		self._muted = False
		self._time = rospy.Duration(0)
		t = rospy.Timer(rospy.Duration(1), self._tick)
		self._keyboard = pykeyboard.PyKeyboard()

	def _send_sequence(self, *args):
		for key in args:
			self._keyboard.press_key(key)
		rospy.sleep(0.25)
		for key in args:
			self._keyboard.release_key(key)

	def _tick(self, *args):
		if not self._paused:
			self._time += rospy.Duration(1)

	def get_state(self):
		return PlayerState(self._time, self._paused, self._muted)

	def toggle_fullscreen(self, *args):
		self._keyboard.tap_key('f')
		return self.get_state()

	def playpause(self):
		self._keyboard.tap_key(' ')
		return self.get_state()

	def play(self, *args):
		if self._paused:
			self._send_sequence(self._keyboard.alt_key, 'p')
			self._paused = False
		return self.get_state()

	def pause(self, *args): 
		if not self._paused:
			self._send_sequence(self._keyboard.alt_key, self._keyboard.control_r_key, 'p')
			self._paused = True
		return self.get_state()

	def back10(self, *args):
		self._time = max(self._time - rospy.Duration(10), rospy.Duration(0))
		# Alt + Left arrow
		self._send_sequence(self._keyboard.alt_key, self._keyboard.left_key)
		return self.get_state()

	def forward10(self, *args):
		self._time += rospy.Duration(10)
		# Alt + Right arrow
		self._send_sequence(self._keyboard.alt_key, self._keyboard.right_key)
		return self.get_state()

	def mute(self, *args):
		self._keyboard.tap_key('m')
		self._muted = not self._muted
		return self.get_state()

	def start_vlc(self, msg):
		vid_path = msg.path
		self._paused = False
		self._muted = False
		self._time = rospy.Duration(0)
		rospy.Timer(
			rospy.Duration(0.00001),
			lambda x: subprocess.call('vlc --play-and-exit %s' % vid_path, shell=True), oneshot=True
		)
		rospy.Timer(rospy.Duration(0.35), self.toggle_fullscreen, oneshot=True)
		print 'ok'
		return StartVideoResponse()
		

if __name__ == '__main__':
	rospy.init_node('vlc')

	vlc = VLCController()

	play_service = rospy.Service('play', Play, vlc.play)	
	pause_service = rospy.Service('pause', Pause, vlc.pause)	
	back_service = rospy.Service('back10', Back10, vlc.back10)	
	forward_service = rospy.Service('forward10', Forward10, vlc.forward10)	
	mute_service = rospy.Service('toggle_mute', MuteToggle, vlc.mute)	
	fullscreen_service = rospy.Service('toggle_fullscreen', FullscreenToggle, vlc.toggle_fullscreen)

	video_play_service = rospy.Service('start_video', StartVideo, vlc.start_vlc)

	rospy.spin()
