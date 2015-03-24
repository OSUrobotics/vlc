#!/usr/bin/env python
import rospy, pykeyboard
import subprocess
from std_msgs.msg import String, Empty, Duration
from vlc.srv import Play, Pause, Stop, Forward10, Back10, MuteToggle, FullscreenToggle, StartVideo, VolUp, VolDn
from vlc.srv import StartVideoResponse
from vlc.msg import PlayerState
import abc
from lxml import objectify
import urllib, urllib2

class VLCController(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, use_http):
        self._paused = False
        self._muted = False
        self._time = rospy.Duration(0)
        self.time_pub = rospy.Publisher('playback_time', Duration)
        self._http = use_http

    def _tick(self, *args):
        self._update_state()
        self.time_pub.publish(self._time)

    @abc.abstractmethod
    def _update_state(self):
        '''Updates the player's state'''
        return

    @abc.abstractmethod
    def _wait_for_vlc(self):
        '''Blocks until VLC is available'''
        return

    @abc.abstractmethod
    def get_state(self):
        '''Returns the player state as a service response'''
        return

    @abc.abstractmethod
    def toggle_fullscreen(self, *args):
        '''Toggles fullscreen'''
        return

    @abc.abstractmethod
    def playpause(self):
        '''Toggles play/pause'''
        return

    @abc.abstractmethod
    def play(self, *args):
        '''Play. Does nothing if already playing'''
        return

    @abc.abstractmethod
    def pause(self, *args): 
        '''Pause. Does nothing if already paused'''
        return

    @abc.abstractmethod
    def back10(self, *args):
        '''Skip back 10 seconds'''
        return

    @abc.abstractmethod
    def forward10(self, *args):
        '''Skip forward 10 seconds'''
        return

    @abc.abstractmethod
    def mute(self, *args):
        '''Toggles mute'''
        return

    @abc.abstractmethod
    def vol_up(self, *args):
        '''Increase volume'''
        return

    @abc.abstractmethod
    def vol_dn(self, *args):
        '''Decrease volume'''
        return

    @abc.abstractmethod
    def stop(self, *args):
        '''Stop playback'''
        return

    def start_vlc(self, msg):
        vid_path = msg.path
        self._paused = False
        self._muted = False
        self._time = rospy.Duration(0)
        tic_timer = rospy.Timer(rospy.Duration(1), self._tick)
        rospy.Timer(
            rospy.Duration(0.00001),
            lambda x: subprocess.call('vlc %s --play-and-exit "%s"' % ('--extraintf http' if self._http else '', vid_path), shell=True), oneshot=True
        )
        self._wait_for_vlc()
        self.toggle_fullscreen()
        rospy.set_param('vlc_ready', True)
        return StartVideoResponse()
        
class HttpController(VLCController):
    def __init__(self):
        super(HttpController, self).__init__(True)
        self._url = 'http://localhost:8080/requests/status.xml'
        self.status = None

    def _wait_for_vlc(self):
        try:
            resp = urllib2.urlopen(self._url).read()
            objectify.fromstring(resp).time
        except Exception as e:
            rospy.sleep(0.01)
            self._wait_for_vlc()

        self._update_state(update_vol=True)
        rospy.sleep(0.05)

    def _send_command(self, command, val=''):
        url = self._url + '?' + urllib.urlencode(dict(command=command, val=val))
        resp = urllib2.urlopen(url).read()
        self.state = objectify.fromstring(resp)
        try:
            self._time = max(rospy.Duration(self.state.time), rospy.Duration(0))
        except AttributeError as e:
            rospy.logwarn("Couldn't get player time")
        return self.state

    def _update_state(self, update_vol=False):
        self.state = self._send_command('')
        if update_vol:
            self._vol = self.state.volume
        try:
            self._time = rospy.Duration(self.state.time)
        except AttributeError as e:
            rospy.logwarn("Couldn't get player time")

    def get_state(self):
        return PlayerState(
            self._time,
            self.state.state == 'paused',
            self._muted
        )

    def play(self, *args):
        self._send_command('pl_play')
        return self.get_state()

    def pause(self, *args):
        self._send_command('pl_pause')
        return self.get_state()

    def stop(self, *args):
        self._send_command('pl_stop')
        return self.get_state()        

    def back10(self, *args):
        '''Go back 10 seconds'''
        self._send_command('seek', '-10')
        return self.get_state()

    def forward10(self, *args):
        '''Skip forward 10 seconds'''
        self._send_command('seek', '+10')
        return self.get_state()

    def mute(self, *args):
        '''Toggles mute'''
        if self._muted:
            self._send_command('volume', self._vol)
            self._muted = False
        else:
            self._send_command('volume', 0)
            self._muted = True
        return self.get_state()

    def vol_up(self, *args):
        '''Increase volume'''
        self._send_command('volume', '+15')
        self._vol = self.state.volume
        return self.get_state()

    def vol_dn(self, *args):
        '''Lower volume'''
        self._send_command('volume', '-15')
        self._vol = self.state.volume
        return self.get_state()

    def toggle_fullscreen(self, *args):
        '''Toggles fullscreen'''
        self._send_command('fullscreen')
        return self.get_state()

    def playpause(self):
        '''Toggles play/pause'''
        if self._paused:
            self.play()
        else:
            self.pause()
        return self.get_state()


class KeyboardController(VLCController):
    def __init__(self):
        super(KeyboardController, self).__init__(False)
        self._keyboard = pykeyboard.PyKeyboard()

    def get_state(self):
        return PlayerState(self._time, self._paused, self._muted)

    def _wait_for_vlc(self):
        rospy.sleep(0.35)

    def _send_sequence(self, *args):
        for key in args:
            self._keyboard.press_key(key)
        rospy.sleep(0.25)
        for key in args:
            self._keyboard.release_key(key)

    def _tick(self, *args):
        if not self._paused:
            self._time += rospy.Duration(1)
        self.time_pub.publish(self._time)

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

    def vol_up(self, *args):
        self._send_sequence(self._keyboard.control_key, self._keyboard.up_key)
        return self.get_state()

    def vol_dn(self, *args):
        self._send_sequence(self._keyboard.control_key, self._keyboard.down_key)
        return self.get_state()


if __name__ == '__main__':
    rospy.init_node('vlc')

    vlc = HttpController()

    play_service = rospy.Service('play', Play, vlc.play)    
    pause_service = rospy.Service('pause', Pause, vlc.pause)   
    stop = rospy.Service('stop', Stop, vlc.stop)
    back_service = rospy.Service('back10', Back10, vlc.back10)  
    forward_service = rospy.Service('forward10', Forward10, vlc.forward10)  
    mute_service = rospy.Service('toggle_mute', MuteToggle, vlc.mute)   
    fullscreen_service = rospy.Service('toggle_fullscreen', FullscreenToggle, vlc.toggle_fullscreen)

    video_play_service = rospy.Service('start_video', StartVideo, vlc.start_vlc)
    vol_up_service = rospy.Service('vol_up', VolUp, vlc.vol_up)
    vol_dn_service = rospy.Service('vol_dn', VolDn, vlc.vol_dn)

    rospy.spin()
