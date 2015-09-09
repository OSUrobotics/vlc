#!/usr/bin/env python
import rospy
import pykeyboard
import subprocess
import shlex
from std_msgs.msg import Duration
from vlc.srv import Play, Pause, Stop, Forward10, Back10, MuteToggle,\
    FullscreenToggle, StartVideo, VolUp, VolDn
from vlc.srv import StartVideoResponse
from vlc.msg import PlayerState
import abc
from lxml import objectify
import urllib
import urllib2
import requests
from requests.auth import HTTPBasicAuth
from functools import partial
from vlc import libvlc        
from multiprocessing import Process, Queue, Manager
import os
import sys

class VLCController(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, use_http):
        self._paused = False
        self._muted = False
        self._time = rospy.Duration(0)
        self.time_pub = rospy.Publisher('playback_time', Duration)
        self._http = use_http
        self._process = None
        self._tic_timer = None

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

    def _start_vlc(self, vid_path, _):
        args = sum([
            ['vlc'],
            #shlex.split('--extraintf http' if self._http else ''),
            shlex.split('--extraintf http --http-password=ROS' if self._http else ''),
            ['--play-and-pause'],
            [vid_path],
        ], [])
        self._process = subprocess.Popen(args)

    def start_vlc(self, msg):
        vid_path = msg.path
        self._paused = False
        self._muted = False
        self._length = -1
        self._time = rospy.Duration(0)

        if self._process is not None:
            self._process.terminate()
            self._tic_timer.shutdown()
            self._process.wait()

        rospy.Timer(
            rospy.Duration(0.00001),
            partial(self._start_vlc, vid_path),
            oneshot=True
        )
        self._wait_for_vlc()
        self.toggle_fullscreen()
        rospy.set_param('vlc_ready', True)
        self._tic_timer = rospy.Timer(rospy.Duration(0.25), self._tick)
        return StartVideoResponse()


class HttpController(VLCController):
    def __init__(self):
        super(HttpController, self).__init__(True)
        self._url = 'http://localhost:8080/requests/status.xml'
        self.status = None
        self._auth = HTTPBasicAuth('', 'ROS')

    def _wait_for_vlc(self):
        try:
            resp = requests.get(self._url, auth=self._auth).content
            objectify.fromstring(resp).time
        except requests.ConnectionError:
            rospy.sleep(0.01)
            self._wait_for_vlc()

        self._update_state(update_vol=True)
        rospy.sleep(0.05)

    def _send_command(self, command, val=''):
        try:
            url = self._url + '?' + urllib.urlencode(dict(command=command, val=val))
            resp = requests.get(url, auth=self._auth).content
            self.state = objectify.fromstring(resp)
            self._time = max(rospy.Duration(self.state.time), rospy.Duration(0))
            self._length = self.state.length
        except AttributeError:
            rospy.logwarn("Couldn't get player time")
        except urllib2.URLError:
            rospy.logwarn("Couldn't get player time (HTTP interface doesn't seem to be ready")
        return self.state

    def _update_state(self, update_vol=False):
        self.state = self._send_command('')
        if update_vol:
            self._vol = self.state.volume
        try:
            self._time = rospy.Duration(self.state.time)
        except AttributeError:
            rospy.logwarn("Couldn't get player time")

    def get_state(self):
        return PlayerState(
            self._time,
            self.state.volume,
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
        self._send_command('volume', '+75')
        self._vol = self.state.volume
        return self.get_state()

    def vol_dn(self, *args):
        '''Lower volume'''
        self._send_command('volume', '-75')
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

    def set_vol(self, req):
        print self.state
        self._send_command('volume', req.vol)
        return self.get_state()

    def seek(self, req):
        self._send_command('seek', int(req.time.to_sec()))
        if req.time.to_sec() > self._length:
            self._paused = True
        return self.get_state()


class KeyboardController(VLCController):
    def __init__(self):
        super(KeyboardController, self).__init__(False)
        self._keyboard = pykeyboard.PyKeyboard()

    def get_state(self):
        return PlayerState(self._time, 0, self._paused, self._muted)

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

class CtypesController(VLCController):

    class SubProcessController(object):
        def __init__(self, queue, state, use_http=False):
            # super(CtypesController.SubProcessController, self).__init__(False)
            self.player = None
            self.marquee_timer = None
            self.state = state
            import sys
            print 'ok...'
            rospy.loginfo('asdfasdf')
            sys.stdout.flush()
            
            r = rospy.Rate(10)
            while not rospy.is_shutdown():
                func, args = queue.get()
                getattr(self, func)(*args)

        def start_vlc(self, msg):
            self.player = libvlc.MediaPlayer('file://' + msg.path)
            self.player.play()
            rospy.Timer(rospy.Duration(0.5), self._update_state)
            self._wait_for_vlc()
            self._update_state()
            return StartVideoResponse()

        def _update_state(self, *_):
            self.state[0] = rospy.Duration(self.player.get_time() / 1000.0)
            self.state[1] = self._vol_pc_to_9(self.player.audio_get_volume())
            self.state[2] = self.player.get_state() == libvlc.State.Paused
            self.state[3] = self.player.audio_get_mute()
            sys.stdout.flush()



        def _wait_for_vlc(self):
            '''Blocks until VLC is available'''
            while not self.player.is_playing() and not rospy.is_shutdown():
                rospy.sleep(0.05)


        # def get_state(self):
        #     '''Returns the player state as a service response'''
        #     return PlayerState(
        #         self._time,
        #         self.state.volume,
        #         self.state.state == 'paused',
        #         self._muted
        #     )

        def clear_marquee(self, _=None):
            self.marquee('')

        def marquee(self, text, timeout=0):
            self.player.video_set_marquee_int(libvlc.VideoMarqueeOption.Enable, 1)
            self.player.video_set_marquee_string(libvlc.VideoMarqueeOption.Text, text)
            if self.marquee_timer:
                self.marquee_timer.shutdown()
            if timeout > 0:
                self.marquee_timer = rospy.Timer(rospy.Duration(timeout), self.clear_marquee, oneshot=True)
            sys.stdout.flush()

        def toggle_fullscreen(self, *args):
            '''Toggles fullscreen'''
            self.player.toggle_fullscreen()

        def playpause(self):
            '''Toggles play/pause'''
            if self.player.get_state() != libvlc.State.Paused:
                self.marquee('Paused')
            else:
                self.clear_marquee()
            self.player.pause()

        def play(self, *args):
            '''Play. Does nothing if already playing'''
            self.player.play()
            self.clear_marquee()
            self._update_state()

        def pause(self, *args):
            '''Pause. Does nothing if already paused'''
            self.player.set_pause(True)
            self._update_state()
            self.state[2] = True
            self.marquee('Paused')

        def back10(self, *args):
            '''Skip back 10 seconds'''
            self.player.set_time(int((self.state[0].to_sec() - 10) * 1000))
            self.marquee('<<', 3)
            self._update_state()

        def forward10(self, *args):
            '''Skip forward 10 seconds'''
            self.player.set_time(int((self.state[0].to_sec() + 10) * 1000))
            self.marquee('>>', 3)
            self._update_state()

        def seek(self, req):
            self.player.set_time(int(req.time.to_sec()) * 1000)
            self._update_state()

        def mute(self, *args):
            '''Toggles mute'''
            self.player.audio_toggle_mute()
            if self.player.audio_get_mute():
                self.marquee('Muted')
            else:
                self.clear_marquee()
            self._update_state()

        def vol_up(self, *args):
            '''Increase volume'''
            newvol = self._vol_9_to_pc(self.state[1] + 75)
            self.player.audio_set_volume(newvol)
            self.marquee('Vol %s%%' % newvol, 3)
            self._update_state()

        def vol_dn(self, *args):
            '''Decrease volume'''
            newvol = self._vol_9_to_pc(self.state[1] - 75)
            self.player.audio_set_volume(newvol)
            self.marquee('Vol %s%%' % newvol, 3)
            self._update_state()

        def set_vol(self, req):
            '''Set the volume to a specific level'''
            self.player.audio_set_volume(self._vol_9_to_pc(req.vol))
            self._update_state()

        def stop(self, *args):
            '''Stop playback'''
            self.player.stop()
            self._update_state()

        def _vol_pc_to_9(self, pc):
            return int(256.0/100 * pc)

        def _vol_9_to_pc(self, nine):
            return int(100.0/256 * nine)


    def __init__(self, use_http=False):
        super(CtypesController, self).__init__(False)
        self.command_queue = Queue()
        manager = Manager()
        self.state = manager.list([rospy.Duration(0), 0, False, False])

        p = Process(target=CtypesController.SubProcessController, args=(self.command_queue, self.state))
        p.start()

    def _update_state(self, *_):
        pass

    def run_in_process(self, cmd, *args):
        self.command_queue.put([cmd, args])

    def start_vlc(self, msg):
        print 'starting', 'file://' + msg.path
        self.run_in_process('start_vlc', msg)

        return StartVideoResponse()

    def _wait_for_vlc(self):
        '''Blocks until VLC is available'''
        return

    def get_state(self):
        '''Returns the player state as a service response'''
        return PlayerState(*self.state)

    def toggle_fullscreen(self, *args):
        '''Toggles fullscreen'''
        self.run_in_process('toggle_fullscreen', *args)
        return self.get_state()


    def playpause(self):
        '''Toggles play/pause'''
        self.run_in_process('playpause', *args)
        return self.get_state()

    def play(self, *args):
        '''Play. Does nothing if already playing'''
        self.run_in_process('play', *args)
        return self.get_state()

    def pause(self, *args):
        '''Pause. Does nothing if already paused'''
        self.run_in_process('pause', *args)
        return self.get_state()

    def back10(self, *args):
        '''Skip back 10 seconds'''
        self.run_in_process('back10', *args)
        return self.get_state()

    def forward10(self, *args):
        '''Skip forward 10 seconds'''
        self.run_in_process('forward10', *args)
        return self.get_state()

    def seek(self, req):
        self.run_in_process('seek', req)
        return self.get_state()

    def mute(self, *args):
        '''Toggles mute'''
        self.run_in_process('mute', *args)
        return self.get_state()

    def vol_up(self, *args):
        '''Increase volume'''
        self.run_in_process('vol_up', *args)
        return self.get_state()

    def vol_dn(self, *args):
        '''Decrease volume'''
        self.run_in_process('vol_dn', *args)
        return self.get_state()

    def set_vol(self, req):
        '''Set the volume to a specific level'''
        self.run_in_process('set_vol', req)
        return self.get_state()

    def stop(self, *args):
        '''Stop playback'''
        self.run_in_process('stop', *args)
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
