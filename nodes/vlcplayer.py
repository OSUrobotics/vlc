#!/usr/bin/env python
from vlc.player import HttpController
from std_msgs.msg import String, Empty, Duration
from vlc.srv import Play, Pause, Forward10, Back10, MuteToggle, FullscreenToggle, StartVideo, VolUp, VolDn
from vlc.srv import StartVideoResponse
from vlc.msg import PlayerState
import rospy

if __name__ == '__main__':
    rospy.init_node('vlc')

    vlc = HttpController()

    play_service = rospy.Service('play', Play, vlc.play)
    pause_service = rospy.Service('pause', Pause, vlc.pause)
    back_service = rospy.Service('back10', Back10, vlc.back10)
    forward_service = rospy.Service('forward10', Forward10, vlc.forward10)
    mute_service = rospy.Service('toggle_mute', MuteToggle, vlc.mute)
    fullscreen_service = rospy.Service('toggle_fullscreen', FullscreenToggle, vlc.toggle_fullscreen)

    video_play_service = rospy.Service('start_video', StartVideo, vlc.start_vlc)
    vol_up_service = rospy.Service('vol_up', VolUp, vlc.vol_up)
    vol_dn_service = rospy.Service('vol_dn', VolDn, vlc.vol_dn)

    rospy.spin()

