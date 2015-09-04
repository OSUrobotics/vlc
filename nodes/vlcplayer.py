#!/usr/bin/env python
from vlc.player import HttpController, CtypesController
from vlc.srv import Play, Pause, Stop, Forward10, Back10, MuteToggle,\
    FullscreenToggle, StartVideo, VolUp, VolDn, SetVol, Seek
import rospy

if __name__ == '__main__':
    rospy.init_node('vlc')

    vlc = CtypesController()

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
    set_vol_service = rospy.Service('set_vol', SetVol, vlc.set_vol)
    seek_service = rospy.Service('seek', Seek, vlc.seek)

    rospy.spin()
