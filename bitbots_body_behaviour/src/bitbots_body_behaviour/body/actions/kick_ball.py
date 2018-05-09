# -*- coding:utf-8 -*-
"""
KickBall
^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>
"""
import time

from bitbots_stackmachine.abstract_action_module import AbstractActionModule

import rospy


class KickBall(AbstractActionModule):
    """
    Kickt nach dem Ball, bekommt im init_data die Information ob links oder rechts gekickt werden soll
    Kicks the ball, gets in init_data the information about the side.
    """

    def __init__(self, connector, args):
        super(KickBall, self).__init__(connector)
        self.side = args
        self.begin = rospy.get_time()
        self.right_kick = "rk_wm2016_unstable" # connector.animation.an_config["kicks"]["rk"]
        self.left_kick = None # connector.animation.an_config["kicks"]["lk"]
        self.right_kick_strong = None #  connector.animation.an_config["kicks"]["rkp"]
        self.left_kick_strong = None # connector.animation.an_config["kicks"]["lkp"]
        self.right_side_kick = None # connector.animation.an_config["kicks"]["lko"]
        self.left_side_kick = None # connector.animation.an_config["kicks"]["rko"]
        self.anim_begin = False

    def perform(self, connector, reevaluate=False):
        self.do_not_reevaluate()
        connector.walking.stop_walking()

        if rospy.get_time() - self.begin > 3.5:  # wait one moment
            self.do_not_reevaluate()  # dont interrrupt the kick

            connector.blackboard.set_one_time_kicked(True)
            # todo make a better behaviour that looks if the ball realy moved

            if not connector.animation.is_animation_busy() and self.anim_begin:
                # if the animation was performed, jump one level higher
                return self.interrupt()

            self.anim_begin = True

            if self.side == "RIGHT_KICK":
                connector.animation.play_animation(self.right_kick)
            elif self.side == "LEFT_KICK":
                connector.animation.play_animation(self.left_kick)
            elif self.side == "RIGHT_KICK_STRONG":
                connector.animation.play_animation(self.right_kick_strong)
            elif self.side == "LEFT_KICK_STRONG":
                connector.animation.play_animation(self.left_kick_strong)
            elif self.side == "LEFT_SIDE_KICK":
                connector.animation.play_animation(self.left_side_kick)
            elif self.side == "RIGHT_SIDE_KICK":
                connector.animation.play_animation(self.right_side_kick)
            else:
                raise NotImplementedError("This kick does not exist")
