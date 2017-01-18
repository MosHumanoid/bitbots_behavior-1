# -*- coding:utf-8 -*-
"""
StandsCorrectDecision
^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""

from stackmachine.abstract_decision_module import AbstractDecisionModule

import rospy
from body.actions.align_on_ball import AlignOnBall
from body.decisions.common.kick_decision import KickDecisionCommon
from stackmachine.model import Connector


class StandsCorrectDecision(AbstractDecisionModule):
    """
    Decides if the robot stands correct and takes care if it doesn't
    """

    def __init__(self, _):
        super(StandsCorrectDecision, self).__init__()
        self.toggle_align_to_goal = rospy.get_param("/Behaviour/Toggles/Fieldie/alignToGoal")
        self.toggle_use_side_kick_in_game = rospy.get_param("/Behaviour/Toggles/Fieldie/useSideKickInGame")
        self.toggle_hack_align = rospy.get_param("/Behaviour/Toggles/Fieldie/hackAlign")
        self.config_kickalign_v = rospy.get_param("/Behaviour/Fieldie/kickAlign")

    def perform(self, connector: Connector, reevaluate=False):

        # get data
        #opp_goal_u = connector.filtered_vision_capsule().get_local_goal_model_opp_goal()[0]
        #opp_goal_v = connector.filtered_vision_capsule().get_local_goal_model_opp_goal()[1]
        #opp_goal_deg = math.degrees(math.atan2(opp_goal_u, opp_goal_v))

        # todo check if left-right is correct
        #opp_goal_left_post_u = connector.filtered_vision_capsule().get_local_goal_model_opp_goal_posts()[1][0]
        #opp_goal_left_post_v = connector.filtered_vision_capsule().get_local_goal_model_opp_goal_posts()[1][1]
        #opp_goal_left_post_deg = math.degrees(math.atan2(opp_goal_left_post_u, opp_goal_left_post_v))

        #opp_goal_right_post_u = connector.filtered_vision_capsule().get_local_goal_model_opp_goal_posts()[0][0]
        #opp_goal_right_post_v = connector.filtered_vision_capsule().get_local_goal_model_opp_goal_posts()[0][1]
        #opp_goal_right_post_deg = math.degrees(math.atan2(opp_goal_right_post_u, opp_goal_right_post_v))

        # Align sidewards to the ball
        # if abs(connector.filtered_vision_capsule().get_local_goal_model_ball()[1]) > self.config_kickalign_v:
        if abs(connector.vision.get_ball_relative()[1]) > self.config_kickalign_v:
            # todo wieder gefilterte daten verwenden

            return self.push(AlignOnBall)

        # Align to the goal
        """
        elif self.toggle_hack_align and connector.get_duty() not in ["PenaltyKickFieldie", "ThrowIn"]:
            return self.push(HackAlign)

        elif self.toggle_align_to_goal and not connector.blackboard_capsule().has_stopped_aligning():

            # Not using a sidekick. Always tries to align for normal kick
            if not self.toggle_use_side_kick_in_game:

                # Goal is in front and I align the way I can score
                if opp_goal_right_post_v < 0 < opp_goal_left_post_v:
                    # the 0 (axis) is between the two posts
                    # Already aligned
                    return self.action_stands_correct(connector)

            else:
                # Also possible to use a sidekick
                if opp_goal_right_post_v < 0 < opp_goal_left_post_v:
                    # allready aligned for frontkick
                    return self.action_stands_correct(connector)

                elif opp_goal_v > 0:
                    # goal is on the left
                    if opp_goal_right_post_u < 0 < opp_goal_left_post_u:
                        # aligned for sidekick
                        return self.action_stands_correct(connector)

                elif opp_goal_v < 0:
                    # goal is on the right
                    if opp_goal_right_post_u > 0 > opp_goal_left_post_u:
                        # aligned for sidekick
                        return self.action_stands_correct(connector)

            return self.push(AlignToGoal)
        """
        # Stands correct
        #else:
        return self.action_stands_correct(connector)

    def action_stands_correct(self, connector):
        connector.blackboard.stop_aligning()
        return self.push(KickDecisionCommon)
