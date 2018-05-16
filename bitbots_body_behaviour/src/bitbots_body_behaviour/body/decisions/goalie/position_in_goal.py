# -*- coding:utf-8 -*-
"""
PositionInGoal
^^^^^^^^^^^^^^

Let the goalie position inside the goal.

History:
* 06.12.14: Created using code from GoalieBehaviourDynamic by Daniel Speck (Marc Bestmann)
"""
import math
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule
from bitbots_body_behaviour.body.actions.go_to import GoToRelativePosition


class PositionInGoal(AbstractDecisionModule):
    def __init__(self,  connector, _):
        super(PositionInGoal, self).__init__(connector)
        goal_width = connector.config["Behaviour"]["Body"]["Common"]["Field"]["goalWidth"]
        max_side_move = connector.config["Behaviour"]["Body"]["Goalie"]["maxSidewardMovement"]
        self.angle_threshold = connector.config["Behaviour"]["Body"]["Goalie"]["sidewardMoveAngleThreshold"]
        self.max_side = goal_width * max_side_move

    def perform(self, connector, reevaluate=False):
        ball_position = connector.personal_model.get_ball_position()
        ball_angle = math.degrees(math.atan2(ball_position[1], ball_position[0]))
        robot_position = connector.world_model.get_current_position()

        # is it necessary to move
        if math.fabs(ball_angle) > self.angle_threshold and abs(robot_position[1]) < self.max_side:
            return self.push(GoToRelativePosition, (0, ball_position[1], 0))
