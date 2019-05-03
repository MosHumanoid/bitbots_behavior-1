import rospy
import math
import time
import tf2_ros as tf2
from humanoid_league_msgs.msg import BallRelative
from tf2_geometry_msgs import PointStamped

class BallFilter(object):
    def __init__(self):
        self.time_threshold = 1.0
        self.drift_threshold = 400
        self.distance_threshold = 20
        self.match_count_threshold = 0.5
        self.ball_queue = []
        rospy.init_node('ball_filter')

        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(5))
        self.tf_listener = tf2.TransformListener(self.tf_buffer)
        
        self.ball_relative_msg = rospy.Subscriber(
            'ball_relative',
            BallRelative,
            self.ball_relative_cb,
            queue_size=1,
            tcp_nodelay=True)

        self.pub_ball_relative_filtered = rospy.Publisher(
            'ball_relative_filtered',
            BallRelative,
            queue_size=1)

    def get_ball_position_uv(self, ball):
        try:
            ball = self.tf_buffer.transform(ball, 'base_footprint', timeout=rospy.Duration(0.3))
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            rospy.logwarn(e)
            return None
        return ball.point.x, ball.point.y

    def ball_relative_cb(self, msg):
        ball_obj = PointStamped(msg.header, msg.ball_relative)
        x, y = self.get_ball_position_uv(ball_obj)
        cb_time = time.time()
        current_ball = (x, y, cb_time)
        self.ball_queue.append(current_ball)
        self.clear_queue()
        if self.ball_valid(current_ball):
            self.publish_ball(current_ball)
        else:
            print("Ball dropped")
    
    def publish_ball(self, ball):
        message = BallRelative()
        message.header.frame_id = 'base_footprint'
        message.ball_relative.x = ball[0]
        message.ball_relative.x = ball[1]
        self.pub_ball_relative_filtered.publish(message)

    def ball_valid(self, current_ball):
        current_time = self.ball_queue[2]
        matched = []
        for ball in self.ball_queue:
            distance = math.sqrt((current_ball[0] - ball[0]) ** 2 + (current_ball[1] - ball[1]) ** 2)
            time_diff = current_time - ball[2]
            allowed_distance = time_diff * self.drift_threshold
            if distance <= allowed_distance:
                matched.append(ball)
        if len(matched) > len(self.ball_queue) * self.match_count_threshold:
            return True
        else:
            return False
                        
    def clear_queue(self):
        new_ball_queue = []
        for ball in self.ball_queue:
            if time.time() - ball[2] < self.time_threshold:
                new_ball_queue.append(ball)
        self.ball_queue = new_ball_queue


if __name__ == "__main__":
    ball_filter = BallFilter()

