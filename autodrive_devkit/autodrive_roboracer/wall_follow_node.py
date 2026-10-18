
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wall_follow_node')

        
        self.theta = 30                 
        self.desired_trajectory = 0.65  
        self.max_wall_distance = 2.0   

        
        
        self.kp = 0.8                   
        self.kh = 1.0                   
        self.max_angle = 1.0            
        self.max_steer_change = 0.3     
        self.prev_angle = 0.0

        
        self.max_vel = 0.3            
        self.min_vel = 0.1             
        self.error_for_min_vel = 0.5    

        
        self.front_slow = 3.0           
        self.front_fast = 5.0          

        
        self.turn_distance = 2.0       
        self.turn_gain = 0.8            
        self.turn_direction = 0.0      

        
        self.side_safe = 0.4         
        self.side_gain = 0.8           

        
        self.steer_pub = self.create_publisher(Float32, '/autodrive/roboracer_1/steering_command', 10)
        self.throttle_pub = self.create_publisher(Float32, '/autodrive/roboracer_1/throttle_command', 10)

        
        self.sub = self.create_subscription(
            LaserScan,
            '/autodrive/roboracer_1/lidar',
            self.callback,
            10
        )
        self.get_logger().info("Wall Follow Node Started")

    def getRange(self, data, theta):
        theta_rad = math.radians(theta)
        center = int(round((theta_rad - data.angle_min) / data.angle_increment))

        valid = []
        for index in range(center - 2, center + 3):
            if index < 0 or index >= len(data.ranges):
                continue
            range_val = data.ranges[index]
            if math.isnan(range_val) or math.isinf(range_val):
                continue
            if range_val < data.range_min or range_val > data.range_max:
                continue
            valid.append(range_val)

        if not valid:
            return float('inf')
        return sum(valid) / len(valid)

    def meanRange(self, data, start, end):
        total, count = 0.0, 0
        for t in range(start, end + 1, 2):
            r = self.getRange(data, t)
            total += data.range_max if math.isinf(r) else r
            count += 1
        return total / count

    def wallGeometry(self, data, side):
        a = self.getRange(data, side * (90 - self.theta))   # 斜前方，与 b 相差 theta
        b = self.getRange(data, side * 90)                  # 正侧方，垂直于墙

        if math.isinf(a) or math.isinf(b) or b > self.max_wall_distance:
            return None

        swing = math.radians(self.theta)
        denominator = a * math.sin(swing)
        if denominator == 0:
            return None

        alpha = math.atan((a*math.cos(swing)-b) / denominator)
        AB = b * math.cos(alpha)
        heading = alpha if side < 0 else -alpha
        return AB, heading

    def callback(self, data):
        right = self.wallGeometry(data, -1)
        left = self.wallGeometry(data, +1)

        if right is not None and left is not None:
            offset = (right[0] - left[0]) / 2.0                
            heading = (right[1] + left[1]) / 2.0
        elif right is not None:
            offset = min(0.0, right[0] - self.desired_trajectory)   
            heading = right[1]
        elif left is not None:
            offset = max(0.0, self.desired_trajectory - left[0])    
            heading = left[1]
        else:
            offset, heading = 0.0, 0.0
        error = offset

        angle = -(self.kp * offset + self.kh * heading)

        front = min(self.getRange(data, t) for t in range(-3, 4, 1))
        w = 0.0                                          
        if front < self.turn_distance:
            open_left = self.meanRange(data, 30, 90)
            open_right = self.meanRange(data, -90, -30)
            if open_left - open_right > 0.5:
                self.turn_direction = 1.0
            elif open_right - open_left > 0.5:
                self.turn_direction = -1.0
            w = 1.0 - front / self.turn_distance
            angle += w * self.turn_gain * self.turn_direction
        else:
            self.turn_direction = 0.0

     
        near_left = min(self.getRange(data, t) for t in range(45, 131, 5))
        near_right = min(self.getRange(data, t) for t in range(-130, -44, 5))
        if near_left < self.side_safe:
            angle -= self.side_gain * (1.0 - near_left / self.side_safe) ** 2
        if near_right < self.side_safe:
            angle += self.side_gain * (1.0 - near_right / self.side_safe) ** 2

        limit = self.max_angle + (1.0 - self.max_angle) * w
        angle = max(-limit, min(limit, angle))

        max_change = self.max_steer_change * (1.0 + 2.0 * w)
        change = max(-max_change, min(max_change, angle - self.prev_angle))
        angle = self.prev_angle + change
        self.prev_angle = angle

        ratio = min(1.0, abs(error) / self.error_for_min_vel)
        vel = self.max_vel - (self.max_vel - self.min_vel) * ratio

        vel = max(self.min_vel, vel * (1.0 - 0.4 * abs(angle)))

        if not math.isinf(front):
            f = (front - self.front_slow) / (self.front_fast - self.front_slow)
            f = max(0.0, min(1.0, f))
            vel = min(vel, self.min_vel + (self.max_vel - self.min_vel) * f)

        self.debug_count = getattr(self, 'debug_count', 0) + 1
        if self.debug_count % 3 == 0:
            fmt = lambda g: 'None' if g is None else f'{g[0]:.2f}'
            self.get_logger().info(
                f'R={fmt(right)} L={fmt(left)} nearR={near_right:.2f} nearL={near_left:.2f} front={front:.2f} '
                f'offset={offset:.2f} heading={math.degrees(heading):.1f}deg '
                f'steer={angle:.2f} vel={vel:.2f}')

        self.publish(angle, vel)

    def publish(self, angle, vel):
        steer_msg = Float32()
        steer_msg.data = float(angle)
        throttle_msg = Float32()
        throttle_msg.data = float(vel)
        self.steer_pub.publish(steer_msg)
        self.throttle_pub.publish(throttle_msg)

def main(args=None):
    rclpy.init(args=args)
    print("Laser node started")

    node = WallFollowNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出时停车
        if rclpy.ok():
            node.publish(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
