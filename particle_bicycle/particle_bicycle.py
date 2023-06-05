import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray

from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point

import tf_transformations

from .Robot import Robot
from .ParticleFilter import ParticleFilter


class ParticleBicycle(Node):

    def __init__(self, robot, particle_filter):
        super().__init__('particle_bicycle')

        self._robot = robot
        self._particle_filter = particle_filter
        
        self._init_filter()
        
        self.create_subscription(
            Float32MultiArray, 'move_robot', self._move_robot_cb, 1)

        self.gt_pub = self.create_publisher(PoseStamped, 'ground_truth', 10)
        self.est_pose_pub = self.create_publisher(PoseStamped, 'position', 10)
        
    def _init_filter(self):
        ref_sensed = self._robot.sense()

        self._particle_filter.particles_sense()
        self._particle_filter.update_particles_weights(ref_sensed)
        
    def _estimate_pose_after_move(self, dist, steer):
        # Moving phase
        self._particle_filter.move_particles(dist, steer)
        
        # Measurement phase
        ref_sensed = self._robot.sense()
        self._particle_filter.particles_sense()
        self._particle_filter.update_particles_weights(ref_sensed)
        
        # Resampling phase
        self._particle_filter.resample()
        
        # Estimation phase
        est_x, est_y, est_heading = self._particle_filter.estimate_pose()
        
        return est_x, est_y, est_heading

    def _move_robot_cb(self, msg):
        # Publishing ground truth
        self._robot.move(msg.data[0], msg.data[1])

        x, y, z, w = tf_transformations.quaternion_from_euler(
            0, 0, self._robot.heading)
        q = Quaternion(x=x, y=y, z=z, w=w)
        p = Point(x=self._robot.x, y=self._robot.y, z=0.0)
        rob_pose = Pose(position=p, orientation=q)
        
        h = Header()
        h.frame_id = "base_link"
        rob_pose_stamped = PoseStamped(header=h, pose=rob_pose)
        self.gt_pub.publish(rob_pose_stamped)


        # Publishing estimated position
        est_x, est_y, est_heading = self._estimate_pose_after_move(msg.data[0], msg.data[1])
        
        x, y, z, w = tf_transformations.quaternion_from_euler(
            0, 0, est_heading)
        q = Quaternion(x=x, y=y, z=z, w=w)
        p = Point(x=est_x, y=est_y, z=0.0)
        est_pose = Pose(position=p, orientation=q)
        
        h = Header()
        h.frame_id = "base_link"
        est_pose_stamped = PoseStamped(header=h, pose=est_pose)
        self.est_pose_pub.publish(est_pose_stamped)
        

    def timer_callback(self):
        my_msg = Float32MultiArray()
        my_msg.data = [10.0, 1.5]
        self.mu_pub.publish(my_msg)


def main(args=None):
    rclpy.init(args=args)

    r = Robot()
    pf = ParticleFilter(r, 100)

    particle_bicycle = ParticleBicycle(r, pf)

    rclpy.spin(particle_bicycle)

    particle_bicycle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
