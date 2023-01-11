import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# from rclpy.clock import clock

from threading import Thread

from std_msgs.msg import String
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# from biometra_driver.biometra_driver import biometra_trobot # TODO:UNCOMMENT THIS WHEN IT IS READY

class BiometraDescriptionClient(Node):

    def __init__(self, NODE_NAME = 'BiometraDescriptionNode'):
        super().__init__(NODE_NAME)

        # self.biometra = biometra_trobot() #USE THIS LINE WHEN IT IS READY

        timer_period = 0.1  # seconds

        self.state = "UNKNOWN"
        joint_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()

        self.statePub = self.create_publisher(String, NODE_NAME + '/state',10)
        # self.stateTimer = self.create_timer(timer_period, callback = self.stateCallback, callback_group = state_cb_group)

        self.joint_publisher = self.create_publisher(JointState,'joint_states', 10, callback_group = joint_cb_group)
        self.joint_state_handler = self.create_timer(timer_period, callback = self.joint_state_publisher_callback, callback_group = joint_cb_group)
    
    
    def stateCallback(self):
        '''
        Publishes the biometra_description state to the 'state' topic. 
        '''
        msg = String()
        msg.data = 'State: %s' % self.state
        self.statePub.publish(msg)
        self.get_logger().info('Publishing State: "%s"' % msg.data)
        self.state = "READY"


    def joint_state_publisher_callback(self):
        
        # self.get_logger().info("BUGG")
        # joint_states = self.biometra.refresh_joint_state() #USE THIS FORMAT WHEN IT IS READY
        joint_states_biometra_1 = [1.2]
        biometra_joint_msg_biometra_1 = JointState()
        biometra_joint_msg_biometra_1.header = Header()
        biometra_joint_msg_biometra_1.header.stamp = self.get_clock().now().to_msg()
        biometra_joint_msg_biometra_1.name = ['Biometra_Cap']
        biometra_joint_msg_biometra_1.position = joint_states_biometra_1
        biometra_joint_msg_biometra_1.velocity = []
        biometra_joint_msg_biometra_1.effort = []

        self.joint_publisher.publish(biometra_joint_msg_biometra_1)
        self.get_logger().info('Publishing joint states: "%s"' % str(joint_states_biometra_1))

        joint_states_biometra_2 = [0.0]
        biometra_joint_msg_biometra_2 = JointState()
        biometra_joint_msg_biometra_2.header = Header()
        biometra_joint_msg_biometra_2.header.stamp = self.get_clock().now().to_msg()
        biometra_joint_msg_biometra_2.name = ['Biometra_Cap2']
        biometra_joint_msg_biometra_2.position = joint_states_biometra_2
        biometra_joint_msg_biometra_2.velocity = []
        biometra_joint_msg_biometra_2.effort = []

        self.joint_publisher.publish(biometra_joint_msg_biometra_2)
        self.get_logger().info('Publishing joint states: "%s"' % str(joint_states_biometra_2))


def main(args=None):
    rclpy.init(args=args)
    try:
        biometra_joint_state_publisher = BiometraDescriptionClient()
        executor = MultiThreadedExecutor()
        executor.add_node(biometra_joint_state_publisher)

        try:
            biometra_joint_state_publisher.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            biometra_joint_state_publisher.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            executor.shutdown()
            biometra_joint_state_publisher.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()