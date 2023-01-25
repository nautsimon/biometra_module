#! /usr/bin/env python3

import rclpy                 # import Rospy
from rclpy.node import Node  # import Rospy Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from std_msgs.msg import String

from wei_services.srv import WeiDescription 
from wei_services.srv import WeiActions   

from time import sleep

from biometra_driver.biometra_driver import biometra_trobot

class BiometraClient(Node):
    '''
    The biometraNode inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback, 
    based on the executed command and publishes the state of the biometra and a description of the biometra to the respective topics.
    '''
    def __init__(self, TEMP_NODE_NAME = "BiometraNode"):
        '''
        The init function is neccesary for the biometraNode class to initialize all variables, parameters, and other functions.
        Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
        '''

        super().__init__(TEMP_NODE_NAME)
        node_name = self.get_name()

        self.declare_parameter("port",8085)
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        self.get_logger().info("Received Port:" + str(self.port))

        self.state = "READY"

        self.connect_robot()

        action_cb_group = ReentrantCallbackGroup()
        description_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()
        
        self.description = {
            'name': node_name,
            'type': 'biometra_thermocicler',
            'actions':
            {
                'status':'',
                'open_lid':'',
                'close_lid':'',
                'run_program':'program_n'
            }
        }

        timer_period = 1  # seconds
        self.statePub = self.create_publisher(String, node_name + '/state', 10)
        self.stateTimer = self.create_timer(timer_period, self.stateCallback, callback_group = state_cb_group)

        self.actionSrv = self.create_service(WeiActions, node_name + "/action_handler", self.actionCallback, callback_group = action_cb_group)
        self.descriptionSrv = self.create_service(WeiDescription, node_name + "/description_handler", self.descriptionCallback, callback_group = description_cb_group)

    def connect_robot(self):
        try:
            self.biometra = biometra_trobot()
        except Exception as error_msg:
            self.state = "BIOMETRA CONNECTION ERROR"
            self.get_logger().error("------- BIOMETRA Error message: " + str(error_msg) +  (" -------"))
        else:
            self.get_logger().info("Biometra is online")

    def stateCallback(self):
        '''
        Publishes the biometra state to the 'state' topic. 
        '''
        msg = String()
        msg.data = 'State: %s' % self.state
        self.statePub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.state = "READY"

    def descriptionCallback(self, request, response):
        """The descriptionCallback function is a service that can be called to showcase the available actions a robot
        can preform as well as deliver essential information required by the master node.

        Parameters:
        -----------
        request: str
            Request to the robot to deliver actions
        response: str
            The actions a robot can do, will be populated during execution

        Returns
        -------
        str
            The robot steps it can do
        """
        response.description_response = str(self.description)

        return response

    def actionCallback(self, request, response):
        '''
        The actionCallback function is a service that can be called to execute the available actions the robot
        can preform.
        '''

        if request.action_handle=='status':
            self.biometra.get_status()
            response.action_response = True
        if request.action_handle=='open_lid':            
            self.state = "BUSY"
            self.stateCallback()
            self.biometra.open_lid()    
            response.action_response = True
        if request.action_handle=='close_lid':            
            self.state = "BUSY"
            self.stateCallback()
            self.biometra.close_lid()    
            response.action_response = True
        if request.action_handle=='close_lid':
            self.state = "BUSY"
            self.stateCallback()
            vars = eval(request.vars)
            print(vars)
            prog = vars.get('program_n')
            self.biometra.run_program(prog)
        self.state = "COMPLETED"

        return response



def main(args = None):
    
    rclpy.init(args=args)  # initialize Ros2 communication
    try:
        biometra_client = BiometraClient()
        executor = MultiThreadedExecutor()
        executor.add_node(biometra_client)

        try:
            biometra_client.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            biometra_client.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            executor.shutdown()
            biometra_client.destroy_node()
    finally:
        rclpy.shutdown()
if __name__ == '__main__':
    main()
