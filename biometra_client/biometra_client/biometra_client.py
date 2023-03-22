#! /usr/bin/env python3
import string
from typing import List, Tuple

import rclpy  # import Rospy
from rclpy.node import Node  # import Rospy Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from std_msgs.msg import String

from wei_services.srv import WeiActions, WeiDescription

from time import sleep

from biometra_driver.biometra import Biometra
from biometra_driver.functions import Functions

class biometraNode(Node):
    '''
    The biometraNode inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback, 
    based on the executed command and publishes the state of the biometra and a description of the biometra to the respective topics.
    '''
    def __init__(self, NODE_NAME = "biometra_client"):
        '''
        The init function is neccesary for the biometraNode class to initialize all variables, parameters, and other functions.
        Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
        '''

        super().__init__(NODE_NAME)
        
        self.biometra = Biometra()
        self.functions = Functions()
        self.state = "READY"
        self.robot_status = ""
        self.action_flag = "READY"
        self.reset_request_count = 0


        
        self.description = {
            'name': NODE_NAME,
            'type': 'biometra_thermocycler',
            'actions':
            {
                'status':'',
                'open_lid':'',
                'close_lid':'',
                'run_program':'program_n'
            }
        }

        action_cb_group = ReentrantCallbackGroup()
        description_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()       
        state_refresher_cb_group = ReentrantCallbackGroup()

        timer_period = 0.5  # seconds
        state_refresher_timer_period = 0.5 # seconds

        self.StateRefresherTimer = self.create_timer(state_refresher_timer_period, callback = self.robot_state_refresher_callback, callback_group = state_refresher_cb_group)

        self.statePub = self.create_publisher(String, self.node_name + "/state", 10)       # Publisher for sealer state
        self.stateTimer = self.create_timer(timer_period, self.stateCallback, callback_group=state_cb_group)   # Callback that publishes to sealer state

        self.actionSrv = self.create_service(WeiActions, self.node_name + "/action_handler", self.actionCallback, callback_group=action_cb_group)
        self.descriptionSrv = self.create_service(WeiDescription, self.node_name + "/description_handler", self.descriptionCallback, callback_group=description_cb_group)


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
        #TODO: maybe check connection here


        while self.state != "READY":
            self.get_logger().warn("Waiting for Biometra to switch to READY state...")
            sleep(0.2)
        
        # action_handle = request.action_handle
        vars = eval(request.vars)
        self.get_logger().info(str(vars))
        self.action_flag = "BUSY"

        if request.action_handle =='status':
            self.get_logger().info('Starting Action ' + request.action_handle.upper())
            try:
                self.functions.get_status()

            except Exception as err:
                self.state = "ERROR"
                response.action_response = -1
                response.action_msg = str("Biometra") + " get_status failed. Error: " + str(err) # TODO: swap name for self.node_name
                self.get_logger().error(response.action_msg)
            
            else:
                self.state = "COMPLETED"
                response.action_response = 0
                response.action_msg = str("Biometra") + " get_status successful"
                self.get_logger().info(response.action_msg)
            
            finally:
                self.get_logger().info('Finished Action: ' + request.action_handle.upper())
                return response


        if request.action_handle=='open_lid':   
            self.get_logger().info('Starting Action ' + request.action_handle.upper())         
            
            try:
                self.functions.open_lid()

            except Exception as err:
                self.state = "ERROR"
                response.action_response = -1
                response.action_msg = str("Biometra") + " get_status failed. Error: " + str(err) # TODO: swap name for self.node_name
                self.get_logger().error(response.action_msg)

            else:
                self.state = "COMPLETED"
                response.action_response = 0
                response.action_msg = str("Biometra") + " get_status successful"
                self.get_logger().info(response.action_msg)

            finally:
                self.get_logger().info('Finished Action: ' + request.action_handle.upper())
                return response

        if request.action_handle=='close_lid':            
            self.get_logger().info('Starting Action ' + request.action_handle.upper())

            try:
                self.functions.close_lid()

            except Exception as err:
                self.state = "ERROR"
                response.action_response = -1
                response.action_msg = str("Biometra") + " get_status failed. Error: " + str(err) # TODO: swap name for self.node_name
                self.get_logger().error(response.action_msg)

            else:
                self.state = "COMPLETED"
                response.action_response = 0
                response.action_msg = str("Biometra") + " get_status successful"
                self.get_logger().info(response.action_msg)

            finally:
                self.get_logger().info('Finished Action: ' + request.action_handle.upper())
                return response

        if request.action_handle=='run_program':
            self.get_logger().info('Starting Action ' + request.action_handle.upper())
            try:
                prog_num = vars.get('program_n')
                self.biometra.run_program(prog_num)

            except Exception as err:
                self.state = "ERROR"
                response.action_response = -1
                response.action_msg = str("Biometra") + " get_status failed. Error: " + str(err) # TODO: swap name for self.node_name
                self.get_logger().error(response.action_msg)

            else:
                self.state = "COMPLETED"
                response.action_response = 0
                response.action_msg = str("Biometra") + " get_status successful"
                self.get_logger().info(response.action_msg)

            finally:
                self.get_logger().info('Finished Action: ' + request.action_handle.upper())
                return response

        else:
            msg = "UNKNOWN ACTION REQUEST! Available actions: status, open_lid, close_lid, run_program"
            response.action_response = -1
            response.action_msg = msg
            self.get_logger().error('Error ' + msg)
            self.state = "ERROR"
            return response

        return response

    def stateCallback(self):
        '''
        Publishes the biometra state to the 'state' topic. 
        '''
        msg = String()

        try:
            biometra_msg = self.functions.get_status() # TODO: translate status messages
        
        except Exception as err:
            self.get_logger().error("BIOMETRA IS NOT RESPONDING! ERROR: " + str(err))
            self.state = "BIOMETRA CONNECTION ERROR"
        
        if self.state != "BIOMETRA CONNECTION ERROR":

            if self.state == "ERROR" or self.biometra.status_msg == -1: #TODO: have biometra output this as variable
                msg.data = 'State: ERROR'
                self.statePub.publish(msg)
                self.get_logger().error(msg.data)
                self.get_logger().error(self.biometra.error_msg.upper()) # TODO:
                self.get_logger().warn('Trying to reset the Biometra')
                self.reset_request_count += 1
                self.action_flag = "READY"
                self.state = "UNKOWN"
            
            elif self.state == "COMPLETED" and self.action_flag == "BUSY":
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)
                self.action_flag = "READY"

            elif self.biometra.status_msg == 1 or self.action_flag == "BUSY": # TODO
                self.state = "BUSY"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)

            elif self.biometra.status_msg == 0 and self.action_flag == "READY":
                self.state = "READY"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)
        
        else:
            msg.data = 'State: %s' % self.state
            self.statePub.publish(msg)
            self.get_logger().error(msg.data)
            self.get_logger().warn("Cannot connect to Biomeetra! Check connection")



def main(args = None):
    # NAME = "biometraNode"
    # rclpy.init(args=args)  # initialize Ros2 communication
    # try:
    #     node = biometraNode(NODE_NAME=NAME)
    #     executor = MultiThreadedExecutor()
    #     executor.add_node(node)

    #     try:
    #         node.get_logger().info('Beginning client, shut down with CTRL-C')
    #         executor.spin()
    #     except KeyboardInterrupt:
    #         node.get_logger().info('Keyboard interupt, shutting down.\n')
    #     finally:
    #         executor.shutdown()
    #         node.destroy_node()
    
    # finally:
    #     rclpy.shutdown()

    NAME = "biometra_client"
    rclpy.init(args=args)  # initialize Ros2 communication
    node = biometraNode(NODE_NAME=NAME)
    rclpy.spin(node)     # keep Ros2 communication open for action node
    rclpy.shutdown()     # kill Ros2 communication


if __name__ == '__main__':
    main()
