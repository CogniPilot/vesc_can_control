#!/usr/bin/env python3
import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter, ParameterType, ParameterDescriptor
from actuator_msgs.msg import Actuators
from can_msgs.msg import Frame

class VESCCANControl(Node):
    def __init__(self):
        super().__init__('vesc_can_control_node')


        actuators_input_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='actuators input topic name.')

        can_output_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='can output topic name.')

        number_vesc_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Number of VESC to control.')

        actuators_pole_pair_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER_ARRAY,
            description='Actuator pole pairs for VESC eRPM.')

        actuators_index_can_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER_ARRAY,
            description='Actuator message index for CAN VESC.')

        can_node_id_actuators_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER_ARRAY,
            description='CAN node IDs mapped to actuator input indexs.')

        self.declare_parameter("actuators_input_topic", "/actuators", 
            actuators_input_topic_descriptor)

        self.declare_parameter("can_output_topic", "/can0_send",
            can_output_topic_descriptor)

        self.declare_parameter("number_vesc", 4, 
            number_vesc_descriptor)

        self.declare_parameter("actuators_pole_pair", [15, 15, 15, 15], 
            actuators_pole_pair_descriptor)

        self.declare_parameter("actuators_index_can", [0, 1, 2, 3], 
            actuators_index_can_descriptor)

        self.declare_parameter("can_node_id_actuators", [50, 51, 52, 53], 
            can_node_id_actuators_descriptor)

        self.NumberVESC = self.get_parameter("number_vesc").value
        self.ActuatorsPolePair = self.get_parameter("actuators_pole_pair").value
        self.ActuatorsIndexCAN = self.get_parameter("actuators_index_can").value
        self.CANNodeIdActuators = self.get_parameter("can_node_id_actuators").value
        self.ActuatorsSubTopic = self.get_parameter("actuators_input_topic").value
        self.CANPubTopic = self.get_parameter("can_output_topic").value

        self.ActuatorsSub = self.create_subscription(Actuators, '{:s}'.format(self.ActuatorsSubTopic), self.ActuatorsCallback, 1)
        self.CANPub = self.create_publisher(Frame, '{:s}'.format(self.CANPubTopic), 20)

    def ActuatorsCallback(self, msgActuators):
        for i in range(self.NumberVESC):
            actuatorIndex=self.ActuatorsIndexCAN[i]
            angularVelocity = msgActuators.velocity[actuatorIndex]
            rpm = angularVelocity*60/(2.0*np.pi)
            erpm = int(rpm*self.ActuatorsPolePair[i])
            data = np.array([erpm>>24 & 255, erpm>>16 & 255, erpm>>8 & 255, erpm & 255], dtype=np.uint8)
            msg = Frame()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "can0"
            msg.id = 768+self.CANNodeIdActuators[i]
            msg.is_rtr = False
            msg.is_extended = True
            msg.is_error = False
            msg.dlc = 4
            if msg.dlc < 8:
                msg.data = np.append(data, np.zeros(8-msg.dlc, dtype=np.uint8))
            else:
                msg.data = np.frombuffer(data, dtype=np.uint8, count=message.dlc)

            self.CANPub.publish(msg)
        return

def main(args=None):
    rclpy.init()
    VCC = VESCCANControl()
    rclpy.spin(VCC)
    VCC.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()