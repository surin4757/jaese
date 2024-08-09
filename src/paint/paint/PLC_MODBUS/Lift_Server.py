#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from paint_msgs.srv import LiftHigh
import subprocess


from paint.app.lift import LiftController

class Lift_Node(Node) :
    def __init__(self) :
        super().__init__("Lift_Node")
        self.SERIAL = LiftController()
        self.lift_service = self.create_service(LiftHigh, 'Lift_High_service', self.callback)
        self.get_logger().info("LiftNodehasbeenstart")
        self.ret = float(0.0)
    
    def send_to_serial(self, com_sig) :
        if com_sig == 0 :
            ret = float(22.222)
            return ret

        elif com_sig == 1 :
            print(com_sig)
            ret = self.SERIAL.sendcommand(self.SERIAL.Read_PositionOfLift)
            ret = self.SERIAL.hex_to_float(ret[3:5])
            self.ret = ret
            return ret
        elif com_sig == 2 :
            print("lift up stat")
            ret = self.SERIAL.sendcommand(self.SERIAL.COMMAND_UP)
            ret = self.SERIAL.hex_to_float(ret[3:5])
            ret = float(self.ret)
            return ret
        elif com_sig == 3 :
            ret = self.SERIAL.sendcommand(self.SERIAL.COMMAND_DOWN)
            ret = self.SERIAL.hex_to_float(ret[3:5])
            ret = float(self.ret)
            return ret
        
    def callback(self, req, res) :
        com_sig = req.req
        res.high = self.send_to_serial(com_sig)
        return res
    def count_tac_time(self) :
        pass

def main(args = None) :
    rclpy.init(args=args)
    node = Lift_Node()
    rclpy.spin(node)
    rclpy.shutdown()