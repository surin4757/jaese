#!/usr/bin/env python3

#timer control -> req/res coltrol
import sys
import threading
import rclpy
from rclpy.node import Node
from paint_msgs.srv import LiftHigh
from functools import partial
from PySide6.QtWidgets import QApplication, QMainWindow
from paint.ui.main_ui import Ui_MainWindow
from PySide6.QtCore import Qt, QObject, Signal, Slot, QRect


class Lift_Node(Node):
    def __init__(self, container):
        super().__init__("Lift_Node")
        self.timer_active = True
        self.container = container

        self.get_logger().info("Lift Node has been started")

        self.lift_service_client = self.create_client(LiftHigh, 'Lift_High_service')
        while not self.lift_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again ...')
        self.container.server_connected = True
        self.request = LiftHigh.Request()

        self.send_request()

    def send_request(self):
        self.request.req = self.container.send_msg

        if self.request.req != 1:  # if req is not the default value (1)
            self.get_logger().info(f"--Sending request with req: {self.request.req}")
            self.future = self.lift_service_client.call_async(self.request)
            self.future.add_done_callback(partial(self.callback_command))
        else:
            self.future = self.lift_service_client.call_async(self.request)
            self.future.add_done_callback(partial(self.callback_get_high))

    def callback_get_high(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"get high: {response.high}")
            self.container.lift_high = response.high
            self.container.update_signal.emit(response.high)

            self.send_request()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def callback_command(self, future):
        try:
            print("up start")
            response = future.result()
            self.get_logger().info(f"Sending request with req: {self.request.req}")
            self.request.req = self.container.send_msg
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        finally :
            self.container.send_msg = 1
            self.send_request()

class MyApp(QMainWindow, Ui_MainWindow):
    def __init__(self, container):
        super().__init__()
        self.setupUi(self)
        self.container = container
        self.container.update_signal.connect(self.update_lift_status)
        self.lift_modulecommand_up.clicked.connect(self.handle_lift_up_command)
        self.lift_modulecommand_down.clicked.connect(self.handle_lift_down_command)

    @Slot(float)
    def update_lift_status(self, value):
        self.lift_module_lcdbar_high.display(value)
        Toplift = 90 - int(value)
        self.Toplift1.setGeometry(QRect(30, Toplift, 50, 120))
        self.Toplift2.setGeometry(QRect(95, Toplift, 50, 120))
        self.Toplift3.setGeometry(QRect(160, Toplift, 50, 120))
        self.Toplift4.setGeometry(QRect(225, Toplift, 50, 120))
    def handle_lift_up_command(self) :
        self.container.send_msg = 2
    def handle_lift_down_command(self) :
        self.container.send_msg = 3

class Container(QObject):
    update_signal = Signal(float)
    send_msg = 1
    lift_high = 0.0
    server_connected = False

    def __init__(self):
        super().__init__()

def gui_handle(container):
    app = QApplication(sys.argv)
    window = MyApp(container)
    window.show()
    app.exec()

def main(args=None):
    rclpy.init(args=args)
    lift_container = Container()
    gui_thread = threading.Thread(target=gui_handle, args=(lift_container,))
    gui_thread.start()
    node = Lift_Node(lift_container)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()