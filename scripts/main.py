import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from diagnostic_updater import Updater, DiagnosticStatusWrapper

class DiagnosticNode(Node):
    def __init__(self):
        super().__init__('diagnostic_node')
        self.publisher_ = self.create_publisher(Int32, 'counter', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.counter_ = 0

        # Setup diagnostic updater
        self.updater_ = Updater(self)
        self.updater_.setHardwareID("DiagnosticNode")
        self.updater_.add("Counter Status", self.check_counter_diagnostic)

    def timer_callback(self):
        msg = Int32()
        self.counter_ += 1
        msg.data = self.counter_
        self.publisher_.publish(msg)

        # Update diagnostics
        self.updater_.update()

    def check_counter_diagnostic(self, stat:DiagnosticStatusWrapper):
        if self.counter_ % 10 == 0:
            stat.summary(DiagnosticStatusWrapper.OK, "Counter is OK")
        elif self.counter_ % 5 == 0:
            stat.summary(DiagnosticStatusWrapper.WARN, "Counter is divisible by 5")
        else:
            stat.summary(DiagnosticStatusWrapper.ERROR, "Counter is not divisible by 10 or 5")
        
        stat.add("Counter Value", str(self.counter_))
        return stat

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
