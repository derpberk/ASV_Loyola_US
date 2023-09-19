from asv_interfaces.msg import Nodeupdate

class Logger():
    def __init__(self, node):
        self.parent=node
        self.publisher=self.parent.create_publisher(Nodeupdate, 'internal_log', 10)  # we create the publisher
        self.msg=Nodeupdate()
        self.msg.node = node.get_name()  # our identity

    def fatal(self, log, once=False, skip=False):
        self.parent.get_logger().fatal(log)
        self.msg.message=log
        self.publisher.pub(self.msg)

    def error(self, log, once=False, skip=False):
        self.parent.get_logger().error(log)
        self.msg.message = log
        self.publisher.pub(self.msg)

    def warning(self, log, once=False, skip=False):
        self.parent.get_logger().warning(log)
        self.msg.message = log
        self.publisher.pub(self.msg)

    def info(self, log, once=False, skip=False):
        self.parent.get_logger().info(log)
        self.msg.message = log
        self.publisher.pub(self.msg)

    def debug(self, log, once=False, skip=False):
        self.parent.get_logger().debug(log)
        self.msg.message = log
        self.publisher.pub(self.msg)