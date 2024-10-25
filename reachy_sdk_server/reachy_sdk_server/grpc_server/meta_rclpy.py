ROS = True


class MetaRclpy:
    if ROS:
        import rclpy

        logger = rclpy.impl.rcutils_logger.RcutilsLogger
        time = rclpy.time
        duration = rclpy.duration
        node = rclpy.node
        action = rclpy.action
        qos = rclpy.qos
    else:
        logger = None
        time = None
        duration = None
        node = None

    @staticmethod
    def init():
        if ROS:
            return rclpy.init()
        else:
            return None

    @staticmethod
    def ok():
        if ROS:
            return rclpy.ok()
        else:
            return None

    @staticmethod
    def spin(*args):
        if ROS:
            return rclpy.spin(*args)
        else:
            return

    @staticmethod
    def spin_once(*args):
        if ROS:
            return rclpy.spin_once(*args)
        else:
            return None

    @staticmethod
    def spin_until_future_complete(*args):
        if ROS:
            return rclpy.spin_until_future_complete(*args)
        else:
            return None

    @staticmethod
    def create_node(*args):
        if ROS:
            return rclpy.create_node(*args)
        else:
            return None

    @staticmethod
    def shutdown():
        if ROS:
            return rclpy.shutdown()
        else:
            return None
