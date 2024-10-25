# TODO how to handle this ?
ROS = True

if ROS:
    import rclpy


class MetaRclpy:
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
    def spin_once(*args, **kwargs):
        if ROS:
            return rclpy.spin_once(*args, **kwargs)
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
