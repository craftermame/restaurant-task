from functools import wraps

import rclpy

def with_rclpy(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        if not rclpy.ok():
            rclpy.init()
        return func(*args, **kwargs)
    return wrapper
