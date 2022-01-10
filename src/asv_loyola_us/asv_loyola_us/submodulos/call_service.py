

def call_service(self, client,  msg):

    # TODO: raise error to avoid infinite wait if service is not up, after all means a module is not active $$ watchdog will be in charge
    while not client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info(f'service {client.srv_name} not available, waiting again...', once=True)
    future = client.call_async(msg)
    while rclpy.ok():
        if future.done():
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                return response
            break