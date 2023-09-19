
import rclpy

from rclpy.node import Node


def call_service(self, client, msg):
    while not client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info(f'service {client.srv_name} not available, waiting again...', once=True)
    future = client.call_async(msg)
    self.get_logger().info(f'{self.get_name()} is calling service {client.srv_name}')
    while rclpy.ok():
        rclpy.spin_once(self)
        if future.done():
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().info(
                    'Service call failed %r' % (e,))
            self.get_logger().debug("answer gotten")
            return response
            break


#TODO: use self.spin_until_future_complete

def call_service_spin(client_ori, msg):
    x=rclpy.create_node("service_call") #we state what node we are
    client=x.create_client(client_ori.srv_type,client_ori.srv_name)
    while not client.wait_for_service(timeout_sec=1.0):
        x.get_logger().info(f'service {client_ori.srv_name} not available, waiting again...', once=True)
    future = client.call_async(msg)
    x.get_logger().debug(f'future created')
    while rclpy.ok():
        rclpy.spin_once(x)
        if future.done():
            try:
                response = future.result()
            except Exception as e:
                x.get_logger().info(
                    'Service call failed %r' % (e,))
                response = False
            x.get_logger().debug("answer gotten")
            x.destroy_node()
            return response
            break
#TODO: add tagg de la persona que llama al servicio


"""original call service function"""
def call_service_orig(client,  msg):
    # TODO: raise error to avoid infinite wait if service is not up, after all means a module is not active $$ watchdog will be in charge
    print(client, msg)
    while not client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info(f'service {client.srv_name} not available, waiting again...', once=True)
    future = client.call_async(msg)
    self.get_logger().info(f'future created {future}')
    while True:
        if future.done():
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().info(
                    'Service call failed %r' % (e,))
            self.get_logger().debug("answer gotten")
            return response
            break

            #TODO: la idea es crear un nuevo nodo, este se le llamará spin, y sera el encargado de gestionar el servidor, pudiendo asi usar el simple client