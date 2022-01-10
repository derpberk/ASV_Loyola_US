servicio de flush waypoints accesible por MQTT

Tenemos un nodo central, que inicia todos los nodos de comunicaciones

Dronekit y MQTT funcionan a la misma frecuencia !!!!



#una idea de log alternativa

x = rclpy.create_node('mission_node') #we state what node we are
#we need to make sure we are subscribed to logs, so we create a dummy publisher as subscriptions happen syncronously
publisher = x.create_publisher(Log, 'rosout', 10) #we create the publisher
while publisher.get_subscription_count() == 0: #while rosout is not up
    sleep(0.01) #we wait
#we publish the error
x.get_logger().fatal(traceback.format_exc())

#presenta el problema de los namespaces y la id del dron





una posible solución es utilizar una clase llamada logger con los mismos atributos de log, que alimente a rosout y al watchdog de cada dron a la vez


