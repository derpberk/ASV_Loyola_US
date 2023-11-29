import mysql.connector
from rclpy.logging import get_logger  # Import the logger if using ROS 2

class Database(object):
    def __init__(self, hostname='golem.us.es', port='6006', username=None, password=None, database=None):
        self.connection = mysql.connector.connect(
            host=hostname,
            port=port,
            user=username,
            password=password,
            database=database
        )

    def insert_record_data(self, Date=None, Latitud=None, Longitud=None, Sensor=None, ASV=None, Data=None):
        logger = get_logger(__name__)  # Use appropriate logger if using ROS 2
        logger.info("Sending data")

        cursor = self.connection.cursor()

        create_table_query = """
        CREATE TABLE IF NOT EXISTS ASV(
            Date TIMESTAMP,
            Latitud NUMERIC(32, 30),
            Longitud NUMERIC(32, 30),
            Sensor VARCHAR(255),
            ASV INT,
            Data FLOAT
        )
        """
        cursor.execute(create_table_query)

        query = 'INSERT INTO ASV (Date, Latitud, Longitud, Sensor, ASV, Data) VALUES (%s, %s, %s, %s, %s, %s)'
        args = [Date, Latitud, Longitud, Sensor, ASV, Data]

        cursor.execute(query, args)
        self.connection.commit()

        # cursor.close()
        # self.connection.close()
