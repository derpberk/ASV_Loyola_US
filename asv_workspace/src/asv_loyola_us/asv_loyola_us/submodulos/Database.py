import pymysql.cursors
import time
import datetime
from brping import Ping1D
import threading
from std_msgs.msg import Float32

import subprocess  # For executing a shell command




class Database(object):
    def __init__(self):
        self.hostname ='127.0.0.1'
        self.port= 3306
        self.username = 'root'
        self.password = 'password'
        self.database = 'Datas'    




    def insert_record(self,ID,SAMPLE_NUM=None, BAT=None, TEMP=None, PH=None, DO=None, LATITUD=None, LONGITUD=None, COND=None, ORP=None,SONAR=None, DATE=None):
        self.hostname ='127.0.0.1'
        self.port= 3306
        self.username = 'root'
        self.password = 'password'
        self.database = 'Datas'    
        query = 'INSERT INTO sensor (ID, SAMPLE_NUM, BAT, TEMP, PH, DO, LATITUD, LONGITUD, COND, ORP,SONAR, DATE) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s,%s)' 
        args = [ID,SAMPLE_NUM, BAT, TEMP, PH, DO, LATITUD, LONGITUD, COND, ORP,SONAR, DATE]
        now = datetime.datetime.now()
        date = now.strftime('%Y-%m-%d %H:%M:%S')
        
        # if prct_batery is not None:
        #     query += ",BAT"
        #     args.append(prct_batery)
        # if latitud is not None:
        #     query += ",latitud"
        #     args.append(latitud)
        # if longitud is not None:
        #     query += ",longitud"
        #     args.append(longitud)
        # if sensor_sonar is not None:
        #     query += ",sensor_sonar"
        #     args.append(sensor_sonar)
        # if date is not None:
        #     query += ",date"
        #     args.append(date)
        # query += ") VALUES (%s" + ",%s"*(len(args)-1) + ")"

        cursor_ = None
        try:
            conn = pymysql.connect(host= self.hostname, user=self.username, passwd=self.password, db=self.database, port=self.port)
            cursor_ = conn.cursor()
            cursor_.execute(query, args)
            conn.commit()

        except Exception as error:
            print(error)
            
        finally:
            cursor_.close()
            conn.close()
