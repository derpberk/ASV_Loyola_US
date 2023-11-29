import pymysql.cursors
import time
import datetime

import threading


import subprocess  # For executing a shell command




class Database(object):
    def __init__(self):
        self.hostname ='golem.us.es'
        self.port= 6006
        self.username = 'root'
        self.password = 'password'
        self.database = 'Datas'    




    def insert_record_sonar(self,ID, LATITUD=None, LONGITUD=None,SONAR=None, DATE=None,):

        

        query = 'INSERT INTO sensor (ID,DATE,LATITUD, LONGITUD,SONAR) VALUES (%s, %s, %s, %s, %s)' 
        args = [ID,DATE, LATITUD, LONGITUD,SONAR]
            

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

    def insert_record_sensor(self,ID, BAT=None, TEMP=None, PH=None, CON=None, LATITUD=None, LONGITUD=None, DATE=None,):

        

        query = 'INSERT INTO sensor (ID,LATITUD, LONGITUD,SONAR, DATE) VALUES (%s, %s, %s, %s, %s)' 
        args = [ID,DATE, LATITUD, LONGITUD,BAT,TEMP,PH,CON]
            

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