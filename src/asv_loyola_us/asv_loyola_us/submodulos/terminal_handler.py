import subprocess  # For executing a shell command
import os
import signal
import psutil
import sys
import signal
from datetime import datetime
def ping_google():
    host="www.google.es"
    command = ["ping", "-c", "1", "-W2", host] #timeout 2 seconds

    return subprocess.run(args=command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0 #this returns 0 if ping is successfull

def check_internet():
    host="www.google.es"
      
    command = ["curl", "-I", "https://linuxhint.com/"]

    return subprocess.run(args=command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0 #this returns 0 if there is internet

def ping_mqtt():
    host="127.0.0.1"
    command = ["ping", "-c", "1", "-W2", host] #timeout 2 seconds

    return subprocess.run(args=command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0 #this returns 0 if ping is successfull
    
def check_ssh_tunelling():

    for proc in psutil.process_iter():
        try:
            # Check if process name contains the given name string.
            pinfo = proc.as_dict(attrs=['pid', 'name', 'create_time'])
            if "ssh" == pinfo['name'].lower():
                return True
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return False;

    
def start_ssh_tunneling():
    #arg i to understand aliases
    #stdout and stderr to do not print bash messages
    #preexec_fn so that we can close the command executed in bash and it doesnt keep running
    if subprocess.run(args=['/bin/bash', '-i', '-c', "mqttssh"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid).returncode == 0:
        return True
    return False

    #TODO: observar que valores se devuelven cuando hay error de ssh, se desconoce si lo que devuelve cuando ssh falla

def kill_ssh_tunelling():
    for proc in psutil.process_iter():
        try:
            # Check if process name contains the given name string.
            pinfo = proc.as_dict(attrs=['pid', 'name', 'create_time'])
            if "ssh" == pinfo['name'].lower():
                proc.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass


def show_ps():
    for proc in psutil.process_iter():
        try:
            # Check if process name contains the given name string.
            pinfo = proc.as_dict(attrs=['pid', 'name', 'create_time'])
            print(pinfo['name'].lower())
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass

def kill_ros2():
    processes=["mission", "mqtt", "drone", "sensors", "watchdog", "_ros2_daemon", "dummy_publisher", "camera","planner"]
    for proc in psutil.process_iter():
        try:
            # Check if process name contains the given name string.
            pinfo = proc.as_dict(attrs=['pid', 'name', 'create_time'])
            if pinfo['name'].lower() in processes:
                print(f"process {pinfo['name'].lower()} was killed")
                proc.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass


def restart_asv():
    subprocess.run(args=['/bin/bash', '-i', '-c', "restart_asv"], preexec_fn=os.setsid)

def start_recording():
    name=datetime.today()
    name=name.strftime('%Y.%m.%d..%H.%M')
    name=str("recording"+name+".svo")
    command="python3 /home/xavier/repositorios/zed-examples/svo\ recording/recording/python/svo_recording.py"+ " " + name

    return subprocess.Popen([command], cwd="/home/xavier/zed_datasets", shell=True, preexec_fn=os.setsid)

def singint_python():
    for proc in psutil.process_iter():
        try:
            # Check if process name contains the given name string.
            pinfo = proc.as_dict(attrs=['pid', 'name', 'create_time'])
            if "python3" == pinfo['name'].lower():
               print(f"process {pinfo['name'].lower()} was interrupted")
               proc.send_signal(signal.SIGINT)
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return False

def singint_pid(proccess):
    pid=proccess.pid
    try:
        os.killpg(os.getpgid(pid), signal.SIGINT)
    except:
        pass
    return False

def kill_python():
    for proc in psutil.process_iter():
        try:
            # Check if process name contains the given name string.
            pinfo = proc.as_dict(attrs=['pid', 'name', 'create_time'])
            if "python3" == pinfo['name'].lower():
               print(f"process {pinfo['name'].lower()} was killed")
               proc.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return False;
