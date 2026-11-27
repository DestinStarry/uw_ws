"""Bounded one-motor CTRL/OFF probe. Default prints a plan; --execute opens UART.
Stop ROS hardware_node/CuteCom first. All servo commands are centered during this probe.
No ROS graph, allocator, IMU or MCU handshake is involved.
"""
import argparse
import math
import os
import shutil
import signal
import subprocess
import time

PINS = {1:"PE9 / TIM1_CH1",2:"PE11 / TIM1_CH2",3:"PE13 / TIM1_CH3",4:"PE14 / TIM1_CH4"}

def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port",default="/dev/ttyTHS2")
    parser.add_argument("--motor",type=int,choices=range(1,5),default=1)
    parser.add_argument("--level",type=float,default=.2)
    parser.add_argument("--seconds",type=float,default=1.)
    parser.add_argument("--execute",action="store_true",help="actually send to the selected serial port")
    opts=parser.parse_args(args)
    if not math.isfinite(opts.level) or not 0<abs(opts.level)<=.2:
        parser.error("level must be finite and 0 < abs(level) <= 0.2")
    if not math.isfinite(opts.seconds) or not .1<=opts.seconds<=2.:
        parser.error("seconds must be in [0.1, 2.0]")
    values=[0.]*7;values[opts.motor-1]=opts.level
    frame="CTRL:"+",".join(f"{x:.4f}" for x in values)+"\n"
    zero="CTRL:0,0,0,0,0,0,0\n"
    print(f"Motor {opts.motor}: {PINS[opts.motor]}; expected configured pulse {1500-int(opts.level*400)} us")
    print("All three servo commands: center. Plan: neutral 3s -> selected motor -> neutral 1s -> OFF.")
    print("Selected frame:",frame.strip())
    if not opts.execute:
        print("Preview only; no port opened. Add --execute only after preparing the physical test.")
        return
    if os.name!="posix":parser.error("execute on the Orin/Linux host")
    if shutil.which("fuser"):
        result=subprocess.run(["fuser",opts.port],capture_output=True,text=True)
        owners=[int(x) for x in result.stdout.split() if x.isdigit() and int(x)!=os.getpid()]
        if owners:parser.error("port already in use by PID(s): "+str(owners)+"; stop ROS/CuteCom first")
    import serial
    import fcntl
    import termios
    running=True
    def stop(signum,frame):
        nonlocal running
        running=False
    old={sig:signal.signal(sig,stop) for sig in (signal.SIGINT,signal.SIGTERM)}
    dev=None
    def send_for(text,seconds):
        end=time.monotonic()+seconds
        while running and time.monotonic()<end:
            dev.write(text.encode("ascii"));time.sleep(.02)
    try:
        dev=serial.Serial(opts.port,115200,timeout=0,write_timeout=.1,exclusive=True)
        fcntl.ioctl(dev.fileno(),termios.TIOCEXCL)
        send_for(zero,3.)
        if running:send_for(frame,opts.seconds)
        if running:send_for(zero,1.)
    finally:
        if dev is not None:
            for _ in range(10):
                try:dev.write(b"OFF\n")
                except (OSError,serial.SerialException):break
                time.sleep(.02)
            try:dev.flush()
            except (OSError,serial.SerialException):pass
            dev.close()
        for sig,handler in old.items():signal.signal(sig,handler)
    print("Probe ended; OFF requested. No MCU execution acknowledgement is available.")

if __name__=="__main__":main()
