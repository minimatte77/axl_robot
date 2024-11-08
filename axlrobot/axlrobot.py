#!/usr/bin/python3
# Description : axlrobot
# Website	 : ewp.it
# Date		: 2024/09/01
import socket
import time
import threading
import math

class Servo_ctrl(threading.Thread):
    def __init__(self, *args, **kwargs):
        super(Servo_ctrl, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.set()
        self.__running = threading.Event()
        self.__running.set()

    def run(self):
        global goal_pos, servo_command, init_get, if_continue, walk_step
        while self.__running.isSet():
            self.__flag.wait()
            if not steadyMode:
                command_GenOut()
                while move_smooth_goal():
                    if goal_command == 'stop':
                        break
                    else:
                        continue
                if goal_command == 'StandUp' or goal_command == 'StayLow' or goal_command == 'Lean-L' or goal_command == 'Lean-R':
                    servoStop()
            else:
                steady()
                time.sleep(0.03)		
            print('loop')

    def pause(self):
        self.__flag.clear()

    def resume(self):
        self.__flag.set()

    def stop(self):
        self.__flag.set()
        self.__running.clear()

class Head_ctrl(threading.Thread):
    def __init__(self, *args, **kwargs):
        super(Head_ctrl, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.set()
        self.__running = threading.Event()
        self.__running.set()

    def run(self):
        global T_command, P_command
        while self.__running.isSet():
            self.__flag.wait()
            if T_command == 'headUp':
                up(PT_speed)
            elif T_command == 'headDown':
                down(PT_speed)
            
            if P_command == 'headRight':
                lookright(PT_speed)
            elif P_command == 'headLeft':
                lookleft(PT_speed)

            if max_dict['P'] == goal_dict['P'] or min_dict['P'] == goal_dict['P']:
                P_command = 'stop'

            if max_dict['T'] == goal_dict['T'] or min_dict['T'] == goal_dict['T']:
                T_command = 'stop'

            if T_command == 'stop' and P_command == 'stop':
                self.pause()

            time.sleep(PT_deley)
            print('loop')



    def pause(self):
        self.__flag.clear()

    def resume(self):
        self.__flag.set()

    def stop(self):
        self.__flag.set()
        self.__running.clear()

class AxlRobot:
    def __init__(self,name):
        self.robotname=name
        self.speed=100
        self.legs_coord = [[0, 0], [0, 0], [0, 0], [0, 0]]
        try:
            import Adafruit_PCA9685
            self.pwm = Adafruit_PCA9685.PCA9685()
            self.pwm.set_pwm_freq(50)
            print('Init SpiderG : Ok...')
        except:
            import os
            os.system("sudo pip3 install adafruit-pca9685")
            import Adafruit_PCA9685
            self.pwm = Adafruit_PCA9685.PCA9685()
            self.pwm.set_pwm_freq(50)
            print('Init SpiderG : *** Error ***')

        MPU_connection = 1
        try:
            from mpu6050 import mpu6050
            import PID
            import Kalman_filter
            self.sensor = mpu6050(0x68)
            print('mpu6050 connected\nmpu6050 is connected and related functions are available.')
            self.mpu_tor = 0
            self.X_steady = 0
            self.Y_steady = 0
            self.P = 0.3
            self.I = 0.1
            self.D = 0
            self.X_pid = PID.PID()
            self.X_pid.SetKp(P)
            self.X_pid.SetKd(I)
            self.X_pid.SetKi(D)
            self.Y_pid = PID.PID()
            self.Y_pid.SetKp(P)
            self.Y_pid.SetKd(I)
            self.Y_pid.SetKi(D)
            self.kalman_filter_X =  Kalman_filter.Kalman_filter(0.001,0.1)
            self.kalman_filter_Y =  Kalman_filter.Kalman_filter(0.001,0.1)
            self.steadyMode = 0
        except:
            self.MPU_connection = 0
            self.steadyMode = 0
            print('mpu6050 disconnected\nmpu6050 is not connected and the related functions are unavailable.')

        self.FLF_port = 3
        self.FLT_port = 2

        self.FRF_port = 1
        self.FRT_port = 0

        self.RLF_port = 5
        self.RLT_port = 4

        self.RRF_port = 7
        self.RRT_port = 6

        self.P_port = 8
        self.T_port = 9

        # LEG I init
        self.FLF_init_pwm = 102
        self.FLF_min_pwm = 102
        self.FLF_max_pwm = 512

        self.FLT_init_pwm = 92
        self.FLT_min_pwm = 92
        self.FLT_max_pwm = 502

        # LEG II init
        self.FRF_init_pwm = 102
        self.FRF_min_pwm = 102
        self.FRF_max_pwm = 512

        self.FRT_init_pwm = 92
        self.FRT_min_pwm = 92
        self.FRT_max_pwm = 502
        
        # LEG III init
        self.RLF_init_pwm = 102 # 110
        self.RLF_min_pwm = 102 # 110
        self.RLF_max_pwm = 512 # 500

        self.RLT_init_pwm = 102 # 110
        self.RLT_min_pwm = 102 # 110
        self.RLT_max_pwm = 512 # 510

        # LEG IV init
        self.RRF_init_pwm = 132
        self.RRF_min_pwm = 132
        self.RRF_max_pwm = 542

        self.RRT_init_pwm = 132
        self.RRT_min_pwm = 132
        self.RRT_max_pwm = 542

        #PAN/TILT
        self.P_init_pwm   = 130
        self.P_min_pwm   = 130
        self.P_max_pwm   = 540

        self.T_init_pwm   = 300
        self.T_min_pwm   = 300
        self.T_max_pwm   = 300

        self.FLF_direction = 1
        self.FLT_direction = -1

        self.FRF_direction = -1
        self.FRT_direction = 1

        self.RLF_direction = 1
        self.RLT_direction = -1

        self.RRF_direction = -1
        self.RRT_direction = 1

        self.P_direction = 1
        self.T_direction = 1

        self.old_command = ''
        self.direction_dict = {self.FLF_port: self.FLF_direction, self.FLT_port: self.FLT_direction,
                    self.FRF_port: self.FRF_direction, self.FRT_port: self.FRT_direction,
                    self.RLF_port: self.RLF_direction, self.RLT_port: self.RLT_direction,
                    self.RRF_port: self.RRF_direction, self.RRT_port: self.RRT_direction,
                    self.P_port: self.P_direction, self.T_port: self.T_direction}

        self.old_dict = {self.FLF_port: self.FLF_init_pwm, self.FLT_port: self.FLT_init_pwm,
                    self.FRF_port: self.FRF_init_pwm, self.FRT_port: self.FRT_init_pwm,
                    self.RLF_port: self.RLF_init_pwm, self.RLT_port: self.RLT_init_pwm,
                    self.RRF_port: self.RRF_init_pwm, self.RRT_port: self.RRT_init_pwm,
                    self.P_port: self.P_init_pwm, self.T_port: self.T_init_pwm}

        self.now_command =''
        self.now_dict = {self.FLF_port: self.FLF_init_pwm, self.FLT_port: self.FLT_init_pwm,
                    self.FRF_port: self.FRF_init_pwm, self.FRT_port: self.FRT_init_pwm,
                    self.RLF_port: self.RLF_init_pwm, self.RLT_port: self.RLT_init_pwm,
                    self.RRF_port: self.RRF_init_pwm, self.RRT_port: self.RRT_init_pwm,
                    self.P_port: self.P_init_pwm, self.T_port: self.T_init_pwm}

        self.goal_command = ''
        self.goal_dict = {self.FLF_port: self.FLF_init_pwm, self.FLT_port: self.FLT_init_pwm,
                    self.FRF_port: self.FRF_init_pwm, self.FRT_port: self.FRT_init_pwm,
                    self.RLF_port: self.RLF_init_pwm, self.RLT_port: self.RLT_init_pwm,
                    self.RRF_port: self.RRF_init_pwm, self.RRT_port: self.RRT_init_pwm,
                    self.P_port: self.P_init_pwm, self.T_port: self.T_init_pwm}

        self.min_dict = {self.FLF_port: self.FLF_min_pwm, self.FLT_port: self.FLT_min_pwm,
                    self.FRF_port: self.FRF_min_pwm, self.FRT_port: self.FRT_min_pwm,
                    self.RLF_port: self.RLF_min_pwm, self.RLT_port: self.RLT_min_pwm,
                    self.RRF_port: self.RRF_min_pwm, self.RRT_port: self.RRT_min_pwm,
                    self.P_port: self.P_min_pwm, self.T_port: self.T_min_pwm}

        self.max_dict = {self.FLF_port: self.FLF_max_pwm, self.FLT_port: self.FLT_max_pwm,
                    self.FRF_port: self.FRF_max_pwm, self.FRT_port: self.FRT_max_pwm,
                    self.RLF_port: self.RLF_max_pwm, self.RLT_port: self.RLT_max_pwm,
                    self.RRF_port: self.RRF_max_pwm, self.RRT_port: self.RRT_max_pwm,
                    self.P_port: self.P_max_pwm, self.T_port: self.T_max_pwm}


        self.FLF_walk_pwm = self.angle_to_pwm(120,self.FLF_port)
        self.FLT_walk_pwm = self.angle_to_pwm(110,self.FLT_port)
        self.FRF_walk_pwm = self.angle_to_pwm(120,self.FRF_port)
        self.FRT_walk_pwm = self.angle_to_pwm(110,self.FRT_port)
        self.RLF_walk_pwm = self.angle_to_pwm(120,self.RLF_port)
        self.RLT_walk_pwm = self.angle_to_pwm(120,self.RLT_port)
        self.RRF_walk_pwm = self.angle_to_pwm(120,self.RRF_port)
        self.RRT_walk_pwm = self.angle_to_pwm(120,self.RRT_port)
        
        self.walk_dict = {self.FLF_port: self.FLF_walk_pwm, self.FLT_port: self.FLT_walk_pwm,
                    self.FRF_port: self.FRF_walk_pwm, self.FRT_port: self.FRT_walk_pwm,
                    self.RLF_port: self.RLF_walk_pwm, self.RLT_port: self.RLT_walk_pwm,
                    self.RRF_port: self.RRF_walk_pwm, self.RRT_port: self.RRT_walk_pwm,
                    self.P_port: 0, self.T_port: 0}


        self.PT_speed = 7
        self.P_command = 'stop'
        self.T_command = 'stop'
        self.PT_deley = 0.07
        self.global_position = 0
        self.gait_set = 1
        # Lunghezze dei segmenti della gamba
        l1 = 50  # Lunghezza del segmento dal corpo al ginocchio
        l2 = 50  # Lunghezza del segmento dal ginocchio al piede
    def getRobotName(self):
        return self.robotname

# FUNZIONI STANDARD MOVIMENTO
#
    def set_calibrate_pose(self):
        for leg in range(0,8):
            self.goal_dict[leg]=self.angle_to_pwm(90,leg)
        self.move_servo_all()
        self.old_command='CALIBRATE'
        self.now_command='CALIBRATE'
        self.goal_command='CALIBRATE'
        return self

    def repose(self):
        self.goal_command='REPOSE'
        self.goal_dict[self.FLF_port]=self.angle_to_pwm(170,self.FLF_port)
        self.goal_dict[self.FRF_port]=self.angle_to_pwm(170,self.FRF_port)
        self.goal_dict[self.FLT_port]=self.angle_to_pwm(30,self.FLT_port)
        self.goal_dict[self.FRT_port]=self.angle_to_pwm(30,self.FRT_port)
        self.move_servo_all()
        #time.sleep(0.2)
        self.goal_dict[self.RLF_port]=self.angle_to_pwm(170,self.RLF_port)
        self.goal_dict[self.RRF_port]=self.angle_to_pwm(170,self.RRF_port)
        self.goal_dict[self.RLT_port]=self.angle_to_pwm(20,self.RLT_port)
        self.goal_dict[self.RRT_port]=self.angle_to_pwm(20,self.RRT_port)
        self.move_servo_all()
        #
        self.save_command()
                
    def attention(self):
        self.goal_command='ATTENTION'
        if(self.now_command=='SITTING'):
            self.goal_dict[self.FLF_port]=self.angle_to_pwm(150,self.FLF_port)
            self.goal_dict[self.FRF_port]=self.angle_to_pwm(150,self.FRF_port)
            self.goal_dict[self.FLT_port]=self.angle_to_pwm(100,self.FLT_port)
            self.goal_dict[self.FRT_port]=self.angle_to_pwm(100,self.FRT_port)
            self.move_servo_all()
            time.sleep(0.5)
        #
        self.goal_dict[self.RLF_port]=self.angle_to_pwm(120,self.RLF_port)
        self.goal_dict[self.RRF_port]=self.angle_to_pwm(120,self.RRF_port)
        self.goal_dict[self.RLT_port]=self.angle_to_pwm(120,self.RLT_port)
        self.goal_dict[self.RRT_port]=self.angle_to_pwm(120,self.RRT_port)
        self.move_servo_all()
        time.sleep(0.2)
        self.goal_dict[self.FLF_port]=self.angle_to_pwm(120,self.FLF_port)
        self.goal_dict[self.FRF_port]=self.angle_to_pwm(120,self.FRF_port)
        self.goal_dict[self.FLT_port]=self.angle_to_pwm(110,self.FLT_port)
        self.goal_dict[self.FRT_port]=self.angle_to_pwm(110,self.FRT_port)
        self.move_servo_all()
        #
        self.save_command()
    
    def sitting(self):
        self.goal_command='SITTING'
        #
        self.goal_dict[self.FLF_port]=self.angle_to_pwm(140,self.FLF_port)
        self.goal_dict[self.FRF_port]=self.angle_to_pwm(140,self.FRF_port)
        self.goal_dict[self.FLT_port]=self.angle_to_pwm(90,self.FLT_port)
        self.goal_dict[self.FRT_port]=self.angle_to_pwm(90,self.FRT_port)
        self.move_servo_all()
        time.sleep(0.5)
        self.goal_dict[self.RLF_port]=self.angle_to_pwm(160,self.RLF_port)
        self.goal_dict[self.RRF_port]=self.angle_to_pwm(160,self.RRF_port)
        self.goal_dict[self.RLT_port]=self.angle_to_pwm(40,self.RLT_port)
        self.goal_dict[self.RRT_port]=self.angle_to_pwm(40,self.RRT_port)
        self.move_servo_all()
        time.sleep(0.2)
        self.goal_dict[self.FLF_port]=self.angle_to_pwm(120,self.FLF_port)
        self.goal_dict[self.FRF_port]=self.angle_to_pwm(120,self.FRF_port)
        self.goal_dict[self.FLT_port]=self.angle_to_pwm(110,self.FLT_port)
        self.goal_dict[self.FRT_port]=self.angle_to_pwm(110,self.FRT_port)
        self.move_servo_all()
        #
        self.save_command()

# FINE FUNZIONI STANDARD
    def save_command(self):
        self.old_command=self.now_command
        self.now_command=self.goal_command
        
    def set_pwm_axl(self,leg):
        if(self.speed==100):
            self.pwm.set_pwm(leg, 0, self.goal_dict[leg])
            #print('Speed: 100%')
        else:
            #print(f'Speed: {self.speed} %')
            if(self.goal_dict[leg]>=self.now_dict[leg]):
                step=int(((self.goal_dict[leg]-self.now_dict[leg])/100)*self.speed)
                if(step==0): step=1
                #print(f'Step : {self.goal_dict[leg]}')
                #print(f'Step : {self.now_dict[leg]}')
                #print(f'Step : {step}')
                for ipos in range(self.now_dict[leg],self.goal_dict[leg], step):
                    self.pwm.set_pwm(leg, 0, ipos)
                self.pwm.set_pwm(leg, 0, self.goal_dict[leg])
            else:
                step=int(((self.now_dict[leg]-self.goal_dict[leg])/100)*self.speed)
                if(step==0): step=1
                step=step*-1
                #print(f'Step : {self.goal_dict[leg]}')
                #print(f'Step : {self.now_dict[leg]}')
                #print(f'Step : {step}')
                for ipos in range(self.now_dict[leg],self.goal_dict[leg], step):
                    self.pwm.set_pwm(leg, 0, ipos)
                self.pwm.set_pwm(leg, 0, self.goal_dict[leg])                
            
 
    def setspeed(self,v_speed):
        if(v_speed>100): v_speed=100
        if(v_speed<=0): v_speed=1
        self.speed=v_speed
        print(f'Set speed {v_speed} %')
    
    def move_servo_all(self):
        for leg in range(0,8):
            #self.pwm.set_pwm(leg, 0, self.goal_dict[leg])
            self.set_pwm_axl(leg)
        #time.sleep(0.5)
        self.save_pose()

    def move_servo_single(self,servo):
        #self.pwm.set_pwm(servo, 0, self.goal_dict[servo])
        #self.set_pwm_axl(servo)
        self.set_pwm_axl(servo)
        
        self.save_pose()

    def angle_to_pwm(self,angle,port):
        if self.direction_dict[port]==-1:
            angle=180-angle
        pulse_min =  self.min_dict[port]  # Min pulse per il servo #102 #
        pulse_max =  self.max_dict[port]  # Max pulse per il servo #512 #
        angle_range = 180  # Gamma degli angoli del servo
        pulse_range = pulse_max - pulse_min
        pulse_width = pulse_min + (angle / angle_range) * pulse_range
        return int(pulse_width)

    def save_pose(self):
        for leg in range(0,8):
            self.old_dict[leg]=self.now_dict[leg]
            self.now_dict[leg]=self.goal_dict[leg]

    def move_leg(self,leg, angle):
        self.goal_dict[leg]=self.angle_to_pwm(angle,leg)
        self.move_servo_single(leg)

    def move_leg_pwm(self,leg, pwm):
        self.goal_dict[leg]=pwm
        self.move_servo_single(leg)

#posizioni
# Funzione per sollevare una gamba
    def move_leg_entire(self,leg1, leg2, angle1, angle2):
        self.goal_dict[leg1]=self.angle_to_pwm(angle1,leg1)
        self.move_servo_single(leg1)
        self.goal_dict[leg2]=self.angle_to_pwm(angle2,leg2)
        self.move_servo_single(leg2)        
    
    # Simula un passo per due gambe diagonali (anteriore sinistra, posteriore destra)
    def step_diagonal(self):
        self.move_leg(self.FRF_port, 120)
        self.move_leg(self.RLF_port, 120)
        # Solleva le zampe diagonali
        self.move_leg(self.FLT_port, 60)  
        self.move_leg(self.RRT_port, 60)  
        time.sleep(0.2)

        # Muove avanti le zampe diagonali
        self.move_leg(self.FLF_port, 80)
        self.move_leg(self.RRF_port, 80)
        time.sleep(0.2)

        # Abbassa le zampe diagonali
        self.move_leg(self.FLT_port, 120) 
        self.move_leg(self.RRT_port, 120) 
        time.sleep(0.2)
        
    # Simula un passo per le altre gambe diagonali (anteriore destra, posteriore sinistra)
    def step_other_diagonal(self):
        self.move_leg(self.FLF_port, 120)
        self.move_leg(self.RRF_port, 120)

        # Solleva le altre zampe diagonali
        self.move_leg(self.FRT_port, 60)  # Solleva anteriore destra
        self.move_leg(self.RLT_port, 60)  # Solleva posteriore sinistra
        time.sleep(0.2)

        # Muove avanti le zampe diagonali
        self.move_leg(self.FRF_port, 80)
        self.move_leg(self.RLF_port, 80)
        time.sleep(0.2)

        # Abbassa le zampe diagonali
        self.move_leg(self.FRT_port, 120)  # Solleva anteriore destra
        self.move_leg(self.RLT_port, 120)  # Solleva posteriore sinistra
        time.sleep(0.2)

    # Funzione per camminare simulando il trotto
    def walk_trotto(self,reset,steps=10):
        if(reset==1):
            self.set_calibrate_pose()  # Imposta la posizione iniziale
            time.sleep(0.5)
            self.attention()
        for _ in range(steps):
            self.step_diagonal()    
            time.sleep(0.2)
            self.step_other_diagonal()
            time.sleep(0.2)
        self.attention()


