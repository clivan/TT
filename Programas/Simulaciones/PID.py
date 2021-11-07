# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt

#Parámetros del vehículo
LARGO=0.364 #[m]
ANCHO=0.19 #[m]
L=0.285 #Distancia entre ejes [m]
POSEJE=0.03 #Distancia de la parte posterior al eje posterior [m]
RUE_LAR=0.0325 #[m]
RUE_AN=0.026 #[m]
HUELLA=0.08 #Separación de las ruedas con las partes laterales [m]
MAX_ANG=math.radians(40.0) #Máximo ángulo de gobierno [rad]
MAX_VELANG=math.radians(30.0) #Máxima velocidad para alcanzar el ángulo [rad/s]
MAX_VEL=5.5/3.6 #Máxima velocidad [m/s]
MIN_VEL=-2.0/3.6 #Mínima velocidad [m/s]
MAX_ACE=0.1 #Aceleración máxima [m/s/s]
dt=0.1 #[s]
K=10
Kp=10.0
Kd=25.0 #
Ki=0.001 #
vel=0.285/5

class Robot:
    def __init__(self, x=0.0, y=0.0, th=0.0, v=0.0):
        """
        Crea un objeto Robot e inicializa su configuración en (0, 0, 0).
        """
        self.x=x #[m]
        self.y=y #[m]
        self.th=th #[rad]
        self.v=v #[m]

    def set(self, x, y, th):
        self.x=x
        self.y=y
        self.th=th%(2.0*np.pi)

    def mover(self, phi, v, tol=0.001):
        if phi>MAX_ANG:
            phi=MAX_ANG
        if phi<-MAX_ANG:
            phi=-MAX_ANG
        if v<0.0:
            v=0.0
        giro=np.tan(phi)*v/L
        if abs(giro)<tol:
            self.x+=v*np.cos(self.th)
            self.y+=v*np.sin(self.th)
            self.th=(self.th+giro)%(2.0*np.pi)
        else:
            r=v/giro
            cx=self.x-(np.sin(self.th)*r)
            cy=self.y+(np.cos(self.th)*r)
            self.th=(self.th+giro)%(2.0*np.pi)
            self.x=cx+(np.sin(self.th)*r)
            self.y=cy-(np.cos(self.th)*r)

def pi2pi(ang):
    return(ang+np.pi)%(2*np.pi)-np.pi

def velPID(obj, act):
    return K*(obj-act)

def sim(robot):
    X=[]
    Y=[]
    TH=[]
    PHI=[]
    V=[]
    prev=robot.y
    cteI=0
    for i in range(15):
        cte=robot.y
        cteD=cte-prev
        prev=cte
        cteI+=cte
        phi=-Kp*cte-Kd*cteD-Ki*cteI
        #a=velPID(MAX_VEL, robot.v)
        robot.mover(phi, vel)
        X.append(robot.x)
        Y.append(robot.y)
        TH.append(robot.th)
        PHI.append(phi)
        print("x:{}\ty:{}\tth:{}\tphi:{}\tv:{}\terrP:{}\terrD:{}\terrI{}\terr:{}".format(robot.x, robot.y, robot.th, phi, robot.v, cte, cteD, cteI, MAX_VEL-robot.v))
    return X, Y, TH, PHI, V

def grafCar(x, y, th, phi):
    Chas=np.matrix([[-POSEJE, (LARGO-POSEJE), (LARGO-POSEJE), -POSEJE, -POSEJE], [ANCHO/2, ANCHO/2, -ANCHO/2, -ANCHO/2, ANCHO/2]])
    Rfd=np.matrix([[RUE_LAR, -RUE_LAR, -RUE_LAR, RUE_LAR, RUE_LAR], [-RUE_AN-HUELLA, -RUE_AN-HUELLA, RUE_AN-HUELLA, RUE_AN-HUELLA, -RUE_AN-HUELLA]])
    Rtd=np.copy(Rfd)
    Rfi=np.copy(Rfd)
    Rfi[1, :]*=-1
    Rti=np.copy(Rtd)
    Rti[1, :]*=-1
    MR1=np.matrix([[math.cos(th), math.sin(th)], [-math.sin(th), math.cos(th)]])
    MR2=np.matrix([[math.cos(phi), math.sin(phi)], [-math.sin(phi), math.cos(phi)]])
    Rfd=(Rfd.T*MR2).T
    Rfi=(Rfi.T*MR2).T
    Rfd[0, :]+=L
    Rfi[0, :]+=L
    Rfd=(Rfd.T*MR1).T
    Rfi=(Rfi.T*MR1).T
    Chas=(Chas.T*MR1).T
    Rtd=(Rtd.T*MR1).T
    Rti=(Rti.T*MR1).T
    Chas[0, :]+=x
    Chas[1, :]+=y
    Rfd[0, :]+=x
    Rfd[1, :]+=y
    Rfi[0, :]+=x
    Rfi[1, :]+=y
    Rtd[0, :]+=x
    Rtd[1, :]+=y
    Rti[0, :]+=x
    Rti[1, :]+=y
    plt.plot(np.array(Chas[0, :]).flatten(), np.array(Chas[1, :]).flatten(), "-k")
    plt.plot(np.array(Rfd[0, :]).flatten(), np.array(Rfd[1, :]).flatten(), "-b")
    plt.plot(np.array(Rfi[0, :]).flatten(), np.array(Rfi[1, :]).flatten(), "-b")
    plt.plot(np.array(Rtd[0, :]).flatten(), np.array(Rtd[1, :]).flatten(), "-b")
    plt.plot(np.array(Rti[0, :]).flatten(), np.array(Rti[1, :]).flatten(), "-b")
    plt.plot(x, y, '*r')

def main():
    robot=Robot()
    robot.set(0, 0.1, 0)
    X, Y, TH, PHI, V=sim(robot)
    n=len(X)
    plt.plot(X, np.zeros(n))
    plt.plot(X, Y)
    grafCar(X[0], Y[0], TH[0], PHI[0])
    grafCar(X[n-1], Y[n-1], TH[n-1], PHI[n-1])
    plt.axis('equal')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Controlador PID')
    plt.show()
    
if __name__=='__main__':
    main()
