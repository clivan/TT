# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt

#Parámetros del vehículo
LARGO=0.364 #[m]
ANCHO=0.19 #[m]
DISEJE=0.285 #[m]
POSEJE=0.08 #Distancia de la parte posterior al eje posterior [m]
RUE_LAR=0.0325 #[m]
RUE_AN=0.026 #[m]
HUELLA=0.08 #Separación de las ruedas con las partes laterales [m]
L=0.2 #Distancia entre ejes [m]
MAX_ANG=math.radians(40.0) #Máximo ángulo de gobierno [rad]
MAX_VELANG=math.radians(30.0) #Máxima velocidad para alcanzar el ángulo [rad/s]
MAX_VEL=5.5/3.6 #Máxima velocidad [m/s]
MIN_VEL=-2.0/3.6 #Mínima velocidad [m/s]
MAX_ACE=0.1 #Aceleración máxima [m/s²]

class Robot(object):
    def __init__(self, longi=DISEJE):
        """
        Crea un objeto Robot e inicializa su configuración en (0, 0, 0).
        """
        self.x=0.0 #[m]
        self.y=0.0 #[m]
        self.th=0.0 #[rad]
        self.longi=longi #[m]

    def set(self, x, y, th):
        """
        Asignar nueva configuación al robot.
        """
        self.x = x
        self.y = y
        self.th=th%(2.0*np.pi)
        
    def mover(self, phi, distancia, tolerancia=0.001):
        """
        Calcula los valores para la nueva configuración
        Dirección=Ángulo de dirección de las ruedas frontales, está limitado por MAX_ANG
        Distancia=Distancia total a recorrer (>0)
        """
        if phi>MAX_ANG:
            phi=MAX_ANG
        if phi<-MAX_ANG:
            phi=-MAX_ANG
        if distancia<0.0:
            distancia=0.0
        giro=np.tan(phi)*distancia/self.longi
        if abs(giro)<tolerancia:
            self.x+=distancia*np.cos(self.th)
            self.y+=distancia*np.sin(self.th)
            self.th=(self.th+giro)%(2.0*np.pi)
        else:
            radio=distancia/giro
            cx=self.x-(np.sin(self.th)*radio)
            cy=self.y+(np.cos(self.th)*radio)
            self.th=(self.th+giro)%(2.0*np.pi)
            self.x=cx+(np.sin(self.th)*radio)
            self.y=cy-(np.cos(self.th)*radio)

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.th)

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
    

def robotito():
    """
    Reinicia el objeto Robot a su posición inicial.
    """
    robot=Robot()
    robot.set(0, 1, 0)
    robot.setP(10/180*np.pi)
    return robot


def run(robot, params, n=100, v=1.0):
    xt=[]
    yt=[]
    e=0
    prev=robot.y
    intcons=0
    for i in range(2*n):
        cte=robot.y
        diff=cte-prev
        intrcons=cte
        prev=cte
        pphi=-params[0]*cte-params[1]*diff-params[2]*intcons
        robot.mover(pphi, v)
        xt.append(robot.x)
        yt.append(robot.y)
        if i>=n:
            e+=cte**2
    return xt, yt, e/n

def girar(tol=0.2):
    p=[0, 0, 0]
    dp=[1, 1, 1]
    robot=robotito()
    xt, yt, me=run(robot, p)
    it=0
    while sum(dp) > tol:
        print("Iteración {}, Error = {}".format(it, me))
        for i in range(len(p)):
            p[i]+=dp[i]
            robot=robotito()
            x, y, e= run(robot, p)
            if e<me:
                me=e
                dp[i]*=1.1
            else:
                p[i]-=2*dp[i]
                robot=robotito()
                xt, yt, e=run(robot, p)
                if e<me:
                    me=e
                    dp[i]*=1.1
                else:
                    p[i]+=dp[i]
                    dp[i]*=0.9
        it += 1
    return p


params=girar()
robot=robotito()
xt, yt, e=run(robot, params)
n=len(xt)
plt.plot(xt, yt, 'g')
plt.plot(xt, np.zeros(n), 'r')
plt.show()
print("break point here")
