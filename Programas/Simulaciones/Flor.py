# -*- coding: utf-8 -*-
import numpy as np
import math
import matplotlib.pyplot as plt
from random import random

#Parámetros del vehículo
LARGO=0.35 #[m]
ANCHO=0.2 #[m]
POSEJE=0.08 #Distancia de la parte posterior al eje posterior [m]
RUE_LAR=0.03 #[m]
RUE_AN=0.02 #[m]
HUELLA=0.08 #Separación de las ruedas con las partes laterales [m]
L=0.2 #Distancia entre ejes [m]
MAX_ANG=math.radians(45.0) #Máximo ángulo de gobierno [rad]
MAX_VELANG=math.radians(30.0) #Máxima velocidad para alcanzar el ángulo [rad/s]
MAX_VEL=5.5/3.6 #Máxima velocidad [m/s]
MIN_VEL=-2.0/3.6 #Mínima velocidad [m/s]
MAX_ACE=0.1 #Aceleración máxima [m/s²]

K_rho=0.5
K_alpha=9
K_beta=-1
dt=0.01

def mover(xi, yi, thi, xf, yf, thf, col):
    x=xi
    y=yi
    th=thi
    xx=xf-x
    yy=yf-y
    X=[]
    Y=[]
    rho=np.sqrt(xx**2+yy**2)
    while rho>0.001:
        X.append(x)
        Y.append(y)
        xx=xf-x
        yy=yf-y
        rho=np.sqrt(xx**2+yy**2)
        alpha=(np.arctan2(yy, xx)-th+np.pi)%(2*np.pi)-np.pi
        beta=(thf-th-alpha+np.pi)%(2*np.pi)-np.pi
        v=K_rho*rho
        w=K_alpha*alpha+K_beta*beta
        if alpha>np.pi/2 or alpha<-np.pi/2:
            v=-v
        th=th+w*dt
        x=x+v*np.cos(th)*dt
        y=y+v*np.sin(th)*dt
        phi=np.arctan2(w*L, v)
    grafCar(xf, yf, thf, phi)
    plt.plot(X, Y, col)
    
        
def grafCar(x, y, th, phi):
    if (phi>MAX_ANG):
        phi=MAX_ANG
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
    plt.plot(x, y, "*r")

def main():
    xi=5
    yi=5
    thi=np.pi/2
    grafCar(xi, yi, thi, np.pi/2)
    x1=5
    y1=9
    th1=0*np.pi
    mover(xi, yi, thi, x1, y1, th1, 'b--')
    x2=7.5
    y2=7.5
    th2=0*np.pi
    mover(xi, yi, thi, x2, y2, th2, 'g--')
    x3=9
    y3=5
    th3=0*np.pi
    mover(xi, yi, thi, x3, y3, th3, 'r--')
    x4=7.5
    y4=2.5
    th4=0*np.pi
    mover(xi, yi, thi, x4, y4, th4, 'c--')
    x5=5
    y5=1
    th5=0*np.pi
    mover(xi, yi, thi, x5, y5, th5, 'm--')
    x6=2.5
    y6=2.5
    th6=0*np.pi
    mover(xi, yi, thi, x6, y6, th6, 'y--')
    x7=1
    y7=5
    th7=0*np.pi
    mover(xi, yi, thi, x7, y7, th7, 'k--')
    x8=2.5
    y8=7.5
    th8=0*np.pi
    mover(xi, yi, thi, x8, y8, th8, 'r--')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()

if __name__=='__main__':
    main()
