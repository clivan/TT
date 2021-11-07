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

K_rho=9
K_alpha=15
K_beta=-3
dt=0.01

def sim(xi, yi, thi, xf, yf, thf):
    x=xi
    y=yi
    th=thi
    xx=xf-x
    yy=yf-y
    X=[]
    Y=[]
    TH=[]
    T=[]
    A=[]
    B=[]
    R=[]
    V=[]
    W=[]
    P=[]
    rho=np.sqrt(xx**2+yy**2)
    t=0
    alpha=0
    beta=0
    v=0
    w=0
    phi=0
    while rho>0.001:
        X.append(x)
        Y.append(y)
        TH.append(th)
        T.append(t)
        A.append(alpha)
        B.append(beta)
        R.append(rho)
        V.append(v)
        W.append(w)
        P.append(phi)
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
        t=t+1
    T=np.multiply(T, 0.01)
    plt.figure()
    plt.arrow(xi, yi, math.cos(thi), math.sin(thi), color='r', width=0.01)
    plt.arrow(xf, yf, math.cos(thf), math.sin(thf), color='g', width=0.01)
    grafCar(xi, yi, thi, np.pi/2)
    grafCar(xf, yf, thf, phi)
    plt.plot(X, Y, 'b--')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.xlim(-0.5, 3.5)
    plt.ylim(-0.5, 3.5)
    plt.figure()
    plt.subplot(2, 2, 1)
    plt.plot(T[1:np.size(T)], X[1:np.size(V)])
    plt.ylabel('Recorrido en x (m)')
    plt.xlabel('Tiempo (s)')
    plt.subplot(2, 2, 2)
    plt.plot(T[1:np.size(T)], Y[1:np.size(W)])
    plt.ylabel('Recorrido en y (m)')
    plt.xlabel('Tiempo (s)')
    plt.subplot(2, 2, 3)
    plt.plot(T[1:np.size(T)], TH[1:np.size(V)])
    plt.ylabel('Orientacion del vehiculo (rad)')
    plt.xlabel('Tiempo (s)')
    plt.subplot(2, 2, 4)
    plt.plot(T[1:np.size(T)], P[1:np.size(W)])
    plt.ylabel('Orientacion de las ruedas (rad)')
    plt.xlabel('Tiempo (s)')
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(T[1:np.size(T)], R[1:np.size(V)])
    plt.ylabel(r'$\rho$')
    plt.xlabel('Tiempo (s)')
    plt.subplot(3, 1, 2)
    plt.plot(T[1:np.size(T)], A[1:np.size(W)])
    plt.ylabel(r'$\alpha$')
    plt.xlabel('Tiempo (s)')
    plt.subplot(3, 1, 3)
    plt.plot(T[1:np.size(T)], B[1:np.size(V)])
    plt.ylabel(r'$\beta$')
    plt.xlabel('Tiempo (s)')
    plt.figure()
    plt.subplot(1, 2, 1)
    plt.plot(T[1:np.size(T)], V[1:np.size(V)])
    plt.ylabel('Velocidad lineal (m/s)')
    plt.xlabel('Tiempo (s)')
    plt.subplot(1, 2, 2)
    plt.plot(T[1:np.size(T)], W[1:np.size(W)])
    plt.ylabel('Velocidad angular (rad/s)')
    plt.xlabel('Tiempo (s)')
        
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
    xi=2
    yi=1
    thi=100*np.pi/180
    xf=0.5
    yf=2.3
    thf=120*np.pi/180
    sim(xi, yi, thi, xf, yf, thf)
    plt.show()

if __name__=='__main__':
    main()
