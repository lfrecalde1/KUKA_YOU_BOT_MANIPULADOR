"""Practica_2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import *
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
import time
import matplotlib as mpl
from funciones_manipulador import *

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(95)# 100 milisegundos equivale a 0.1 segundos

## Parte de declaracion de los motores del robot
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']

for i in range(4):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

## Parte de declaracion de las articulaciones del brazo
arm_elements=[]
arm_elements_names=['arm1','arm2','arm3','arm4','arm5']
for i in range(5):
    arm_elements.append(robot.getMotor(arm_elements_names[i]))
    arm_elements[i].setVelocity(0.1)

# DEFINICION DE LOS SENSORES D POSICION DE LAS ARTICULACIONES
arm_elements_sensor=[]
arm_elements_sensor_names=['arm1sensor','arm2sensor','arm3sensor','arm4sensor','arm5sensor']
for i in range(5):
    arm_elements_sensor.append(PositionSensor(arm_elements_sensor_names[i]))
    arm_elements_sensor[i].enable(timestep)

gps = GPS("gps1")
gps.enable(timestep)

home(arm_elements,5,robot)
# Definir el tiempo de sampleo del sistema
t_sample=0.1
t_final=30+t_sample
t=np.arange(0,t_final,t_sample)
t=t.reshape(1,t.shape[0])

#Parametros del BRAZO ROBOTICO
l0=0.14
l1=0.14
a1=0.033
l2=0.1550
l3=0.135
l4=0.2180

# Angulo control de cada articulacion
q1_ref=0*np.ones((t.shape[0],t.shape[1]+1))
q2_ref=0*np.ones((t.shape[0],t.shape[1]+1))
q3_ref=0*np.ones((t.shape[0],t.shape[1]+1))
q4_ref=0*np.ones((t.shape[0],t.shape[1]+1))
q5_ref=0*np.ones((t.shape[0],t.shape[1]+1))

# Velocidad control de cada Articulacion
q1p_ref=0*np.ones((t.shape[0],t.shape[1]))
q2p_ref=0*np.ones((t.shape[0],t.shape[1]))
q3p_ref=0*np.ones((t.shape[0],t.shape[1]))
q4p_ref=0*np.ones((t.shape[0],t.shape[1]))
q5p_ref=0*np.ones((t.shape[0],t.shape[1]))

# Angulo real de cada articulacion
q1=0*np.ones((t.shape[0],t.shape[1]+1))
q2=0*np.ones((t.shape[0],t.shape[1]+1))
q3=0*np.ones((t.shape[0],t.shape[1]+1))
q4=0*np.ones((t.shape[0],t.shape[1]+1))
q5=0*np.ones((t.shape[0],t.shape[1]+1))

# Velocidad real de cada Articulacion
q1p=0*np.ones((t.shape[0],t.shape[1]+1))
q2p=0*np.ones((t.shape[0],t.shape[1]+1))
q3p=0*np.ones((t.shape[0],t.shape[1]+1))
q4p=0*np.ones((t.shape[0],t.shape[1]+1))
q5p=0*np.ones((t.shape[0],t.shape[1]+1))

#Posiciones del robot
x=np.zeros((t.shape[0],t.shape[1]+1))
y=np.zeros((t.shape[0],t.shape[1]+1))
z=np.zeros((t.shape[0],t.shape[1]+1))

# Vector de posiciones deseadas
hxd=np.zeros((t.shape[0],t.shape[1]+1))
hyd=np.zeros((t.shape[0],t.shape[1]+1))
hzd=np.zeros((t.shape[0],t.shape[1]+1))


if robot.step(timestep) != -1:
    q1[0,0]=arm_elements_sensor[0].getValue()
    q2[0,0]=arm_elements_sensor[1].getValue()
    q3[0,0]=arm_elements_sensor[2].getValue()
    q4[0,0]=arm_elements_sensor[3].getValue()
    q5[0,0]=arm_elements_sensor[4].getValue()

    q1_ref[0,0]=arm_elements_sensor[0].getValue()
    q2_ref[0,0]=arm_elements_sensor[1].getValue()
    q3_ref[0,0]=arm_elements_sensor[2].getValue()
    q4_ref[0,0]=arm_elements_sensor[3].getValue()
    q5_ref[0,0]=arm_elements_sensor[4].getValue()

    q1p[0,0]=arm_elements[0].getVelocity()
    q2p[0,0]=arm_elements[1].getVelocity()
    q3p[0,0]=arm_elements[2].getVelocity()
    q4p[0,0]=arm_elements[3].getVelocity()
    q5p[0,0]=arm_elements[4].getVelocity()

    x[0,0]=-np.cos(q1[0,0])*(-a1+l2*np.sin(q2[0,0])+l3*np.sin(q2[0,0]+q3[0,0])+l4*np.sin(q2[0,0]+q3[0,0]+q4[0,0]))
    y[0,0]=-np.sin(q1[0,0])*(-a1+l2*np.sin(q2[0,0])+l3*np.sin(q2[0,0]+q3[0,0])+l4*np.sin(q2[0,0]+q3[0,0]+q4[0,0]))
    z[0,0]=l0+l1+l2*np.cos(q2[0,0])+l3*np.cos(q2[0,0]+q3[0,0])+l4*np.cos(q2[0,0]+q3[0,0]+q4[0,0])

    posicion_deseada = gps.getValues()
    x_objetivo,y_objetivo,z_objetivo=tranformacion_cordenadas(posicion_deseada[2],posicion_deseada[0],posicion_deseada[1],-np.pi/2)
    hxd[0,0]=x_objetivo
    hyd[0,0]=y_objetivo
    hzd[0,0]=z_objetivo

# DEFINICION DE LAS POCIONES DESEADASD EL ROBOT MANIPULADOR

# Trayectoria Deseada
xd=0.3*np.ones((t.shape[0],t.shape[1]))
yd=0.0*np.ones((t.shape[0],t.shape[1]))
zd=0.4*np.ones((t.shape[0],t.shape[1]))

#trayectoria deseada derivada
xd_p=0*np.ones((t.shape[0],t.shape[1]))
yd_p=0*np.ones((t.shape[0],t.shape[1]))
zd_p=0*np.ones((t.shape[0],t.shape[1]))

q1d=0*np.ones((t.shape[0],t.shape[1]))
q2d=-45*np.pi/180*np.ones((t.shape[0],t.shape[1]))
q3d=-45*np.pi/180*np.ones((t.shape[0],t.shape[1]))
q4d=-45*np.pi/180*np.ones((t.shape[0],t.shape[1]))
q5d=0*np.pi/180*np.sin(0.5*t)

#ganancias del controlador
k1=1
k2=0.8
k3=1
k4=1

# Errores de control
herrx=np.zeros((t.shape[0],t.shape[1]))
herry=np.zeros((t.shape[0],t.shape[1]))
herrz=np.zeros((t.shape[0],t.shape[1]))
# Main loop:
for k in range(0,t.shape[1]):
    if robot.step(timestep) != -1:

        if k > 1:
            hxdp= (hxd[0,k] - hxd_1) / t_sample
            hydp= (hyd[0,k] - hyd_1) / t_sample
            hzdp= (hzd[0,k] - hzd_1) / t_sample
        else:
            hxdp= 0
            hydp= 0
            hzdp= 0
        # Seccion para almacenar los valores de los errors del sistema
        herrx[0,k]=hxd[0,k]-x[0,k]
        herry[0,k]=hyd[0,k]-y[0,k]
        herrz[0,k]=hzd[0,k]-z[0,k]
        # Definicion del vector de posiciones
        h=np.array([[x[0,k]],[y[0,k]],[z[0,k]]])

        #Definicion del vector de posiciones deseadas
        hd=np.array([[hxd[0,k]],[hyd[0,k]],[hzd[0,k]]])

        # Definicion de la derivada de la posicion deseada
        hdp=np.array([[hxdp],[hydp],[hzdp]])
       
        #Defincion de estados internos del sistema
        q=np.array([[q1[0,k]],[q2[0,k]],[q3[0,k]],[q4[0,k]],[q5[0,k]]])
        qd=np.array([[q1d[0,k]],[q2d[0,k]],[q3d[0,k]],[q4d[0,k]],[q5d[0,k]]])
        #CONTROLADOR CINEMATICO CON ESPACIO NULO PARA POSICIONES DE ESLABONES
        q1p_ref[0,k],q2p_ref[0,k],q3p_ref[0,k],q4p_ref[0,k],q5p_ref[0,k]=controlador(h,hd,hdp,q,qd,l0,l1,a1,l2,l3,l4,k1,k2,k3,k4)
        
        # INTEGRACION DEL CONTROLADOR PARA OBTENER LAS LOS ANGULOS DE CONTROL
        q1_ref[0,k+1]=euler(q1_ref[0,k],q1p_ref[0,k],t_sample)
        q2_ref[0,k+1]=euler(q2_ref[0,k],q2p_ref[0,k],t_sample)
        q3_ref[0,k+1]=euler(q3_ref[0,k],q3p_ref[0,k],t_sample)
        q4_ref[0,k+1]=euler(q4_ref[0,k],q4p_ref[0,k],t_sample)
        q5_ref[0,k+1]=euler(q5_ref[0,k],q5p_ref[0,k],t_sample)

        # Envio de las velocidades de cada articualcion 
        arm_elements[0].setVelocity(float(np.abs(q1p_ref[0,k])))
        arm_elements[1].setVelocity(float(np.abs(q2p_ref[0,k])))
        arm_elements[2].setVelocity(float(np.abs(q3p_ref[0,k])))
        arm_elements[3].setVelocity(float(np.abs(q4p_ref[0,k])))
        arm_elements[4].setVelocity(float(np.abs(q5p_ref[0,k])))

        # ENvio de posiciones para las articulaciones
        arm_elements[0].setPosition(float(q1_ref[0,k+1]))
        arm_elements[1].setPosition(float(q2_ref[0,k+1]))
        arm_elements[2].setPosition(float(q3_ref[0,k+1]))
        arm_elements[3].setPosition(float(q4_ref[0,k+1]))
        arm_elements[4].setPosition(float(q5_ref[0,k+1]))
       
        # OBTENCION DE LAS POSCIONES Y VELOCIDADES DE CADA ARTICULACION
        q1p[0,k+1]=arm_elements[0].getVelocity()
        q2p[0,k+1]=arm_elements[1].getVelocity()
        q3p[0,k+1]=arm_elements[2].getVelocity()
        q4p[0,k+1]=arm_elements[3].getVelocity()
        q5p[0,k+1]=arm_elements[4].getVelocity()

        q1[0,k+1]=arm_elements_sensor[0].getValue()
        q2[0,k+1]=arm_elements_sensor[1].getValue()
        q3[0,k+1]=arm_elements_sensor[2].getValue()
        q4[0,k+1]=arm_elements_sensor[3].getValue()
        q5[0,k+1]=arm_elements_sensor[4].getValue()

        # CINEMATICA DIRECTA DEL MANIPULADOR
        x[0,k+1]=-np.cos(q1[0,k+1])*(-a1+l2*np.sin(q2[0,k+1])+l3*np.sin(q2[0,k+1]+q3[0,k+1])+l4*np.sin(q2[0,k+1]+q3[0,k+1]+q4[0,k+1]))
        y[0,k+1]=-np.sin(q1[0,k+1])*(-a1+l2*np.sin(q2[0,k+1])+l3*np.sin(q2[0,k+1]+q3[0,k+1])+l4*np.sin(q2[0,k+1]+q3[0,k+1]+q4[0,k+1]))
        z[0,k+1]=l0+l1+l2*np.cos(q2[0,k+1])+l3*np.cos(q2[0,k+1]+q3[0,k+1])+l4*np.cos(q2[0,k+1]+q3[0,k+1]+q4[0,k+1])

        

        # TOMA DATOS DE LA POSICION DESEADA
        posicion_deseada = gps.getValues()
        x_objetivo,y_objetivo,z_objetivo=tranformacion_cordenadas(posicion_deseada[2],posicion_deseada[0],posicion_deseada[1],-np.pi/2)
        hxd[0,k+1]=x_objetivo
        hyd[0,k+1]=y_objetivo
        hzd[0,k+1]=z_objetivo

        print(hxd[0,k+1],hyd[0,k+1],hzd[0,k+1])

        hxd_1=hxd[0,k+1]
        hyd_1=hyd[0,k+1]
        hzd_1=hzd[0,k+1]


#grafica('default','Trayectoria',hxd[0,:],hyd[0,:],hzd[0,:],'$\mathbf{\eta_{d}(t)}$','$x[m]$','$y[m]$','$z[m]$','g')
grafica_c('default','Trayectoria',x[0,:],y[0,:],z[0,:],'$\mathbf{\eta(t)}$','$x[m]$','$y[m]$','$z[m]$','b',hxd[0,:],hyd[0,:],hzd[0,:],'$\mathbf{\eta_{d}(t)}$','g')
print("FINALIZACION DEL PROGRAMA")
# Enter here exit cleanup code.


