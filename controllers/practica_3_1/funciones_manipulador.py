import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

def grafica(sty,titulo,x,y,z,etiqueta,ejex,ejey,ejez,color):
    mpl.style.use(sty)
    ax=plt.axes(projection="3d")
    ax.set_title(titulo.format(sty), color='0')
    ax.set_xlabel(ejex)
    ax.set_ylabel(ejey)
    ax.set_zlabel(ejez)
 
    ax.plot3D(x, y, z, color, label=etiqueta)
    ax.grid(linestyle='--', linewidth='0.3', color='black')
    legend = ax.legend(loc='upper right', shadow=False, fontsize='small')
    plt.show()
    


def grafica_c(sty,titulo,x,y,z,etiqueta,ejex,ejey,ejez,color,x_1,y_1,z_1,etiqueta_1,color_1):
    mpl.style.use(sty)
    ax=plt.axes(projection="3d")
    ax.set_title(titulo.format(sty), color='0')

    ax.set_xlabel(ejex)
    ax.set_ylabel(ejey)
    ax.set_zlabel(ejez)
 
    ax.plot3D(x,y,z, color, label=etiqueta)
    ax.plot3D(x_1,y_1,z_1,color_1,label=etiqueta_1)

    ax.grid(linestyle='--', linewidth='0.2', color='black')
    legend = ax.legend(loc='upper right', shadow=False, fontsize='small')
    plt.show()

def home(arm_elements,tiempo_home,robot):

    # Envio de las velocidades de cada articualcion
    t_sample=0.1
    t_final=tiempo_home+t_sample
    t=np.arange(0,t_final,t_sample)
    t=t.reshape(1,t.shape[0])
    timestep = int(95)# 100 milisegundos equivale a 0.1 segundos
    for k in range(0,t.shape[1]):
        if robot.step(timestep) != -1:
            # Envio de las velocidades de cada articualcion 
            arm_elements[0].setVelocity(0.5)
            arm_elements[1].setVelocity(0.5)
            arm_elements[2].setVelocity(0.5)
            arm_elements[3].setVelocity(0.5)
            arm_elements[4].setVelocity(0.5)

            # ENvio de posiciones para las articulaciones
            arm_elements[0].setPosition(0)
            arm_elements[1].setPosition(float(-np.pi/4))
            arm_elements[2].setPosition(float(-np.pi/4))
            arm_elements[3].setPosition(float(-np.pi/8))
            arm_elements[4].setPosition(float(0))
            print("SYSTEM HOME")
       
def controlador(h,hd,hdp,q,qd,l0,l1,a1,l2,l3,l4,k1,k2,k3,k4):
    K1=k1*np.eye(3,3)
    K2=k2*np.eye(3,3)
    K3=k3*np.eye(5,5)
    K4=k4*np.eye(5,5)
    W=np.eye(5,5)
    
    W_1=np.linalg.inv(W)
    
    herr=hd-h
    
    nulo=qd-q
    
    I=np.eye(5,5)

    J11=np.sin(q[0,0])*(l2*np.sin(q[1,0])+l3*np.sin(q[1,0]+q[2,0])+l4*np.sin(q[1,0]+q[2,0]+q[3,0])-a1)

    J12=-np.cos(q[0,0])*(l2*np.cos(q[1,0])+l3*np.cos(q[1,0]+q[2,0])+l4*np.cos(q[1,0]+q[2,0]+q[3,0]))

    J13=-np.cos(q[0,0])*(l3*np.cos(q[1,0]+q[2,0])+l4*np.cos(q[1,0]+q[2,0]+q[3,0]))

    J14=-np.cos(q[0,0])*(l4*np.cos(q[1,0]+q[2,0]+q[3,0]))

    J15=0

    J21=-np.cos(q[0,0])*(l2*np.sin(q[1,0])+l3*np.sin(q[1,0]+q[2,0])+l4*np.sin(q[1,0]+q[2,0]+q[3,0])-a1)

    J22=-np.sin(q[0,0])*(l2*np.cos(q[1,0])+l3*np.cos(q[1,0]+q[2,0])+l4*np.cos(q[1,0]+q[2,0]+q[3,0]))

    J23=-np.sin(q[0,0])*(l3*np.cos(q[1,0]+q[2,0])+l4*np.cos(q[1,0]+q[2,0]+q[3,0]))

    J24=-np.sin(q[0,0])*(l4*np.cos(q[1,0]+q[2,0]+q[3,0]))

    J25=0

    J31=0

    J32=-(l2*np.sin(q[1,0])+l3*np.sin(q[1,0]+q[2,0])+l4*np.sin(q[1,0]+q[2,0]+q[3,0]))
 
    J33=-(l3*np.sin(q[1,0]+q[2,0])+l4*np.sin(q[1,0]+q[2,0]+q[3,0]))

    J34=-(l4*np.sin(q[1,0]+q[2,0]+q[3,0]))

    J35=0

    J=np.matrix([[J11,J12,J13,J14,J15],[J21,J22,J23,J24,J25],[J31,J32,J33,J34,J35]])

    J_m=W_1@J.transpose()@np.linalg.inv(J@W_1@J.transpose())

    control=J_m@(hdp+K2@np.tanh(np.linalg.inv(K2)@K1@herr))+(I-J_m@J)@K3@np.tanh(np.linalg.inv(K3)@K4@nulo)
    
    return control[0,0], control[1,0], control[2,0], control[3,0], control[4,0]


def euler(z,zp,t_sample):
    z=z+zp*t_sample
    return z

def tranformacion_cordenadas(x,y,z,phi):
    T=np.matrix([[np.cos(phi),-np.sin(phi),0],[np.sin(phi),np.cos(phi),0],[0,0,1]])
    relativo=np.array([[x],[y],[z]])
    real=T@relativo
    return real[0,0],real[1,0],real[2,0]