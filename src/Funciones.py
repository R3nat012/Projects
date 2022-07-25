import numpy as np
import rbdl
from copy import copy

cos=np.cos; sin=np.sin; pi=np.pi


def Tdh(d, th, a, alpha):
    cth = np.cos(th);    sth = np.sin(th)
    ca = np.cos(alpha);  sa = np.sin(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                    [sth,  ca*cth, -sa*cth, a*sth],
                    [0,        sa,     ca,      d],
                    [0,         0,      0,      1]])
    return T

def fkine(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    """
    # Longitudes (en metros)

    # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
    T1 = Tdh(0,q[0],0.205,0)
    T2 = Tdh(0,q[1] - np.pi/2,-0.1,0)
    T3 = Tdh(0,q[2],0, np.pi/2)
    T4 = Tdh(q[3]+0.117,0,0,0)
    # Efector final con respecto a la base
    T = T1.dot(T2).dot(T3).dot(T4)
    return T

def jacobiano(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x4 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4]
    """
    # Crear una matriz 3x4
    J = np.zeros((3,4))
    # Transformacion homogenea inicial (usando q)
    T = fkine(q)
    # Iteracion para la derivada de cada columna
    for i in xrange(4):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i] = dq[i]+delta
        # Transformacion homogenea luego del incremento (q+delta)
        Ti = fkine(dq)
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[:, i] = 1/delta * \
            np.transpose(np.array([[Ti[0, 3]-T[0, 3]], [Ti[1, 3]-T[1, 3]], [Ti[2, 3]-T[2, 3]]]))
    return J
 
 
def ikine(xdes, q0):
    """
    Calcular la cinematica inversa numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo de newton
    """
    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001
 
    q  = copy(q0)
    for i in range(max_iter):
        # Main loop
        J = jacobiano(q, delta)
        f = fkine(q)
        f = f[0:3, 3]
        e = xdes - f
        q = q + np.dot(np.linalg.pinv(J), e)
        error = np.linalg.norm(e)
        print(error)              
        if (np.linalg.norm(e) < epsilon):
            break
    return q

class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('../urdf/robot.urdf')

    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq




