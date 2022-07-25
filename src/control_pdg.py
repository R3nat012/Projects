#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from markers import *
from Funciones import *
from roslib import packages

import rbdl


rospy.init_node("control_pdg")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker_actual  = BallMarker(color['RED'])
bmarker_deseado = BallMarker(color['GREEN'])
# Archivos donde se almacenara los datos
fqact = open("/tmp/qactual.dat", "w")
fqdes = open("/tmp/qdeseado.dat", "w")
fxact = open("/tmp/xactual.dat", "w")
fxdes = open("/tmp/xdeseado.dat", "w")

# Nombres de las articulaciones
jnames = ['revolucion12', 'revolucion4', 'revolucion5','corredera7', 'revolucion13', 'revolucion14', 'revolucion15', 'revolucion16']
# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Valores del mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames

# =============================================================
# Configuracion articular inicial (en radianes)
q = np.array([0.0, 0.0, 0.0, 0.0])
# Velocidad inicial
dq = np.array([0., 0., 0., 0. ])
# Configuracion articular deseada
qdes = np.array([1.0, -1.0, 1.0, 0.3])
# =============================================================

# Posicion resultante de la configuracion articular deseada
xdes = fkine(qdes)[0:3,3]
# Copiar la configuracion articular en el mensaje a ser publicado
qrueda = np.array([ 0. , 0. , 0. , 0. ])
qfinal = np.concatenate((q,qrueda))
jstate.position = qfinal
pub.publish(jstate)

# Modelo RBDL
modelo = rbdl.loadModel('../urdf/robot.urdf')
ndof = 4

# Frecuencia del envio (en Hz)
freq = 20
dt = 1.0/freq
rate = rospy.Rate(freq)

# Simulador dinamico del robot
robot = Robot(q, dq, ndof, dt)

# Se definen las ganancias del controlador
valores = 0.1*np.array([1.0, 1.0, 1.0, 1.0])
Kp = np.diag(valores)
Kd = 2*np.sqrt(Kp)

# Para la gravedad
g     = np.zeros(ndof)          

# Bucle de ejecucion continua
t = 0.0
while not rospy.is_shutdown():

    # Leer valores del simulador
    q  = robot.read_joint_positions()
    dq = robot.read_joint_velocities()
    # Posicion actual del efector final
    x = fkine(q)[0:3,3]
    # Tiempo actual (necesario como indicador para ROS)
    jstate.header.stamp = rospy.Time.now()

    # Almacenamiento de datos
    fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
    fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
    fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+' '+ str(q[3])+'\n ')
    fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' '+ str(qdes[2])+' '+ str(qdes[3])+'\n ')

    # ----------------------------
    # Control dinamico (COMPLETAR)
    # ----------------------------
    e = qdes - q
 
    ceros = np.array([0.0, 0.0, 0.0, 0.0 ])
    # Compensacion de gravedad
    rbdl.InverseDynamics(modelo, q, ceros, ceros, g)
 
    # Ley de control
    u = np.dot(Kp,e) - np.dot(Kd,dq) + g 

    # Simulacion del robot
    robot.send_command(u)

    # Publicacion del mensaje
    qrueda = np.array([ 0, 0, 0, 0])
    qfinal = np.concatenate((q,qrueda))
    jstate.position = qfinal
    pub.publish(jstate)
    bmarker_deseado.xyz(xdes)
    bmarker_actual.xyz(x)
    t = t+dt
    # Esperar hasta la siguiente  iteracion
    rate.sleep()
    
    print(np.linalg.norm(e))
    if np.linalg.norm(e)<0.3:
      print('llegue')
      break
        

fqact.close()
fqdes.close()
fxact.close()
fxdes.close()


