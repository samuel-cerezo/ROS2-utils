import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

# Definir el robot utilizando la convención DH
# Aún falta ajustar los parámetros a y d según el LBR iisy
LBR_iisy = rtb.DHRobot([
    rtb.RevoluteDH(d=0.34, a=0.075, alpha=-np.pi/2),  # A1
    rtb.RevoluteDH(d=0, a=0.36, alpha=0),             # A2
    rtb.RevoluteDH(d=0, a=0.1, alpha=-np.pi/2),       # A3
    rtb.RevoluteDH(d=0.1, a=0, alpha=np.pi/2),        # A4
    rtb.RevoluteDH(d=0, a=0, alpha=-np.pi/2),         # A5
    rtb.RevoluteDH(d=0.1, a=0, alpha=0)               # A6
], name='LBR iisy')

# Ángulos de las articulaciones (en radianes) que ya tienes
q = [np.deg2rad(45), np.deg2rad(-30), np.deg2rad(90), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)]

# Calcular la cinemática directa (FK)
T_flange = LBR_iisy.fkine(q)
print(f"T_base_flange:\n {T_flange}")

# Supongamos que tenemos los valores de rotación y traslación en el sistema de coordenadas "mundo"
# Componentes de traslación (TX, TY, TZ)
TX, TY, TZ = 1.0, 1.0, 1.0  # Estas coordenadas deben ser obtenidas de tu sistema de captura

# Componentes de rotación (RX, RY, RZ) en radianes
RX, RY, RZ = np.deg2rad(0), np.deg2rad(0), np.deg2rad(45)  # Ejemplo, puedes cambiar los valores

# Crear la matriz de rotación a partir de los ángulos de rotación
R = SE3.RPY(RX, RY, RZ)

# Crear la transformación del flange respecto al mundo
T_world_flange = R * SE3(TX, TY, TZ)
print(f'T_world_flange: \n{T_world_flange}')

# Calcular la posición de la base en el sistema del mundo
T_world_base = T_world_flange * T_flange.inv()


print(f"T_world_base:\n{T_world_base}")
