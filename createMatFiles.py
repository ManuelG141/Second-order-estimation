import scipy.io as sio
import numpy as np
import control as ct

# Parámetros del sistema subamortiguado
wn = 1  # Frecuencia natural
zeta = 1  # Coeficiente de amortiguamiento


# Crear una señal de respuesta al impulso subamortiguada
t = np.arange(0,20.01,0.01)  # Tiempo
sys = ct.tf([0,0,wn**2],[1,2*wn*zeta,wn**2])
response = ct.step_response(sys, t)

# Guardar la señal en un archivo .mat
data = {'t': t, 'y': response.outputs}
sio.savemat('response.mat', data)

print("Archivo .mat guardado con éxito.")