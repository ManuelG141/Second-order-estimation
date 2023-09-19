from scipy.io import loadmat
import control as ct
import pandas as pd
import matplotlib.pyplot as plt
from EstimateFunc import secondOrderSystem as cs

data = loadmat("Laboratorio II\P5.3\pruebalab.mat")
t = data['t'].ravel().tolist()
y = data['y'].ravel().tolist()
df = pd.DataFrame({'t': t, 'y': y})

wn = 1
zeta = 0.5
sys = ct.tf([0,0,wn**2],[1,wn*zeta,wn**2])
response = ct.step_response(sys)
plt.plot(response.time,response.outputs)
estimateFunc = cs(response.time,response.outputs)
plt.show()

