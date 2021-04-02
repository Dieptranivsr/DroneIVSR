import matplotlib.pyplot as plt
import numpy as np

TEMP1 = [22.4, 14.4, 12.3]
temperatures = []
floatTemperatures = np.array(temperatures, dtype = np.float64)
print(floatTemperatures)
floatTemperatures = np.append(TEMP1, 34.435425253425)
print(floatTemperatures)

t = [1,2,3,4]
fig = plt.figure()
fig.suptitle('How are you ?')
plt.plot(t, floatTemperatures)
plt.xlabel('t')
plt.ylabel('floatTemperatures')
plt.show()