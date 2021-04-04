import matplotlib.pyplot as plt
import numpy as np
'''
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
plt.xlabel('x')
plt.ylabel('floatTemperatures')
plt.show()
'''

print("Enter your name:")
x = raw_input()
print("Hello, " + x)

x = 0.00000033022002
if x != 0:
    print ("Hi")
else:
    print("No")

x = np.arange(0.,10,0.1)
a = np.cos(x)
b = np.sin(x)
c = np.exp(x/10)
d = np.exp(-x/10)
la = plt.plot(x,a,'b-',label='cosine')
lb = plt.plot(x,b,'g-',label='sine')
lc = plt.plot(x,c,'r-',label='exp(+x)')
ld = plt.plot(x,d,'y-', linewidth = 5,label='exp(-x)')
ll = plt.legend(loc='upper left')
lx = plt.xlabel('xaxis')
ly = plt.ylabel('yaxis')
plt.show()