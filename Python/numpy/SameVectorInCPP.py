'''
import numpy as np
proc_ranges = np.array([np.inf, np.log(-1.), 4, 5, 6])
proc_ranges = proc_ranges[~np.isnan(proc_ranges) & ~np.isinf(proc_ranges)]
print(proc_ranges)
'''

'''
import numpy as np

proc_ranges = np.array([ 41, 243, 4, 6, 99])
proc_ranges[proc_ranges < 66] = 0
print(proc_ranges)

proc_ranges[1 : 3] = 0
print(proc_ranges)
'''


import numpy as np

proc_ranges = np.array([ 41, 0, 4, 53, 0, 0, 0, 6, 0, 45, 43, 17, 0, 0, 0, 0, 99])
zero_mat = np.where(proc_ranges == 0)[0]
print(zero_mat)
bound_mat = np.split(zero_mat, np.where(np.diff(zero_mat) != 1)[0]+1)
print(np.where(np.diff(zero_mat) != 1)[0]+1)

'''
# Start and End matrix declarations
strt = []  # start is a collection of all indices where the zeroes start
end = []  # end is a collection of all indices where the zeroes end

i = 0
for i in range(len(bound_mat)):
    strt.append(bound_mat[i][0])
    end.append(bound_mat[i][-1])

strt.append(len(proc_ranges))  # Putting length of ranges to the end
end.insert(0, -1)  # Adding -1 to the start for proper gap length calculation

# Array for all gap lengths
comp = [0]

# Getting the indices of the max gap
j = 0
s_idx = 0
e_idx = 0
for j in range(len(strt)):
    sz = strt[j] - end[j] - 1
    comp.append(sz)

    if comp[j + 1] > comp[j]:
        s_idx = end[j]
        e_idx = strt[j]

mg_idx = [s_idx, e_idx]

print(mg_idx)
'''
