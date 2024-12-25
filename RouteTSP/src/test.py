from optimize_new import optimize
from local_SP import local_SP
import pickle
import os
import numpy as np
from tools import getBusIndBeforeJunc, nextNode
rootPath = r'E:\workspace\python\BusRouteTSP'

# timeStepList = np.arange(150, 170, 5)
timeStepList = [3510]
for timeStep in timeStepList:
    with open(f"{rootPath}\\RouteTSP\\result\\inputData\\time={timeStep}.pkl", "rb") as f:
        inputDict = pickle.load(f)
        BUS_POS = inputDict['p_bus_0'][:inputDict['N']]
        I = inputDict['I']
    with open(f"{rootPath}\\RouteTSP\\result\\outputData\\time={timeStep}.pkl", "rb") as f:
        outputDict = pickle.load(f)
    tlsPlan, busArrTimePlan, theta = optimize(**inputDict)
    # busArrTimePlan = [[plan[0][1:] + timeStep, plan[1][1:]] for plan in busArrTimePlan]
    pass

    busArrTimePlan = [np.array([plan[0] + timeStep, plan[1]]) for plan in busArrTimePlan]
    tlsPlan_ = []
    busArrTimePlan_ = [np.array([[plan[0, 0]-timeStep], [plan[1, 0]]]) for plan in busArrTimePlan]
    for i in range(I):
        busInd = getBusIndBeforeJunc(BUS_POS, i)
        t_arr = np.array([busArrTimePlan[ind][0, 2*i + 1 - (2*I + 2 - len(busArrTimePlan[ind][0]))] for ind in busInd]) - timeStep
        r = np.array([busArrTimePlan[ind][0, 2*i + 2 - (2*I + 2 - len(busArrTimePlan[ind][0]))] for ind in busInd]) - timeStep
        t_arr_next = np.array([busArrTimePlan[ind][0, 2*(i+1) + 1 - (2*I + 2 - len(busArrTimePlan[ind][0]))] for ind in busInd]) - timeStep
        tlsPlani_, busArrTimePlani_ = local_SP(i, t_arr, r, t_arr_next, theta[i], busInd, **inputDict)
        tlsPlan_.append(tlsPlani_)
        for n in busInd:
            if len(busArrTimePlan_[n][0]) == 1:
                if nextNode(busArrTimePlan_[n][1, 0]) == 'STOP':
                    plan = (busArrTimePlan[n][:, 1] - [timeStep, 0]).reshape(-1, 1)
                    busArrTimePlan_[n] = np.append(busArrTimePlan_[n], plan, axis=1)
            busArrTimePlan_[n] = np.append(busArrTimePlan_[n], busArrTimePlani_[n - busInd[0]], axis=1)
    busArrTimePlan_ = [np.array([plan[0] + timeStep, plan[1]]) for plan in busArrTimePlan_]
    pass

print('Output in main.py:')
for k, v in outputDict.items():
    print(f"{k}: {v}\n")

print('Output in test.py:')
print(f'tlsPlan: {tlsPlan}')
print(f'busArrPlan: {busArrTimePlan}')