from optimize_new import optimize
# from optimize_SA_only import optimize
from local_SP import local_SP
import pickle
import os
import numpy as np
from tools import getBusIndBeforeJunc, nextNode, getNextStopPos
rootPath = r'E:\workspace\python\BusRouteTSP'

# timeStepList = np.arange(10, 3600, 100)
timeStepList = [410]
for timeStep in timeStepList:
    with open(f"{rootPath}\\RouteTSP\\result\\inputData\\time={timeStep}.pkl", "rb") as f:
        inputDict = pickle.load(f)
        BUS_POS = inputDict['p_bus_0'][:inputDict['N']]
        I = inputDict['I']
        N = inputDict['N']
    with open(f"{rootPath}\\RouteTSP\\result\\outputData\\time={timeStep}.pkl", "rb") as f:
        outputDict = pickle.load(f)
    tlsPlan, busArrTimePlan, theta, t_coord = optimize(**inputDict)
    # busArrTimePlan = [[plan[0][1:] + timeStep, plan[1][1:]] for plan in busArrTimePlan]
    pass

    busArrTimePlan = [np.array([plan[0] + timeStep, plan[1]]) for plan in busArrTimePlan]
    tlsPlan_ = []
    busArrTimePlan_ = [np.array([[plan[0, 0]-timeStep], [plan[1, 0]]]) for plan in busArrTimePlan]
    INI = -10000
    t_arr_next = INI * np.ones(N)
    busArrTimePlan_temp = [[] for _ in range(N)]
    for i in range(I-1, -1, -1):
        busInd = getBusIndBeforeJunc(BUS_POS, i)
        t_arr = np.array([busArrTimePlan[ind][0, 2*i + 1 - (2*I + 2 - len(busArrTimePlan[ind][0]))] for ind in busInd]) - timeStep
        # r = np.array([busArrTimePlan[ind][0, 2*i + 2 - (2*I + 2 - len(busArrTimePlan[ind][0]))] for ind in busInd]) - timeStep
        # t_arr_next = np.array([busArrTimePlan[ind][0, 2*(i+1) + 1 - (2*I + 2 - len(busArrTimePlan[ind][0]))] for ind in busInd]) - timeStep
        for n in busInd:
            if t_arr_next[n] == INI:
                t_arr_next[n] = busArrTimePlan[n][0, 2*(i+1) + 1 - (2*I + 2 - len(busArrTimePlan[n][0]))] - timeStep
        tlsPlani_, busArrTimePlani_, t_arr_ = local_SP(i, t_arr, t_arr_next[busInd], theta[i], t_coord[i], busInd, **inputDict)
        tlsPlan_.insert(0, tlsPlani_)
        for n in busInd:
            # if len(busArrTimePlan_[n][0]) == 1:
            #     if nextNode(busArrTimePlan_[n][1, 0]) == 'STOP':
            #         plan = (busArrTimePlan[n][:, 1] - [timeStep, 0]).reshape(-1, 1)
            #         busArrTimePlan_[n] = np.append(busArrTimePlan_[n], plan, axis=1)
            if len(busArrTimePlan_[n][0]) == 1 and nextNode(BUS_POS[n]) == 'STOP' and getNextStopPos(BUS_POS[n]) == inputDict['POS_stop'][i]:
                    busArrTimePlan_[n] = np.append(busArrTimePlan_[n], np.array([t_arr_[n - busInd[0]], inputDict['POS_stop'][i]]).reshape(-1, 1), axis=1)
            # busArrTimePlan_[n] = np.append(busArrTimePlan_[n], busArrTimePlani_[n - busInd[0]], axis=1)
            busArrTimePlan_temp[n].append(np.flip(busArrTimePlani_[n - busInd[0]], axis=1))
            t_arr_next[n] = t_arr_[n - busInd[0]]
    for n in range(len(busArrTimePlan_)):
        busArrTimePlan_[n] = np.append(busArrTimePlan_[n], np.flip(np.concatenate(np.array(busArrTimePlan_temp[n]), axis=1), axis=1), axis=1)
    busArrTimePlan_ = [np.array([plan[0] + timeStep, plan[1]]) for plan in busArrTimePlan_]

    pass

print('Output in main.py:')
for k, v in outputDict.items():
    print(f"{k}: {v}\n")

print('Output in test.py:')
print(f'tlsPlan: {tlsPlan}')
print(f'busArrPlan: {busArrTimePlan}')