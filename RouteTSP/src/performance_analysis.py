import sys
rootPath = r'E:\workspace\python\BusRouteTSP'
sys.path.append(rootPath)
import numpy as np
from ScenarioGenerator.nodeGen import posJunc
from ScenarioGenerator.busStopGen import posSet
from tools import performanceAnalysis

if __name__ == '__main__':
    # testDir = 'origin_0.7'
    # testDir = 'origin_0.7_5-25-10_nolb'
    # testDir = 'blank'
    # testDir = 'TSP_only_ref12'
    # testDir = 'TSP_only_ref13'
    # testDir = 'SA_only'
    # testDir = 'origin_exact'
    # testDir = 'blank_exact'
    # testDir = 'SP_0.7_5-25-10_nolb'
    # testDirList = ['blank_exact', 'TSP_only_ref12', 'SA_only', 'origin_exact']
    # testDirList = ['blank', 'origin_0.7_5-25-10_nolb', 'SP_0.7_5-25-10_nolb']
    # testDirList = ['blank_test', 'origin_exact_avg10_max12_Ts30']
    # testDirList = ['blank_noQ', 'origin_avg10_max12_Ts30_dev10_noQ', 'SP_avg10_max12_Ts30_dev10_extra8_H240']
    # testDirList = ['blank_avg10_max12_noQ_lowV', 'origin_avg10_max12_Ts30_dev10_noQ_lowV', 'SP_avg10_max12_Ts30_dev10_noQ_lowV']
    # testDirList = ['blank_avg9_max12_noQ_lowV', 'origin_avg9_max12_Ts30_dev10_noQ_lowV', 'SP_avg9_max12_Ts30_dev10_noQ_lowV_50']
    testDirList = ['blank_test', 'origin_test', 'SP_test']

    for testDir in testDirList:
        SIMTIME = 3600
        busArrTime = np.load(f'E:\\workspace\\python\\BusRouteTSP\\RouteTSP\\result\\case study\\{testDir}\\busArrTime.npy')
        # BUS_DEP_HW = 2*60
        BUS_DEP_HW = 2*60
        POS_JUNC = np.array(posJunc).cumsum()
        POS_STOP = np.concatenate([[0], POS_JUNC]) + np.array(posSet[0])
        V_AVG = 10
        STOP_DUR = 30*np.array([1, 1, 1, 1, 1, 1])
        TIMETABLE = np.array([i*BUS_DEP_HW + (POS_STOP)/V_AVG + np.delete(np.insert(STOP_DUR, 0, 0), -1).cumsum() for i in range(100)])
        BG_PHASE_LEN = np.load(r'E:\workspace\python\BusRouteTSP\tools\result\BG_PHASE_LEN.npy')
        tlsPlanList = [np.load(f'E:\\workspace\\python\\BusRouteTSP\\RouteTSP\\result\\case study\\{testDir}\\tlsPlan_nt{i}.npy') for i in range(1, 6)]
        busArrTimeDev, busArrTimeDevStd, lateRate, busHeadwayVar, delay = performanceAnalysis(
            testDir, busArrTime, TIMETABLE, tlsPlanList, BG_PHASE_LEN, SIMTIME)
        np.save(f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\busArrTimeDev.npy', np.array(busArrTimeDev))
        np.save(f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\busArrTimeDevStd.npy', np.array(busArrTimeDevStd))
        np.save(f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\lateRate.npy', np.array(lateRate))
        np.save(f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\busHeadwayVar.npy', np.array(busHeadwayVar))
        np.save(f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\delay.npy', np.array(delay))