import sys
rootPath = r'E:\workspace\python\BusRouteTSP'
sys.path.append(rootPath)
import numpy as np
from optimize import optimize
from local_SP import local_SP
from tools import local_SP_plot
from ScenarioGenerator.busStopGen import posSet
from ScenarioGenerator.nodeGen import posJunc

I = 5
MIN_STOP_DUR = 5
PER_BOARD_DUR = 3
PLANNED_BUS_NUM = 6
PLANNED_CYCLE_NUM = 10
PAD_CYCLE_NUM = 3
BG_CYCLE_LEN = 100
BG_PHASE_SEQ = np.load(r'E:\workspace\python\BusRouteTSP\tools\result\BG_PHASE_SEQ.npy')
BG_PHASE_LEN = np.load(r'E:\workspace\python\BusRouteTSP\tools\result\BG_PHASE_LEN.npy')
OFFSET = np.load(r'E:\workspace\python\BusRouteTSP\tools\result\offset.npy')
VOLUME = np.load(r'E:\workspace\python\BusRouteTSP\tools\result\volume.npy')
S = np.array([[1700, 3600, 1700, 1800, 1700, 3600, 1700, 1800],
            [1700, 3600, 1700, 1800, 1700, 3600, 1700, 1800],
            [1700, 3600, 1700, 1800, 1700, 3600, 1700, 1800],
            [1700, 3600, 1700, 1800, 1700, 3600, 1700, 1800],
            [1700, 3600, 1700, 1800, 1700, 3600, 1700, 1800]])
YR = 4  # 损失时间
BG_PHASE_SPLIT = np.insert(np.delete(BG_PHASE_LEN, -1, axis=2), 0, 0, axis=2).cumsum(axis=2)
FIRST_PHASE = BG_PHASE_SEQ[:, :, 0]
LAST_PHASE = BG_PHASE_SEQ[:, :, -1]
BAR_PHASE = BG_PHASE_SEQ[:, :, [0, 2]]
COORD_PHASE = [2, 6]
BUS_PHASE = [2]
G_MIN = 8  # 最小绿灯时间
COEFF_XC = 1.2  # 临界饱和系数
INI = -10000
POS_JUNC = np.array(posJunc).cumsum()
POS_STOP = np.concatenate([[0], POS_JUNC]) + np.array(posSet[0])
L_APP = POS_JUNC - POS_STOP[:-1]
L_DEP = POS_STOP[1:] - POS_JUNC
V_AVG = 10
BUS_DEP_HW = 2*60
V_MAX = 15
STOP_DUR = np.array([10, 10, 10, 10, 10, 10])
TIMETABLE = np.array([20 + i*BUS_DEP_HW + (POS_STOP - POS_STOP[0])/V_AVG + np.delete(np.insert(STOP_DUR, 0, 0), -1).cumsum() for i in range(100)])
tlsPadT = [[np.array([[14., 48., 23., 15.], [36., 26., 12., 26.]]), 
            np.array([[14., 48., 23., 15.], [36., 26., 12., 26.]]),
            np.array([[14., 48., 23., 15.], [36., 26., 12., 26.]]),
            np.array([[0., 0., 0., 0.], [0., 0., 0., 0.]])],
            [np.array([[17., 45., 12., 26.], [40., 22., 12., 26.]]), 
            np.array([[17., 45., 12., 26.], [40., 22., 12., 26.]]),
            np.array([[17., 27., 0., 0.], [40., 4., 0., 0.]])],
            [np.array([[17., 37., 12., 34.], [17., 37., 13., 33.]]), 
            np.array([[17., 37., 12., 27.], [17., 37., 13., 26.]])],
            [np.array([[40., 23., 12., 25.], [17., 46., 12., 25.]]), 
            np.array([[40., 8., 0., 0.], [17., 31., 0., 0.]])],
            [np.array([[36., 27., 12., 0.], [14., 49., 12., 0.]])]
        ]
jCurrList = [np.array([0, 0], dtype=int),
            np.array([1, 1], dtype=int),
            np.array([3, 3], dtype=int),
            np.array([1, 1], dtype=int),
            np.array([3, 3], dtype=int)]
tlsPadt = []
for p in tlsPadT:
    tlsPadt.append(np.insert(np.delete(p, -1, axis=-1), 0, 0, axis=-1).cumsum(axis=-1))

def genInput(timeStep):
    # 构造优化模型输入
    inputDict = {'I': 5, 'N': PLANNED_BUS_NUM, 'K': PLANNED_CYCLE_NUM, 'K_ini': PAD_CYCLE_NUM, 'C': BG_CYCLE_LEN, 'J': BG_PHASE_SEQ, 
                 'J_first': FIRST_PHASE, 'J_last': LAST_PHASE, 'J_barrier': BAR_PHASE, 'J_coord': COORD_PHASE, 'J_bus': BUS_PHASE,
                 'T_opt': BG_PHASE_LEN, 't_opt': BG_PHASE_SPLIT, 'POS': POS_JUNC, 'YR': YR, 'G_min': G_MIN, 'Xc': COEFF_XC,
                 'POS_stop': POS_STOP, 'L_app': L_APP, 'L_dep': L_DEP, 'v_avg': V_AVG, 'v_max': V_MAX, 'T_board': STOP_DUR, 'cnt': 1
                }
    inputDict['V_ij'] = VOLUME
    inputDict['S_ij'] = S
    inputDict['p_bus_0'] = [-10000]*PLANNED_BUS_NUM
    inputDict['t_arr_plan'] = TIMETABLE[0:PLANNED_BUS_NUM, :] - timeStep
    inputDict['Q_ij'] = 0
    inputDict['tls_pad_T'] = tlsPadT.copy()
    inputDict['tls_pad_t'] = tlsPadt.copy()
    inputDict['j_curr'] = jCurrList.copy()
    return inputDict

tlsPlan, busArrTimePlan, theta = optimize(**genInput(0)) 
busArrTimePlan = [np.array(plan) for plan in busArrTimePlan]
tlsPlan_ = []
busArrTimePlan_ = [plan[:, 0].reshape([-1, 1]) for plan in busArrTimePlan]
for i in range(I):
    t_arr = [plan[0, 2*i + 1] for plan in busArrTimePlan]
    t_arr_next = [plan[0, 2*(i+1) + 1] for plan in busArrTimePlan]
    tlsPlani_, busArrTimePlani_ = local_SP(i, t_arr, t_arr_next, theta[i], **genInput(0))
    tlsPlan_.append(tlsPlani_)
    busArrTimePlan_ = [np.append(busArrTimePlan_[n], busArrTimePlani_[n][:, 1:], axis=1) for n in range(len(busArrTimePlan_))]

for i in range(len(tlsPlan_)):
    np.save(f'E:\\workspace\\python\\BusRouteTSP\\RouteTSP\\result\\tlsPlan_nt{i+1}.npy', np.array(tlsPlan_[i]))

MAXITER = 100
fail_cnt_origin = 0
fail_cnt_SP = 0
for cnt in range(MAXITER):
    Ts = np.random.normal(10, PER_BOARD_DUR, 3000)
    # Ts = 10 * np.ones(3000)
    fail_cnt_origin += local_SP_plot([np.array(plan) for plan in tlsPlan], busArrTimePlan, [[0, 1], [0, 1], [0, 1], [0, 0], [0, 0]],
                   PER_BOARD_DUR, V_MAX, TIMETABLE, 'original', cnt, Ts)
    fail_cnt_SP += local_SP_plot([np.array(plan) for plan in tlsPlan_], busArrTimePlan_, [[0, 1], [0, 1], [0, 1], [0, 0], [0, 0]], 
                  PER_BOARD_DUR, V_MAX, TIMETABLE, 'local_SP', cnt, Ts)
print(fail_cnt_origin/(MAXITER * 10 * 6))
print(fail_cnt_SP/(MAXITER * 10 * 6))