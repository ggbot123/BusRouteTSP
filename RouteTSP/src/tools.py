import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np
import sys
rootPath = r'E:\workspace\python\BusRouteTSP'
sys.path.append(rootPath)
from ScenarioGenerator.nodeGen import posJunc
from ScenarioGenerator.busStopGen import posSet
from tqdm import tqdm
from bisect import bisect_left

# phaseDict = {1: 12, 2: 11, 3: 9, 4: 8, 5: 6, 6: 5, 7: 3, 8: 2}
phaseDict = {1: [7], 2: [12, 13], 3: [10], 4: [2], 5: [14], 6: [5, 6], 7: [3], 8: [9]}
RGY2J = {2: 4, 3: 7, 5: 6, 6: 6, 7: 1, 9: 8, 10: 3, 12: 2, 13: 2, 14: 5}
POS_JUNC = np.array(posJunc).cumsum()
POS_STOP = np.concatenate([[0], POS_JUNC]) + np.array(posSet[0])

def round_and_adjust(T):
    t1 = np.round(T[0][0] + T[0][1])
    C = np.round(np.sum(T[0]))
    t2 = C - t1
    T = np.round(T)
    for l in range(T.shape[0]):
        diff_t1 = T[l][0] + T[l][1] - t1
        T[l][0] -= diff_t1
        diff_t2 = T[l][2] + T[l][3] - t2
        T[l][2] -= diff_t2
    return T
        
def getBusIndBeforeJunc(p_bus, id):
    return np.where((p_bus <= POS_JUNC[id]))[0]

def nextNode(p_bus):
    if np.where(p_bus < POS_STOP)[0][0] == np.where(p_bus <= POS_JUNC)[0][0]:
        return 'STOP'
    else:
        return 'JUNC'

def getIniTlsCurr(T, t):
    # 初始化结果数组
    result = np.zeros_like(T)
    # 逐行计算使得前几个元素的和等于目标值
    for i in range(T.shape[0]):
        current_sum = 0
        for j in range(T.shape[1]):
            if current_sum + T[i, j] <= t:
                result[i, j] = T[i, j]
                current_sum += T[i, j]
            else:
                result[i, j] = t - current_sum
                current_sum = t
                break  # 达到目标和，停止本行计算
    return result

# T：K*(2*4), K：周期，I：交叉口编号, 2：双环, 4：四相位
def getSumoTLSProgram(J, T, YR):
    K = T.shape[0]
    YR_ = YR * np.ones(T.shape)
    Tplan = np.zeros(T.shape[:-1] + (T.shape[-1]*2, ))
    Tplan[..., ::2] = T - YR_
    Tplan[..., 1::2] = YR_
    for i in range(T.shape[1]):
        for j in range(T.shape[2]):
            if T[0, i, j] < YR:
                Tplan[0, i, 2*j] = 0
                Tplan[0, i, 2*j + 1] = T[0, i, j]
                if T[0, i, j] > 0:
                    YRfirst = 1
    tPlan = np.insert(np.delete(Tplan, -1, axis=-1), 0, 0, axis=-1).cumsum(axis=-1)
    # print(tPlan)
    RGYplan = [[] for _ in range(K)]

   # 14 links (3/4 lanes)
    phaseDict = {1: [7], 2: [12, 13], 3: [10], 4: [2], 5: [14], 6: [5, 6], 7: [3], 8: [9]}

    for k in range(K):
        tk = tPlan[k]
        tSplit = np.sort(np.unique(tk.flatten(), axis=-1))
        Tsplit = np.diff(tSplit)
        for j, t in enumerate(tSplit):
            phaseDur = Tsplit[j] if j < len(Tsplit) else YR
            RGY = 'GrrGrrrGrrGrrr'
            for b in [0, 1]:
                tInd = np.searchsorted(tk[b], t, side='right')
                currentPhaseList = phaseDict[J[b][int((tInd - 1)/2)]]
                for currentPhase in currentPhaseList:
                    if tInd % 2 == 0:
                        RGY = RGY[:(currentPhase - 1)] + 'y' + RGY[currentPhase:]
                    else:
                        RGY = RGY[:(currentPhase - 1)] + 'G' + RGY[currentPhase:]
            RGYplan[k].append((RGY, phaseDur))
    return RGYplan

def getIndfromId(type, id):
    myDict = {
        'tls': {
            'nt1': 0,
            'nt2': 1,
            'nt3': 2,
            'nt4': 3,
            'nt5': 4,
        }
    }
    if type == 'tls':
        return myDict[type][id]
    if type == 'det':
        jId, pId = id.split('_')[1], id.split('_')[2]
        return myDict['tls'][jId], int(pId) - 1
    if type == 'stop':
        sId = id.split('_')[1]
        return int(sId)
    else:
        return int(id)
    
def getBusOrder(busId, arr):
    # order = bisect_left(arr, busId)
    order = arr.index(busId)
    return order
    
def savePlan(timeStep, tlsPlan, busArrTimePlan, cnt):
    # 定义要保存的文件路径
    file_path = f"{rootPath}\\RouteTSP\\result\\optimizer_output\\{cnt}.txt"
    # 保存一系列数据到txt文件
    with open(file_path, 'w') as f:
        f.write(f"Record\n")
        # 保存timeStep
        f.write(f"timeStep: {timeStep}\n\n")

        # 保存tlsPlan
        # f.write("tlsPlan:\n")
        # np.savetxt(f, tlsPlan.reshape(-1, tlsPlan.shape[-1]), fmt='%.4f', delimiter=", ")
        # f.write("\n\n")

        # 保存busArrTimePlan
        f.write("busArrTimePlan:\n")
        for plan in busArrTimePlan:
            np.savetxt(f, plan, fmt='%.1f', delimiter=', ')
            f.write("\n---\n")  # 用---分隔每一组数据
    print(f"Data series saved to {file_path}")

def readPlan():
    # 定义要读取的文件路径
    file_path = f"{rootPath}\\optimizer_output.txt"

    # 读取txt文件中的一系列数据
    data_series = []
    with open(file_path, 'r') as f:
        while True:
            line = f.readline().strip()
            if not line:  # 文件结束
                break
            
            if line.startswith("Record"):
                # 读取timeStep
                timeStep_line = f.readline().strip()
                timeStep = int(timeStep_line.split(": ")[1])
                
                # 跳过空行和tlsPlan标识行
                f.readline()  # 空行
                f.readline()  # "tlsPlan:"
                
                # 读取tlsPlan
                tlsPlan_data = []
                for _ in range(5 * 2):  # 5*2=10行数据
                    line = f.readline().strip()
                    row_data = [float(x) for x in line.split(", ")]
                    tlsPlan_data.append(row_data)
                tlsPlan = np.array(tlsPlan_data).reshape(5, 2, 4)
                
                # 跳过空行和busArrTimePlan标识行
                f.readline()  # 空行
                f.readline()  # "busArrTimePlan:"
                
                # 读取busArrTimePlan
                busArrTimePlan_data = []
                for _ in range(6 * 2):  # 6*2=12行数据
                    line = f.readline().strip()
                    row_data = [float(x) for x in line.split(", ")]
                    busArrTimePlan_data.append(row_data)
                busArrTimePlan = np.array(busArrTimePlan_data).reshape(6, 2, 10)
                
                # 保存当前组数据
                data_series.append({
                    "timeStep": timeStep,
                    "tlsPlan": tlsPlan,
                    "busArrTimePlan": busArrTimePlan
                })
                
                # 跳过分隔符 ---
                f.readline()
    return data_series

def myplot(POS, POS_stop, phase, timetable):
    PLAN_START = 10
    PLAN_STEP = 50
    # 读取文件
    bus_speed_file = f'{rootPath}\\RouteTSP\\result\\bus_speed_profile.csv'
    tls_state_file = f'{rootPath}\\RouteTSP\\result\\tlsState_profile.csv'

    # 读取公交速度数据和信号灯状态数据
    bus_speed_df = pd.read_csv(bus_speed_file)
    tls_state_df = pd.read_csv(tls_state_file)

    # 提取时间列
    time_range = bus_speed_df.iloc[:, 0].values

    # 1. 预处理信号灯状态数据
    tls_state_df = tls_state_df.iloc[:, 1:]  # 跳过第一列（时间列）
    num_intersections = len(tls_state_df.columns)
    num_time_points = len(tls_state_df)

    # 将信号灯状态分解为字符数组
    phase = phaseDict[phase][0] - 1
    tls_states = np.array([[state[phase] for state in tls_state_df[col].values] 
                           for col in tls_state_df.columns])

    # 生成用于绘图的线段
    fig, ax1 = plt.subplots(figsize=(10, 8))

    # 定义颜色映射
    color_map = {'r': 'red', 'G': 'green', 'y': 'yellow'}
    lines = []
    colors = []

    # 为每个交叉口生成线段集合
    for i in range(num_intersections):
        for j in range(1, num_time_points):
            lines.append([(time_range[j-1], POS[i]), (time_range[j], POS[i])])
            colors.append(color_map[tls_states[i][j]])

    # 使用 LineCollection 绘制信号灯状态
    lc = LineCollection(lines, colors=colors, linewidths=2)
    ax1.add_collection(lc)

    ax1.set_ylabel('Position (m)')
    ax1.set_xlabel('Time (s)')
    ax1.set_title('P-t curve')

    # 2. 绘制公交位置-时间曲线
    for col in bus_speed_df.columns[1:]:
        bus_speed = bus_speed_df[col].fillna(0).values
        bus_position = np.cumsum(bus_speed)
        ax1.plot(time_range, bus_position, label=f'{col}', linewidth=2)

    # 3. 绘制到站时刻表散点
    for m in range(len(timetable)):
        # 获取公交车 m 在所有站点的到达时间
        times = timetable[m]
        if times[0] > time_range[-1]:
            break
        # 过滤仅在 time_range 内的时间点和对应的站点位置
        valid_times = times[times <= time_range[-1]]
        valid_positions = POS_stop[times <= time_range[-1]]

        # 绘制筛选后的到达时间和站点位置
        plt.scatter(valid_times, valid_positions, label=f'Bus {m+1}', s=20)

    # 手动绘制网格线
    for x_value in np.arange(PLAN_START, time_range[-1], PLAN_STEP):
        ax1.axvline(x=x_value, color='gray', linestyle='--', linewidth=0.7)

    plt.tight_layout()
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.savefig(f'{rootPath}\\RouteTSP\\result\\P-t curve.png')
    plt.show()

def local_SP_plot(timeStep, tlsPlan, busArrPlan, busPhasePos, PER_BOARD_DUR, V_MAX, timetable, T_board_past, DIRNAME, cnt, sample=None):
    YR = 3
    N = len(busArrPlan)
    I = len(tlsPlan)
    INI = -10000
    # I = 1
    num_sample = 10
    sample_cnt = 0
    fail_cnt = 0

    # 绘制信号配时
    fig, ax1 = plt.subplots(figsize=(10, 8))
    lines = []
    colors = []
    TPlanAll = []
    tPlanAll = []
    for i in range(I):
        busLoop = busPhasePos[i][0]
        busPhase = busPhasePos[i][1]
        TPlan = tlsPlan[i][:, busLoop, :].flatten()
        tPlan = np.insert(TPlan, 0, timeStep).cumsum()
        for j, phaseLen in enumerate(TPlan):
            if j % 4 != busPhase:
                lines.append([(tPlan[j], POS_JUNC[i]), (tPlan[j] + phaseLen, POS_JUNC[i])])
                colors.append('red')
            else:
                lines.append([(tPlan[j], POS_JUNC[i]), (tPlan[j] + phaseLen - YR, POS_JUNC[i])])
                colors.append('green')
                lines.append([(tPlan[j] + phaseLen - YR, POS_JUNC[i]), (tPlan[j] + phaseLen, POS_JUNC[i])])
                colors.append('yellow')
        # 使用 LineCollection 绘制信号灯状态
        lc = LineCollection(lines, colors=colors, linewidths=2)
        ax1.add_collection(lc)
        TPlanAll.append(TPlan)
        tPlanAll.append(tPlan)

    for n in range(N):
        plt.scatter(timetable[n, :], POS_STOP, color='blue', marker='*')

    for s in range(num_sample):
        arrTimeList = INI * np.ones([N, 6])
        Ts_past = T_board_past.copy()
        for n in range(N):
            isFail = 0
            pos = busArrPlan[n][1, 0]
            t = busArrPlan[n][0, 0]
            if Ts_past[n] > 0:
                busArrPlan[n] = np.insert(busArrPlan[n], 0, np.array([t, pos]), axis=1)
            traj_t, traj_x = [t], [pos] 
            for t_arr in busArrPlan[n][:, 1:].T:
                t_arr_plan = t_arr[0]
                nextPOS = t_arr[1]
                # 驶向路口/站点
                if (nextPOS - pos)/(t_arr_plan - t) <= V_MAX and (nextPOS - pos)/(t_arr_plan - t) >= 0:
                    t_arr_act = t_arr_plan
                else:
                    t_arr_act = t + (nextPOS - pos)/V_MAX
                traj_t.append(t_arr_act)
                traj_x.append(nextPOS)

                if nextPOS in POS_STOP:
                    # 停站
                    if sample is None:
                        Ts = np.random.normal(0, 1)*PER_BOARD_DUR
                    else:
                        Ts = sample[sample_cnt]
                        sample_cnt += 1
                    Ts = Ts - Ts_past[n]
                    Ts_past[n] = 0
                    stopId = int(np.where(POS_STOP == nextPOS)[0])
                    arrTimeList[n, stopId] = t_arr_act
                    t_arr_act += Ts
                    traj_t.append(t_arr_act)
                    traj_x.append(nextPOS)
                else:
                    # 路口等待
                    tlsId = int(np.where(POS_JUNC == nextPOS)[0])
                    busPhase = busPhasePos[tlsId][1]
                    ind = np.searchsorted(tPlanAll[tlsId][1:] - YR, t_arr_act)
                    arrPhase = ind % 4
                    arrCycle = int(ind / 4)
                    if arrPhase < busPhase:
                        t_arr_act = tPlanAll[tlsId][4*arrCycle + busPhase]
                        # isFail = 1
                    elif arrPhase > busPhase:
                        t_arr_act = tPlanAll[tlsId][4*(arrCycle + 1) + busPhase]
                        isFail = 1
                    traj_t.append(t_arr_act)
                    traj_x.append(nextPOS)
                t = t_arr_act
                pos = nextPOS
            ax1.plot(traj_t, traj_x, color='b')
            fail_cnt += isFail
        # np.save(f'E:\\workspace\\python\\BusRouteTSP\\RouteTSP\\result\\busArrTime\\busArrTime_{s}.npy', np.array(arrTimeList))
        # print(arrTimeList)
    # print(np.array(arrTimeMat))

    ax1.set_xlabel('time(s)')
    ax1.set_ylabel('distance(m)')
    ax1.set_title('Traffic Signal Timing Plan')
    plt.savefig(f'{rootPath}\\RouteTSP\\result\\SP_compare_multi\\{DIRNAME}\\{cnt}.png')
    # plt.show()
    return fail_cnt

def local_SP_plot_(tlsPlan, busArrPlan, busPhasePos, PER_BOARD_DUR, V_MAX, DIRNAME, cnt, sample=None):
    POS_JUNC = np.array(posJunc).cumsum()
    POS_STOP = np.concatenate([[0], POS_JUNC]) + np.array(posSet[0])
    YR = 4
    N = len(busArrPlan)
    # I = len(tlsPlan)
    I = 1
    num_sample = 10
    busLoop = busPhasePos[0]
    busPhase = busPhasePos[1]
    TPlan = tlsPlan[:, busLoop, :].flatten()
    tPlan = np.insert(TPlan, 0, 0).cumsum()
    sample_cnt = 0

    # 绘制信号配时
    fig, ax1 = plt.subplots(figsize=(10, 8))
    lines = []
    colors = []
    for i in range(I):
        for j, phaseLen in enumerate(TPlan):
            if j % 4 != busPhase:
                lines.append([(tPlan[j], POS_JUNC[i]), (tPlan[j] + phaseLen, POS_JUNC[i])])
                colors.append('red')
            else:
                lines.append([(tPlan[j], POS_JUNC[i]), (tPlan[j] + phaseLen - YR, POS_JUNC[i])])
                colors.append('green')
                lines.append([(tPlan[j] + phaseLen - YR, POS_JUNC[i]), (tPlan[j] + phaseLen, POS_JUNC[i])])
                colors.append('yellow')
    # 使用 LineCollection 绘制信号灯状态
    lc = LineCollection(lines, colors=colors, linewidths=2)
    ax1.add_collection(lc)
    ax1.set_xlabel('time(s)')
    ax1.set_ylabel('distance(m)')
    ax1.set_title('Traffic Signal Timing Plan')

    for n in range(N):
        for _ in range(num_sample):
            pos = busArrPlan[n][1, 0]
            t = busArrPlan[n][0, 0]
            traj_t, traj_x = [t], [pos]
            for t_arr in busArrPlan[n][:, 1:].T:
                t_arr_plan = t_arr[0]
                nextPOS = t_arr[1]
                # 驶向路口/站点
                if (nextPOS - pos)/(t_arr_plan - t) <= V_MAX and (nextPOS - pos)/(t_arr_plan - t) >= 0:
                    t_arr_act = t_arr_plan
                else:
                    t_arr_act = t + (nextPOS - pos)/V_MAX
                traj_t.append(t_arr_act)
                traj_x.append(nextPOS)

                if nextPOS in POS_STOP:
                    # 停站
                    if sample is None:
                        Ts = np.random.normal(0, 1)*PER_BOARD_DUR
                    else:
                        Ts = sample[sample_cnt]
                        sample_cnt += 1
                    t_arr_act += Ts
                    traj_t.append(t_arr_act)
                    traj_x.append(nextPOS)
                else:
                    # 路口等待
                    ind = np.searchsorted(tPlan[1:] - YR, t_arr_act)
                    arrPhase = ind % 4
                    arrCycle = int(ind / 4)
                    if arrPhase < busPhase:
                        t_arr_act = tPlan[4*arrCycle + busPhase]
                    elif arrPhase > busPhase:
                        t_arr_act = tPlan[4*(arrCycle + 1) + busPhase]
                    traj_t.append(t_arr_act)
                    traj_x.append(nextPOS)

                t = t_arr_act
                pos = nextPOS
            ax1.plot(traj_t, traj_x, color='b')

    # plt.savefig(f'{rootPath}\\RouteTSP\\result\\SP_compare\\{DIRNAME}\\{cnt}.png')
    # plt.show()

def performanceAnalysis(busArrTimeList, TIMETABLE, tlsPlanList, bgPlan):
    INI = -10000
    def calBusArrTimeDev(busArrTime, TIMETABLE):
        mask = busArrTime != INI
        arrStopNum = np.sum(mask)
        busArrTime_mask = np.where(mask, busArrTime, 0)
        timetable_mask = np.where(mask, TIMETABLE, 0)
        return np.sum(np.abs(busArrTime_mask - timetable_mask)) / arrStopNum
    
    def calBusHeadwayDev(busArrTime):
        busArrTime = np.where(busArrTime == INI, np.nan, busArrTime)
        filtered_cols = [busArrTime[:, col][~np.isnan(busArrTime[:, col])] for col in range(busArrTime.shape[1])]
        variances = []
        for col_data in filtered_cols:
            if len(col_data) > 1:  # 至少有两个元素才能计算差值
                diff = np.diff(col_data)  # 计算相邻元素的差值
                var = np.std(diff)  # 计算方差
                variances.append(var)
            else:
                variances.append(np.nan)
        return variances
    
    # def calTlsPlanDev(tlsPlan, bgPlan, padCycleNum):
    #     cycleNum = tlsPlan.shape[0] - padCycleNum
    #     dev = tlsPlan - np.broadcast_to(bgPlan, tlsPlan.shape)
    #     return np.sum(np.abs(dev)) / cycleNum
    
    def calTlsPlanDev(tlsPlan, bgPlan, padCycleNum):
        cycleNum = tlsPlan.shape[0] - padCycleNum
        dev = tlsPlan[padCycleNum:] - np.broadcast_to(bgPlan, tlsPlan[padCycleNum:].shape)
        return np.sum(np.abs(dev)) / cycleNum
    
    # padCycleNumList = [4, 3, 2, 2, 1]
    padCycleNumList = [0, 1, 1, 1, 1]
    tlsDev = 0
    busArrTimeDev = 0
    busHeadwayVarSum = 0
    for i, tlsPlan in enumerate(tlsPlanList):
        tlsDev += calTlsPlanDev(tlsPlan, bgPlan[i, :], padCycleNumList[i])
    for i, busArrTime in enumerate(busArrTimeList):
        TIMETABLE = TIMETABLE[:busArrTime.shape[0], :]
        busArrTimeDev += calBusArrTimeDev(busArrTime, TIMETABLE)
        busHeadwayVar = calBusHeadwayDev(busArrTime)
        busHeadwayVarSum += np.mean(busHeadwayVar[:-1])

    print(f"Bus Arrive Time Deviation Avg (s): {busArrTimeDev/10}")
    print(f"Bus Time Headway stdDev Avg (s): {busHeadwayVarSum/10}")
        # print(f"Traffic Light Plan {i} Deviation Avg (s): {tlsDev}")
    print(f"Traffic Light Plan Deviation Avg (s): {tlsDev/5}")
    # print(busArrTime)
    # print(TIMETABLE)
    # print(tlsPlanList[0])
    # print(bgPlan[0, :])

    
if __name__ == '__main__':
    # J = np.array([[1, 2, 3, 4], [6, 5, 7, 8]])
    # T_opt = np.array([[[19, 40, 16, 25], [44, 15, 19, 22]],
    #                   [[44, 16, 29, 11], [11, 49, 22, 18]]])
    # YR = 4
    # RGYplan = getSumoTLSProgram(J, T_opt, YR)
    # print(RGYplan)
    # for k in range(len(RGYplan)):
    #     for phase in RGYplan[k]:
    #         # print(f'Junc{i}: {phase}')
    #         print(f'Period {k}: {phase}')
    # POS_JUNC = np.array(posJunc).cumsum()
    # POS_STOP = np.concatenate([[0], POS_JUNC]) + np.array(posSet[0])
    # TIMETABLE = np.array([20 + i*120 + (POS_STOP - POS_STOP[0])/12 for i in range(100)])
    # myplot(POS_JUNC, POS_STOP, 2, TIMETABLE)
    # busArrTime = np.load(r'E:\workspace\python\BusRouteTSP\RouteTSP\result\busArrTime_{s}.npy')
    busArrTimeList = [np.load(f'E:\\workspace\\python\\BusRouteTSP\\RouteTSP\\result\\busArrTime\\busArrTime_{s}.npy') for s in range(10)]
    BUS_DEP_HW = 2*60
    POS_JUNC = np.array(posJunc).cumsum()
    POS_STOP = np.concatenate([[0], POS_JUNC]) + np.array(posSet[0])
    V_AVG = 10
    STOP_DUR = 10*np.array([1, 1, 1, 1, 1, 1])
    TIMETABLE = np.array([20 + i*BUS_DEP_HW + (POS_STOP - POS_STOP[0])/V_AVG + np.delete(np.insert(STOP_DUR, 0, 0), -1).cumsum() for i in range(100)])
    BG_PHASE_LEN = np.load(r'E:\workspace\python\BusRouteTSP\tools\result\BG_PHASE_LEN.npy')
    tlsPlanList = [np.load(f'E:\\workspace\\python\\BusRouteTSP\\RouteTSP\\result\\tlsPlan_nt{i}.npy') for i in range(1, 6)]
    performanceAnalysis(busArrTimeList, TIMETABLE, tlsPlanList, BG_PHASE_LEN)