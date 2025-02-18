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
import xml.etree.ElementTree as ET
import pickle
import ast

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
    if np.array_equal(T[0, :, :], np.zeros_like(T[0])):
        T = T[1:, :, :]
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
    tPlan = np.insert(np.delete(Tplan, -1, axis=-1), 0, 0, axis=-1).cumsum(axis=-1)
    # print(tPlan)
    RGYplan = [[] for _ in range(K)]

   # 14 links (3/4 lanes)
    # phaseDict = {1: [7], 2: [12, 13], 3: [10], 4: [2], 5: [14], 6: [5, 6], 7: [3], 8: [9]}
    phaseDict = {1: [7], 2: [11, 12, 13], 3: [10], 4: [2], 5: [14], 6: [4, 5, 6], 7: [3], 8: [9]}

    for k in range(K):
        tk = tPlan[k]
        tSplit = np.sort(np.unique(tk.flatten(), axis=-1))
        Tsplit = np.diff(tSplit)
        for j, t in enumerate(tSplit):
            phaseDur = Tsplit[j] if j < len(Tsplit) else Tplan[k, 0, -1]
            # RGY = 'GrrGrrrGrrGrrr'
            RGY = 'GrrrrrrGrrrrrr'
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
    
def calOffsetForCoordPhase(OFFSET, BG_PHASE_LEN, BG_PHASE_SEQ, BG_CYCLE_LEN, COORD_PHASE):
    offset_ib = np.zeros_like(OFFSET)
    for i in range(len(OFFSET)-1, -1, -1):
        offset_ib[i] = (OFFSET[i] - OFFSET[-1]) % BG_CYCLE_LEN
        if i + 1 < len(OFFSET) and offset_ib[i] < offset_ib[i+1]:
            offset_ib[i] += BG_CYCLE_LEN * (1 + int((offset_ib[i+1] - offset_ib[i]) / BG_CYCLE_LEN))
    offset_coord = np.array([OFFSET, offset_ib])
    for l in range(2):
        for i in range(len(OFFSET)):
            ind = np.where(BG_PHASE_SEQ[i, l, :] == COORD_PHASE[l])[0][0]
            offset_coord[l, i] += BG_PHASE_LEN[i, l, :ind].sum()
    offset_coord[0, :] -= offset_coord[0, 0]
    offset_coord[1, :] -= offset_coord[1, -1]
    return offset_coord

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

def myplot(testDir, POS, POS_stop, phase, timetable):
    PLAN_START = 10
    PLAN_STEP = 50
    # 读取文件
    bus_speed_file = f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\bus_speed_profile.csv'
    tls_state_file = f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\tlsState_profile.csv'

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
    plt.savefig(f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\P-t curve.png')
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

def performanceAnalysis(testDir, busArrTime, TIMETABLE, tlsPlanList, bgPlan, SIMTIME, v_avg):
    INI = -10000
    def calBusArrTimeDev(busArrTime, TIMETABLE):
        mask = busArrTime != INI
        # arrStopNum = np.sum(mask)
        busArrTime_mask = np.where(mask, busArrTime, np.nan)
        timetable_mask = np.where(mask, TIMETABLE, np.nan)
        # return np.sum(np.abs(busArrTime_mask - timetable_mask)) / arrStopNum
        busArrTimeDev = np.abs(busArrTime_mask - timetable_mask)
        # 计算准点率
        late_mask = busArrTimeDev > 30
        late_count = np.sum(late_mask, axis=0)
        total_valid_count = np.sum(~np.isnan(busArrTimeDev), axis=0)
        late_rate = late_count / total_valid_count
        return np.nanmean(busArrTimeDev, axis=0), np.nanstd(busArrTimeDev, axis=0), late_rate
    
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
        return np.array(variances)
    
    def calTlsPlanDev(tlsPlan, bgPlan, padCycleNum):
        cycleNum = tlsPlan.shape[0] - padCycleNum
        dev = tlsPlan[padCycleNum:] - np.broadcast_to(bgPlan, tlsPlan[padCycleNum:].shape)
        return np.sum(np.abs(dev)) / cycleNum
    
    def cal_veh_delay(SIMTIME, testDir, v_avg):
        def calculate_route_distance(route_tuple, v_avg):
            route_tuple = ast.literal_eval(route_tuple)
            dist_dict = {'np1_nt1': 500, 'np2_nt1': 500, 'np3_nt1': 500, 'nt1_nt2': 700, 'nt1_np1': 500, 'nt1_np2': 500, 'nt1_np3': 500, 'nt2_nt1': 700,
                     'np4_nt2': 500, 'np5_nt2': 500, 'nt2_nt3': 640, 'nt2_np4': 500, 'nt2_np5': 500, 'nt3_nt2': 640,
                     'np6_nt3': 500, 'np7_nt3': 500, 'nt3_nt4': 560, 'nt3_np6': 500, 'nt3_np7': 500, 'nt4_nt3': 560,
                     'np8_nt4': 500, 'np9_nt4': 500, 'nt4_nt5': 670, 'nt4_np8': 500, 'nt4_np9': 500, 'nt5_nt4': 670,
                     'np10_nt5': 500, 'np11_nt5': 500, 'nt5_np12': 500, 'nt5_np10': 500, 'nt5_np11': 500, 'np12_nt5': 500}
            return sum(dist_dict.get(route, 0) for route in route_tuple)/v_avg
        veh_file = f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\veh_delay.csv'
        veh_df = pd.read_csv(veh_file)
        veh_df = veh_df.set_index(veh_df.columns[0], drop=True).T.dropna()
        veh_df['arrive'] = pd.to_numeric(veh_df['arrive'], errors='coerce')  # 将无法转换的值变为 NaN
        veh_df['depart'] = pd.to_numeric(veh_df['depart'], errors='coerce')  # 将无法转换的值变为 NaN
        veh_df = veh_df[veh_df['depart'] < SIMTIME - 1]
        # veh_df = veh_df[veh_df.index.str.startswith('f')]
        veh_num = len(veh_df)
        veh_arr = veh_df.iloc[:, 0]
        veh_dep = veh_df.iloc[:, 1]
        avg_delay = (veh_dep - veh_arr).mean()
        veh_df['passingTime'] = veh_dep - veh_arr
        # 按照 'routes' 列进行分组，并计算每组的均值
        grouped = veh_df.groupby('route').agg(
            avgPassingTime=('passingTime', 'mean'),
            count=('route', 'size')
        ).reset_index()
        grouped['idealPassingTime'] = grouped['route'].apply(lambda x: calculate_route_distance(x, v_avg))
        grouped['delay'] = grouped['avgPassingTime'] - grouped['idealPassingTime']
        grouped.to_csv(f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\veh_delay_in_route.csv', index=False)
        return veh_num, avg_delay
    
    def cal_traffic_impact(testDir):
        # 解析XML文件
        xmlfile = f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\output_E2_3500.xml'
        tree = ET.parse(xmlfile)
        root = tree.getroot()
        data_dict = {}

        # 遍历所有interval节点
        for interval in root.findall('interval'):
            # 获取id
            interval_id = interval.get('id')
            # 提取所需的指标
            mean_time_loss = interval.get('meanTimeLoss')
            n_veh_seen = interval.get('nVehSeen')
            max_jam_length_in_vehicles = interval.get('maxJamLengthInVehicles')
            started_halts = interval.get('startedHalts')
            # 将提取的指标添加到字典中
            metrics = {
                'meanTimeLoss': float(mean_time_loss),
                'nVehSeen': int(n_veh_seen),
                'maxJamLengthInVehicles': int(max_jam_length_in_vehicles),
                'startedHalts': float(started_halts)
            }
            if interval_id not in data_dict:
                data_dict[interval_id] = []
            data_dict[interval_id].append(metrics)

        PI_dict_move = {}
        PI_dict_int = {}
        for id, metricsSeries in data_dict.items():
            veh_sum = np.sum([metrics['nVehSeen'] for metrics in metricsSeries])
            avg_delay = np.sum([metrics['meanTimeLoss']*metrics['nVehSeen'] for metrics in metricsSeries])/veh_sum
            avg_max_queue_length = np.mean([metrics['maxJamLengthInVehicles'] for metrics in metricsSeries])
            max_max_queue_length = np.max([metrics['maxJamLengthInVehicles'] for metrics in metricsSeries])
            all_halts = np.sum([metrics['startedHalts'] for metrics in metricsSeries])
            PI_dict_move[id] = {
                'vehSum': veh_sum,
                'avgDelay': avg_delay,
                'avgMaxQueueLength': avg_max_queue_length,
                'maxQueueLength': max_max_queue_length,
                'halts': all_halts
            }
            intId = id.split('_')[1]
            if intId not in PI_dict_int:
                PI_dict_int[intId] = {}
            PI_dict_int[intId][id] = PI_dict_move[id]
        
        for intId, metricsSeries in PI_dict_int.items():
            veh_sum = np.sum([metrics['vehSum'] for metrics in metricsSeries.values()])
            avg_delay = np.sum([metrics['avgDelay']*metrics['vehSum'] for metrics in metricsSeries.values()])/veh_sum
            avg_max_queue_length = np.mean([metrics['avgMaxQueueLength'] for metrics in metricsSeries.values()])
            max_max_queue_length = np.max([metrics['maxQueueLength'] for metrics in metricsSeries.values()])
            all_halts = np.sum([metrics['halts'] for metrics in metricsSeries.values()])
            PI_dict_int[intId] = {
                'vehSum': veh_sum,
                'avgDelay': avg_delay,
                'avgMaxQueueLength': avg_max_queue_length,
                'maxQueueLength': max_max_queue_length,
                'halts': all_halts
            }
        
        veh_sum_total = np.sum([metrics['vehSum'] for metrics in PI_dict_int.values()])
        avg_delay_total = np.sum([metrics['avgDelay']*metrics['vehSum'] for metrics in PI_dict_int.values()])/veh_sum_total
        avg_max_queue_length_total = np.mean([metrics['avgMaxQueueLength'] for metrics in PI_dict_int.values()])
        max_max_queue_length_total = np.max([metrics['maxQueueLength'] for metrics in PI_dict_int.values()])
        avg_halts_total = np.sum(metrics['halts'] for metrics in PI_dict_int.values())/veh_sum_total

        return PI_dict_int, PI_dict_move, data_dict, veh_sum_total, avg_delay_total, avg_max_queue_length_total, max_max_queue_length_total, avg_halts_total
    
    padCycleNumList = [4, 3, 2, 2, 1]
    # padCycleNumList = [0, 1, 1, 1, 1]
    tlsDev = 0
    for i, tlsPlan in enumerate(tlsPlanList):
        tlsDev += calTlsPlanDev(tlsPlan, bgPlan[i, :], padCycleNumList[i])

    TIMETABLE = TIMETABLE[:busArrTime.shape[0], :]
    busArrTimeDev, busArrTimeDevStd, lateRate = calBusArrTimeDev(busArrTime, TIMETABLE)
    busHeadwayVar = calBusHeadwayDev(busArrTime)
    veh_num, avg_delay = cal_veh_delay(SIMTIME, testDir, v_avg)
    PI_dict_int, PI_dict_move, data_dict, veh_sum_total, avg_delay_total, avg_max_queue_length_total, max_max_queue_length_total, avg_halts_total = cal_traffic_impact(testDir)

    print(f"=========Performance Index for {testDir}=========")
    print(f"Bus Arrive Time Deviation Avg (s): {busArrTimeDev}, {np.mean(busArrTimeDev[1:])}")
    print(f"Bus Arrive Time Deviation stdDev (s): {busArrTimeDevStd}, {np.mean(busArrTimeDevStd[1:])}")
    print(f"Bus Late Rate (%): {100*lateRate}, {np.mean(100*lateRate[1:])}")
    print(f"Bus Time Headway stdDev Avg (s): {busHeadwayVar}, {np.mean(busHeadwayVar[1:])}")
    # print(f"Traffic Light Plan {i} Deviation Avg (s): {tlsDev}")
    print(f"Traffic Light Plan Deviation Avg (s): {tlsDev/len(tlsPlanList)}")
    print(f'Passed vehicle number: {veh_num}')
    print(f'Average Passing Time: {avg_delay}s')
    print(f'Vehicle Sum: {veh_sum_total}')
    print(f'Average Vehicle Delay: {avg_delay_total}s')
    print(f'Average Max Queue Length: {avg_max_queue_length_total}')
    print(f'Total Max Queue Length: {max_max_queue_length_total}')
    print(f'Average Vehicle Stops: {avg_halts_total}')
    # print(busArrTime)
    # print(TIMETABLE)
    # print(tlsPlanList[0])
    # print(bgPlan[0, :])
    print("=====================================================")
    return busArrTimeDev, busArrTimeDevStd, lateRate, busHeadwayVar, avg_delay, PI_dict_int, PI_dict_move, data_dict

def plot_result(saveDIR, testDirList, PI_name, yAxis_name, Algo_name):
    data = []
    delay_data = []
    for testDir in testDirList:
        busArrTimeDev = np.load(f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\busArrTimeDev.npy')
        busArrTimeDevStd = np.load(f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\busArrTimeDevStd.npy')
        lateRate = np.load(f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\lateRate.npy')
        busHeadwayVar = np.load(f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\busHeadwayVar.npy')
        data.append(np.array([busArrTimeDev, busArrTimeDevStd, 100*lateRate, busHeadwayVar]))
        delay = np.load(f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\delay.npy')
        delay_data.append(delay)
    data = np.array(data)
    delay_data = np.array(delay_data)

    n = len(testDirList)
    # 为每个数组绘制独立的柱状图
    for i in range(4):
        plt.figure(figsize=(10, 5))  # 创建一个新的图，figsize 控制图形大小

        # 设置柱状图宽度和位置
        width = 0.15  # 每个柱状图的宽度
        x = np.arange(5)  # x 轴上的位置，长度为 6

        # 绘制每组数据的柱状图
        for j in range(n):
            plt.bar(x + j * width, data[j, i, 1:], width=width, label=f"{Algo_name[j]}")

        # 设置标题、X轴标签、Y轴标签
        plt.title(f"{PI_name[i]}")
        plt.xlabel("Bus Stop")
        plt.ylabel(f"{yAxis_name[i]}")
        plt.xticks(x + (n-1) * width / 2, [f"{i+1}" for i in range(5)])

        # 添加图例
        plt.legend()

        # 显示图表
        plt.tight_layout()
        plt.savefig(f'{rootPath}\\RouteTSP\\result\\simulation result\\{saveDIR}\\{PI_name[i]}.png')
        plt.show()
    
    # delay
    plt.figure(figsize=(8, 5))
    # 设置柱状图位置
    x = np.arange(n)

    # 绘制柱状图
    plt.bar(x, delay_data, width=0.6, color='skyblue', edgecolor='black')

    # 设置标题和标签
    plt.title("Vehicle Passing Time")
    plt.xlabel("Algorithm")
    plt.ylabel("Time (s)")
    plt.xticks(x, [f"{Algo_name[i]}" for i in range(n)])

    plt.ylim(180, 220)

    # 显示图表
    plt.tight_layout()
    plt.savefig(f'{rootPath}\\RouteTSP\\result\\simulation result\\{saveDIR}\\delay.png')
    plt.show()

def plot_result_for_each_intersection(saveDIR, testDirList):
    data = []
    for testDir in testDirList:
        with open(f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\PI_dict_int_3500.pkl', "rb") as f:
            PI_dict = pickle.load(f)
        node_order = ['nt1', 'nt2', 'nt3', 'nt4', 'nt5']
        data.append(pd.DataFrame(PI_dict).T.reindex(node_order)) 
    for metric in data[0].columns:
        if metric == 'vehSum':
            continue
        combined_df = pd.DataFrame({f'{testDirList[i]}': data[i][metric] for i in range(len(data))})
        # Plot grouped bar chart
        combined_df.plot(kind='bar', figsize=(10, 6), edgecolor='black')
        plt.title(f'Comparison of {metric}', fontsize=14)
        plt.xlabel('Node', fontsize=12)
        plt.ylabel(metric, fontsize=12)
        plt.xticks(rotation=0, fontsize=10)
        plt.legend(fontsize=10)
        plt.grid(axis='y', linestyle='--', alpha=0.7)
        plt.tight_layout()
        plt.savefig(f'{rootPath}\\RouteTSP\\result\\simulation result\\{saveDIR}\\traffic impact\\intersection\\{metric}.png')
        plt.show()

def plot_result_for_each_movement(saveDIR, testDirList):
    data = []
    for testDir in testDirList:
        with open(f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\PI_dict_move_3500.pkl', "rb") as f:
            PI_dict = pickle.load(f)
        grouped_data = {}
        for key, value in PI_dict.items():
            intId = key.split('_')[1]
            moveId = key.split('_')[2] if len(key.split('_')) == 3 else (key.split('_')[2] + key.split('_')[3])
            if intId not in grouped_data:
                grouped_data[intId] = {}
            grouped_data[intId][moveId] = value
        nodeOrder = ['WL', 'WT1', 'WT2', 'NL', 'NT', 'EL', 'ET1', 'ET2', 'SL', 'ST']
        for key, value in grouped_data.items():
            grouped_data[key] = pd.DataFrame(value).T.reindex(nodeOrder)
        data.append(grouped_data)

    for ntx in data[0]:
        for metric in data[0][ntx].columns:
            if metric == 'vehSum':
                continue
            combined_df = pd.DataFrame({f'{testDirList[i]}': data[i][ntx][metric] for i in range(len(data))})
            # Plot grouped bar chart
            combined_df.plot(kind='bar', figsize=(10, 6), edgecolor='black')
            plt.title(f'Comparison of {metric} in {ntx}', fontsize=14)
            plt.xlabel('Node', fontsize=12)
            plt.ylabel(metric, fontsize=12)
            plt.xticks(rotation=0, fontsize=10)
            plt.legend(fontsize=10)
            plt.grid(axis='y', linestyle='--', alpha=0.7)
            plt.tight_layout()
            plt.savefig(f'{rootPath}\\RouteTSP\\result\\simulation result\\{saveDIR}\\traffic impact\\movement\\{ntx}\\{metric}.png')
            # plt.show()

def plot_result_for_route(testDirList, route, phase):
    for testDir in testDirList:
        veh_file = f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\veh_delay.csv'
        veh_df = pd.read_csv(veh_file)
        veh_df = veh_df.set_index(veh_df.columns[0], drop=True).T.dropna()
        veh_df = veh_df[veh_df['route'] == route]
        
        fig, ax1 = plt.subplots(figsize=(10, 8))
        # 遍历 DataFrame 的每一行，提取轨迹并绘制
        for i, row in veh_df.iterrows():
            row['traj'] = ast.literal_eval(row['traj'])
            t_values = [point[0] for point in row['traj']]
            x_values = [point[1] for point in row['traj']]
            color = 'blue' if row.name.startswith('f') else 'purple'
            plt.plot(t_values, x_values, color=color, linewidth=1)
        
        tls_state_file = f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\tlsState_profile.csv'
        tls_state_df = pd.read_csv(tls_state_file)
        time_range = tls_state_df.iloc[:, 0].values
        # 1. 预处理信号灯状态数据
        tls_state_df = tls_state_df.iloc[:, 1:]  # 跳过第一列（时间列）
        num_intersections = len(tls_state_df.columns)
        num_time_points = len(tls_state_df)
        # 将信号灯状态分解为字符数组
        phase = phaseDict[phase][0] - 1
        tls_states = np.array([[state[phase] for state in tls_state_df[col].values] 
                            for col in tls_state_df.columns])
        # 定义颜色映射
        color_map = {'r': 'red', 'G': 'green', 'y': 'yellow'}
        lines = []
        colors = []
        # 为每个交叉口生成线段集合
        POS = np.array(posJunc).cumsum()
        for i in range(num_intersections):
            for j in range(1, num_time_points):
                lines.append([(time_range[j-1], POS[i]), (time_range[j], POS[i])])
                colors.append(color_map[tls_states[i][j]])
        # 使用 LineCollection 绘制信号灯状态
        lc = LineCollection(lines, colors=colors, linewidths=2)
        ax1.add_collection(lc)

        # 设置坐标轴标签和标题
        plt.xlabel('Time (t)')
        plt.ylabel('X (position)')
        plt.title('All Trajectories')
        plt.grid(True)
        plt.savefig(f'{rootPath}\\RouteTSP\\result\\case study\\{testDir}\\traj\\{route}.svg', dpi=300, format="svg")
        plt.show()

if __name__ == '__main__':
    Ti = [[[0, 0, 0, 3], [0, 0, 0, 3]],
          [[15, 33, 12, 34], [15, 33, 13, 33]],
          [[15, 39, 12, 34], [17, 37, 13, 33]]]
    RGYplan = getSumoTLSProgram(np.array([[1, 2, 3, 4], [5, 6, 7, 8]]), np.array(Ti), 4)
    pass