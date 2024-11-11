import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np
import sys
rootPath = r'E:\workspace\python\BusRouteTSP'
sys.path.append(rootPath)
from ScenarioGenerator.nodeGen import posJunc
from tqdm import tqdm
from bisect import bisect_left

# phaseDict = {1: 12, 2: 11, 3: 9, 4: 8, 5: 6, 6: 5, 7: 3, 8: 2}
phaseDict = {1: [7], 2: [12, 13], 3: [10], 4: [2], 5: [14], 6: [5, 6], 7: [3], 8: [9]}
RGY2J = {2: 4, 3: 7, 5: 6, 6: 6, 7: 1, 9: 8, 10: 3, 12: 2, 13: 2, 14: 5}

# T：K*(2*4), K：周期，I：交叉口编号, 2：双环, 4：四相位
# def getSumoTLSProgram(J, T, YR):
#     K = T.shape[0]
#     YR_ = YR * np.ones(T.shape)
#     Tplan = np.zeros(T.shape[:-1] + (T.shape[-1]*2, ))
#     Tplan[..., ::2] = T - YR_
#     Tplan[..., 1::2] = YR_
#     tPlan = np.insert(np.delete(Tplan, -1, axis=-1), 0, 0, axis=-1).cumsum(axis=-1)
#     # print(tPlan)
#     RGYplan = [[] for _ in range(K)]
#     phaseDict = {1: 12, 2: 11, 3: 9, 4: 8, 5: 6, 6: 5, 7: 3, 8: 2}
#     for k in range(K):
#         tk = tPlan[k]
#         tSplit = np.sort(np.unique(tk.flatten(), axis=-1))
#         Tsplit = np.diff(tSplit)
#         # print(tSplit)
#         for j, t in enumerate(tSplit):
#             phaseDur = Tsplit[j] if j < len(Tsplit) else YR
#             RGY = 'GrrGrrGrrGrr'
#             for b in [0, 1]:
#                 tInd = np.searchsorted(tk[b], t, side='right')
#                 currentPhase = phaseDict[J[b][int((tInd - 1)/2)]] - 1
#                 if tInd % 2 == 0:
#                     RGY = RGY[:currentPhase] + 'y' + RGY[currentPhase+1:]
#                 else:
#                     RGY = RGY[:currentPhase] + 'G' + RGY[currentPhase+1:]
#             RGYplan[k].append((RGY, phaseDur))
#     return RGYplan

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
    else:
        return int(id)
    
def getBusOrder(busId, arr):
    order = bisect_left(arr, busId)
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

# def myplot(POS, phase):
#     # 读取文件
#     bus_speed_file = f'{rootPath}\\RouteTSP\\result\\bus_speed_profile.csv'
#     tls_state_file = f'{rootPath}\\RouteTSP\\result\\tlsState_profile.csv'

#     # 读取公交速度数据和信号灯状态数据
#     bus_speed_df = pd.read_csv(bus_speed_file)
#     tls_state_df = pd.read_csv(tls_state_file)

#    # 提取时间列
#     time_range = bus_speed_df.iloc[:, 0].values

#     # 1. 绘制信号灯相位图
#     fig, ax1 = plt.subplots(figsize=(10, 8))

#     # 将信号灯状态颜色映射
#     color_map = {'r': 'red', 'G': 'green', 'y': 'yellow'}

#     # 遍历每个交叉口，绘制信号灯颜色随时间的变化
#     phase = phaseDict[phase][0] - 1
#     for i, col in tqdm(enumerate(tls_state_df.columns[1:])):  # 跳过第一列（时间）
#         tls_state = tls_state_df[col].values
#         for j, state in enumerate(tls_state):
#             ax1.scatter(time_range[j], POS[i], color=color_map[state[phase]], s=10)  # 使用第三个灯色

#     ax1.set_ylabel('Position (m)')
#     ax1.set_xlabel('time (s)')
#     ax1.set_title('P-t curve')

#     # 2. 绘制公交位置-时间曲线
#     # 遍历每一辆公交车（从第二列开始，因为第一列是时间）
#     for col in bus_speed_df.columns[1:]:
#         bus_speed = bus_speed_df[col].fillna(0).values
#         bus_position = np.cumsum(bus_speed)
#         # 绘制公交车的位置-时间曲线
#         ax1.plot(time_range, bus_position, label=f'{col}', linewidth=2)

#     # 添加图例
#     # ax1.legend(loc='best')

#     plt.tight_layout()
#     plt.savefig(f'{rootPath}\\RouteTSP\\result\\P-t curve.png')
#     plt.show()

def myplot(POS, phase):
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

    plt.tight_layout()
    plt.savefig(f'{rootPath}\\RouteTSP\\result\\P-t curve.png')
    plt.show()

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
    POS_JUNC = np.array(posJunc).cumsum()
    myplot(POS_JUNC, 2)