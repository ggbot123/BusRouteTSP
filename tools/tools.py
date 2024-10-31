import numpy as np
import pandas as pd
import re

def volume(junctions, i, dir, move):
    if junctions[i][dir][0] < 0:
        print(f'error: junc{i, dir, move}')
        return
    if move == 'left':
        ratio = junctions[i][dir][1][0]
    elif move == 'through':
        ratio = junctions[i][dir][1][1]
    else:
        ratio = junctions[i][dir][1][2]
    return junctions[i][dir][0] * ratio

def vc_ratio(traffic_demand, i, dir, move, lane_num=1):
    if move == 'left':
        c = 1700 # 右转饱和流率
    elif move == 'through':
        c = 1800 # 直行饱和流率
    else:
        c = 1500 # 右转饱和流率
    return volume(traffic_demand, 'I' + str(i+1), dir, move)/(c * lane_num)

def all_vc_ratio(traffic_demand):
    output = np.zeros([len(traffic_demand), 8])
    for i in range(len(traffic_demand)):
        output[i] = [vc_ratio(traffic_demand, i, 'E', 'left'), vc_ratio(traffic_demand, i, 'W', 'through', 2),
                     vc_ratio(traffic_demand, i, 'S', 'left'), vc_ratio(traffic_demand, i, 'N', 'through'),
                     vc_ratio(traffic_demand, i, 'W', 'left'), vc_ratio(traffic_demand, i, 'E', 'through', 2),
                     vc_ratio(traffic_demand, i, 'N', 'left'), vc_ratio(traffic_demand, i, 'S', 'through')]
    # return {f'I{j+1}': vc for j, vc in enumerate(output)}
    return output

# T：K*(2*4), K：周期，I：交叉口编号, 2：双环, 4：四相位
def getSumoTLSProgram(J, T, YR):
    K = T.shape[0]
    YR_ = YR * np.ones(T.shape)
    Tplan = np.zeros(T.shape[:-1] + (T.shape[-1]*2, ))
    Tplan[..., ::2] = T - YR_
    Tplan[..., 1::2] = YR_
    tPlan = np.insert(np.delete(Tplan, -1, axis=-1), 0, 0, axis=-1).cumsum(axis=-1)
    # print(tPlan)
    RGYplan = [[] for _ in range(K)]
    
    # 12 links (3 lanes)
    # phaseDict = {1: 12, 2: 11, 3: 9, 4: 8, 5: 6, 6: 5, 7: 3, 8: 2}

    # 14 links (3/4 lanes)
    phaseDict = {1: [7], 2: [12, 13], 3: [10], 4: [2], 5: [14], 6: [5, 6], 7: [3], 8: [9]}
    # phaseDict = {1: [12], 2: [11], 3: [9], 4: [8], 5: [6], 6: [5], 7: [3], 8: [2]}

    for k in range(K):
        tk = tPlan[k]
        tSplit = np.sort(np.unique(tk.flatten(), axis=-1))
        Tsplit = np.diff(tSplit)
        # print(tSplit)
        for j, t in enumerate(tSplit):
            phaseDur = Tsplit[j] if j < len(Tsplit) else YR
            # RGY = 'GrrGrrGrrGrr'
            RGY = 'GrrGrrrGrrGrrr'
            for b in [0, 1]:
                tInd = np.searchsorted(tk[b], t, side='right')
                # currentPhase = phaseDict[J[b][int((tInd - 1)/2)]] - 1
                # if tInd % 2 == 0:
                #     RGY = RGY[:currentPhase] + 'y' + RGY[currentPhase+1:]
                # else:
                #     RGY = RGY[:currentPhase] + 'G' + RGY[currentPhase+1:]
                currentPhaseList = phaseDict[J[b][int((tInd - 1)/2)]]
                for currentPhase in currentPhaseList:
                    if tInd % 2 == 0:
                        RGY = RGY[:(currentPhase - 1)] + 'y' + RGY[currentPhase:]
                    else:
                        RGY = RGY[:(currentPhase - 1)] + 'G' + RGY[currentPhase:]
            RGYplan[k].append((RGY, phaseDur))
    return RGYplan


def round_and_adjust(arr, N):
    """
    将一个2x4的浮点数组取整，并调整每行的总和为100，同时保证每行前两个元素的和相等。
    
    参数：
    arr: numpy.ndarray, 形状为 (2, 4)，浮点数组
    
    返回：
    numpy.ndarray, 调整后的整数数组
    """
    # 将浮点数组四舍五入为整数
    rounded_arr = np.round(arr).astype(int)
    
    # 调整每行的和为100
    for i in range(2):
        row_sum = np.sum(rounded_arr[i])
        diff = N - row_sum
        
        # 如果总和不为100，调整最后一个元素
        rounded_arr[i, -1] += diff

    # 确保每行前两个元素的和相等
    row1_sum = np.sum(rounded_arr[0, :2])
    row2_sum = np.sum(rounded_arr[1, :2])
    row_diff = row1_sum - row2_sum

    adjustment = abs(row_diff) // 2
    # 如果前两个元素的和不相等
    if row_diff > 0:
        rounded_arr[0, 0] -= (adjustment + 1)
        rounded_arr[1, 0] += adjustment
        rounded_arr[0, -1] += 1
    elif row_diff < 0:
        rounded_arr[0, 0] += adjustment
        rounded_arr[1, 0] -= (adjustment + 1)
        rounded_arr[1, -1] += 1


    return rounded_arr

def get_volume_ratio(traffic_demand, from_junc, to_junc, from_node, from_move, to_move):
    n = len(traffic_demand)
    moveDict = {'left': 0, 'through': 1, 'right': 2}
    if from_node == 1:
        from_dir = 'W'
    elif from_node == 2*n+2:
        from_dir = 'E'
    elif from_node % 2 == 0:
        from_dir = 'N'
    else:
        from_dir = 'S'
    if from_junc == to_junc:
        if to_move is None:
            return traffic_demand['I' + str(from_junc)][from_dir][1][moveDict[from_move]]
        if from_move is None:
            return traffic_demand['I' + str(from_junc)][from_dir][1][moveDict[to_move]]
        else:
            print('ERROR in calculating volume ratio!')
            return -1
    elif from_junc < to_junc:
        if from_move is None:
            from_move = 'through'
        ratio_to_next_node = traffic_demand['I' + str(from_junc)][from_dir][1][moveDict[from_move]]
        return ratio_to_next_node * get_volume_ratio(traffic_demand, from_junc + 1, to_junc, 1, None, to_move)
    else:
        if from_move is None:
            from_move = 'through'
        ratio_to_next_node = traffic_demand['I' + str(from_junc)][from_dir][1][moveDict[from_move]]
        return ratio_to_next_node * get_volume_ratio(traffic_demand, from_junc - 1, to_junc, 2*n+2, None, to_move)

def demand_to_route(traffic_demand):
    n = len(traffic_demand)
    from_edges = ['%s_%s' % (x, 'nt1') for x in ['np1', 'np2', 'np3']] + \
            ['%s_%s' % (x, 'nt2') for x in ['np4', 'np5']] + \
            ['%s_%s' % (x, 'nt3') for x in ['np6', 'np7']] + \
            ['%s_%s' % (x, 'nt4') for x in ['np8', 'np9']] + \
            ['%s_%s' % (x, 'nt5') for x in ['np10', 'np11', 'np12']]
    to_edges = ['%s_%s' % ('nt1', x) for x in ['np1', 'np2', 'np3']] + \
            ['%s_%s' % ('nt2', x) for x in ['np4', 'np5']] + \
            ['%s_%s' % ('nt3', x) for x in ['np6', 'np7']] + \
            ['%s_%s' % ('nt4', x) for x in ['np8', 'np9']] + \
            ['%s_%s' % ('nt5', x) for x in ['np10', 'np11', 'np12']]
    OD_volume = pd.DataFrame(np.zeros([len(from_edges), len(to_edges)]), index=from_edges, columns=to_edges)
    for O in from_edges:
        for D in to_edges:
            from_junc = int(re.findall(r'\d+', O)[1])
            from_node = int(re.findall(r'\d+', O)[0])
            to_junc = int(re.findall(r'\d+', D)[0])
            to_node = int(re.findall(r'\d+', D)[1])

            if from_node == to_node:
                volume_ratio = 0
                continue

            if from_junc == to_junc:
                if from_node == 1 and to_node == 2 or from_node == 3 and to_node == 1 \
                    or from_node == 2*n+2 and to_node == 2*n+1 or from_node == 2*n and to_node == 2*n+2:
                    from_move = 'left'
                elif from_node == 2 and to_node == 1 or from_node == 1 and to_node == 3 \
                    or from_node == 2*n+1 and to_node == 2*n+2 or from_node == 2*n+2 and to_node == 2*n:
                    from_move = 'right'
                else:
                    from_move = 'through'
                volume_ratio = get_volume_ratio(traffic_demand, from_junc, to_junc, from_node, from_move, None)
            elif from_junc < to_junc:
                if from_node == 1:
                    from_move = 'through'
                elif from_node % 2 == 0:
                    from_move = 'left'
                else:
                    from_move = 'right'
                if to_node == 2*n+2:
                    to_move = 'through'
                elif to_node % 2 == 0:
                    to_move = 'left'
                else:
                    to_move = 'right'
                volume_ratio = get_volume_ratio(traffic_demand, from_junc, to_junc, from_node, from_move, to_move)
            else:
                if from_node == 2*n+2:
                    from_move = 'through'
                elif from_node % 2 == 0:
                    from_move = 'right'
                else:
                    from_move = 'left'
                if to_node == 1:
                    to_move = 'through'
                elif to_node % 2 == 0:
                    to_move = 'right'
                else:
                    to_move = 'left'
                volume_ratio = get_volume_ratio(traffic_demand, from_junc, to_junc, from_node, from_move, to_move)

            if from_node == 1:
                from_dir = 'W'
            elif from_node == 2*n+2:
                from_dir = 'E'
            elif from_node % 2 == 0:
                from_dir = 'N'
            else:
                from_dir = 'S'
            OD_volume.loc[O, D] = volume_ratio * traffic_demand['I' + str(from_junc)][from_dir][0]
    # OD_volume.to_csv(r'E:\workspace\python\BusRouteTSP\tools\result\routes.csv')
    OD_volume.to_csv(r'E:\workspace\python\BusRouteTSP\tools\result\OD_volume.csv')
