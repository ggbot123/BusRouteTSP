import sys
rootPath = r'E:\workspace\python\BusRouteTSP'
sys.path.append(rootPath)
from maxband import MaxBAND_versatile
from tools import vc_ratio, all_vc_ratio, getSumoTLSProgram, round_and_adjust, demand_to_route, volume
from ScenarioGenerator.nodeGen import posJunc
import numpy as np
import json
import pickle
from plot import plot_signal_timing

def cal_green_split(junctions):
    green_split = np.zeros([len(junctions), 8])   # 顺序：EL-WT-SL-NT-WL-ET-SL-NT
    for i in range(len(junctions)):
        vc_main = max((vc_ratio(junctions, i, 'W', 'through', 2) + vc_ratio(junctions, i, 'E', 'left')),
                      (vc_ratio(junctions, i, 'W', 'left') + vc_ratio(junctions, i, 'E', 'through', 2)))
        vc_cross = max((vc_ratio(junctions, i, 'N', 'through') + vc_ratio(junctions, i, 'S', 'left')),
                       (vc_ratio(junctions, i, 'N', 'left') + vc_ratio(junctions, i, 'S', 'through')))
        MM = vc_main / (vc_main + vc_cross)
        green_split[i] = [MM * vc_ratio(junctions, i, 'E', 'left') / (vc_ratio(junctions, i, 'W', 'through', 2) + vc_ratio(junctions, i, 'E', 'left')),
                          MM * vc_ratio(junctions, i, 'W', 'through', 2) / (vc_ratio(junctions, i, 'W', 'through', 2) + vc_ratio(junctions, i, 'E', 'left')),
                          (1 - MM) * vc_ratio(junctions, i, 'S', 'left') / (vc_ratio(junctions, i, 'N', 'through') + vc_ratio(junctions, i, 'S', 'left')),
                          (1 - MM) * vc_ratio(junctions, i, 'N', 'through') / (vc_ratio(junctions, i, 'N', 'through') + vc_ratio(junctions, i, 'S', 'left')),
                          MM * vc_ratio(junctions, i, 'W', 'left') / (vc_ratio(junctions, i, 'W', 'left') + vc_ratio(junctions, i, 'E', 'through', 2)),
                          MM * vc_ratio(junctions, i, 'E', 'through', 2) / (vc_ratio(junctions, i, 'W', 'left') + vc_ratio(junctions, i, 'E', 'through', 2)),
                          (1 - MM) * vc_ratio(junctions, i, 'N', 'left') / (vc_ratio(junctions, i, 'N', 'left') + vc_ratio(junctions, i, 'S', 'through')),
                          (1 - MM) * vc_ratio(junctions, i, 'S', 'through') / (vc_ratio(junctions, i, 'N', 'left') + vc_ratio(junctions, i, 'S', 'through'))
                        ]
        pair_dict = {0: 1, 1: 0, 2: 3, 3: 2, 4: 5, 5: 4, 6: 7, 7: 6}
        minG = 0.12
        for j in range(len(green_split[i])):
            if green_split[i, j] < minG:
                green_split[i, j] = minG
                if j in [0, 1, 4, 5]:
                    green_split[i, pair_dict[j]] = MM - minG
                else:
                    green_split[i, pair_dict[j]] = 1 - MM - minG
    return np.array(green_split)

# inputs
with open(r'E:\workspace\python\BusRouteTSP\tools\result\traffic_demand.json', 'r') as json_file:
    traffic_demand = json.load(json_file)
print(all_vc_ratio(traffic_demand))
print(cal_green_split(traffic_demand))

num_signals = len(traffic_demand)
green_split = cal_green_split(traffic_demand)
red_time = 1 - green_split[:, [1, 5]].T
queue_clearance_time = 0.1* np.array([[1, 1, 1, 1, 1],
                                      [1, 1, 1, 1, 1]])
two_way_vol_ratio = 1
cycle_range = [1/100, 1/100]
speed_range = 10 * np.ones((2, num_signals - 1, 2))
speed_range[:, :, 1] = 14
speed_change_range = -30 * np.ones((2, num_signals - 2, 2))
speed_change_range[:, :, 1] = 30
distance = np.array([posJunc[1:], posJunc[1:]])
left_turn_time = green_split[:, [4, 0]].T

# build versatile MaxBAND model and solve
maxband = MaxBAND_versatile(num_signals=num_signals)
b, w, m, z, t, sig = maxband.solve(red_time, queue_clearance_time,
                                   two_way_vol_ratio, cycle_range, speed_range,
                                   speed_change_range, distance, left_turn_time)
w = np.array(w).reshape(2, -1)
t = np.array(t).reshape(2, -1)
sig = np.array(sig).reshape(2, -1)

print(f"b: {b}")
print(f"w: {w}")
print(f"m: {m}")
print(f"z: {z}")
print(f"t: {t}")
print(f"sig: {sig}")

cycle_length = round(1/z[0])
progression_speed = distance / (t*cycle_length)
offset_WT = np.insert((t[0, :] - np.diff(w[0, :]) - queue_clearance_time[0, 1:])*cycle_length, 0, 0)
delta = ((2*sig[0, :]-1) * left_turn_time[0, :] - (2*sig[1, :]-1) * left_turn_time[1, :])/2
inbound_dev = ((red_time[1, :] - red_time[0, :])/2 - delta)*cycle_length
green_ratio = green_split[:, [1, 5]].T

print(f"cycle length: {cycle_length}s")
print(f"progression speed: {progression_speed}m/s")
print(f"outbound offset (WT): {offset_WT}s")
print(f"inbound-outbound deviation: {inbound_dev}s")
print(f"green ratio: {green_ratio}")

YR = 4
J = np.zeros([num_signals, 2, 4], dtype=int)
T = np.zeros([1, num_signals, 2, 4])
offset = offset_WT.copy()
allRGYplan = []
for i in range(num_signals):
    if sig[0, i] == 0 and sig[1, i] == 1:
        J[i] = np.array([[2, 1, 3, 4], [5, 6, 7, 8]])
    elif sig[0, i] == 1 and sig[1, i] == 0:
        J[i] = np.array([[1, 2, 3, 4], [6, 5, 7, 8]])
        offset[i] = offset_WT[i] - left_turn_time[1, i]*cycle_length
    elif sig[0, i] == 0 and sig[1, i] == 0:
        J[i] = np.array([[1, 2, 3, 4], [5, 6, 7, 8]])
    else:
        J[i] = np.array([[2, 1, 3, 4], [6, 5, 7, 8]])
        offset[i] = offset_WT[i] - left_turn_time[1, i]*cycle_length
    T[0, i] = (cycle_length * green_split[i, (J[i].flatten() - 1)]).reshape(2, 4)
    T[0, i] = round_and_adjust(T[0, i], cycle_length)
    RGYplan = getSumoTLSProgram(J[i], np.array([T[0, i]]), YR)
    # print(RGYplan)
    allRGYplan.append(RGYplan[0])
# print(allRGYplan)
print(J)
print(T)
offset = np.round(offset - offset[0]).cumsum()
print(offset)

vc = all_vc_ratio(traffic_demand)
vc = np.array([vc[i, J[i].flatten() - 1] for i in range(num_signals)])
print( vc / ((T[0].reshape(num_signals, -1)-YR) / cycle_length))

# 保存生成车流所需的routes
demand_to_route(traffic_demand)

# plot_signal_timing(num_signals, cycle_length, offset_WT, green_ratio, inbound_dev, progression_speed[0, :], queue_clearance_time[0, :], distance=np.cumsum(np.insert(distance[0, :], 0, 0)))

# 计算流量矩阵
V = np.zeros([num_signals, 8])
for i in range(num_signals):
    V[i] = np.array([volume(traffic_demand, 'I' + str(i+1), 'E', 'left'), volume(traffic_demand, 'I' + str(i+1), 'W', 'through'),
                     volume(traffic_demand, 'I' + str(i+1), 'S', 'left'), volume(traffic_demand, 'I' + str(i+1), 'N', 'through'),
                     volume(traffic_demand, 'I' + str(i+1), 'W', 'left'), volume(traffic_demand, 'I' + str(i+1), 'E', 'through'),
                     volume(traffic_demand, 'I' + str(i+1), 'N', 'left'), volume(traffic_demand, 'I' + str(i+1), 'S', 'through')])

# 将数据保存到文件
np.save(r'E:\workspace\python\BusRouteTSP\tools\result\offset.npy', offset)
np.save(r'E:\workspace\python\BusRouteTSP\tools\result\BG_PHASE_SEQ.npy', J)
np.save(r'E:\workspace\python\BusRouteTSP\tools\result\BG_PHASE_LEN.npy', T[0])
np.save(r'E:\workspace\python\BusRouteTSP\tools\result\volume.npy', V)
with open(r'E:\workspace\python\BusRouteTSP\tools\result\data.pkl', 'wb') as file:
    pickle.dump(allRGYplan, file)