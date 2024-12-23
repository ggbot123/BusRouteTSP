import numpy as np
import sys
rootPath = r'E:\workspace\python\BusRouteTSP'
sys.path.append(rootPath)
from ScenarioGenerator.busStopGen import posSet
from ScenarioGenerator.nodeGen import posJunc

SIM_TIME = 7200
ARRIVAL_RATE = 3/(2*60)
V_AVG = 10
POS_JUNC = np.array(posJunc).cumsum()
POS_STOP = np.concatenate([[0], POS_JUNC]) + np.array(posSet[0])

def output_personFlow(person, person_walks, person_rides):
    str_adds = '   <vType id="PED" vClass="pedestrian" color="yellow" guiShape="pedestrian"/>\n'
    # 乘客均匀到达
    cnt = 0
    # from_edge = [['np1_nt1', 'nt1_nt2', 'nt2_nt3', 'nt3_nt4', 'nt4_nt5'],
    #              ['np12_nt5', 'nt5_nt4', 'nt4_nt3', 'nt3_nt2', 'nt2_nt1']]
    from_edge = [['np1_nt1', 'nt1_nt2', 'nt2_nt3', 'nt3_nt4', 'nt4_nt5']]
    from_pos = posSet
    # to_edge = ["AtoB_4", "BtoA_4"]
    to_edge = ["AtoB_4"]
    # busStopSet = [["AtoB_0", "AtoB_1", "AtoB_2", "AtoB_3", "AtoB_4"],
    #               ["BtoA_0", "BtoA_1", "BtoA_2", "BtoA_3", "BtoA_4"]]
    busStopSet = [["AtoB_0", "AtoB_1", "AtoB_2", "AtoB_3", "AtoB_4"]]

    start_i = (POS_STOP - POS_STOP[0])/V_AVG
    print(start_i)
    for j in range(len(from_edge[0])):
        for i in range(int(start_i[j]), SIM_TIME, int(1/ARRIVAL_RATE)):
            cnt = cnt + 1
            str_adds += (person % (str(cnt), i, from_pos[0][j]) + person_walks % (from_edge[0][j], busStopSet[0][j]) + person_rides % (to_edge[0], "A2B"))
            str_adds += '   </person>\n'

    return str_adds