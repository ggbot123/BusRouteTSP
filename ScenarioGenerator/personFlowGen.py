import numpy as np
import sys
rootPath = r'E:\workspace\python\BusRouteTSP'
sys.path.append(rootPath)
from ScenarioGenerator.busStopGen import posSet

SIM_TIME = 7200
ARRIVAL_RATE = 10/(4*60)

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

    for i in range(0, SIM_TIME, int(1/ARRIVAL_RATE)):
        for j in range(len(from_edge[0])):
            cnt = cnt + 1
            str_adds += (person % (str(cnt), i, from_pos[0][j]) + person_walks % (from_edge[0][j], busStopSet[0][j]) + person_rides % (to_edge[0], "A2B"))
            str_adds += '   </person>\n'

    return str_adds