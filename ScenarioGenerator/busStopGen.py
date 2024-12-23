import sys
rootPath = r'E:\workspace\python\BusRouteTSP'
sys.path.append(rootPath)
from ScenarioGenerator.nodeGen import posJunc

posJunc = posJunc + [500]
posSet = [[pos/2 for pos in posJunc],
          [pos/2 for pos in posJunc[::-1]]]
# print(f"posset:{posSet}")

def output_busStops(stops): 
    str_adds = ""
    #（基本）等间距分布，站点间距420m
    laneIDSet = [['np1_nt1_0', 'nt1_nt2_0', 'nt2_nt3_0', 'nt3_nt4_0', 'nt4_nt5_0', 'nt5_np12_0'],
                 ['np12_nt5_0', 'nt5_nt4_0', 'nt4_nt3_0', 'nt3_nt2_0', 'nt2_nt1_0', 'nt1_np1_0']]
    # posSet = [[100, 300, 70, 490, 350],
    #           [100, 320, 70, 490, 30]]
    # laneIDSet = [['np1_nt1_0', 'nt1_nt2_0', 'nt2_nt3_0', 'nt3_nt4_0', 'nt4_nt5_0'],
    #              ['np12_nt5_0', 'nt5_nt4_0', 'nt4_nt3_0', 'nt3_nt2_0', 'nt2_nt1_0']]
    
    for i in range(6):
        if i == 0:
            str_adds += stops % (('AtoB_' + str(i)), laneIDSet[0][i], posSet[0][i] - 20, posSet[0][i], "A2B") 
        else:
            str_adds += stops % (('AtoB_' + str(i)), laneIDSet[0][i], posSet[0][i] - 20 - 13.6*2, posSet[0][i] - 13.6*2, "A2B") 
    for i in range(5):
        str_adds += stops % (('BtoA_' + str(i)), laneIDSet[1][i], posSet[1][i] - 20 - 13.6, posSet[1][i] - 13.6, "B2A") 
    
    return str_adds