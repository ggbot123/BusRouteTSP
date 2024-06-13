def output_busStops(stops): 
    str_adds = ""
    #（基本）等间距分布，站点间距420m
    laneIDSet = [['np1_nt1_0', 'nt1_nt2_0', 'nt3_nt4_0', 'nt3_nt4_0', 'nt4_nt5_0'],
                 ['np12_nt5_0', 'nt5_nt4_0', 'nt4_nt3_0', 'nt4_nt3_0', 'nt2_nt1_0']]
    # posSet = [[100, 300, 70, 490, 350],
    #           [100, 320, 70, 490, 30]]
    # laneIDSet = [['np1_nt1_0', 'nt1_nt2_0', 'nt2_nt3_0', 'nt3_nt4_0', 'nt4_nt5_0'],
    #              ['np12_nt5_0', 'nt5_nt4_0', 'nt4_nt3_0', 'nt3_nt2_0', 'nt2_nt1_0']]
    posSet = [[100, 175, 160, 280, 335],
              [100, 335, 280, 160, 175]]

    for i in range(5):
        str_adds += stops % (('AtoB_' + str(i)), laneIDSet[0][i], posSet[0][i], posSet[0][i] + 20, "A2B") 
    for i in range(5):
        str_adds += stops % (('BtoA_' + str(i)), laneIDSet[1][i], posSet[1][i], posSet[1][i] + 20, "B2A") 
    
    return str_adds