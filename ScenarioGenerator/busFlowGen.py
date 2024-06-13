import numpy as np

SIM_TIME = 7200
BUS_DEP_PERIOD = 4*60
BUS_DEP_ERR_STD = 30
BUS_DURATION = 20

def output_busFlow(trips, stops):
    str_adds = '   <vType id="BUS" vClass="bus" accel="2.6" decel="4.5" sigma="0" length="12" minGap="3" maxSpeed="70" color="red" guiShape="bus"/>\n'
    # 等间隔发车，发车间隔4min
    cnt = 0
    from_edge = ["np1_nt1", "np12_nt5"]
    to_edge = ["nt5_np12", "nt1_np1"]
    busStopSet = [["AtoB_0", "AtoB_1", "AtoB_2", "AtoB_3", "AtoB_4"],
                  ["BtoA_0", "BtoA_1", "BtoA_2", "BtoA_3", "BtoA_4"]]
    for i in range(0, SIM_TIME, BUS_DEP_PERIOD):
        time_err = np.random.normal(0, BUS_DEP_ERR_STD, 1)
        cnt = cnt + 1
        str_adds += trips % (str(cnt), max(0, i + time_err), from_edge[0], to_edge[0])
        for j in range(len(busStopSet[0])):
            str_adds += stops % (busStopSet[0][j], BUS_DURATION)
        str_adds += '   </trip>\n'

    return str_adds