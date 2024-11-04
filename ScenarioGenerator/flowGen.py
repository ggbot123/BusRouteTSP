import numpy as np
import pandas as pd

FLOW_MULTIPLIER = 1.0 #流量乘数
SIM_TIME = 7200
SIM_TIME_SEG = 1200

OD_volume = pd.read_csv(r'E:\workspace\python\BusRouteTSP\tools\result\OD_volume.csv', index_col=0)
print(OD_volume)

def output_flows(flow): 
    str_flows = '<routes>\n'
    str_flows += '  <vType id="type1" length="5" accel="5" decel="10"/>\n' 
    edges = []
    flows = []
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
    for f in from_edges:
        for t in to_edges:
            if f.split("_")[0] != t.split("_")[1]:
                edges.append([f, t])  
    # print(edges)
    times = range(0, SIM_TIME + 1, SIM_TIME_SEG) #1~7200s(2h),间隔1200s(20min)
    for i in range(len(times) - 1): 
        t_begin, t_end = times[i], times[i + 1] 
        for j in range(len(edges)):
            prob = OD_volume.loc[edges[j][0], edges[j][1]]/3600
            if prob >= 0.001:
                str_flows += flow % (str(j) + '_' + str(i), edges[j][0], edges[j][1], t_begin, t_end, 'free', prob)
            # print(prob)
            # if edges[j][0] == from_edges[0] or edges[j][0] == from_edges[-1]:
                # str_flows += flow % (str(j) + '_' + str(i), edges[j][0], edges[j][1], t_begin, t_end, 0.015)
            # else:
            #     str_flows += flow % (str(j) + '_' + str(i), edges[j][0], edges[j][1], t_begin, t_end, 0.015/(len(to_edges) - 1))
    str_flows += '</routes>\n'
    return str_flows