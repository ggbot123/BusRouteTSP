import numpy as np

FLOW_MULTIPLIER = 1.0 #流量乘数
SIM_TIME = 7200
SIM_TIME_SEG = 1200

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
            if f.split("_")[0] not in t:
                edges.append([f, t])  
    # print(edges)
    times = range(0, SIM_TIME + 1, SIM_TIME_SEG) #1~7200s(2h),间隔1200s(20min)
    for i in range(len(times) - 1): 
        t_begin, t_end = times[i], times[i + 1] 
        for j in range(len(edges)): 
            str_flows += flow % (str(j) + '_' + str(i), edges[j][0], edges[j][1], t_begin, t_end, 0.005)
    str_flows += '</routes>\n'
    return str_flows

# def output_flows(flow, num_car_hourly): 
#     prob = '%.2f' % (num_car_hourly / float(3600)) #每秒发车率（0~1之间）
#     flow1 = '  <flow id="mf_%s" departPos="random_free" begin="%d" end="%d" probability="' + \
#             prob + '" type="type1">\n' + \
#             '    <route edges="%s"/>\n  </flow>\n'
#     routes = ['nt1_npc npc_nt5 nt5_np11', 
#               'nt1_npc npc_nt5 nt5_nt6 nt6_np12',
#               'nt4_nt5 nt5_np11',
#               'nt4_nt5 nt5_nt6 nt6_np12',
#               'nt1_nt2 nt2_np4',
#               'nt1_nt6 nt6_np13',
#               'nt1_npc npc_nt3 nt3_np6',
#               'nt1_npc npc_nt3 nt3_nt2 nt2_np5',
#               'nt4_nt3 nt3_np6',
#               'nt4_nt3 nt3_nt2 nt2_np5']
#     cases = [(3, 4, 5), (0, 3, 4), (1, 2, 5), (4, 5, 9), (5, 6, 9), (4, 7, 8)]
#     str_flows = '<routes>\n'
#     str_flows += '  <vType id="type1" length="5" accel="5" decel="10"/>\n' 

#     #source roads的输入流量（仿真2h）
#     flows = [[500, 100, 700, 800, 550, 550, 100, 200, 250, 250, 400, 800],
#              [600, 700, 100, 200, 50, 100, 1000, 500, 450, 150, 400, 200],
#              [100, 400, 400, 200, 600, 550, 100, 500, 500, 800, 400, 200],
#              [100, 200, 300, 300, 300, 400, 600, 600, 800, 500, 400, 300],
#              [600, 400, 400, 600, 800, 400, 300, 300, 300, 200, 250, 250]]
#     edges = ['%s_%s' % (x, 'nt1') for x in ['np1', 'np2', 'np3']] + \
#             ['%s_%s' % (x, 'nt4') for x in ['np8', 'np9']]
#     times = range(0, 7201, 1200) #1~7200s(2h),间隔1200s(20min)
#     times1 = range(0, 7201, 600) #1~7200s(2h),间隔600s(10min)
#     for i in range(len(times) - 1):
#         t_begin, t_end = times[i], times[i + 1]
#         for c in cases[i]:
#             name = str(c) + '_' + str(i)
#             str_flows += flow1 % (name, t_begin, t_end, routes[c])
#         for i0 in [i * 2, i * 2 + 1]: 
#             t_begin, t_end = times1[i0], times1[i0 + 1] 
#             for j in range(5): 
#                 str_flows += flow % (str(j) + '_' + str(i0), edges[j], t_begin, t_end,
#                                      int(flows[j][i0] * FLOW_MULTIPLIER))
#     str_flows += '</routes>\n'
#     return str_flows