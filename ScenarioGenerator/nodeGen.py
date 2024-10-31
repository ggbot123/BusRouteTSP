import numpy as np

#路段长度参数
# posJunc = [500, 350, 320, 560, 670]
posJunc = [500, 700, 640, 560, 670]
L_in, L0, L1, L2, L3 = posJunc
L_end = 500

#定义节点node
def output_nodes(node):
    str_nodes = '<nodes>\n'  #加上 The opening tag
    # 定义信号控制节点
    # 以nt1号交叉口为原点
    str_nodes += node % ('nt1', 0, 0, 'traffic_light') 
    str_nodes += node % ('nt2', L0, 0, 'traffic_light')
    str_nodes += node % ('nt3', L0 + L1, 0, 'traffic_light')
    str_nodes += node % ('nt4', L0 + L1 + L2, 0, 'traffic_light')
    str_nodes += node % ('nt5', L0 + L1 + L2 + L3, 0, 'traffic_light')
    # 定义主路优先放行节点
    str_nodes += node % ('np1', -L_in, 0, 'priority')
    str_nodes += node % ('np2', 0, L_end, 'priority')
    str_nodes += node % ('np3', 0, -L_end, 'priority')
    str_nodes += node % ('np4', L0, L_end, 'priority')
    str_nodes += node % ('np5', L0, -L_end, 'priority')
    str_nodes += node % ('np6', L0 + L1, L_end, 'priority')
    str_nodes += node % ('np7', L0 + L1, -L_end, 'priority')
    str_nodes += node % ('np8', L0 + L1 + L2, L_end, 'priority')
    str_nodes += node % ('np9', L0 + L1 + L2, -L_end, 'priority')
    str_nodes += node % ('np10', L0 + L1 + L2 + L3, L_end, 'priority')
    str_nodes += node % ('np11', L0 + L1 + L2 + L3, -L_end, 'priority')
    str_nodes += node % ('np12', L0 + L1 + L2 + L3 + L_in, 0, 'priority')
    str_nodes += '</nodes>\n' #加上 The closing tag
    return str_nodes