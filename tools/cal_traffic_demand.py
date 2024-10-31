import json

## 计算连续交叉口输入流量
junctions = {'I1': {'W': [1593, [0.17, 0.78, 0.05]],
                    'N': [251, [0.15, 0.75, 0.1]],
                    'S': [501, [0.55, 0.3, 0.15]],
                    'E': [-1, [0.17, 0.78, 0.05]]
                    },
             'I2': {'W': [-1, [0.17, 0.78, 0.05]],
                    'N': [494, [0.2, 0.7, 0.1]],
                    'S': [498, [0.2, 0.7, 0.1]],
                    'E': [-1, [0.17, 0.78, 0.05]]
                    },
             'I3': {'W': [-1, [0.17, 0.78, 0.05]],
                    'N': [691, [0.2, 0.7, 0.1]],
                    'S': [538, [0.2, 0.7, 0.1]],
                    'E': [-1, [0.17, 0.78, 0.05]]
                    },
             'I4': {'W': [-1, [0.17, 0.78, 0.05]],
                    'N': [431, [0.15, 0.75, 0.1]],
                    'S': [484, [0.15, 0.75, 0.1]],
                    'E': [-1, [0.17, 0.78, 0.05]]
                    },
             'I5': {'W': [-1, [0.17, 0.78, 0.05]],
                    'N': [515, [0.25, 0.65, 0.1]],
                    'S': [462, [0.25, 0.65, 0.1]],
                    'E': [1593, [0.17, 0.78, 0.05]]
                    }
            }
def volume(i, dir, move):
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


def cal_flow(i, dir):
    if dir == 'W':
        if i == 1:
            return junctions['I1']['W'][0]
        else:
            up_junc = 'I' + str(i-1)
            junctions['I' + str(i)]['W'][0] = cal_flow(i-1, dir)*junctions[up_junc][dir][1][1] + volume(up_junc, 'N', 'left') + volume(up_junc, 'S', 'right')
            return junctions['I' + str(i)]['W'][0]

    elif dir == 'E':
        if i == 5:
            return junctions['I5']['E'][0]
        else:
            up_junc = 'I' + str(i+1)
            junctions['I' + str(i)]['E'][0] = cal_flow(i+1, dir)*junctions[up_junc][dir][1][1] + volume(up_junc, 'N', 'right') + volume(up_junc, 'S', 'left')
            return junctions['I' + str(i)]['E'][0]

print(cal_flow(5, 'W'))
print(cal_flow(1, 'E'))

print(junctions)
with open('./result/traffic_demand.json', 'w') as json_file:
    json.dump(junctions, json_file, indent=4)