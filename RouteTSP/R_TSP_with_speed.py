import gurobipy as gb
import numpy as np
from plotTSTP import plotTSTP
import datetime

FILENAME = './result/R-TSPwithSpeed.png'
# 基本参数
I = 5
K = 10
N = 3
# 背景信号参数
C = 100
J = np.array([[[1, 2, 3, 4], [6, 5, 7, 8]], 
              [[2, 1, 4, 3], [5, 6, 8, 7]],
              [[1, 2, 4, 3], [6, 5, 8, 7]],
              [[2, 1, 4, 3], [6, 5, 8, 7]],
              [[2, 1, 4, 3], [5, 6, 7, 8]]])
J_first = J[:, :, 0]
J_last = J[:, :, -1]
J_barrier = J[:, :, [0, 2]]
J_coord = [2, 6]
J_bus = [2]
T_opt = np.array([[[19, 40, 16, 25], [44, 15, 19, 22]],
                  [[44, 16, 29, 11], [11, 49, 22, 18]],
                  [[19, 48, 20, 13], [51, 16, 20, 13]],
                  [[42, 22, 20, 16], [48, 16, 26, 10]],
                  [[42, 21, 27, 10], [15, 48, 16, 21]]])
assert np.array_equal(T_opt.sum(axis=2), np.ones(T_opt.shape[0:2]) * C), "相位时长与信号周期不相同\n"
t_opt = np.insert(np.delete(T_opt, -1, axis=2), 0, 0, axis=2).cumsum(axis=2)
OF = np.array([0, 59, 56, 16, 61]).cumsum()
POS = np.array([200, 350, 320, 560, 670]).cumsum()
YR = 4  # 损失时间
G_min = 10 # 最小绿灯时间
Xc = 0.9
# 交叉口各相位流量
V_ij = 0.5* np.array([[156, 858, 125, 530, 109, 1092, 140, 390],
                 [100, 942, 70, 550, 50, 1220, 150, 450],
                 [130, 982, 140, 400, 180, 1470, 100, 250],
                 [180, 882, 300, 340, 130, 1210, 20, 500],
                 [150, 592, 80, 720, 160, 1270, 240, 250]])
# 交叉口各相位饱和流率
S_ij = np.array([[1550, 2000, 1550, 2000, 1550, 2000, 1550, 2000],
                 [1550, 2000, 1550, 2000, 1550, 2000, 1550, 2000],
                 [1550, 2000, 1550, 2000, 1550, 2000, 1550, 2000],
                 [1550, 2000, 1550, 2000, 1550, 2000, 1550, 2000],
                 [1550, 2000, 1550, 2000, 1550, 2000, 1550, 2000]])
# 公交车站位置(m)
POS_stop = [200/2, 200+350/2, 200+350+320/2, 200+350+320+560/2, 200+350+320+560+670/2, 200+350+320+560+670+200/2]
L_app = POS - POS_stop[:I]
L_dep = POS_stop[1:] - POS
# 到站时刻表(不包括首发站)
v_avg = 12
t_arr_plan = np.zeros([N, I])
for n in range(N):
    # 发车时刻表
    H = 4*60
    T_dep_0 = np.arange(200, 200 + N*H, H)
    t_arr_plan[n] = T_dep_0[n] + np.array((L_app + L_dep)/v_avg).cumsum()
# 公交最大车速(m/s)
v_max = 15
# 排队时间(s)
Q_ij = 0
# 辅助变量
M = 100000

if __name__ == '__main__':
    # 创建一个新的模型 调用内置函数Model()创建模型
    model = gb.Model()
    # model.setParam('LogFile', '.\\log\\gurobi_%s.log' % datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d-%H-%M'))  # 将日志输出到gurobi.log文件
    # 创建变量
    t = {}
    g = {}
    y = {}
    theta = {}
    T_app = {}
    T_dep = {}
    beta = {}
    t_dev = {}
    t_arr = {}
    d = {}
    d_ = {}
    r = {}
    for n in range(N):
        for i in range(I):
            T_app[n, i] = model.addVar(name=f'T_app{n}{i}')
            T_dep[n, i] = model.addVar(name=f'T_dep{n}{i}')
            beta[n, i] = model.addVar(vtype=gb.GRB.BINARY, name=f'beta{n}{i}')
            t_dev[n, i] = model.addVar(name=f't_dev{n}{i}')
            t_arr[n, i] = model.addVar(name=f't_arr{n}{i}')
            r[n, i] = model.addVar(name=f'r{n}{i}')
            d[n, i] = model.addVar(name=f'd{n}{i}')
            d_[n, i] = model.addVar(name=f'd_{n}{i}', lb=-float('inf'))   # 辅助变量d_
            # 初始解
            T_app[n, i].start = L_app[i]/12
            T_dep[n, i].start = L_dep[i]/12
        t_arr[n, I] = model.addVar(name=f't_arr{n}{I}')
    for i in range(I):
        for k in range(K):
            for j in np.concatenate((J[i][0], J[i][1])):
                t[i, j, k] = model.addVar(name=f't{i}{j}{k}')
                g[i, j, k] = model.addVar(name=f'g{i}{j}{k}')
                y[i, j, k] = model.addVar(name=f'y{i}{j}{k}')
                # 初始解
                t[i, j, k].start = t_opt[i][np.where(J[i] == j)][0] + k*C
                g[i, j, k].start = T_opt[i][np.where(J[i] == j)][0] - YR
                y[i, j, k].start = 0
            for j in J_bus:
                for n in range(N):
                    theta[i, j, k, n] = model.addVar(vtype=gb.GRB.BINARY, name=f'theta{i}{j}{k}{n}')

    # 设置目标函数
    w_c = 0.5
    w_b = 0.5
    objective_expr = gb.LinExpr()
    for i in range(I):
        for j in range(1, 9):
            for k in range(K):
                objective_expr += w_c * y[i, j, k]
        for n in range(N):
            objective_expr += w_b * (t_dev[n, i] + 0.0001*d[n, i])
    model.setObjective(objective_expr, gb.GRB.MINIMIZE)

    # 设置约束
    for i in range(I):
        for l in [0, 1]:
            for k in range(K):
                for j_ind in range(len(J[i][l])):
                    # Ring-Barrier结构约束
                    j = J[i][l][j_ind]
                    if j in J_first[i] and k == 0:
                        # 信号起始时刻（各交叉口local time）
                        model.addConstr(t[i, j, k] == 0)                    
                    if j not in J_last[i]:
                        j_next = J[i][l][j_ind + 1]
                        model.addConstr(t[i, j_next, k] == t[i, j, k] + g[i, j, k] + YR)
                    else:
                        j_next = J[i][l][0]
                        if k == K - 1:
                            model.addConstr(t[i, j, k] + g[i, j, k] + YR == K*C)
                        else:
                            model.addConstr(t[i, j_next, k + 1] == t[i, j, k] + g[i, j, k] + YR)                    
                    if j in J_barrier[i][0]:
                        j_oth = J_barrier[i][1][np.where(J_barrier[i][0] == j)][0]
                        model.addConstr(t[i, j, k] == t[i, j_oth, k])                   
                    # 目标函数辅助约束
                    model.addConstr(y[i, j, k] >= T_opt[i][np.where(J[i] == j)][0] - g[i, j, k] - YR)
                    model.addConstr(y[i, j, k] >= 0)
                    # print(V_ij[i][j_ind]*C/(S_ij[i][j_ind]*Xc))
                    # 最小绿时约束
                    model.addConstr(g[i, j, k] >= G_min)
                    model.addConstr(g[i, j, k] >= V_ij[i][j-1]*C/(S_ij[i][j-1]*Xc))

                    

    # 公交运行过程约束
    for n in range(N):
        model.addConstr(t_arr[n, 0] == T_dep_0[n])
        for i in range(I):
            # 公交行驶时间模型约束
            model.addConstr(r[n, i] == t_arr[n, i] + T_app[n, i])
            model.addConstr(sum(theta[i, j, k, n] for j in J_bus for k in range(K)) == 1) # 采用theta表征公交到达交叉口时刻对应的信号周期
            for j in J_bus:
                for k in range(K):
                    # 采用theta表征公交到达交叉口时刻对应的信号周期
                    if k > 0:
                        model.addConstr(r[n, i] >= OF[i] + t[i, j, k - 1] + g[i, j, k - 1] - (1 - theta[i, j, k, n])*M)
                    model.addConstr(r[n, i] <= OF[i] + t[i, j, k] + g[i, j, k] + (1 - theta[i, j, k, n])*M)
                    # 交叉口公交车辆延误
                    model.addConstr(d_[n, i] >= OF[i] + t[i, j, k] + Q_ij - r[n, i] - (1 - theta[i, j, k, n])*M)
                    model.addConstr(d_[n, i] <= OF[i] + t[i, j, k] + Q_ij - r[n, i] + (1 - theta[i, j, k, n])*M)
            # 辅助约束，保证延误非负
            # model.addConstr(d_[n, i] <= (1 - beta[n, i])*M)
            # model.addConstr(d_[n, i] >= -beta[n, i]*M)
            # model.addConstr(d[n, i] >= d_[n, i] - beta[n, i]*M)
            model.addConstr(d[n, i] >= d_[n, i])
            model.addConstr(d[n, i] >= 0)
            model.addConstr(r[n, i] + T_dep[n, i] + d[n, i] == t_arr[n, i + 1])
            # 公交车速约束
            model.addConstr(T_app[n, i] >= L_app[i]/v_max)
            model.addConstr(T_dep[n, i] >= L_dep[i]/v_max)
            # 目标函数辅助约束(晚点时刻)
            # model.addConstr(t_dev[n, i] >= t_arr[n, i + 1] - t_arr_plan[n][i])
            # model.addConstr(t_dev[n, i] >= 0)
            # 目标函数辅助约束(偏差时刻)
            model.addConstr(t_dev[n, i] >= t_arr[n, i + 1] - t_arr_plan[n][i])
            model.addConstr(t_dev[n, i] >= -(t_arr[n, i + 1] - t_arr_plan[n][i]))
            


    # 执行优化函数
    model.optimize()

    # 检查求解状态
    if model.status == gb.GRB.OPTIMAL:
        # 打印最优解
        # print('最优解是:')
        # for i in range(I):
        #     for k in range(K):
        #         for j in range(1, 9):
        #             print(f'y{i}{j}{k}: {y[i, j, k].x}')
        #             print(f't{i}{j}{k}: {t[i, j, k].x}')
        #             print(f'g{i}{j}{k}: {g[i, j, k].x}')
        # for n in range(N):
        #     for i in range(I + 1):    
        #         print(f't_arr{n}{i}: {t_arr[n, i].x}')
        print('社会车辆目标函数值:\n%s\n', sum(y[i, j, k].x for i in range(I) for j in range(1, 9) for k in range(K)))
        print('公交车辆目标函数值:\n%s\n', sum(t_dev[n, i].x for i in range(I) for n in range(N)))
        # 绘制信号方案&公交车辆轨迹图
        T_sol = np.zeros([K] + list(J.shape))
        Traj_sol = np.array([])
        for k in range(K):
            for i in range(I):    
                T_sol[k][i] = np.array([g[i, j, k].x + YR for j in np.concatenate((J[i][0], J[i][1]))]).reshape(J[i].shape)
        for n in range(N):
            traj_t, traj_x = [], []
            for i in range(I):
                traj_t += [t_arr[n, i].x, t_arr[n, i].x+T_app[n, i].x, t_arr[n, i+1].x-T_dep[n, i].x]
                traj_x += [POS_stop[i], POS[i], POS[i]]
            traj_t += [t_arr[n, I].x]
            traj_x += [POS_stop[I]]
            Traj_sol = np.append(Traj_sol, [traj_t, traj_x])
        plotTSTP(FILENAME, J, T_sol, np.concatenate((T_dep_0.reshape([N, -1]), t_arr_plan), axis=1), Traj_sol.reshape([N, 2, -1]), 2)
    else:
        print('未找到最优解。')