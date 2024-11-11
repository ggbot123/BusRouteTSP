import gurobipy as gb
import numpy as np
from plotTSTP import plotTSTP
import datetime

rootPath = r'E:\workspace\python\BusRouteTSP'
def optimize(**kwargs):
    for key, value in kwargs.items():
        try:
            exec(f'{key} = {repr(value)}', globals())
        except NameError:
            if key == 'tls_pad_T':
                tls_pad_T = kwargs[key]
            elif key == 'tls_pad_t':
                tls_pad_t = kwargs[key]
            elif key == 'j_curr':
                j_curr = kwargs[key]
            else:
                exec(f'{key} = np.{repr(value)}', globals())
    FILENAME = f'{rootPath}\\RouteTSP\\result\\Rolling\\%d.png' % cnt
    M = 100000
    INI = -10000

    t_ini = []
    for i in range(I):
        # 0-kCurr周期起始时刻距离规划时刻的时长
        t_ini.append(np.flip(np.flip(np.sum(np.array(tls_pad_T[i])[:, 0, :], axis=-1)).cumsum()))
        OF[i] = 0
    t_lb = -t_ini[0][0]

    # if len(T_opt_plan) > 0:
    #     t_ini = np.flip(T_opt_plan[:, :, 0, :].sum(axis=-1), axis=0).cumsum(axis=0)
    #     t_lb = min([t_lb, -t_ini[-1][0]])

    # 创建一个新的模型 调用内置函数Model()创建模型
    model = gb.Model()
    model.setParam('OutputFlag', 0)
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
            # T_app[n, i].start = L_app[i]/12
            # T_dep[n, i].start = L_dep[i]/12
        t_arr[n, I] = model.addVar(name=f't_arr{n}{I}')
    for i in range(I):
        for k in range(K + K_ini):
            for j in np.concatenate((J[i][0], J[i][1])):
                t[i, j, k] = model.addVar(vtype=gb.GRB.INTEGER, name=f't{i}{j}{k}', lb=t_lb)
                g[i, j, k] = model.addVar(vtype=gb.GRB.INTEGER, name=f'g{i}{j}{k}')
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
            for k in range(K + K_ini):
                objective_expr += w_c * y[i, j, k]
        for n in range(N):
            objective_expr += w_b * (t_dev[n, i] + 0.0001*d[n, i])
    model.setObjective(objective_expr, gb.GRB.MINIMIZE)

    # 设置约束
    for i in range(I):
        for l in [0, 1]:
            for k in range(K_ini + K):
                for j_ind in range(len(J[i][l])):
                    # Ring-Barrier结构约束
                    j = J[i][l][j_ind]
                    if (k < len(tls_pad_T[i]) - 1) or ((k == len(tls_pad_T[i]) - 1) and (j_ind < j_curr[i][l])):
                        # 已经结束的周期/相位，采用原有的信号配时
                        model.addConstr(t[i, j, k] == -t_ini[i][k] + tls_pad_t[i][k][l][j_ind])
                        model.addConstr(g[i, j, k] == tls_pad_T[i][k][l][j_ind] - YR)
                        # if len(T_opt_plan) > 0:
                        #     model.addConstr(t[i, j, k] == -t_ini[K_ini-k-1][0] + t_opt_plan[k][i][l][j_ind])
                        #     model.addConstr(g[i, j, k] == T_opt_plan[k][i][l][j_ind] - YR)
                        # else:
                        #     model.addConstr(t[i, j, k] == -(K_ini - k)*C + t_opt[i][l][j_ind])
                        #     model.addConstr(g[i, j, k] == T_opt[i][l][j_ind] - YR)
                    else:
                        if (k == len(tls_pad_T[i]) - 1 and j_ind == j_curr[i][l]):
                            model.addConstr(t[i, j, k] == -t_ini[i][k] + tls_pad_t[i][k][l][j_ind])
                            model.addConstr(g[i, j, k] >= tls_pad_T[i][k][l][j_ind] - YR)
                        if j not in J_last[i]:
                            j_next = J[i][l][j_ind + 1]
                            model.addConstr(t[i, j_next, k] == t[i, j, k] + g[i, j, k] + YR)
                        else:
                            j_next = J[i][l][0]
                            if k == K + K_ini - 1:
                                model.addConstr(t_ini[i][0] + t[i, j, k] + g[i, j, k] + YR == (K + K_ini)*C)
                            else:
                                model.addConstr(t[i, j_next, k + 1] == t[i, j, k] + g[i, j, k] + YR)
                        if j in J_barrier[i][0]:
                            j_oth = J_barrier[i][1][np.where(J_barrier[i][0] == j)][0]
                            model.addConstr(t[i, j, k] == t[i, j_oth, k])                   
                        # 目标函数辅助约束
                        model.addConstr(y[i, j, k] >= T_opt[i][np.where(J[i] == j)][0] - g[i, j, k] - YR)
                        model.addConstr(y[i, j, k] >= 0)
                        # model.addConstr(y[i, j, k] >= g[i, j, k] + YR - T_opt[i][np.where(J[i] == j)][0])
                        # 最小绿时约束
                        model.addConstr(g[i, j, k] >= G_min)
                        # model.addConstr(g[i, j, k] >= V_ij[i][j-1]*C/(S_ij[i][j-1]*Xc))

                    

    # 公交运行过程约束
    for n in range(N):
        # 初始化
        nextStopInd = np.where(p_bus_0[n] < POS_stop)[0][0] # 下一站点索引，若后面无站点，该车将不会进入算法
        nextIntInd = np.where(p_bus_0[n] < POS)[0][0] if len(np.where(p_bus_0[n] < POS)[0]) > 0 else I  # 下一交叉口索引，若后面无交叉口，则填I
        # case 1：还没发车
        if p_bus_0[n] == INI or nextStopInd == 0:
            model.addConstr(t_arr[n, 0] == T_dep_0[n])
        # case 2：处于交叉口-下一站点之间
        elif nextIntInd == nextStopInd:
            model.addConstr(t_arr[n, nextStopInd] == T_dep[n, nextStopInd-1])
            model.addConstr(T_dep[n, nextStopInd-1] >= (POS_stop[nextStopInd] - p_bus_0[n])/v_max)
        # case 3: 处于上一站点-交叉口之间
        else:
            model.addConstr(r[n, nextIntInd] == T_app[n, nextIntInd])
            model.addConstr(T_app[n, nextIntInd] >= (POS[nextIntInd] - p_bus_0[n])/v_max)

        for i in range(nextIntInd, I):
            # 公交行驶时间模型约束
            model.addConstr(r[n, i] == t_arr[n, i] + T_app[n, i])
            model.addConstr(sum(theta[i, j, k, n] for j in J_bus for k in range(K + K_ini)) == 1) # 采用theta表征公交到达交叉口时刻对应的信号周期
            for j in J_bus:
                for k in range(K + K_ini):
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
            if i == nextIntInd:
                # case 3 这个约束前面已经加过了，这里不加
                if p_bus_0[n] == INI or nextStopInd == 0 or nextIntInd == nextStopInd:
                    model.addConstr(T_app[n, i] >= L_app[i]/v_max)
            else:
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
    model.write(f'{rootPath}\\RouteTSP\\log\\BusRouteTSP.lp')
    if model.status == gb.GRB.OPTIMAL:
        T_sol = []
        Traj_sol = [[[], []] for _ in range(N)]
        for i in range(I):
            T_sol_i = []
            for k in range((len(tls_pad_T[i]) - 1), (K + K_ini)):
                T_sol_i.append(np.array([g[i, j, k].x + YR for j in np.concatenate((J[i][0], J[i][1]))]).reshape(J[i].shape))
            for l in [0, 1]:
                T_sol_i[0][l][0:(j_curr[i][l] + 1)] -= tls_pad_T[i][-1][l][0:(j_curr[i][l] + 1)]
            T_sol.append(T_sol_i)
        for n in range(N):
            traj_t, traj_x = [], []
            nextStopInd = np.where(p_bus_0[n] < POS_stop)[0][0] # 下一站点索引，若后面无站点，该车将不会进入算法
            nextIntInd = np.where(p_bus_0[n] < POS)[0][0] if len(np.where(p_bus_0[n] < POS)[0]) > 0 else I  # 下一交叉口索引，若后面无交叉口，则填I
            # 边界条件处理
            if nextStopInd != 0:
                traj_t += [0]
                traj_x += [p_bus_0[n]]
            else:
                if p_bus_0[n] == INI:
                    traj_t += [T_dep_0[n]-(POS_stop[0]/v_avg), T_dep_0[n]]
                    traj_x += [0, POS_stop[0]]
                else:
                    traj_t += [0, t_arr[n, 0].x]
                    traj_x += [p_bus_0[n], POS_stop[0]]
            # 填写traj
            for i in range(nextIntInd, I):
                if d_[n, i].x <= 0:
                    traj_t += [t_arr[n, i].x+T_app[n, i].x, t_arr[n, i+1].x]
                    traj_x += [POS[i], POS_stop[i+1]]
                else:
                    traj_t += [t_arr[n, i].x+T_app[n, i].x, t_arr[n, i+1].x-T_dep[n, i].x, t_arr[n, i+1].x]
                    traj_x += [POS[i], POS[i], POS_stop[i+1]]
            Traj_sol[n][0] = np.append(Traj_sol[n][0], traj_t)
            Traj_sol[n][1] = np.append(Traj_sol[n][1], traj_x)
        # plotTSTP(FILENAME, J, T_sol, t_arr_plan, Traj_sol, 2, plotShow=False)
    else:
        # 计算 IIS
        model.computeIIS()
        model.write(f'{rootPath}\\RouteTSP\\log\\model.ilp')  # 可以选择将 IIS 写入文件

        # 输出哪些约束导致不可行性
        for constr in model.getConstrs():
            if constr.IISConstr:  # 标记为 IIS 的约束
                print(f"Infeasible constraint: {constr.ConstrName}")
        print('未找到最优解。')

    return T_sol, Traj_sol