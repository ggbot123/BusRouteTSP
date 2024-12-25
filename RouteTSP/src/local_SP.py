import numpy as np
import gurobipy as gb
import sys
rootPath = r'E:\workspace\python\BusRouteTSP'
sys.path.append(rootPath)
from ScenarioGenerator.busStopGen import posSet
from ScenarioGenerator.nodeGen import posJunc
from tools import round_and_adjust

def local_SP(id, tlsPlan, t_arr, t_arr_next, theta, busInd, **kwargs):
    for key, value in kwargs.items():
        try:
            exec(f'{key} = {repr(value)}', globals())
        except NameError:
            if key == 'tls_pad_T':
                tls_pad_T = kwargs[key][id]
            elif key == 'tls_pad_t':
                tls_pad_t = kwargs[key][id]
            elif key == 'j_curr':
                j_curr = kwargs[key][id]
            elif key == 'tls_curr_T':
                tls_curr_T = kwargs[key][id]
            elif key == 'tls_curr_t':
                tls_curr_t = kwargs[key][id]
            elif key == 'p_bus_0':
                p_bus_0 = kwargs[key][busInd]
            else:
                exec(f'{key} = np.{repr(value)}', globals())
    N = len(busInd)
    k_tar = np.where(np.abs(theta - 1) < 0.01)[0]
    M = 100000
    INI = -10000
    t_ini = np.flip(np.flip(np.sum(np.array(tls_pad_T)[:, 0, :], axis=-1)).cumsum())
    t_lb = -t_ini[0]
    tls_pad_t = np.insert(np.delete(tls_pad_T, -1, axis=-1), 0, 0, axis=-1).cumsum(axis=-1)

    # 随机变量样本生成：1000个样本，每个样本是长度为N的向量，服从N(0, 1)
    num_samples = 50  # 样本数量
    np.random.seed(0)
    Ts_means = T_board[id]
    Ts_devs = 10
    Z = np.random.normal(0, 1, (num_samples, I + 1))
    # Ts = np.maximum(Ts_means + Z * Ts_devs, 0*np.ones_like(Ts_means))
    Ts = np.maximum(Ts_means + Z * Ts_devs, 10*np.ones_like(Ts_means))
    Ts = np.minimum(Ts, 30*np.ones_like(Ts_means))

    # 创建Gurobi模型
    model = gb.Model("Local_SP")
    model.setParam('OutputFlag', 0)

    # 决策变量-期望到达交叉口时刻r(n)
    r = model.addVars(N, name="r")

    # 决策变量-信号配时t(j,k) & g(j,k)
    t = {}
    g = {}
    y = {}
    for k in range(K + K_ini):
        for j in np.concatenate((J[id][0], J[id][1])):
            t[j, k] = model.addVar(name=f't{j}{k}', lb=t_lb)
            g[j, k] = model.addVar(name=f'g{j}{k}', lb=0)
            y[j, k] = model.addVar(name=f'y{j}{k}', lb=0, ub=10)
            # y[j, k] = model.addVar(name=f'y{j}{k}', lb=-gb.GRB.INFINITY)
            # y[j, k] = model.addVar(name=f'y{j}{k}', lb=-5, ub=5)
            # y[j, k] = model.addVar(name=f'y{j}{k}', lb=-10, ub=10)

    # （辅助）决策变量-实际到达交叉口时刻 r_act(n)
    r_act = model.addVars(num_samples, N, lb=-gb.GRB.INFINITY, name="r_act")
    t_arr_next_ = model.addVars(num_samples, N,  lb=-gb.GRB.INFINITY, name=f't_arr_next_')
    t_dev = model.addVars(num_samples, N, name=f't_dev')
    beta1 = model.addVars(num_samples, N,  vtype=gb.GRB.BINARY, name=f'beta1')
    beta2 = model.addVars(num_samples, N,  vtype=gb.GRB.BINARY, name=f'beta2')
    beta3 = model.addVars(num_samples, N,  vtype=gb.GRB.BINARY, name=f'beta3')

    # 0-1变量 z(s) 表示每个样本是否满足机会约束
    # z = model.addVars(num_samples, vtype=gb.GRB.BINARY, name="z")

    # 设置目标函数
    wc = 0.3
    wb = 0.7
    objective_expr = gb.LinExpr()
    for j in range(1, 9):
        for k in range(K + K_ini):
            objective_expr += wc * y[j, k]
    for s in range(num_samples):
        for n in range(N):
            objective_expr += wb * (t_dev[s, n])/num_samples
    
    model.setObjective(objective_expr, gb.GRB.MINIMIZE)

    # 信号灯结构约束
    for l in [0, 1]:
        for k in range(K_ini + K):
            for j_ind in range(len(J[id][l])):
                # Ring-Barrier结构约束
                j = J[id][l][j_ind]
                if (k < len(tls_pad_T) - 1) or ((k == len(tls_pad_T) - 1) and (j_ind < j_curr[l])):
                    # 已经结束的周期/相位，采用原有的信号配时
                    model.addConstr(t[j, k] == -t_ini[k] + tls_pad_t[k][l][j_ind])
                    model.addConstr(g[j, k] == tls_pad_T[k][l][j_ind] - YR) 
                elif k == (len(tls_pad_T) - 1) and j_ind >= j_curr[l]:
                    if j_ind == j_curr[l]:
                        model.addConstr(t[j, k] == -t_ini[k] + tls_pad_t[k][l][j_ind])
                        model.addConstr(g[j, k] == tls_pad_T[k][l][j_ind] + tls_curr_T[l, j_ind] - YR)
                    else:
                        model.addConstr(t[j, k] == tls_curr_t[l, j_ind])
                        model.addConstr(g[j, k] == tls_curr_T[l, j_ind] - YR)
                else:
                    if j in J_first[id] and k == len(tls_pad_T):
                        model.addConstr(t[j, k] == t[J_last[id][l], k - 1] + g[J_last[id][l], k - 1] + YR)
                    if j not in J_last[id]:
                        j_next = J[id][l][j_ind + 1]
                        model.addConstr(t[j_next, k] == t[j, k] + g[j, k] + YR)
                    else:
                        j_next = J[id][l][0]
                        if k == K + K_ini - 1:
                            model.addConstr(t_ini[0] + t[j, k] + g[j, k] + YR == (K + K_ini)*C)
                        else:
                            model.addConstr(t[j_next, k + 1] == t[j, k] + g[j, k] + YR)
                    if j in J_barrier[id][0]:
                        j_oth = J_barrier[id][1][np.where(J_barrier[id][0] == j)][0]
                        model.addConstr(t[j, k] == t[j_oth, k])                   
                    # 目标函数辅助约束
                    model.addConstr(y[j, k] >= T_opt[id][np.where(J[id] == j)][0] - g[j, k] - YR)
                    model.addConstr(T_opt[id][np.where(J[id] == j)][0] - g[j, k] - YR >= -10)
                    # model.addConstr(y[j, k] == T_opt[id][np.where(J[id] == j)][0] - g[j, k] - YR)
                    # model.addConstr(y[j, k] == tlsPlan[id][k - (len(tls_pad_T) - 1)][np.where(J[id] == j)][0] - g[j, k] - YR)
                    # 最小绿时约束
                    model.addConstr(g[j, k] >= G_min)
                    model.addConstr(g[j, k] >= V_ij[id, (j-1)]*C/(S_ij[id, (j-1)]*Xc))

    # 设置分段线性函数约束，使用 PWLConstr 表示每个样本的分段函数
    for i in range(num_samples):
        for n in range(N):
            if p_bus_0[n] <= POS_stop[id]:
                # model.addConstr(r_act[i, n] == t_arr[n] + Ts[i, id] + L_app[id]/v_max)
                T_board_left = Ts[i, id] if T_board_past[busInd][n] == 0 else np.max([Ts[i, id] - T_board_past[busInd][n], 0])
                # model.addGenConstrPWL(r[n], r_act[i, n], 
                #                     [t_arr[n], (t_arr[n] + T_board_left + L_app[id]/v_max), 
                #                      (t_arr[n] + T_board_left + L_app[id]/v_min), 
                #                      (t_arr[n] + T_board_left + L_app[id]/v_min) + 1],
                #                     [(t_arr[n] + T_board_left + L_app[id]/v_max), 
                #                      (t_arr[n] + T_board_left + L_app[id]/v_max), 
                #                      (t_arr[n] + T_board_left + L_app[id]/v_min), 
                #                      (t_arr[n] + T_board_left + L_app[id]/v_min)], name=f"pwl_{i}_{n}")
                model.addGenConstrPWL(r[n], r_act[i, n], 
                                    [t_arr[n], (t_arr[n] + T_board_left + L_app[id]/v_max), 
                                     (t_arr[n] + T_board_left + L_app[id]/v_max) + 1],
                                    [(t_arr[n] + T_board_left + L_app[id]/v_max), 
                                     (t_arr[n] + T_board_left + L_app[id]/v_max), 
                                     (t_arr[n] + T_board_left + L_app[id]/v_max) + 1], name=f"pwl_{i}_{n}")
                # model.addConstrs((r[n] >= t_arr[n] + L_app[id]/v_max for n in range(N)), name="r_lb_1")
                # model.addConstrs((r[n] <= t_arr[n] + 30 + L_app[id]/v_min for n in range(N)), name="r_ub_1")
            elif p_bus_0[n] < POS[id]:
                # model.addConstr(r_act[i, n] == t_arr[n] + T_board_left + L_app[id]/v_max)
                # model.addGenConstrPWL(r[n], r_act[i, n], 
                #                      [0, (L_app[id] - (p_bus_0[n] - POS_stop[id]))/v_max,
                #                       (L_app[id] - (p_bus_0[n] - POS_stop[id]))/v_min,
                #                       (L_app[id] - (p_bus_0[n] - POS_stop[id]))/v_min + 1],
                #                      [(L_app[id] - (p_bus_0[n] - POS_stop[id]))/v_max, 
                #                       (L_app[id] - (p_bus_0[n] - POS_stop[id]))/v_max, 
                #                       (L_app[id] - (p_bus_0[n] - POS_stop[id]))/v_min, 
                #                       (L_app[id] - (p_bus_0[n] - POS_stop[id]))/v_min], name=f"pwl_{i}_{n}")
                model.addGenConstrPWL(r[n], r_act[i, n], 
                                    [0, (L_app[id] - (p_bus_0[n] - POS_stop[id]))/v_max, 
                                     (L_app[id] - (p_bus_0[n] - POS_stop[id]))/v_max + 1],
                                    [(L_app[id] - (p_bus_0[n] - POS_stop[id]))/v_max, 
                                     (L_app[id] - (p_bus_0[n] - POS_stop[id]))/v_max, 
                                     (L_app[id] - (p_bus_0[n] - POS_stop[id]))/v_max + 1], name=f"pwl_{i}_{n}")
                # model.addConstrs((r[n] >= t_arr[n] + (L_app[id] - (p_bus_0[n] - POS_stop[id]))/v_max for n in range(N)), name="r_lb_2")
                # model.addConstrs((r[n] <= t_arr[n] + (L_app[id] - (p_bus_0[n] - POS_stop[id]))/v_min for n in range(N)), name="r_ub_2")
            else: 
                continue
            for j in J_bus:
                # model.addConstr(beta1[i, n] + beta2[i, n] + beta3[i, n] == 1)
                model.addConstr(beta2[i, n] + beta3[i, n] == 1)
                # beta1 = 1：在k_tar绿灯亮起前到达
                # model.addConstr(r_act[i, n] >= t[j, k_tar[n] - 1] - M * (1 - beta1[i, n]))
                # model.addConstr(r_act[i, n] <= t[j, k_tar[n]] + M * (1 - beta1[i, n]))
                # beta2 = 1：在k_tar绿灯时段到达
                model.addConstr(r_act[i, n] >= t[j, k_tar[n]] - M * (1 - beta2[i, n]))
                model.addConstr(r_act[i, n] <= t[j, k_tar[n]] + g[j, k_tar[n]] + M * (1 - beta2[i, n]))
                # beta3 = 1：在k_tar绿灯结束后到达
                model.addConstr(r_act[i, n] >= t[j, k_tar[n]] + g[j, k_tar[n]] - M * (1 - beta3[i, n]))
                model.addConstr(r_act[i, n] <= t[j, k_tar[n] + 1] + M * (1 - beta3[i, n]))
                
                # model.addConstr(t_arr_next_[i, n] == beta1[i, n] * t[j, k_tar[n]] + beta2[i, n] * r_act[i, n] + 
                #                 beta3[i, n] * t[j, k_tar[n] + 1] + L_dep[id]/v_max)
                model.addConstr(t_arr_next_[i, n] == beta2[i, n] * r_act[i, n] + 
                                beta3[i, n] * t[j, k_tar[n] + 1] + L_dep[id]/v_max)
                model.addConstr(t_dev[i, n] >= t_arr_next_[i, n] - t_arr_next[n])

    # 期望到达时刻约束
    model.addConstrs((r[n] <= t_arr_next[n] + 10 - L_dep[id]/v_max for n in range(N)), name="r_ub_3")
    model.addConstrs((r[n] >= t_arr_next[n] - 10 - L_dep[id]/v_min for n in range(N)), name="r_lb_3")
    

    # 求解
    # model.update()
    model.optimize()

    # 检查求解结果
    if model.status == gb.GRB.OPTIMAL:
        T_sol = []
        for k in range((len(tls_pad_T) - 1), (K + K_ini)):
            Tk = round_and_adjust(np.array([g[j, k].x + YR for j in np.concatenate((J[id][0], J[id][1]))]).reshape(J[id].shape))
            T_sol.append(Tk)
        for l in [0, 1]:
            T_sol[0][l][0:(j_curr[l] + 1)] -= tls_pad_T[-1][l][0:(j_curr[l] + 1)]
        r_opt = [r[i].X for i in range(N)]
        J_tls = 0
        J_arr = 0
        for s in range(num_samples):
            for n in range(N):
                J_arr += (t_dev[s, n].x)/num_samples
        for j in range(1, 9):
            for k in range(len(tls_pad_T) - 1, K + K_ini):
                J_tls += y[j, k].x
        # print("Intersection:", id)
        # print("Arrival time:", r_opt)
        # print("Tls plan:", T_sol)
        # print("Objective value:", model.ObjVal)
        # print("Tls deviation:", J_tls)
        # print("Bus arrival deviation:", J_arr)
        print("Local SP Solver runtime (seconds):", model.Runtime)

        busArrPlan = []
        for n in range(N):
            traj = np.array([[r_opt[n], t_arr_next[n]], [POS[id], POS_stop[id + 1]]])
            busArrPlan.append(traj)

    else:
        # 计算 IIS
        model.computeIIS()
        model.write(f'{rootPath}\\RouteTSP\\log\\local_SP_model.ilp')  # 可以选择将 IIS 写入文件

        # 输出哪些约束导致不可行性
        for constr in model.getConstrs():
            if constr.IISConstr:  # 标记为 IIS 的约束
                print(f"Infeasible constraint: {constr.ConstrName}")
        print("Optimization failed.")
    return T_sol, busArrPlan