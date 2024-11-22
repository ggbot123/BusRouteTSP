import numpy as np
import gurobipy as gb
import sys
rootPath = r'E:\workspace\python\BusRouteTSP'
sys.path.append(rootPath)
from ScenarioGenerator.busStopGen import posSet
from ScenarioGenerator.nodeGen import posJunc
from tools import round_and_adjust, local_SP_plot

def local_SP(id, t_arr, t_arr_next, theta, **kwargs):
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
            else:
                exec(f'{key} = np.{repr(value)}', globals())
    k_tar = np.where(theta == 1)[0]
    M = 100000
    t_ini = np.flip(np.flip(np.sum(np.array(tls_pad_T)[:, 0, :], axis=-1)).cumsum())
    t_lb = -t_ini[0]
    tls_pad_t = np.insert(np.delete(tls_pad_T, -1, axis=-1), 0, 0, axis=-1).cumsum(axis=-1)

    # 随机变量样本生成：1000个样本，每个样本是长度为N的向量，服从N(0, 1)
    num_samples = 100  # 样本数量
    alpha = 0.1  # 风险水平（1 - alpha 为机会约束概率）
    np.random.seed(0)
    # Ts_means = np.array([2, 2, 2, 2, 2, 2])
    Ts_means = T_board
    Ts_devs = np.array([1, 1, 1, 1, 1, 1])*3
    Z = np.random.normal(0, 1, (num_samples, N))
    Ts = (Ts_means + Z * Ts_devs)   # Ts: 1000 * N

    # 创建Gurobi模型
    model = gb.Model("Local_SP")
    model.setParam('OutputFlag', 0)

    # 决策变量-期望到达交叉口时刻r(n)
    r = model.addVars(N, lb=-gb.GRB.INFINITY, name="r")

    # 决策变量-信号配时t(j,k) & g(j,k)
    t = {}
    g = {}
    y = {}
    for k in range(K + K_ini):
        for j in np.concatenate((J[id][0], J[id][1])):
            t[j, k] = model.addVar(name=f't{j}{k}', lb=t_lb)
            g[j, k] = model.addVar(name=f'g{j}{k}')
            y[j, k] = model.addVar(name=f'y{j}{k}', lb=-gb.GRB.INFINITY)

    # （辅助）决策变量-实际到达交叉口时刻 r_act(n)
    r_act = model.addVars(num_samples, N, lb=-gb.GRB.INFINITY, name="r_act")

    # 0-1变量 z(s) 表示每个样本是否满足机会约束
    z = model.addVars(num_samples, vtype=gb.GRB.BINARY, name="z")

    # 设置目标函数
    objective_expr = gb.LinExpr()
    for j in range(1, 9):
        for k in range(K + K_ini):
            objective_expr += y[j, k]**2
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
                else:
                    if (k == len(tls_pad_T) - 1 and j_ind == j_curr[l]):
                        model.addConstr(t[j, k] == -t_ini[k] + tls_pad_t[k][l][j_ind])
                        model.addConstr(g[j, k] >= tls_pad_T[k][l][j_ind] - YR)
                    if j not in J_last[0]:
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
                    # model.addConstr(y[j, k] >= T_opt[0][np.where(J[0] == j)][0] - g[j, k] - YR)
                    model.addConstr(y[j, k] == T_opt[id][np.where(J[id] == j)][0] - g[j, k] - YR)
                    # 最小绿时约束
                    model.addConstr(g[j, k] >= G_min)
                    model.addConstr(g[j, k] >= V_ij[id, (j-1)]*C/(S_ij[id, (j-1)]*Xc))

    # 设置分段线性函数约束，使用 PWLConstr 表示每个样本的分段函数
    for i in range(num_samples):
        for n in range(N):
            # 添加分段线性约束
            model.addGenConstrPWL(r[n], r_act[i, n], [t_arr[n], (t_arr[n] + Ts[i, n] + L_app[id]/v_max), (t_arr[n] + Ts[i, n] + L_app[id]/v_max) + 1],
                                [(t_arr[n] + Ts[i, n] + L_app[id]/v_max), (t_arr[n] + Ts[i, n] + L_app[id]/v_max), (t_arr[n] + Ts[i, n] + L_app[id]/v_max) + 1], name=f"pwl_{i}_{n}")
    # 期望到达时刻约束
    model.addConstrs((r[n] <= t_arr_next[n] - L_dep[id]/v_max for n in range(N)), name="arrival_plan")

    # 机会约束 - 在给定相位通过
    for j in J_bus:
        model.addConstrs((r_act[i, n] >= t[j, k_tar[n]] - M * (1 - z[i]) for i in range(num_samples) for n in range(N)), name="lower_bound")
        model.addConstrs((r_act[i, n] <= t[j, k_tar[n]] + g[j, k_tar[n]] - YR + M * (1 - z[i]) for i in range(num_samples) for n in range(N)), name="upper_bound")

    # 设置满足比例约束
    model.addConstr(gb.quicksum(z[i] for i in range(num_samples)) >= (1 - alpha) * num_samples)

    # 求解
    # model.update()
    model.optimize()

    # 检查求解结果
    if model.status == gb.GRB.OPTIMAL:
        # T_sol = np.zeros([K] + list(J[id].shape))
        T_sol = []
        for k in range((len(tls_pad_T) - 1), (K + K_ini)):
            Tk = round_and_adjust(np.array([g[j, k].x + YR for j in np.concatenate((J[id][0], J[id][1]))]).reshape(J[id].shape))
            T_sol.append(Tk)
        for l in [0, 1]:
            T_sol[0][l][0:(j_curr[l] + 1)] -= tls_pad_T[-1][l][0:(j_curr[l] + 1)]
        # for k in range(K):   
        #     T_sol[k] = round_and_adjust(np.array([g[j, k].x + YR for j in np.concatenate((J[id][0], J[id][1]))]).reshape(J[id].shape))
        r_opt = [r[i].X for i in range(N)]
        print("Arrival time:", r_opt)
        print("Tls plan:", T_sol)
        print("Objective value:", model.ObjVal)
        print("Solver runtime (seconds):", model.Runtime)

        if id == 0:
            busArrPlan = [np.array([[t_arr[n] - 20, t_arr[n], r_opt[n], t_arr_next[n]], [0, POS_stop[0], POS[0], POS_stop[1]]]) for n in range(N)]
        else:
            busArrPlan = [np.array([[t_arr[n], r_opt[n], t_arr_next[n]], [POS_stop[id], POS[id], POS_stop[id + 1]]]) for n in range(N)]
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