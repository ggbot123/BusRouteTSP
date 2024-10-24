import matplotlib.pyplot as plt
import numpy as np
import sys
rootPath = r'E:\workspace\python\BusRouteTSP'
sys.path.append(rootPath)
from ScenarioGenerator.busStopGen import posSet
from ScenarioGenerator.nodeGen import posJunc

def plotTSTP(FILENAME, J, T_opt_K, t_arr_plan, traj_bus, selected_phase, plotShow=True):
    # 示例输入参数
    I = T_opt_K.shape[1] # 交叉口数量
    N = t_arr_plan.shape[0]
    K_ini = 2
    K = T_opt_K.shape[0]
    C0 = 100
    t_opt_K = np.insert(np.delete(T_opt_K, -1, axis=3), 0, 0, axis=3).cumsum(axis=3)
    # assert np.array_equal(T_opt.sum(axis=2), np.ones(T_opt.shape[0:2]) * C), "相位时长与信号周期不相同\n"
    OF = np.array([0, 59, 56, 16, 61]).cumsum()
    POS = np.array(posJunc).cumsum()
    POS_stop = np.concatenate([[0], POS]) + np.array(posSet[0])
    YR = 4  # 损失时间

    # 时间轴的范围（秒）
    time_end = C0*(K - K_ini)  # 可以根据需要调整时间范围
    time_step_size = 1
    time_steps = range(-K_ini*C0, time_end, time_step_size)  # 时间步
    
    # 绘制每个交叉口的信号灯变化
    plt.subplots(figsize=(10, 6))
    for i in range(I):
        for l in range(J[i].shape[0]):
            if selected_phase in J[i][l]:
                for t in time_steps:
                    if len(T_opt_K.shape) == 4:
                        C = np.array(T_opt_K.sum(axis=3)[:, i, 0]).cumsum()
                        k = np.where(t + K_ini*C0 - OF[i] < C)[0][0]
                        t_opt = t_opt_K[k]
                        T_opt = T_opt_K[k]
                        if k > 0:
                            t_local = (t + K_ini*C0 - OF[i] - C[k-1]) % C[k]
                        else:
                            t_local = (t + K_ini*C0 - OF[i]) % C[k]
                    else:
                        C = 100
                        t_opt = t_opt_K
                        T_opt = T_opt_K
                        t_local = (t - OF[i]) % C
                    j = np.where(t_local >= t_opt[i][l])[0][-1]
                    if J[i][l][j] == selected_phase:
                        if T_opt[i][l][j] - (t_local - t_opt[i][l][j]) <= YR:
                            light_status = 'y'
                        else:
                            light_status = 'g'
                    else:
                        light_status = 'r'
                    plt.hlines(POS[i], t, t + time_step_size, colors = light_status, linestyles = "-")
    for n in range(N):
        plt.plot(traj_bus[n][0], traj_bus[n][1], color='b')
        plt.scatter(t_arr_plan[n], POS_stop, color='b', marker='*')

    plt.plot([0, 0], [0, POS_stop[-1]], linestyle='--', color='k')
    plt.plot([(K-K_ini)*C0, (K-K_ini)*C0], [0, POS_stop[-1]], linestyle='--', color='k')
    # plt.grid(True)
    plt.xlabel('time(s)')
    plt.ylabel('distance(m)')
    plt.title('Traffic Signal Timing Plan')
    # plt.yticks(POS, labels = [f'intersection {i+1}' for i in range(I)])
    plt.savefig(FILENAME)
    if plotShow is True:
        plt.show()
