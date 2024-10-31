import gurobipy as grb
import numpy as np
import matplotlib.pyplot as plt

class MaxBAND_versatile():
    def __init__(self, num_signals):
        self.num_signals = num_signals
    
    
    def build_program(self, red_time, queue_clearance_time,
                      two_way_vol_ratio, cycle_range, speed_range,
                      speed_change_range, distance, left_turn_time):
        
        assert red_time.shape == (2, self.num_signals)
        assert queue_clearance_time.shape == (2, self.num_signals)
        assert left_turn_time.shape == (2, self.num_signals)
        assert speed_range.shape == (2, self.num_signals - 1, 2)
        assert speed_change_range.shape == (2, self.num_signals - 2, 2)
        assert distance.shape == (2, self.num_signals - 1)
        
        # create model
        maxband = grb.Model("MaxBAND_versatile")
        
        # create varibles
        b = maxband.addVars(2, lb=0, ub=1, obj=1, vtype=grb.GRB.CONTINUOUS)
        w = maxband.addVars(2, self.num_signals, lb=0, ub=1, obj=1, vtype=grb.GRB.CONTINUOUS)
        m = maxband.addVars(self.num_signals, lb=1, obj=1, vtype=grb.GRB.INTEGER)
        z = maxband.addVars(1, lb=cycle_range[1], ub=cycle_range[0], obj=1, vtype=grb.GRB.CONTINUOUS)
        t = maxband.addVars(2, (self.num_signals - 1), lb=0, obj=1, vtype=grb.GRB.CONTINUOUS)
        sig = maxband.addVars(2, self.num_signals, obj=1, vtype=grb.GRB.BINARY)
        
        # build objective
        k = two_way_vol_ratio
        obj = b[0] + k * b[1]
        maxband.setObjective(obj, grb.GRB.MAXIMIZE)
        
        r = red_time
        tao = queue_clearance_time
        L = left_turn_time
        d = distance
        e, f = speed_range[:,:,0], speed_range[:,:,1]
        g, h = speed_change_range[:,:,0], speed_change_range[:,:,1]
        
        # add constraints
        # No.1: Directional Interference
        for j in range(2):
            for i in range(self.num_signals):
                maxband.addConstr(w[j, i] + b[j] <= 1 - r[j, i])

        # No.2: Loop Integer
        for i in range(self.num_signals - 1):
            # left-hand side
            term1 = w[0, i] + w[1, i]
            term2 = -(w[0, i + 1] + w[1, i + 1])
            term3 = t[0, i] + t[1, i]
            term4 = sig[0, i] * L[0, i] - sig[1, i] * L[1, i]
            term4 -= sig[0, i + 1] * L[0, i + 1] - sig[1, i + 1] * L[1, i + 1]
            term5 = -m[i]
            lhs = term1 + term2 + term3 + term4 + term5
            
            # right-hand side
            term1 = r[0, i + 1] - r[0, i]
            term2 = tao[0, i] + tao[1, i]
            rhs = term1 + term2
            
            maxband.addConstr(lhs == rhs)

        # No.3: Weighted inbound and outbound bandwidth
        # maxband.addConstr((1 - k) * b[1] >= (1 - k) * k * b[0])
        maxband.addConstr(b[1] == b[0])

        
        # No.4: Speed Range
        for i in range(self.num_signals - 1):
            maxband.addConstr(t[0, i] >= (d[0, i] / f[0, i]) * z[0])
            maxband.addConstr(t[0, i] <= (d[0, i] / e[0, i]) * z[0])
            maxband.addConstr(t[1, i] >= (d[1, i] / f[1, i]) * z[0])
            maxband.addConstr(t[1, i] <= (d[1, i] / e[1, i]) * z[0])
            
        # No.5: Speed Change Range
        for i in range(self.num_signals - 2):
            maxband.addConstr(((d[0, i] / d[0, i + 1]) * t[0, i + 1] - t[0, i]) <= (d[0, i] / h[0, i]) * z[0])
            maxband.addConstr(((d[0, i] / d[0, i + 1]) * t[0, i + 1] - t[0, i]) >= (d[0, i] / g[0, i]) * z[0])
            maxband.addConstr(((d[1, i] / d[1, i + 1]) * t[1, i + 1] - t[1, i]) <= (d[1, i] / h[1, i]) * z[0])
            maxband.addConstr(((d[1, i] / d[1, i + 1]) * t[1, i + 1] - t[1, i]) >= (d[1, i] / g[1, i]) * z[0])
            
        
        return maxband

    
    def solve(self, red_time, queue_clearance_time,
              two_way_vol_ratio, cycle_range, speed_range,
              speed_change_range, distance, left_turn_time):
        
        maxband = self.build_program(red_time, queue_clearance_time,
                                     two_way_vol_ratio, cycle_range, speed_range,
                                     speed_change_range, distance, left_turn_time)
        maxband.optimize()
        
        b_range = 2
        w_range = b_range + 2 * self.num_signals
        m_range = w_range + self.num_signals
        t_range = m_range + 1 + 2 * (self.num_signals - 1)
        solution_b = [v.x for v in maxband.getVars()[:b_range]]
        solution_w = [v.x for v in maxband.getVars()[b_range:w_range]]
        solution_m = [v.x for v in maxband.getVars()[w_range:m_range]]
        solution_z = [v.x for v in maxband.getVars()[m_range:m_range + 1]]
        solution_t = [v.x for v in maxband.getVars()[m_range + 1:t_range]]
        solution_sig = [v.x for v in maxband.getVars()[t_range:]]
        
        return solution_b, solution_w, solution_m, solution_z, solution_t, solution_sig