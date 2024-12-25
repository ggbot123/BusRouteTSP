import sys
rootPath = r'E:\workspace\python\BusRouteTSP'
sys.path.append(rootPath)
import traci
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import logging
import datetime
import pickle
from tqdm import tqdm
from ScenarioGenerator.busStopGen import posSet
from ScenarioGenerator.nodeGen import posJunc
from tools import getSumoTLSProgram, getIndfromId, myplot, RGY2J, getIniTlsCurr
from optimize_new import optimize
# from flexible_optimize import optimize

# sumoBinary = "E:\\software\\SUMO\\bin\\sumo-gui.exe"
sumoBinary = "sumo"
sumoCmd = [sumoBinary, "-c", f"{rootPath}\\ScenarioGenerator\\Scenario\\exp.sumocfg"]
logFile = f'{rootPath}\\RouteTSP\\log\\sys_%s.log' % datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d-%H-%M')
logging.basicConfig(filename=logFile, level=logging.INFO)
logging.info('Simulation start')
SIM_TIME = 3600
SIM_STEP = 1
LOWER_CONTROL_STEP = 1
PLAN_START = 10
PLAN_STEP = 50
BUS_DEP_INI = 0
MIN_STOP_DUR = 10
PER_BOARD_DUR = 3
PLANNED_BUS_NUM = 6
PLANNED_CYCLE_NUM = 10
PAD_CYCLE_NUM = 3
BG_CYCLE_LEN = 100
BG_PHASE_SEQ = np.load(r'E:\workspace\python\BusRouteTSP\tools\result\BG_PHASE_SEQ.npy')
BG_PHASE_LEN = np.load(r'E:\workspace\python\BusRouteTSP\tools\result\BG_PHASE_LEN.npy')
OFFSET = np.load(r'E:\workspace\python\BusRouteTSP\tools\result\offset.npy')
VOLUME = np.load(r'E:\workspace\python\BusRouteTSP\tools\result\volume.npy')
S = np.array([[1700, 3600, 1700, 1800, 1700, 3600, 1700, 1800],
              [1700, 3600, 1700, 1800, 1700, 3600, 1700, 1800],
              [1700, 3600, 1700, 1800, 1700, 3600, 1700, 1800],
              [1700, 3600, 1700, 1800, 1700, 3600, 1700, 1800],
              [1700, 3600, 1700, 1800, 1700, 3600, 1700, 1800]])
YR = 4  # 损失时间
BG_PHASE_SPLIT = np.insert(np.delete(BG_PHASE_LEN, -1, axis=2), 0, 0, axis=2).cumsum(axis=2)
FIRST_PHASE = BG_PHASE_SEQ[:, :, 0]
LAST_PHASE = BG_PHASE_SEQ[:, :, -1]
BAR_PHASE = BG_PHASE_SEQ[:, :, [0, 2]]
COORD_PHASE = [2, 6]
BUS_PHASE = [2]
G_MIN = 8  # 最小绿灯时间
COEFF_XC = 0.9  # 临界饱和系数
INI = -10000
POS_JUNC = np.array(posJunc).cumsum()
POS_STOP = np.concatenate([[0], POS_JUNC]) + np.array(posSet[0])
L_APP = POS_JUNC - POS_STOP[:-1]
L_DEP = POS_STOP[1:] - POS_JUNC
V_AVG = 10
BUS_DEP_HW = 2*60
V_MAX = 15
V_MIN = 10
Ts_means = 15
Ts_devs = 0
np.random.seed(0)
Z = np.random.normal(0, 1, (100, 6))
Z[0, 0] = 0
STOP_DUR = (Ts_means + 5)*np.array([1, 1, 1, 1, 1, 1])
TIMETABLE = np.array([20 + BUS_DEP_INI + i*BUS_DEP_HW + (POS_STOP - POS_STOP[0])/V_AVG + np.delete(np.insert(STOP_DUR, 0, 0), -1).cumsum() 
                      for i in range(100)])
DETBUFFERLEN = 10
E1_INT = 60
recordTimeList = np.arange(0, 3600, 5)

class Vehicle:
    def __init__(self, vehId, timeStep):
        self.id = vehId
        self.depart = timeStep
        self.speedProfile = pd.Series([pd.NA]*(timeStep+1))
    def update(self, timeStep):
        self.roadID = traci.vehicle.getRoadID(self.id)
        self.laneID = traci.vehicle.getLaneID(self.id)
        self.pos = traci.vehicle.getPosition(self.id)
        self.speed = traci.vehicle.getSpeed(self.id)
        self.speedProfile[timeStep] = self.speed

class Bus(Vehicle):
    def __init__(self, vehId, timeStep):
        super().__init__(vehId, timeStep)
        self.atBusStop = None
        self.waitTimeDict = {stopId: 0 for stopId in sumoEnv.busStopDict}
        self.arrTimeDict = {stopId: INI for stopId in sumoEnv.busStopDict}
        self.avgTimeRefPlan = None
        self.avgTimeRef = None
        self.nextUpdatePos = None
        self.waitTime = 0
        # traci.vehicle.setSpeedMode(vehId, 0b011000)
    def update(self, timeStep):
        super().update(timeStep)
        self.personNum = traci.vehicle.getPersonNumber(self.id)
        # 刚到站时，根据客流量更新站点停站时长
        if traci.vehicle.isAtBusStop(self.id):
            if self.atBusStop is None:
                # 查询站点编号
                stopId = traci.vehicle.getStops(self.id, 1)[0].stoppingPlaceID
                # 设置停站时间
                Ts = np.maximum(Ts_means + Z[getIndfromId('bus', self.id), getIndfromId('stop', stopId)] * Ts_devs, 5*np.ones_like(Ts_means))
                Ts = np.minimum(Ts, 25*np.ones_like(Ts_means))
                # traci.vehicle.setBusStop(self.id, stopId, duration=(MIN_STOP_DUR + PER_BOARD_DUR*sumoEnv.busStopDict[stopId].personNum))
                traci.vehicle.setBusStop(self.id, stopId, duration=Ts)
                self.atBusStop = stopId
                self.arrTimeDict[self.atBusStop] = timeStep
            else:
                # 注：每辆公交只经过每个站点1次
                self.waitTime += 1
                self.waitTimeDict[self.atBusStop] += SIM_STEP
        else:
            self.atBusStop = None
            self.waitTime = 0
    def updatePlan(self, plan):
        self.avgTimeRefPlan = np.array([plan[1], plan[0]])
        self.nextUpdatePos = self.avgTimeRefPlan[0, 0]
    def upperControl(self):
        if self.nextUpdatePos is not None:
            if self.pos[0] >= self.nextUpdatePos - 0.1:
                self.avgTimeRefPlan = np.delete(self.avgTimeRefPlan, 0, axis=1)
                if self.avgTimeRefPlan.shape[1] > 0:
                    self.nextUpdatePos = self.avgTimeRefPlan[0, 0]
                else:
                    self.nextUpdatePos = float('inf')
                    return
            self.avgTimeRef = self.avgTimeRefPlan[1, 0]
    def lowerControl(self, timeStep):
        if traci.vehicle.isAtBusStop(self.id):
            traci.vehicle.setSpeed(self.id, 0)
        elif self.avgTimeRef is not None:
            if self.nextUpdatePos in POS_JUNC:
                # 过路口时留5s裕量
                speedRef = (self.nextUpdatePos - self.pos[0])/max(self.avgTimeRef - 5 - timeStep, 1)
                # speedRef = V_MAX
            elif self.nextUpdatePos in POS_STOP:
                speedRef = (self.nextUpdatePos - self.pos[0])/max(self.avgTimeRef - timeStep, 1)
            else:
                speedRef = V_MAX
            # traci.vehicle.setSpeed(self.id, min(speedRef, V_MAX))
            traci.vehicle.setSpeed(self.id, max(min(speedRef, V_MAX), V_MIN))

class BusStop:
    def __init__(self, stopId):
        self.id = stopId
        self.pos = traci.busstop.getStartPos(stopId)
        self.personNumProfile = pd.Series()
    def update(self, timeStep):
        self.personNum = traci.busstop.getPersonCount(self.id)
        self.personNumProfile[timeStep] = self.personNum

class TrafficLight:
    def __init__(self, signalId):
        self.id = signalId
        self.phaseSeq = BG_PHASE_SEQ[getIndfromId('tls', signalId)]
        bgPhaseLen = np.array(BG_PHASE_LEN[getIndfromId('tls', signalId)])
        bgOffset = OFFSET[getIndfromId('tls', signalId)]
        # 多补一个周期，避免后面调用出现空数组
        self.kCurr = int((PAD_CYCLE_NUM*BG_CYCLE_LEN - bgOffset)/BG_CYCLE_LEN) + 1
        self.kLast = self.kCurr - 1
        self.tlsPlanPast = [bgPhaseLen for _ in range(self.kCurr)]
        self.tlsPlanCurr = getIniTlsCurr(bgPhaseLen, -bgOffset % BG_CYCLE_LEN)
        self.RGYProfile = pd.Series()
    def update(self, timeStep):
        self.curState = traci.trafficlight.getRedYellowGreenState(self.id)
        self.RGYProfile[timeStep] = self.curState
        self.recordTLS()
    def recordTLS(self):
        gPosInRGY = [i for i, char in enumerate(self.curState) if char == 'G' or char == 'y']
        curPhase = np.unique(np.array([RGY2J[pos+1] for pos in gPosInRGY if (pos+1) in RGY2J]))
        if np.array_equal(curPhase, self.phaseSeq[:, 0]) and (self.tlsPlanCurr[0, -1] > 0):
            self.tlsPlanPast.append(self.tlsPlanCurr)
            self.tlsPlanCurr = np.zeros([2, 4])
            self.kCurr += 1
        ind = np.where(np.isin(self.phaseSeq, list(curPhase)))
        self.jInd = ind[1]
        self.tlsPlanCurr[ind] += 1
    def control(self):
        self.kLast = self.kCurr
        RGYplan = getSumoTLSProgram(BG_PHASE_SEQ[getIndfromId('tls', self.id)], np.array(sumoEnv.tlsPlan[getIndfromId('tls', self.id)]), YR)
        traci.trafficlight.setProgramLogic(self.id, (traci.trafficlight.Logic(
            'MRTSP-SA', 0, 0, phases= [traci.trafficlight.Phase(RGY[1], RGY[0]) for RGYplanK in RGYplan for RGY in RGYplanK])))
        traci.trafficlight.setPhase(self.id, 0)
        sumoEnv.tlsPlan[getIndfromId('tls', self.id)][0] += self.tlsPlanCurr

class sumoEnv:
    allBusDict = {}
    runningBusDict = {}
    runningBusListSorted = []
    busStopDict = {}
    trafficLightDict = {}
    volumeData = np.zeros([BG_PHASE_SEQ.shape[0], BG_PHASE_SEQ.shape[-1]*2, DETBUFFERLEN])
    busArrTimePlan = []
    tlsPlan = []
    def __init__(self):
        sumoEnv.busStopDict = {bsId: BusStop(bsId) for bsId in traci.busstop.getIDList()}
        sumoEnv.trafficLightDict = {tlsId: TrafficLight(tlsId) for tlsId in traci.trafficlight.getIDList()}
        sumoEnv.tlsPlan = [np.array([BG_PHASE_LEN[i] for _ in range(PLANNED_CYCLE_NUM)]) for i in range(len(BG_PHASE_LEN))]

    def update(self, vehIdList, timeStep):
        busIdList = [vehId for vehId in vehIdList if vehId[0] != 'f']
        sumoEnv.runningBusDict = {key: veh for key, veh in sumoEnv.runningBusDict.items() if key in busIdList}
        for vehId in busIdList:
            # 对于新生成的车辆，初始化对象
            if vehId not in sumoEnv.runningBusDict:
                sumoEnv.runningBusDict.update({vehId: Bus(vehId, timeStep)})
            # 对于已存在的车辆，更新状态
            sumoEnv.runningBusDict[vehId].update(timeStep)
            if sumoEnv.runningBusDict[vehId].pos[0] >= POS_STOP[-1]:
                del sumoEnv.runningBusDict[vehId]
        sumoEnv.allBusDict.update(self.runningBusDict)
        # 对于新生成的尚无计划的公交，更新其计划
        for busId, bus in sumoEnv.runningBusDict.items():
            if bus.avgTimeRefPlan is None:
                sumoEnv.runningBusListSorted = sorted(sumoEnv.runningBusDict.values(), key=lambda x: x.pos[0], reverse=True) 
                if busId in sumoEnv.busArrTimePlan:
                    bus.updatePlan(sumoEnv.busArrTimePlan[busId])
            bus.upperControl()
        for stopId in sumoEnv.busStopDict:
            sumoEnv.busStopDict[stopId].update(timeStep)
        for tlsId in sumoEnv.trafficLightDict:
            sumoEnv.trafficLightDict[tlsId].update(timeStep)
        sumoEnv.volumeData = np.concatenate((sumoEnv.volumeData[..., 1:], np.zeros((sumoEnv.volumeData.shape[0], sumoEnv.volumeData.shape[1], 1))), axis=-1)
        for detId in traci.inductionloop.getIDList():
            sumoEnv.volumeData[getIndfromId('det', detId)][-1] = traci.inductionloop.getIntervalVehicleNumber(detId)      

    def genInput(self, timeStep):
        # 构造优化模型输入
        sumoEnv.runningBusListSorted = sorted(sumoEnv.runningBusDict.values(), key=lambda x: x.pos[0], reverse=True)
        minRunningInd = getIndfromId('bus', (min([int(busId) for busId in sumoEnv.runningBusDict])))
        maxRunningInd = getIndfromId('bus', (max([int(busId) for busId in sumoEnv.runningBusDict])))
        maxPlanInd = np.where(TIMETABLE[:, 0] > timeStep + (PLANNED_CYCLE_NUM - 1)*BG_CYCLE_LEN)[0][0]
        
        inputDict = {'I': len(sumoEnv.trafficLightDict), 'N': PLANNED_BUS_NUM, 'K': PLANNED_CYCLE_NUM, 'K_ini': PAD_CYCLE_NUM, 'C': BG_CYCLE_LEN,
                     'J': BG_PHASE_SEQ, 'J_first': FIRST_PHASE, 'J_last': LAST_PHASE, 'J_barrier': BAR_PHASE, 'J_coord': COORD_PHASE, 'J_bus': BUS_PHASE,
                     'T_opt': BG_PHASE_LEN, 't_opt': BG_PHASE_SPLIT, 'POS': POS_JUNC, 'YR': YR, 'G_min': G_MIN, 'Xc': COEFF_XC, 'T_board': STOP_DUR,
                     'POS_stop': POS_STOP, 'L_app': L_APP, 'L_dep': L_DEP, 'v_avg': V_AVG, 'v_max': V_MAX, 'v_min': V_MIN,
                     'cnt': int(timeStep/(SIM_STEP*PLAN_STEP))
                    }
        # inputDict['V_ij'] = 3600/E1_INT * np.mean(sumoEnv.volumeData, axis=-1)
        inputDict['V_ij'] = VOLUME
        inputDict['S_ij'] = S
        inputDict['p_bus_0'] = np.array([bus.pos[0] for bus in sumoEnv.runningBusListSorted] + [INI]*(maxPlanInd - maxRunningInd - 1))
        # 注：隐含意思是不区分完成时刻表的具体车辆，即发生超车时前后车时刻表也要交换
        inputDict['t_arr_plan'] = TIMETABLE[minRunningInd:maxPlanInd, :] - timeStep
        inputDict['Q_ij'] = 0
        inputDict['T_board_past'] = [bus.waitTime for bus in sumoEnv.runningBusListSorted]
        tlsPadT = []
        tlsPadt = []
        jCurrList = []
        tlsCurrT = []
        tlsCurrt = []
        kCurr = sumoEnv.trafficLightDict['nt1'].kCurr
        for tls in sumoEnv.trafficLightDict.values():
            tlsPadT.append(tls.tlsPlanPast[(kCurr - PAD_CYCLE_NUM):tls.kCurr] + [tls.tlsPlanCurr])
            jCurrList.append(tls.jInd)
            tlsCurrT.append(sumoEnv.tlsPlan[getIndfromId('tls', tls.id)][tls.kCurr - tls.kLast] - tls.tlsPlanCurr)
        for p in tlsPadT:
            tlsPadt.append(np.insert(np.delete(p, -1, axis=-1), 0, 0, axis=-1).cumsum(axis=-1))
        for p in tlsCurrT:
            tlsCurrt.append(np.insert(np.delete(p, -1, axis=-1), 0, 0, axis=-1).cumsum(axis=-1))
        inputDict['tls_pad_T'] = tlsPadT.copy()
        inputDict['tls_pad_t'] = tlsPadt.copy()
        inputDict['j_curr'] = jCurrList.copy()
        inputDict['tls_curr_T'] = tlsCurrT.copy()
        inputDict['tls_curr_t'] = tlsCurrt.copy()
        if timeStep in recordTimeList:
            with open(f"{rootPath}\\RouteTSP\\result\\inputData\\time={timeStep}.pkl", "wb") as f:
                pickle.dump(inputDict, f)
        return inputDict

    def plan(self, timeStep):
        if timeStep >= 10:
            pass
        if not sumoEnv.runningBusDict:
            return
        # 调用MRTSP-SA算法
        sumoEnv.tlsPlan, sumoEnv.busArrTimePlan, _ = optimize(**self.genInput(timeStep))
        # 记录优化模型输出
        if timeStep in recordTimeList:
            with open(f"{rootPath}\\RouteTSP\\result\\outputData\\time={timeStep}.pkl", "wb") as f:
                outputDict = {'tlsPlan': sumoEnv.tlsPlan, 'busArrPlan': sumoEnv.busArrTimePlan}
                pickle.dump(outputDict, f)
        # 记录公交到达时刻规划结果
        sumoEnv.busArrTimePlan = {str(getIndfromId('bus', sumoEnv.runningBusListSorted[0].id) + i): [plan[0][1:] + timeStep, plan[1][1:]] 
                                  for i, plan in enumerate(sumoEnv.busArrTimePlan)}
        # 更新信号配时计划
        for tls in sumoEnv.trafficLightDict.values():
            tls.control()
        # 更新公交到达时刻计划
        for bus in sumoEnv.runningBusListSorted:
            bus.updatePlan(sumoEnv.busArrTimePlan[bus.id])

    def lowerControl(self, timeStep):
        for bus in sumoEnv.runningBusDict.values():
            bus.lowerControl(timeStep)

    def record(self):
        csvBusStopFilePath = f'{rootPath}\\RouteTSP\\result\\personNum_profile.csv'
        csvTLSFilePath = f'{rootPath}\\RouteTSP\\result\\tlsState_profile.csv'
        csvBusSpeedFilePath = f'{rootPath}\\RouteTSP\\result\\bus_speed_profile.csv'
        busSpeedProfileDict = {'Bus ' + vehId: veh.speedProfile for vehId, veh in sumoEnv.allBusDict.items()}
        pd.DataFrame(busSpeedProfileDict).to_csv(csvBusSpeedFilePath)
        personNumProfileDict = {'BusStop ' + stopId: stop.personNumProfile for stopId, stop in self.busStopDict.items()}
        pd.DataFrame(personNumProfileDict).to_csv(csvBusStopFilePath)
        tlsStateProfileDict = {'TrafficLight ' + tlsId: tls.RGYProfile for tlsId, tls in self.trafficLightDict.items()}
        pd.DataFrame(tlsStateProfileDict).to_csv(csvTLSFilePath)
        busArrTimeMat = [[bus.arrTimeDict[stopId] for stopId in ['AtoB_0', 'AtoB_1', 'AtoB_2', 'AtoB_3', 'AtoB_4', 'AtoB_5']] 
                         for bus in sumoEnv.allBusDict.values()]
        np.save(r'E:\workspace\python\BusRouteTSP\RouteTSP\result\busArrTime.npy', np.array(busArrTimeMat))
        for id, tls in sumoEnv.trafficLightDict.items():
            tlsPlan = tls.tlsPlanPast
            np.save(f'E:\\workspace\\python\\BusRouteTSP\\RouteTSP\\result\\tlsPlan_{id}.npy', np.array(tlsPlan))

if __name__ == '__main__':
    traci.start(sumoCmd)
    env = sumoEnv()

    for step in tqdm(range(SIM_TIME)):
        timeStep = SIM_STEP*step
        env.update(traci.vehicle.getIDList(), timeStep)
        if step >= PLAN_START and (step - PLAN_START) % PLAN_STEP == 0:
            print("Planning...\n")
            env.plan(timeStep)
        if step % LOWER_CONTROL_STEP == 0:
            env.lowerControl(timeStep)
        traci.simulationStep()

    env.record()
    print("Ploting...\n")
    myplot(POS_JUNC, POS_STOP, BUS_PHASE[0], TIMETABLE)
    traci.close()