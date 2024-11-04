import sys
rootPath = r'E:\workspace\python\BusRouteTSP'
sys.path.append(rootPath)
import traci
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import logging
import datetime
from tqdm import tqdm
from ScenarioGenerator.busStopGen import posSet
from ScenarioGenerator.nodeGen import posJunc
from tools import getSumoTLSProgram, getIndfromId, myplot, savePlan, getBusOrder
from optimize import optimize

# sumoBinary = "E:\\software\\SUMO\\bin\\sumo-gui.exe"
sumoBinary = "sumo"
sumoCmd = [sumoBinary, "-c", f"{rootPath}\\ScenarioGenerator\\Scenario\\exp.sumocfg"]
logFile = f'{rootPath}\\RouteTSP\\log\\sys_%s.log' % datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d-%H-%M')
logging.basicConfig(filename=logFile, level=logging.INFO)
logging.info('Simulation start')
SIM_TIME = 3600
SIM_STEP = 1
UPPER_CONTROL_STEP = 1
LOWER_CONTROL_STEP = 1
PLAN_STEP = 300
PLAN_START = 10
BUS_DEP_INI = 800
MIN_STOP_DUR = 5
PER_BOARD_DUR = 3
PLANNED_BUS_NUM = 6
PLANNED_CYCLE_NUM = 10
PAD_CYCLE_NUM = 2
BG_CYCLE_LEN = 100
BG_PHASE_SEQ = np.load(r'E:\workspace\python\BusRouteTSP\tools\result\BG_PHASE_SEQ.npy')
BG_PHASE_LEN = np.load(r'E:\workspace\python\BusRouteTSP\tools\result\BG_PHASE_LEN.npy')
OFFSET = np.load(r'E:\workspace\python\BusRouteTSP\tools\result\offset.npy')
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
V_AVG = 12
BUS_DEP_HW = 2*60
V_MAX = 15
TIMETABLE = np.array([20 + i*BUS_DEP_HW + (POS_STOP - POS_STOP[0])/V_AVG for i in range(100)])
DETBUFFERLEN = 10
E1_INT = 60

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
        self.avgTimeRefPlan = None
        self.avgTimeRef = None
        self.nextUpdatePos = None
        # 关闭所有sumo速度限制
        # traci.vehicle.setSpeedMode(vehId, 0b011000)
        logging.info('Bus %s created' % vehId)
    def update(self, timeStep):
        super().update(timeStep)
        self.personNum = traci.vehicle.getPersonNumber(self.id)
        # 刚到站时，根据客流量更新站点停站时长
        if traci.vehicle.isAtBusStop(self.id):
            if self.atBusStop is None:
                # 查询站点编号
                stopId = traci.vehicle.getStops(self.id, 1)[0].stoppingPlaceID
                logging.info('[%s] Arrived Stop Id:\n%s\n' % (str(timeStep), str(stopId)))
                # 设置停站时间
                logging.info('[%s] %s\n' % (str(timeStep), str(sumoEnv.busStopDict)))
                # traci.vehicle.setBusStop(self.id, stopId, duration=(MIN_STOP_DUR + PER_BOARD_DUR*sumoEnv.busStopDict[stopId].personNum))
                traci.vehicle.setBusStop(self.id, stopId, duration=0)
                self.atBusStop = stopId
            else:
                # 注：每辆公交只经过每个站点1次
                self.waitTimeDict[self.atBusStop] += SIM_STEP
        else:
            self.atBusStop = None
        logging.info('[%s] Bus Loading Time:\n%s\n' % (str(timeStep), str(self.waitTimeDict)))
    def updatePlan(self, order):
        self.avgTimeRefPlan = np.array([sumoEnv.busArrTimePlan[order][1],
                                        sumoEnv.busArrTimePlan[order][0]])
        self.nextUpdatePos = self.avgTimeRefPlan[0, 0]
    def upperControl(self):
        self.avgTimeRefPlan = np.delete(self.avgTimeRefPlan, 0, axis=1)
        if self.avgTimeRefPlan.shape[1] > 0:
            self.nextUpdatePos = self.avgTimeRefPlan[0, 0]
            self.avgTimeRef = self.avgTimeRefPlan[1, 0]
        else:
            self.nextUpdatePos = float('inf')
    def lowerControl(self, timeStep):
        if self.avgTimeRef is not None:
            speedRef = (self.nextUpdatePos - self.pos[0])/max(self.avgTimeRef - timeStep, 1)
            traci.vehicle.setSpeed(self.id, min(speedRef, V_MAX))
        # TODO：补充车端控制规则约束（前车、信号灯等）

class BusStop:
    def __init__(self, stopId):
        self.id = stopId
        self.pos = traci.busstop.getStartPos(stopId)
        self.personNumProfile = pd.Series()
    def update(self, timeStep):
        self.personNum = traci.busstop.getPersonCount(self.id)
        self.personNumProfile[timeStep] = self.personNum

class TrafficLight:
    tlsGlobal0Time = None
    def __init__(self, signalId):
        self.id = signalId
        self.RGYplan = getSumoTLSProgram(BG_PHASE_SEQ[getIndfromId('tls', signalId)], np.array([BG_PHASE_LEN[getIndfromId('tls', signalId)]]), YR)
        self.nextRGYplan = []
        self.RGYProfile = pd.Series()
        self.nextUpdate = None
        self.lastUpdate = 0
    def update(self, timeStep):
        self.curState = traci.trafficlight.getRedYellowGreenState(self.id)
        self.RGYProfile[timeStep] = self.curState
    def getNextCycleTime(self, timeStep):
        tEnd = 0
        for RGYplanK in self.RGYplan:
            tEnd += sum([phase[1] for phase in RGYplanK])
            if (timeStep - self.lastUpdate) <= tEnd:
                return tEnd - (timeStep - self.lastUpdate)
    def updatePlan(self, timeStep):
        if getIndfromId('tls', self.id) == 0:
            TrafficLight.tlsGlobal0Time = timeStep + self.getNextCycleTime(timeStep)
        self.nextUpdate = TrafficLight.tlsGlobal0Time + OFFSET[getIndfromId('tls', self.id)]
        self.nextRGYplan = getSumoTLSProgram(BG_PHASE_SEQ[getIndfromId('tls', self.id)], sumoEnv.tlsPlan[:, getIndfromId('tls', self.id), :, :], YR)
    def control(self, timeStep):
        self.RGYplan = self.nextRGYplan.copy()
        self.nextRGYplan = []
        self.nextUpdate = None
        self.lastUpdate = timeStep
        traci.trafficlight.setProgramLogic(self.id, (traci.trafficlight.Logic(
            'MRTSP', 0, 0, phases= [traci.trafficlight.Phase(RGY[1], RGY[0]) for RGYplanK in self.RGYplan for RGY in RGYplanK])))

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
    def update(self, vehIdList, timeStep):
        logging.info('[%s] Running Bus Dict:\n%s\n' % (str(timeStep), str(sumoEnv.runningBusDict)))
        busIdList = [vehId for vehId in vehIdList if vehId[0] != 'f']
        sumoEnv.runningBusDict = {key: veh for key, veh in sumoEnv.runningBusDict.items() if key in busIdList}
        for vehId in busIdList:
            # 对于新生成的车辆，初始化对象
            if vehId not in sumoEnv.runningBusDict:
                sumoEnv.runningBusDict.update({vehId: Bus(vehId, timeStep)})
            # 对于已存在的车辆，更新状态
            sumoEnv.runningBusDict[vehId].update(timeStep)
        sumoEnv.allBusDict.update(self.runningBusDict)
        for stopId in sumoEnv.busStopDict:
            sumoEnv.busStopDict[stopId].update(timeStep)
        for tlsId in sumoEnv.trafficLightDict:
            sumoEnv.trafficLightDict[tlsId].update(timeStep)
        sumoEnv.volumeData = np.concatenate((sumoEnv.volumeData[..., 1:], np.zeros((sumoEnv.volumeData.shape[0], sumoEnv.volumeData.shape[1], 1))), axis=-1)
        for detId in traci.inductionloop.getIDList():
            sumoEnv.volumeData[getIndfromId('det', detId)][-1] = traci.inductionloop.getIntervalVehicleNumber(detId)

    def genInput(self, timeStep):
        # 构造优化模型输入
        inputDict = {'I': len(self.trafficLightDict), 'N': PLANNED_BUS_NUM, 'K': PLANNED_CYCLE_NUM, 'K_ini': PAD_CYCLE_NUM, 'C': BG_CYCLE_LEN,
                     'J': BG_PHASE_SEQ, 'J_first': FIRST_PHASE, 'J_last': LAST_PHASE, 'J_barrier': BAR_PHASE, 'J_coord': COORD_PHASE, 'J_bus': BUS_PHASE,
                     'T_opt': BG_PHASE_LEN, 't_opt': BG_PHASE_SPLIT, 'OF': OFFSET, 'POS': POS_JUNC, 'YR': YR, 'G_min': G_MIN, 'Xc': COEFF_XC,
                     'POS_stop': POS_STOP, 'L_app': L_APP, 'L_dep': L_DEP, 'v_avg': V_AVG, 'v_max': V_MAX, 'cnt': int(timeStep/(SIM_STEP*PLAN_STEP))
                    }
        inputDict['V_ij'] = 3600/E1_INT * np.mean(sumoEnv.volumeData, axis=-1)
        inputDict['S_ij'] = np.array([[1700, 3600, 1700, 1800, 1700, 3600, 1700, 1800],
                                      [1700, 3600, 1700, 1800, 1700, 3600, 1700, 1800],
                                      [1700, 3600, 1700, 1800, 1700, 3600, 1700, 1800],
                                      [1700, 3600, 1700, 1800, 1700, 3600, 1700, 1800],
                                      [1700, 3600, 1700, 1800, 1700, 3600, 1700, 1800]])
        inputDict['p_bus_0'] = np.array([bus.pos[0] for bus in sumoEnv.runningBusListSorted] + [-10000]*(PLANNED_BUS_NUM - len(sumoEnv.runningBusDict)))
        # TODO: 适应更一般的情况，例如当0车抛锚时，minRunningInd = 0，而maxRunningInd可能大于目前路上行驶的公交总数
        minRunningInd = getIndfromId('bus', (min([int(busId) for busId in sumoEnv.runningBusDict])))
        maxRunningInd = getIndfromId('bus', (max([int(busId) for busId in sumoEnv.runningBusDict])))
        # 注：隐含意思是不区分完成时刻表的具体车辆，即发生超车时前后车时刻表也要交换
        inputDict['t_arr_plan'] = TIMETABLE[minRunningInd:minRunningInd + PLANNED_BUS_NUM, :] - timeStep
        inputDict['T_dep_0'] = np.concatenate([np.array([(POS_STOP[0] - sumoEnv.runningBusDict[str(busId)].pos[0])/V_AVG
                                                          for busId in range(minRunningInd, maxRunningInd + 1)]),
                                                          TIMETABLE[(maxRunningInd + 1):(minRunningInd + PLANNED_BUS_NUM), 0] - timeStep])
        inputDict['Q_ij'] = 0
        return inputDict

    def plan(self, timeStep):
        # TODO: 改成适应输入无公交的情形
        if not sumoEnv.runningBusDict:
            return
        sumoEnv.runningBusListSorted = sorted(sumoEnv.runningBusDict.values(), key=lambda x: x.pos[0], reverse=True)      
        # 调用MRTSP-SA算法，输出以t=timeStep为0时刻的未来K周期信号配时与全程公交行驶(arrTime, pos)列表
        sumoEnv.tlsPlan, sumoEnv.busArrTimePlan = optimize(**self.genInput(timeStep))
        sumoEnv.busArrTimePlan = [[plan[0] + timeStep, plan[1]] for plan in sumoEnv.busArrTimePlan] # 对齐优化模型与仿真环境0时刻
        # 更新信号配时计划，包括将目前配时替换为新规划配时的时刻，以及对应的配时方案
        for tls in sumoEnv.trafficLightDict.values():
            tls.updatePlan(timeStep)
        # 更新公交到达时刻计划，包括按顺序经过未来每个节点（交叉口or站点）的计划时刻与对应位置
        for i, bus in enumerate(sumoEnv.runningBusListSorted):
            bus.updatePlan(i)
        # 记录优化模型输出
        savePlan(timeStep, sumoEnv.tlsPlan, sumoEnv.busArrTimePlan, int(timeStep/(SIM_STEP*PLAN_STEP)))

    def upperControl(self, timeStep):
        for busId, bus in sumoEnv.runningBusDict.items():
            # TODO: 适应更一般的情形
            # 对于新生成的尚无计划的公交，更新其计划
            if bus.nextUpdatePos is None:
                planInd = getBusOrder(busId, [bus.id for bus in sumoEnv.runningBusListSorted])
                if planInd < len(sumoEnv.busArrTimePlan):
                    bus.updatePlan(planInd)
            # 对于已有计划的公交，检查是否该更新参考速度
            if bus.nextUpdatePos is not None and bus.pos[0] >= bus.nextUpdatePos:
                bus.upperControl()
        # 检查是否需要切换信号配时方案
        for tls in sumoEnv.trafficLightDict.values():
            if tls.nextUpdate is not None and timeStep >= tls.nextUpdate:
                tls.control(timeStep)
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

if __name__ == '__main__':
    traci.start(sumoCmd)
    env = sumoEnv()

    for step in tqdm(range(SIM_TIME)):
        timeStep = SIM_STEP*step

        if timeStep == 1200:
            pass

        env.update(traci.vehicle.getIDList(), timeStep)
        if step >= PLAN_START and (step - PLAN_START) % PLAN_STEP == 0:
            print("Planning...\n")
            env.plan(timeStep)
        if step % UPPER_CONTROL_STEP == 0:
            env.upperControl(timeStep)
        if step % LOWER_CONTROL_STEP == 0:
            env.lowerControl(timeStep)
        traci.simulationStep()

    env.record()
    print("Ploting...\n")
    myplot(POS_JUNC, BUS_PHASE[0])
    traci.close()