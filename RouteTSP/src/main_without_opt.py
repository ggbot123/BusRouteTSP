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
from tools import getSumoTLSProgram, getIndfromId, myplot

# sumoBinary = "E:\\software\\SUMO\\bin\\sumo-gui.exe"
sumoBinary = "sumo"
sumoCmd = [sumoBinary, "-c", f"{rootPath}\\ScenarioGenerator\\Scenario\\exp.sumocfg"]
logFile = f'{rootPath}\\RouteTSP\\log\\sys_%s.log' % datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d-%H-%M')
logging.basicConfig(filename=logFile, level=logging.INFO)
logging.info('Simulation start')
SIM_TIME = 3600
SIM_STEP = 1
BUS_DEP_INI = 0
MIN_STOP_DUR = 5
PER_BOARD_DUR = 3
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
G_MIN = 10  # 最小绿灯时间
COEFF_XC = 0.9  # 临界饱和系数
INI = -10000
POS_JUNC = np.array(posJunc).cumsum()
POS_STOP = np.concatenate([[0], POS_JUNC]) + np.array(posSet[0])
BUS_DEP_HW = 2*60
V_AVG = 12
TIMETABLE = np.array([20 + i*BUS_DEP_HW + (POS_STOP - POS_STOP[0])/V_AVG for i in range(100)])

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
                # 按人数停站
                traci.vehicle.setBusStop(self.id, stopId, duration=(MIN_STOP_DUR + PER_BOARD_DUR*sumoEnv.busStopDict[stopId].personNum))
                # traci.vehicle.setBusStop(self.id, stopId, duration=0)
                self.atBusStop = stopId
            else:
                # 注：每辆公交只经过每个站点1次
                self.waitTimeDict[self.atBusStop] += SIM_STEP
        else:
            self.atBusStop = None
        logging.info('[%s] Bus Loading Time:\n%s\n' % (str(timeStep), str(self.waitTimeDict)))

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

class sumoEnv:
    allBusDict = {}
    runningBusDict = {}
    busStopDict = {}
    trafficLightDict = {}
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
        env.update(traci.vehicle.getIDList(), timeStep)
        traci.simulationStep()

    env.record()
    print("Ploting...\n")
    myplot(POS_JUNC, BUS_PHASE[0])
    traci.close()