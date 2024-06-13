import traci
import matplotlib.pyplot as plt
import pandas as pd
import logging
import datetime
# import time
# import traci._busstop
# import traci.constants
# import traci.domain

sumoBinary = "E:\\software\\SUMO\\bin\\sumo-gui.exe"
sumoCmd = [sumoBinary, "-c", "../ScenarioGenerator/Scenario/exp.sumocfg"]
logFile = '.\\log\\sys_%s.log' % datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d-%H-%M')
logging.basicConfig(filename=logFile, level=logging.INFO)
logging.info('Simulation start')
SIM_TIME = 200
SIM_STEP = 1
CONTROL_STEP = 30
MIN_STOP_DUR = 5
PER_BOARD_DUR = 2

class Vehicle:
    def __init__(self, vehId, timeStep):
        self.id = vehId
        self.depart = timeStep
        self.roadID = traci.vehicle.getRoadID(vehId)
        self.laneID = traci.vehicle.getLaneID(vehId)
        self.lanePos = traci.vehicle.getLanePosition(vehId)
        self.speed = traci.vehicle.getSpeed(vehId)
        self.speedProfile = pd.Series([pd.NA]*timeStep + [self.speed])
    def update(self, timeStep):
        self.roadID = traci.vehicle.getRoadID(self.id)
        self.laneID = traci.vehicle.getLaneID(self.id)
        self.lanePos = traci.vehicle.getLanePosition(self.id)
        self.speed = traci.vehicle.getSpeed(self.id)
        self.speedProfile[timeStep] = self.speed

class Bus(Vehicle):
    def __init__(self, vehId, timeStep):
        super(Bus, self).__init__(vehId, timeStep)
        self.personNum = traci.vehicle.getPersonNumber(vehId)
        self.atBusStop = None
        self.waitTimeDict = {stopId: 0 for stopId in sumoEnv.busStopDict}
        logging.info('Bus created')
    def update(self, timeStep):
        super(Bus, self).update(timeStep)
        self.personNum = traci.vehicle.getPersonNumber(self.id)
        # logging.info('[%s] At Bus Stop:%s\n' % (str(timeStep), str(traci.vehicle.isAtBusStop(self.id))))
        # logging.info('[%s] Condition:%s\n' % (str(timeStep), str(not self.atBusStop and traci.vehicle.isAtBusStop(self.id))))

        # 刚到站时，根据客流量更新站点停站时长
        if traci.vehicle.isAtBusStop(self.id):
            if self.atBusStop is None:
                # 查询站点编号
                stopList = traci.vehicle.getNextStops(self.id)
                logging.info('[%s] Bus Stop List:\n%s\n' % (str(timeStep), str(stopList)))
                stopId = stopList[0][2]
                logging.info('[%s] Arrived Stop Id:\n%s\n' % (str(timeStep), str(stopId)))
                # 设置停站时间
                logging.info('[%s] %s\n' % (str(timeStep), str(sumoEnv.busStopDict)))
                traci.vehicle.setBusStop(self.id, stopId, duration=(MIN_STOP_DUR + PER_BOARD_DUR*sumoEnv.busStopDict[stopId].personNum))
                self.atBusStop = stopId
            else:
                # 注：每辆公交只经过每个站点1次
                self.waitTimeDict[self.atBusStop] += SIM_STEP
        else:
            self.atBusStop = None
        logging.info('[%s] Bus Loading Time:\n%s\n' % (str(timeStep), str(self.waitTimeDict)))
        # logging.info('[%s] Speed Profile for Bus <%s>:\n%s\n' % (str(timeStep), str(self.id), str(self.speedProfile)))
    # def control(self, speedRef):

class BusStop:
    def __init__(self, stopId):
        self.id = stopId
        self.personNum = traci.busstop.getPersonCount(stopId)
        self.personNumProfile = pd.Series([self.personNum])
    def update(self, timeStep):
        self.personNum = traci.busstop.getPersonCount(self.id)
        self.personNumProfile[timeStep] = self.personNum

class TrafficLight:
    def __init__(self, signalId):
        self.id = signalId
        self.curState = traci.trafficlight.getRedYellowGreenState(signalId)
        self.RGYProfile = pd.Series([self.curState])
    def update(self, timeStep):
        self.curState = traci.trafficlight.getRedYellowGreenState(self.id)
        self.RGYProfile[timeStep] = self.curState
        
class sumoEnv:
    allVehicleDict = {}
    allBusDict = {}
    allCarDict = {}
    runningVehicleDict = {}
    runningBusDict = {}
    runningCarDict = {}
    junctionDict = {}
    busStopDict = {}
    trafficLightDict = {}
    def __init__(self):
        sumoEnv.busStopDict = {"AtoB_0": BusStop("AtoB_0"), "AtoB_1": BusStop("AtoB_1"), "AtoB_2": BusStop("AtoB_2"), "AtoB_3": BusStop("AtoB_3"), "AtoB_4": BusStop("AtoB_4")}
        tlsIdList = traci.trafficlight.getIDList()
        sumoEnv.trafficLightDict = {tlsId: TrafficLight(tlsId) for tlsId in tlsIdList}
    def update(self, vehIdList, timeStep):
        # logging.info('[%s] Running Vehicle Id List:\n%s\n' % (str(timeStep), str(vehIdList)))
        logging.info('[%s] Running Vehicle Dict:\n%s\n' % (str(timeStep), str(self.runningVehicleDict)))
        self.runningVehicleDict = {key: veh for key, veh in self.runningVehicleDict.items() if key in vehIdList}
        for vehId in vehIdList:
            # 对于新生成的车辆，初始化对象
            if vehId not in self.runningVehicleDict:
                if traci.vehicle.getVehicleClass(vehId) != 'bus':
                    self.runningVehicleDict.update({vehId: Vehicle(vehId, timeStep)})
                    self.runningCarDict.update({vehId: Vehicle(vehId, timeStep)})
                else:
                    self.runningVehicleDict.update({vehId: Bus(vehId, timeStep)})
                    self.runningBusDict.update({vehId: Bus(vehId, timeStep)})
            # 对于已存在的车辆，更新状态
            else:
                self.runningVehicleDict[vehId].update(timeStep)
                if traci.vehicle.getVehicleClass(vehId) != 'bus':
                    self.runningCarDict[vehId].update(timeStep)
                else:
                    self.runningBusDict[vehId].update(timeStep)
        self.allVehicleDict.update(self.runningVehicleDict)
        self.allBusDict.update(self.runningBusDict)
        self.allCarDict.update(self.runningCarDict)

        for stopId in self.busStopDict:
            self.busStopDict[stopId].update(timeStep)

        for tlsId in self.trafficLightDict:
            self.trafficLightDict[tlsId].update(timeStep)
        

    def record(self):
        xlsSpeedFilePath = './result/speed_profile.xlsx'
        xlsBusStopFilePath = './result/personNum_profile.xlsx'
        xlsTLSFilePath = './result/tlsState_profile.xlsx'
        xlsBusSpeedFilePath = './result/bus_speed_profile.xlsx'
        speedProfileDict = {'Vehicle ' + vehId: veh.speedProfile for vehId, veh in self.allVehicleDict.items()}
        pd.DataFrame(speedProfileDict).to_excel(xlsSpeedFilePath)
        busSpeedProfileDict = {'Bus ' + vehId: veh.speedProfile for vehId, veh in self.allBusDict.items()}
        pd.DataFrame(busSpeedProfileDict).to_excel(xlsBusSpeedFilePath)
        personNumProfileDict = {'BusStop ' + stopId: stop.personNumProfile for stopId, stop in self.busStopDict.items()}
        pd.DataFrame(personNumProfileDict).to_excel(xlsBusStopFilePath)
        tlsStateProfileDict = {'TrafficLight ' + tlsId: tls.RGYProfile for tlsId, tls in self.trafficLightDict.items()}
        logging.info('TLS State List:\n%s\n' % (str(tlsStateProfileDict)))
        pd.DataFrame(tlsStateProfileDict).to_excel(xlsTLSFilePath)


    def plot(self):
        # 车辆速度曲线
        plt.figure(figsize=(10, 6))
        for vehId, veh in self.allVehicleDict.items():
            df = veh.speedProfile.dropna()
            plt.plot(SIM_STEP*df.index, df)       
        plt.xlabel('Time')
        plt.ylabel('Speed')
        plt.title('Speed-Time Sequences')
        # plt.legend()
        plt.grid(True)
        plt.savefig('./result/speed_plot.png')
        plt.show()

        # 公交车辆速度曲线
        plt.figure(figsize=(10, 6))
        for vehId, veh in self.allBusDict.items():
            df = veh.speedProfile.dropna()
            plt.plot(SIM_STEP*df.index, df)       
        plt.xlabel('Time')
        plt.ylabel('Speed')
        plt.title('Speed-Time Sequences')
        # plt.legend()
        plt.grid(True)
        plt.savefig('./result/busSpeed_plot.png')
        plt.show()

        # 公交站点客流曲线
        plt.figure(figsize=(10, 6))
        for stopId, stop in self.busStopDict.items():
            df = stop.personNumProfile.dropna()
            plt.plot(SIM_STEP*df.index, df)       
        plt.xlabel('Time')
        plt.ylabel('PersonNum')
        plt.title('PersonNum-Time Sequences')
        # plt.legend()
        plt.grid(True)
        plt.savefig('./result/personNum_plot.png')
        plt.show()

if __name__ == '__main__':
    traci.start(sumoCmd)
    env = sumoEnv()
    step = 0

    while step < SIM_TIME/SIM_STEP:
        env.update(traci.vehicle.getIDList(), step)
        # if step % CONTROL_STEP == 0:
        #     env.control()
        traci.simulationStep()
        step += 1

    env.record()
    env.plot()
    traci.close()