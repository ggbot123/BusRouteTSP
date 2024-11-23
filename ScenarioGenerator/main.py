import os
import nodeGen
import edgeGen
import connectionGen
import roadTypeGen
import signalTimingGen
import netconfigGen
import flowGen
import sumocfgGen
import busStopGen
import busFlowGen
import personFlowGen


#定义修改文件的函数
def write_file(path, content):
    with open(path, 'w') as f:
        f.write(content)

def generate_net():
    # nod.xml file
    node = '  <node id="%s" x="%.2f" y="%.2f" type="%s"/>\n'
    write_file('./Scenario/exp.nod.xml', nodeGen.output_nodes(node))

    # typ.xml file
    write_file('./Scenario/exp.typ.xml', roadTypeGen.output_road_types())

    # edg.xml file
    edge = '  <edge id="%s" from="%s" to="%s" type="%s"/>\n'
    write_file('./Scenario/exp.edg.xml', edgeGen.output_edges(edge))

    # con.xml file
    con = '  <connection from="%s" to="%s" fromLane="%d" toLane="%d"/>\n'
    write_file('./Scenario/exp.con.xml', connectionGen.output_connections(con))

    # tll.xml file
    tls = '  <tlLogic id="%s" programID="0" offset="%s" type="static">\n' 
    phase = '    <phase duration="%d" state="%s"/>\n'
    write_file('./Scenario/exp.tll.xml', signalTimingGen.output_tls(tls, phase))

    # net config file
    write_file('./Scenario/exp.netccfg', netconfigGen.output_netconfig()) 

    # generate net.xml file
    os.system('netconvert -c ./Scenario/exp.netccfg') #相当于在Windows的cmd窗口中输入的命令；对netconvert执行netccfg配置文件以此生成net.xml文件

def generate_route():
    # raw.rou.xml file
    # flow = '  <flow id="f_%s" from="%s" to="%s" begin="%d" end="%d" vehsPerHour="%f" type="type1"/>\n'
    flow = '  <flow id="f_%s" from="%s" to="%s" begin="%d" end="%d" departLane="%s" probability="%.3f" type="type1"/>\n'
    write_file('./Scenario/exp.raw.rou.xml', flowGen.output_flows(flow))
    os.system('duarouter -n ./Scenario/exp.net.xml -t ./Scenario/exp.raw.rou.xml -o ./Scenario/exp.rou.xml')

def generate_add():
    # add.xml file
    str_adds = '<additional>\n'

    stops = '   <busStop id="%s" lane="%s" color="blue" startPos="%f" endPos="%f" line="%s"/>\n'
    trip = '   <trip id="%s" type="BUS" depart="%f" departPos="%f" from="%s" to="%s">\n'
    trip_stops = '      <stop busStop="%s" duration="%f"/>\n'
    person = '   <person id="%s" depart="%f" departPos="%f" type="PED">\n'
    person_walks = '      <walk from="%s" busStop="%s"/>\n'
    person_rides = '      <ride busStop="%s" line="%s"/>\n'

    str_adds += (busStopGen.output_busStops(stops) + busFlowGen.output_busFlow(trip, trip_stops) + personFlowGen.output_personFlow(person, person_walks, person_rides) + '</additional>\n') 
    # str_adds += (busStopGen.output_busStops(stops) + busFlowGen.output_busFlow(trip, trip_stops) + '</additional>\n') 

    write_file('./Scenario/exp.add.xml', str_adds)

def main():
    generate_net()
    generate_route()
    generate_add()

    # config file
    write_file('./Scenario/exp.sumocfg', sumocfgGen.output_config())

if __name__ == '__main__':
    main()