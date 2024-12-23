import numpy as np
import pickle

# 定义信号配时方案
def output_tls(tls, phase):
    str_adds = '<additional>\n'
    # allPhases = [[('GrrGGrGrrGrG', 15.0), ('GrrGGrGrrGry', 4.0), ('GrrGGrGrrGGr', 21.0), ('GrrGyrGrrGGr', 4.0), ('GrrGrGGrrGGr', 11.0), ('GrrGryGrrGyr', 4.0), 
    #           ('GrGGrrGrGGrr', 12.0), ('GrGGrrGryGrr', 3.0), ('GryGrrGryGrr', 1.0), ('GryGrrGGrGrr', 3.0), ('GGrGrrGGrGrr', 18.0), ('GyrGrrGyrGrr', 4)],
              
    #           [('GrrGrGGrrGGr', 7.0), ('GrrGryGrrGGr', 4.0), ('GrrGGrGrrGGr', 29.0), ('GrrGGrGrrGyr', 4.0), ('GrrGGrGrrGrG', 12.0), ('GrrGyrGrrGry', 4.0), 
    #           ('GGrGrrGGrGrr', 18.0), ('GyrGrrGGrGrr', 4.0), ('GrGGrrGGrGrr', 3.0), ('GrGGrrGyrGrr', 4.0), ('GrGGrrGrGGrr', 7.0), ('GryGrrGryGrr', 4)],

    #           [('GrrGGrGrrGrG', 15.0), ('GrrGGrGrrGry', 4.0), ('GrrGGrGrrGGr', 28.0), ('GrrGyrGrrGGr', 4.0), ('GrrGrGGrrGGr', 12.0), ('GrrGryGrrGyr', 4.0), 
    #            ('GGrGrrGGrGrr', 16.0), ('GyrGrrGyrGrr', 4.0), ('GrGGrrGrGGrr', 9.0), ('GryGrrGryGrr', 4)],

    #           [('GrrGGrGrrGGr', 38.0), ('GrrGGrGrrGyr', 4.0), ('GrrGGrGrrGrG', 2.0), ('GrrGyrGrrGrG', 4.0), ('GrrGrGGrrGrG', 12.0), ('GrrGryGrrGry', 4.0), 
    #            ('GGrGrrGGrGrr', 16.0), ('GGrGrrGyrGrr', 4.0), ('GGrGrrGrGGrr', 2.0), ('GyrGrrGrGGrr', 4.0), ('GrGGrrGrGGrr', 6.0), ('GryGrrGryGrr', 4)],

    #           [('GrrGrGGrrGGr', 11.0), ('GrrGryGrrGGr', 4.0), ('GrrGGrGrrGGr', 23.0), ('GrrGGrGrrGyr', 4.0), ('GrrGGrGrrGrG', 17.0), ('GrrGyrGrrGry', 4.0), 
    #            ('GrGGrrGGrGrr', 12.0), ('GryGrrGGrGrr', 4.0), ('GGrGrrGGrGrr', 7.0), ('GGrGrrGyrGrr', 4.0), ('GGrGrrGrGGrr', 6.0), ('GyrGrrGryGrr', 4)]]
    # offset = np.array([0, 59, 56, 16, 61]).cumsum()
    
    with open(r'E:\workspace\python\BusRouteTSP\tools\result\data.pkl', 'rb') as file:
        allPhases = pickle.load(file)
    offset = np.load(r'E:\workspace\python\BusRouteTSP\tools\result\offset.npy')
    # print(allPhases)
    # print(offset)

    for i in range(len(allPhases)):
        str_adds += tls % ('nt' + str(1 + i), str(offset[i]))
        phases = allPhases[i]
        for state, duration in phases: 
            #enumerate()函数用于将一个可遍历的数据对象(如列表、元组或字符串)
            #组合为一个索引序列，同时列出数据下标和数据
            str_adds += phase % (duration, state)
        str_adds += '  </tlLogic>\n'
    str_adds += '</additional>\n'
    return str_adds