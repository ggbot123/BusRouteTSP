# 定义信号配时方案
def output_tls(tls, phase):
    str_adds = '<additional>\n'
    # consider cross with 4 phases
    four_phases = ['GrGGrrGrGGrr', 'GryGrrGryGrr', 'GGrGrrGGrGrr', 'GyrGrrGyrGrr',
                   'GrrGrGGrrGrG', 'GrrGryGrrGry', 'GrrGGrGrrGGr', 'GrrGyrGrrGyr']
    # two_phases = ['GGrr', 'yyrr', 'rrGG', 'rryy']
    phase_duration = [30, 3]
    for i in range(1, 6):
        str_adds += tls % ('nt' + str(i))
        for k, p in enumerate(four_phases): 
            #enumerate()函数用于将一个可遍历的数据对象(如列表、元组或字符串)
            #组合为一个索引序列，同时列出数据下标和数据
            str_adds += phase % (phase_duration[k % 2], p)
        str_adds += '  </tlLogic>\n'
    str_adds += '</additional>\n'
    return str_adds