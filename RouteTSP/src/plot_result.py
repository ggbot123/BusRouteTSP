from tools import plot_result, plot_result_for_each_intersection, plot_result_for_each_movement, plot_result_for_route

if __name__ == '__main__':
    testDirList = ['blank_exact', 'TSP_only_ref12', 'SA_only', 'origin_exact']
    saveDIR = 'Ts dev = 0'
    Algo_name = ['Blank', 'TSP Only', 'Speed Only', 'Proposed']

    testDirList = ['blank', 'origin_0.7_5-25-10_nolb', 'SP_0.7_5-25-10_nolb']
    saveDIR = 'Ts dev = 10'
    Algo_name = ['Blank', 'Vanilla TSP-SA', 'TSP-SA + SP']

    testDirList = ['blank_avg10_max12_Ts30_dev10_extra8', 'origin_exact_avg10_max12_Ts30_dev10_extra8', 'SP_avg10_max12_Ts30_dev10_extra8']
    saveDIR = 'Ts30_dev10_max12_avg10'
    Algo_name = ['Blank', 'Vanilla TSP-SA', 'TSP-SA + SP']

    testDirList = ['blank_avg10_max12_noQ_lowV', 'origin_avg10_max12_Ts30_dev10_noQ_lowV', 'SP_avg10_max12_Ts30_dev10_noQ_lowV']
    saveDIR = 'Ts30_dev10_max12_avg10_noQ_lowV'
    Algo_name = ['Blank', 'Vanilla TSP-SA', 'TSP-SA + SP']

    # testDirList = ['blank_test', 'origin_test', 'SP_test']
    # testDirList = ['blank_test', 'SP_test']
    testDirList = ['blank_test']
    testDirList = ['origin_test_YP']
    testDirList = ['origin_test']

    saveDIR = 'test'
    Algo_name = ['Blank', 'Vanilla TSP-SA', 'TSP-SA + SP']

    PI_name = ['busArrTimeDev', 'busArrTimeDevStd', 'lateRate', 'busHeadwayVar']
    yAxis_name = ['Deviation (s)', 'Std Var of Deviation (s)', 'Late Rate (%)', 'Std Var of Time Headway (s)']
    
    # plot_result(saveDIR, testDirList, PI_name, yAxis_name, Algo_name)
    # plot_result_for_each_intersection(saveDIR, testDirList)
    # plot_result_for_each_movement(saveDIR, testDirList)

    route = "('np1_nt1', 'nt1_nt2', 'nt2_nt3', 'nt3_nt4', 'nt4_nt5', 'nt5_np12')"
    phase = 2
    plot_result_for_route(testDirList, route, phase)