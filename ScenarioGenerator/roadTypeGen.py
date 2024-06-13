SPEED_LIMIT = 20 #速度限制

#定义路段类型type
def output_road_types():
    str_types = '<types>\n' 
    str_types += '  <type id="a" numLanes="2" speed="%.2f"/>\n' % SPEED_LIMIT 
    str_types += '</types>\n' 
    return str_types