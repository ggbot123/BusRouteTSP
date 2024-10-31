SPEED_LIMIT = 15 #速度限制

#定义路段类型type
def output_road_types():
    str_types = '<types>\n' 
    str_types += '  <type id="mainStreet" numLanes="4" speed="%.2f"/>\n' % SPEED_LIMIT 
    str_types += '  <type id="crossStreet" numLanes="3" speed="%.2f"/>\n' % SPEED_LIMIT 
    str_types += '</types>\n' 
    return str_types