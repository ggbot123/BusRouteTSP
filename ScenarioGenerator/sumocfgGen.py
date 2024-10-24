def output_config(thread=None):
    if thread is None:
        out_file = 'exp.rou.xml'
    else:
        out_file = 'exp_%d.rou.xml' % int(thread)
    str_config = '<configuration>\n  <input>\n'
    str_config += '    <net-file value="exp.net.xml"/>\n'
    str_config += '    <route-files value="%s"/>\n' % out_file
    str_config += '    <additional-files value="exp.add.xml,E1_info.xml"/>\n'
    str_config += '  </input>\n  <time>\n'
    str_config += '    <begin value="0"/>\n    <end value="7200"/>\n'
    str_config += '  </time>\n</configuration>\n'
    return str_config