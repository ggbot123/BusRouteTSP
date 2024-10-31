def get_con_str(con, from_node, cur_node, to_node_straight, to_node_left, to_node_right, type):
    from_edge = '%s_%s' % (from_node, cur_node) #from_edge和to_edge都是edge_id
    to_edge_left = '%s_%s' % (cur_node, to_node_left)
    to_edge_straight = '%s_%s' % (cur_node, to_node_straight)
    to_edge_right = '%s_%s' % (cur_node, to_node_right)
    # return con % (from_edge, to_edge_left, 1, 1) + con % (from_edge, to_edge_straight, 0, 0) + con % (from_edge, to_edge_right, 0, 0)
    if type == 'mainStreet':
        return con % (from_edge, to_edge_left, 3, 2) + con % (from_edge, to_edge_straight, 2, 2) + con % (from_edge, to_edge_straight, 1, 1) + con % (from_edge, to_edge_right, 0, 0)
    else:
        return con % (from_edge, to_edge_left, 2, 3) + con % (from_edge, to_edge_straight, 1, 1) + con % (from_edge, to_edge_right, 0, 0)

#定义连接器connection
def output_connections(con):
    str_cons = '<connections>\n' 
    # cross nt1
    str_cons += get_con_str(con, 'np1', 'nt1', 'nt2', 'np2', 'np3', 'mainStreet')
    str_cons += get_con_str(con, 'np2', 'nt1', 'np3', 'nt2', 'np1', 'crossStreet')
    str_cons += get_con_str(con, 'np3', 'nt1', 'np2', 'np1', 'nt2', 'crossStreet')
    str_cons += get_con_str(con, 'nt2', 'nt1', 'np1', 'np3', 'np2', 'mainStreet')
    # cross nt2
    str_cons += get_con_str(con, 'nt1', 'nt2', 'nt3', 'np4', 'np5', 'mainStreet')
    str_cons += get_con_str(con, 'np4', 'nt2', 'np5', 'nt3', 'nt1', 'crossStreet')
    str_cons += get_con_str(con, 'np5', 'nt2', 'np4', 'nt1', 'nt3', 'crossStreet')
    str_cons += get_con_str(con, 'nt3', 'nt2', 'nt1', 'np5', 'np4', 'mainStreet')
    # cross nt3
    str_cons += get_con_str(con, 'nt2', 'nt3', 'nt4', 'np6', 'np7', 'mainStreet')
    str_cons += get_con_str(con, 'np6', 'nt3', 'np7', 'nt4', 'nt2', 'crossStreet')
    str_cons += get_con_str(con, 'np7', 'nt3', 'np6', 'nt2', 'nt4', 'crossStreet')
    str_cons += get_con_str(con, 'nt4', 'nt3', 'nt2', 'np7', 'np6', 'mainStreet')
    # cross nt4
    str_cons += get_con_str(con, 'nt3', 'nt4', 'nt5', 'np8', 'np9', 'mainStreet')
    str_cons += get_con_str(con, 'np8', 'nt4', 'np9', 'nt5', 'nt3', 'crossStreet')
    str_cons += get_con_str(con, 'np9', 'nt4', 'np8', 'nt3', 'nt5', 'crossStreet')
    str_cons += get_con_str(con, 'nt5', 'nt4', 'nt3', 'np9', 'np8', 'mainStreet')
    # cross nt5
    str_cons += get_con_str(con, 'nt4', 'nt5', 'np12', 'np10', 'np11', 'mainStreet')
    str_cons += get_con_str(con, 'np10', 'nt5', 'np11', 'np12', 'nt4', 'crossStreet')
    str_cons += get_con_str(con, 'np11', 'nt5', 'np10', 'nt4', 'np12', 'crossStreet')
    str_cons += get_con_str(con, 'np12', 'nt5', 'nt4', 'np11', 'np10', 'mainStreet')
    str_cons += '</connections>\n'
    return str_cons