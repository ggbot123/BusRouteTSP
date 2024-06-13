def get_edge_str(edge, from_node, to_node):
    edge_id = '%s_%s' % (from_node, to_node)
    edge_id_inv = '%s_%s' % (to_node, from_node)
    return edge % (edge_id, from_node, to_node) + edge % (edge_id_inv, to_node, from_node)

#定义边edge
def output_edges(edge):
    str_edges = '<edges>\n' 
    # source roads
    str_edges += get_edge_str(edge, 'np1', 'nt1')
    str_edges += get_edge_str(edge, 'np2', 'nt1')
    str_edges += get_edge_str(edge, 'np3', 'nt1')
    str_edges += get_edge_str(edge, 'np4', 'nt2')
    str_edges += get_edge_str(edge, 'np5', 'nt2')
    str_edges += get_edge_str(edge, 'np6', 'nt3')
    str_edges += get_edge_str(edge, 'np7', 'nt3')
    str_edges += get_edge_str(edge, 'np8', 'nt4')
    str_edges += get_edge_str(edge, 'np9', 'nt4')
    str_edges += get_edge_str(edge, 'np10', 'nt5')
    str_edges += get_edge_str(edge, 'np11', 'nt5')
    # network roads
    str_edges += get_edge_str(edge, 'nt1', 'nt2')
    str_edges += get_edge_str(edge, 'nt2', 'nt3')
    str_edges += get_edge_str(edge, 'nt3', 'nt4')
    str_edges += get_edge_str(edge, 'nt4', 'nt5')
    # sink roads
    str_edges += get_edge_str(edge, 'nt5', 'np12')
    str_edges += '</edges>\n'
    return str_edges