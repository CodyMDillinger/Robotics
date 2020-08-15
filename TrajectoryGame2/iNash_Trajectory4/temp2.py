def near_vertices2(vertex_new, current_vertex, prev_vertex, k, vertices_near, axis_):           # uses kd tree to efficiently return approximate nearest vertex
    if k == 1:
        radius = Dimensions.eta
    else:
        radius = min(Dimensions.gamma * sqrt(log(k) / k), Dimensions.eta)       # function of numPoints
    if radius > dist(current_vertex, vertex_new) > 0 and current_vertex.at_goal_set is False:
        vertices_near.append(current_vertex)

    if getattr(vertex_new, axis_.axis) > getattr(current_vertex,
                                                 axis_.axis):  # if new vertex is to the right of or above the current vertex
        next_vert1 = current_vertex.right_child  # then traverse towards the new vertex (right)
        next_vert2 = current_vertex.left_child
    else:
        next_vert1 = current_vertex.left_child  # else traverse towards the new vertex (left)
        next_vert2 = current_vertex.right_child
    if next_vert1 is not None:  # if there is a child in this direction
        vertices_near = near_vertices2(vertex_new, next_vert1, current_vertex, k, vertices_near, axis_.next_)
    if next_vert2 is not None:  # if there is a child in this direction
        if abs(getattr(current_vertex, axis_.axis) - getattr(vertex_new, axis_.axis)) > radius:
            # prune subtree
            pass
        else:
            vertices_near = near_vertices2(vertex_new, next_vert2, current_vertex, k, vertices_near, axis_.next_)

    return vertices_near