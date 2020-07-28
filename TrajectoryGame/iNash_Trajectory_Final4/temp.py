class Vertex:
    def __init__(self, x_val, y_val):  # initialize with x, y coordinates
        self.x = x_val
        self.y = y_val
        self.children = []
        self.parents = []
        self.paths = []
        self.costs = []

    def add_child(self, child):
        self.children.append(child)

    def add_parent(self, parent):
        self.parents.append(parent)

    def add_path(self, path_):
        self.paths.append(path_)     # for list of paths from root node to this vertex

    def add_cost(self, cost_):
        self.costs.append(cost_)     # for list of costs associated with paths in self.paths

    x = 0                           # x,y coordinates
    y = 0
    obs_next = None                 # for shape edges (obstacles and goal set)
    at_goal_set = False
    k_nearest = 0                   # for avoiding checking points multiple times in recursion searches
    k_near = 0
    left_child = None               # for k-d tree (spatial sorting algorithm, faster near/nearest functions)
    right_child = None


a = Vertex(1, 1)
b = Vertex(10, 10)
c = Vertex(45, 23)

a.add_path([1, 1, 1])
a.add_path([2, 2, 2, 2])
b.add_path([7, 7])

print a.paths
print b.paths
print c.paths