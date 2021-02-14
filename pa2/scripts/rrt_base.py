import numpy as np
import copy

class GoalBiasedGreedySteerKNeighborhoodRRTStarBase:
    # define rrt* node
    class Node:
        def __init__(self):
            self.children = []
            self.parent = None
            self.c= None
            self.goal = None
            self.cost = 0

    def __init__(self, seed):
        '''Constructor with seed, to be supplied to the np.random.RandomState object held inside. Feel free to add things.'''
        self.random = np.random.RandomState(seed)
        self.root = None
        # keep configs in the list
        self.configs = []
    # you will not implement this
    def distance(self, c1, c2):
        '''returns the distance between two configurations c1, c2.'''
        pass
    # not yet
    # If the distance between the last point tested and c2 is less than ss or d(c1,c2) < ss, c2 is tested against collision.
    def steer(self, c0, c, step_size):
        '''starting from the configuration c0, gets closer to
        c in discrete steps of size step_size in terms of distance(c0, c). returns the last no-collision
        configuration. If no collisions are hit during steering, returns c2. If
        steering is not possible at all, returns None.'''
        pass
    # you will not implement this
    def allclose(self, c1, c2):
        '''returns True if two configurations are numerically very close (just use np.allclose default values).'''
        pass
    # you will not implement this
    def sample(self, p):
        '''either returns goal configuration with some goal bias probability p or returns
        a no collision configuration sample with 1-p. The goal bias becomes 0 as soon as goal node is found.'''
        pass
    # not tested
    def neighborhood(self, c, k):
        '''returns a list of k closest nodes to configuration c in terms of distance(q.value, c)'''
        # Stack to store the nodes 
        nodes=[]

        # keep [node, distance] pairs in a list
        distances = []

        # push the current node onto the stack 
        nodes.append(self.root) 
    
        # Tree traversal
        # loop while the stack is not empty 
        while (len(nodes)):  
    
            # store the current node and pop it from the stack 
            curr = nodes[0] 
            nodes.pop(0) 
    
            # if the current node and c is not super close, add it to distances
            if self.allclose(c,curr.c) != True:
                dist = self.distance(c, curr.c)
                n_d = [curr, dist]
                distances.append(n_d)

            # store all the children of current node from 
            # right to left. 
            for it in range(len(curr.children)-1,-1,-1):  
                nodes.insert(0,curr.children[it])
    

        # Using set and list comprehension, get rid of duplicates in nearest neighbours
        seen = set()        
        new_distances = [(a, b) for a, b in distances 
                if not (b in seen or seen.add(b))]

        # sort neigbours according to distances
        new_distances.sort(key=lambda tup: tup[1])

        # extract nodes from distances as distances contain [node, distance] pairs
        neighbours = [item[0] for item in new_distances]

        # get first k elements
        if len(neighbours) > k:
            neighbours = neighbours[:k]

      
        return neighbours

    def init_rrt(self, c_init, c_goal):
        '''initializes/resets rrt with the root node holding c_init and goal configuration c_goal.'''
        # set c_init and c_goal in root
        root = self.Node()
        root.c = c_init
        root.goal = c_goal
        self.root = root
        
    # you will not implement this
    def valid(self, c):
        '''returns True if configuration c is non-colliding.'''
        pass
    # not yet
    def collision_free(self, c1, c2, step_size):
        '''returns True if the linear trajectory between c1 and c2 are collision free.'''
        pass

    # check if this configuration is already in the tree
    def exists_c(self, c):
        for i in self.configs:
            if self.allclose(i,c):
                return True

        return False


    def add_node(self, p, k, step_size):
        '''adds a node to the rrt with goal bias probability p, near function with k closest neighbors,
        and step_size for greedy steering. returns the new configuration that is now part of the tree.'''
        # random configuration
        sample_c = self.sample(p)
        # k closest neighbours
        near = self.neighborhood(sample_c, k)

        # nearest neighbour's configuration
        nearest_c = near[0].c
        # After steering as much as possible towards sample config from nearest neighbour
        new_c = self.steer(nearest_c, sample_c, step_size)
        while new_c is None or self.exists_c(new_c):

            sample_c = self.sample(p)
            # k closest neighbours
            near = self.neighborhood(sample_c, k)

            # nearest neighbour's configuration
            nearest_c = near[0].c

            # After steering as much as possible towards sample config from nearest neighbour
            new_c = self.steer(nearest_c, sample_c, step_size)

        # nearest neighbour
        x_min = near[0]
        # cost from nearest neighbour to new config
        cost_min = x_min.cost + self.distance(x_min.c, new_c)
        # connect along a minimum cost path
        for x_near in near:
            collision_free_p = self.collision_free(x_near.c, new_c, step_size)
            x_near_new_cost = x_near.cost + self.distance(x_near.c, new_c)
            if collision_free_p and x_near_new_cost < cost_min:
                x_min = x_near
                cost_min = x_near_new_cost
        
        # add new config to tree
        x_new = self.Node()
        x_new.c = new_c
        x_new.parent = x_min
        x_new.cost = cost_min
        x_min.children.append(x_new)
        self.configs.append(x_min.c)


        # Rewire the tree
        for x_n in near:
            collision_free_p = self.collision_free(x_n.c, x_new.c, step_size)
            x_near_new_cost = x_new.cost + self.distance(x_new.c, x_n.c)
            # if collision free and cost via new node is smaller than old cost
            if collision_free_p and x_near_new_cost < x_n.cost:

                parent = x_n.parent
                # delete x_near from its parents children list
                if parent != None:
                    for i in range(len(parent.children)):
                        if parent.children[i] is x_n:
                            parent.children.pop(i)
                            break
                    # set new parent and cost
                    x_n.cost = x_near_new_cost
                    x_n.parent = x_new
                    x_new.children.append(x_n)

        return x_new.c


    def get_path_to_goal(self):
        '''returns the path to goal node as a list of tuples of configurations[(c_init, c1),(c1, c2),...,(cn,c_goal)].
        If the goal is not reachable yet, returns None.'''

        # get goal
        config_goal = self.root.goal
        goal_node = None

        # First find the goal node via tree traversal
        # Stack to store the nodes 
        nodes=[]

        # push the current node onto the stack 
        nodes.append(self.root) 
    
        # loop while the stack is not empty 
        while (len(nodes)):  
    
            # store the current node and pop it from the stack 
            curr = nodes[0] 
            nodes.pop(0) 
    
            # check if current node is the goal
            if self.allclose(curr.c, config_goal):
                goal_node = curr
                break
            # store all the children of current node from 
            # right to left. 
            for it in range(len(curr.children)-1,-1,-1):  
                nodes.insert(0,curr.children[it])

        # goal not founf
        if goal_node is None:
            return None

        path = []
        # get path by going backwards from goal
        if goal_node != None:
            current = goal_node
            p = current.parent
            # only root has a none parent
            while p is not None:
                edge = (p.c, current.c)
                path.append(edge)
                current = p
                p = current.parent

        # reverse path
        path.reverse()
        if path is not None:
            return path

        return None

        

    def is_goal_reachable(self):
        '''returns True if goal configuration is reachable.'''
        # get goal
        config_goal = self.root.goal
    
        # First find the goal node
        # Stack to store the nodes 
        nodes=[]

        # push the current node onto the stack 
        nodes.append(self.root) 
    
        # loop while the stack is not empty 
        while (len(nodes)):  
    
            # store the current node and pop it from the stack 
            curr = nodes[0] 
            nodes.pop(0) 
    
            # check if current node is the goal
            # return True if yes
            if self.allclose(curr.c, config_goal):
                return True
            # store all the children of current node from 
            # right to left. 
            for it in range(len(curr.children)-1,-1,-1):  
                nodes.insert(0,curr.children[it])
        
        return False

    def simplify_path(self, path, step_size):
        '''greedily removes redundant edges from a configuration path represented as a list of tuples
        of configurations [(c_init,c1),(c1,c2),(c2,c3)...(cn-1,cn)(cn,c_goal)], as described
        Principles of Robot Motion, Theory, Algorithms and Implementations (2005), p. 213,
        Figure 7.7.(use the default version, always try to connect to c_goal, not the other way around'''
        simplified_path = []
        if path is not None:
            # copy path not to change it
            path_copy = copy.deepcopy(path)
            path_len = len(path)
            i = 0
            t = path_len-1
            # add the goal as a tuple to the end to only traverse first elements of tuples
            path_copy.append((path[t][1], path[t][1]))
            t = len(path_copy) - 1
            # if we had not reached initial node
            while t != 0:
                ci = path_copy[i][0]
                ct = path_copy[t][0]
                # if collision free, set qgoal to the connected node
                # add that piece to simplified path
                if self.collision_free(ci, ct, step_size):
                    simplified_path.append((ci, ct))
                    t = i
                    i = 0
                else:
                    i = i+1
        return simplified_path


    def get_all_edges(self):
        '''returns all of the edges in the tree as a list of tuples of configurations [(c1,c2),(c3,c4)...]. The
        order of the edges, The order of edges in the list and their direction is not important.'''

        # keep the edges
        edges = []

        # tree traversal
        # Stack to store the nodes 
        nodes=[]

        # push the current node onto the stack 
        nodes.append(self.root) 
    
        # loop while the stack is not empty
        while (len(nodes)):  
    
            # store the current node and pop it from the stack 
            curr = nodes[0] 
            nodes.pop(0) 
    
            # add the (parent, current) edge to list
            if curr is not self.root:
                e = (curr.parent.c, curr.c)
                edges.append(e)
            
            # store all the children of current node from 
            # right to left. 
            for it in range(len(curr.children)-1,-1,-1):  
                nodes.insert(0,curr.children[it])

        return edges
        

    def get_goal_cost(self):
        '''returns the non-simplified goal path length in terms of distance. Returns np.Inf if goal is not reachable.'''

        # check if goal is reachable, return np.Inf if not
        goal_reachable = self.is_goal_reachable()
        if goal_reachable == False:
            return np.Inf
        # get path to goal
        path = self.get_path_to_goal()

        # check path size for safety
        if path is not None and len(path) > 0:
            first_edge = path[0]
            dist = self.distance(first_edge[0], first_edge[1])
            # add all the edge distances between configs
            for i in range(1, len(path)):
                e = path[i]
                d = self.distance(e[0],e[1])
                dist = dist + d

            return dist
        # return inf if goal not reached
        return np.Inf
        

