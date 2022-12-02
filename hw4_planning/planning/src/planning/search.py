"""Implementation of A* and shortcutting."""
from __future__ import absolute_import, division, print_function
import collections
import numpy as np
import networkx as nx

from itertools import count
from cs4750.utils import PriorityQueue
from planning.RRTTree import RRTTree
from planning import problems
import time
import sys
from matplotlib import pyplot as plt

# Entries in the PriorityQueue are prioritized in lexicographic order, i.e.  by
# the first element of QueueEntry (the f-value). Ties are broken by proceeding
# to the next element, a unique counter. This prevents the underlying
# PriorityQueue from breaking ties by comparing nodes. QueueEntry objects
# contain the node, a possible parent, and cost-to-come via that parent. These
# can be accessed via dot notation, e.g. `entry.node`.
QueueEntry = collections.namedtuple(
    "QueueEntry",
    ["f_value", "counter", "node", "parent", "cost_to_come"],
)

NULL = -1

class ASTARPlanner(object):
    def __init__(self, rm):
        self.rm = rm 

    def Plan(self, start, goal):
        """Compute the shortest path from start to goal on a roadmap.

        Args:
            start, goal: integer labels for the start and goal states

        Returns:
            vpath: a sequence of node labels, including the start and goal
        """
        if start not in self.rm.graph or goal not in self.rm.graph:
            msg = "Either start {} or goal {} is not in G"
            raise nx.NodeNotFound(msg.format(start, goal))

        # expanded is a Boolean array that tracks whether each node has previously
        # been expanded, in order to avoid expanding nodes multiple times.
        expanded = np.zeros(self.rm.num_vertices, dtype=bool)

        # parents is an int array that tracks the best parent for each node. It
        # defaults to NULL (-1) for each node.
        parents = NULL * np.ones(self.rm.num_vertices, dtype=int)

        # heuristic_cache is a float array that caches the returned heuristic value.
        heuristic_cache = NULL * np.ones(self.rm.num_vertices)

        c = count()
        queue = PriorityQueue()
        queue.push(QueueEntry(self.rm.heuristic(start, goal), next(c), start, NULL, 0))

        while len(queue) > 0:
            entry = queue.pop()
            if expanded[entry.node]:
                continue

            expanded[entry.node] = True
            parents[entry.node] = entry.parent

            if entry.node == goal:
                path = self.extract_path(parents, goal)
                return path, parents

            for neighbor, w in self.rm.graph[entry.node].items():
                # Get the edge weight (length) and cache the heuristic value
                weight = w.get("weight", 1)
                if heuristic_cache[neighbor] == NULL:
                    heuristic_cache[neighbor] = self.rm.heuristic(neighbor, goal)
                h = heuristic_cache[neighbor]

                # Compute this neighbor's cost-to-come via entry.node, and insert a
                # new QueueEntry.
                
                ### BEGIN SOLUTION "QUESTION 1.2"
                if expanded[neighbor]:
                    continue
                ncost = entry.cost_to_come + weight
                queue.push(QueueEntry(ncost + h, next(c), neighbor, entry.node, ncost))
                ### END SOLUTION
        raise nx.NetworkXNoPath("Node {} not reachable from {}".format(goal, start))


    def extract_path(self, parents, goal):
        """Extract the shortest path from start to goal.

        Args:
            parents: np.array of integer node labels
            goal: integer node label for the goal state

        Returns:
            vpath: a sequence of node labels from the start to the goal
        """
        # Follow the parents of the node until a NULL entry is reached
        ### BEGIN SOLUTION "QUESTION 1.2" ALT="raise NotImplementedError"
        path = []
        node = goal
        while node != NULL:
            path.append(node)
            node = parents[node]
        path.reverse()
        return path
        ### END SOLUTION



class RRTPlanner(object):

    def __init__(self, problem, map, bias = 0.05, eta = 1.0, max_iter = 100000, show_tree = False, batch_size = 1, shortcut = False):
        self.prob = problem         # Problem Environment
        self.map = map              # Map Environment
        self.tree = RRTTree(self.prob)
        self.bias = bias                # Goal Bias
        self.max_iter = max_iter        # Max Iterations
        self.eta = eta                  # Distance to extend
        self.start = None
        self.end = None
        self.show_tree = show_tree
        self.collision = 0
        self.batch_size = batch_size
        self.shortcut = shortcut
        self.shorten_path = None

    def Plan(self, start_config, goal_config, epsilon=0.001):
        
        # Initialize an empty plan.
        plan_time = time.time()
        self.start = start_config.reshape((1,-1))
        self.end = goal_config.reshape((1,-1))

        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(self.start)
        goal_id = -1

        if isinstance(self.prob, problems.R2Problem) and self.show_tree:
            plt.ion()
            fig = plt.figure()
            ax = fig.add_subplot(111)
            ax.imshow(self.map, cmap=plt.cm.gray, aspect="equal", interpolation="none", vmin=0, vmax=1, origin="lower",extent=self.prob.extents.ravel()[:4],)
            plt.scatter(self.start[:, 0], self.start[:, 1], c="g", zorder=3)
            plt.scatter(self.end[:, 0], self.end[:, 1], c="r", zorder=3)
        if isinstance(self.prob, problems.JointSpace):
            dim = 6
        else:
            dim = 2
        found = False
        batch_size = self.batch_size
            
        for _ in range(self.max_iter):
            
            if batch_size > 1:
                xs_rand = self.sample_batch(self.end, batch_size)
            else:
                xs_rand = self.sample(self.end)
            new_eid = []
            for i in range(xs_rand.shape[0]):
                x_rand = xs_rand[i].reshape((1,dim))
                ''' BEGIN QUESTION 2.1
                + we provide you, x_rand, a point sampled from space
                + From the tree, find the nearest node to x_rand
                + Step towards from x_near to x_rand (Hint: self.extend)
                + If the step is collision free, add vertex and edge to tree
                + stop iteration if distance to goal is less than epsilon threshold
                '''
                sid, x_near = self.tree.GetNearestVertex(x_rand)
                x_new = self.extend(x_near, x_rand)
                if x_new is not None:
                    # Add to tree
                    eid = self.tree.AddVertex(x_new)
                    new_eid.append(eid)
                    self.tree.AddEdge(sid, eid)
                    
                    if self.prob.compute_distance(x_new, self.end) < epsilon:
                        goal_id = eid
                        found = True
                        break
            '''
            End QUESTION 2.1
            '''
            if isinstance(self.prob, problems.R2Problem) and self.show_tree:
                for eid in new_eid:
                    # if eid == self.tree.GetRootID():
                    #     continue
                    econfig = self.tree.vertices[eid]
                    sconfig = self.tree.vertices[self.tree.edges[eid]]
                    x = [sconfig[:, 0], econfig[:, 0]]
                    y = [sconfig[:, 1], econfig[:, 1]]
                    ax.plot(x, y, 'r')

                fig.canvas.draw()
                fig.canvas.flush_events()

            if found:
                break

        if goal_id < 0:
            print("WARNING: RRT Failed!")
            sys.exit(-1)

        # Construct plan
        '''TODO
            Construct plan.
        '''
        plan = self.end
        
        cid = goal_id
        root_id = self.tree.GetRootID()
        while cid != root_id:
            cid = self.tree.edges[cid]
            plan = np.vstack((self.tree.vertices[cid], plan))
        '''
        End TODO
        '''

        plan_time = time.time() - plan_time
        cost = 0
        for i in range(plan.shape[0] - 1):
            cost += np.linalg.norm(plan[i+1] - plan[i])

        print("Planning complete!")
        print("Cost: %f" % cost)
        print("Planning Time: %ds" % plan_time)
        print("Sampled Collisions: %d" % self.collision)
        if self.shortcut:
            self.shorten_path = self.ShortenPath(plan)
        return plan

    def sample(self, goal):
        # Sample random point from map
        if self.prob.name == "JointSpace":
            if np.random.uniform() < self.bias:
                return goal
            return self.prob.sample()

        if np.random.uniform() < self.bias:
            return goal
        clear = np.argwhere(self.map == True)
        idx = np.random.choice(len(clear))
        return clear[idx, :].reshape((1, 2)).astype(float)

    def sample_batch(self, goal, n):
        if self.prob.name == "JointSpace":
            if np.random.uniform() < self.bias:
                for i in range(n - 1):
                    if i == 0:
                        samples = self.prob.sample()
                    else:
                        samples = np.vstack((samples, self.prob.sample()))
                samples = np.vstack((samples, self.end))
            else:
                for i in range(n):
                    if i == 0:
                        samples = self.prob.sample()
                    else:
                        samples = np.vstack((samples, self.prob.sample()))
            return np.array(samples)
        clear = np.argwhere(self.map == True)
        if np.random.uniform() < self.bias:
            idx = np.random.choice(len(clear), n-1)
            return np.vstack((goal, clear[idx, :].reshape((n-1, 2)).astype(float)))
        idx = np.random.choice(len(clear), n)
        return clear[idx, :].reshape((n, 2)).astype(float)

    def extend(self, x_near, x_rand):
        '''
        BEGIN QUESTION 2.1
        Implement an extend logic.
        Inputs: 
            x_near: node in the current tree that is the closest to x_rand
            x_rand: newly sampled random point
        Return:
            A point on the path from x_near to x_rand, step size depends on self.eta. 
            Remember to use self.env.edge_validity_checker() to check for collision
            Return None if the new edge is in collision. 
        '''
        x_goal = x_near + self.eta * (x_rand - x_near)
        if self.prob.check_edge_validity(x_near, x_goal):
            return x_goal
        '''END QUESTION 2.1'''
        self.collision += 1
        return None

    def ShortenPath(self, path):
        '''
        Q2.2 Graduate Student Only
        Input:
            path: plan generated by vanilla RRT, np.array of size (n, dim)
            Hint: use self.prob.check_edge_validity to check for collision.
                  The function will return True if collision free.
        '''
        i = 0
        num = path.shape[0]
        while True:
            if i + 2 >= path.shape[0]:
                i = 0
                if path.shape[0] == num:
                    break
                else:
                    num = path.shape[0]
            if self.prob.check_edge_validity(path[i], path[i+2]):
                path = np.delete(path, i+1, 0)
            i += 1
        return path
        

    def visualize_plan(self, plan, tree = None, visited = None):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        '''

        # disable interactive mode
        plt.ioff()

        visit_map = np.copy(self.map)

        if visited is not None:
            visit_map[visited == 1] = 0.5

        plt.imshow(
            visit_map,
            cmap=plt.cm.gray,
            aspect="equal",
            interpolation="none",
            vmin=0,
            vmax=1,
            origin="lower",
            extent=self.prob.extents.ravel()[:4],
        )

        for i in range(np.shape(plan)[0] - 1):
            x = [plan[i, 0], plan[i+1, 0]]
            y = [plan[i, 1], plan[i+1, 1]]
            plt.plot(x, y, 'b')
        
        if self.shortcut:
            for i in range(np.shape(self.shorten_path)[0] - 1):
                x = [self.shorten_path[i, 0], self.shorten_path[i+1, 0]]
                y = [self.shorten_path[i, 1], self.shorten_path[i+1, 1]]
                plt.plot(x, y, 'g')
        

        plt.scatter(self.start[:, 0], self.start[:, 1], c="g", zorder=3)
        plt.scatter(self.end[:, 0], self.end[:, 1], c="r", zorder=3)
        plt.show()