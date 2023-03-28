
"""
King's College London 
6CCE3ROS & 7CCEMROB Robotic Systems 
Week 28 Tutorial Question 2 - To complete
Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)
"""
import time

import math
import random

import matplotlib.pyplot as plt
import numpy as np

show_animation = True
node_dis = 3.0

class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
            self.g = 0.0

    class AreaBounds:

        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])


    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=4.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500, #500
                 play_area=None,
                 robot_radius=0.0,
                 ):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        play_area:stay inside this area [xmin,xmax,ymin,ymax]
        robot_radius: robot body modeled as circle with given radius
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius

    def neighbor(self,node):
        
        ln = self.Node(node.x - self.expand_dis,node.y)
        rn = self.Node((node.x + self.expand_dis),node.y)
        tn = self.Node(node.x, (node.y +self.expand_dis))
        bn = self.Node(node.x,(node.y -self.expand_dis))
        trn = self.Node(node.x+ self.expand_dis,node.y + self.expand_dis)
        tln = self.Node(node.x- self.expand_dis,node.y + self.expand_dis)
        brn = self.Node(node.x+ self.expand_dis,node.y - self.expand_dis)
        bln = self.Node(node.x- self.expand_dis,node.y - self.expand_dis)
        
        neighbor_ls = [ln,rn,tn,bn,trn,tln,brn,bln]
        legal_neighbor = []
        for neighbor in neighbor_ls:
            if self.check_if_outside_play_area(neighbor, self.play_area):
                legal_neighbor.append(neighbor)
        return legal_neighbor

    # Define the Euclidean distance as the heuristic function
    def euclidean_distance(self,a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

    def planning(self, animation=True):
        """
        rrt path planning
        animation: flag for animation on or off
        """
        
        #todo: complete the code for RRT path planning
        # Generate nodes
        self.node_list = [self.start]  # start and goal are always nodes
        
        open_set = {self.start:self.euclidean_distance(self.start,self.end)}
        closed_set={}
        sh = 0
        while open_set:
            current_node = min(open_set, key=open_set.get)
            
            if self.calc_dist_to_goal(current_node.x,current_node.y) <= self.expand_dis:
                final_node = self.steer(current_node, self.end, self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list, self.robot_radius):
                    self.node_list.append(final_node)
                    return self.generate_final_course(len(self.node_list) - 1)
            
            #hn = self.euclidean_distance(current_node,self.end)
            closed_set[current_node] = open_set[current_node]
            del open_set[current_node]
            for n in self.neighbor(current_node):
                cost = current_node.g + self.expand_dis
                #if self.check_collision(n, self.obstacle_list, self.robot_radius) or current_node in closed_set.keys():
                #    continue
                if n in open_set.keys():
                    if cost< n.g:
                        del open_set[n]
                        self.node_list.remove(n)
                if n in closed_set.keys():
                    if cost<n.g:
                        del closed_set[n]
                        self.node_list.remove(n)
                if n not in open_set.keys() and n not in closed_set.keys():
                    new_node = self.steer(current_node, n,self.expand_dis)
                    if self.check_collision(new_node, self.obstacle_list, self.robot_radius):
                        new_node.g = cost #current_node.g + self.path_resolution
                        open_set[new_node] = self.euclidean_distance(new_node,self.end) + new_node.g
                        self.node_list.append(new_node)
                    if animation and sh%10==0:
                        self.draw_graph(new_node)
            sh+=1
            '''
                    else: #n in open_set.keys():
                        if open_set[n] >= self.euclidean_distance(n,self.end) +g + self.path_resolution:
                            continue
                        else:
                    


        for i in range(self.max_iter):
            #complete here
            # Random node
            rnd_node = self.get_random_node()
            k_neighbor = 4
            k_set = []
            rest = list(self.node_list)
            #print(len(self.node_list))
            for k in range(k_neighbor):
                if len(rest)<= k:
                    break
                nearest_ind = self.get_nearest_node_index(rest, rnd_node)
                nearest_node = rest[nearest_ind]           
                

                new_node = self.steer(nearest_node, rnd_node, self.expand_dis)
                
                if self.check_if_outside_play_area(new_node, self.play_area) and \
                    self.check_collision(
                    new_node, self.obstacle_list, self.robot_radius):
                        self.node_list.append(new_node)
                rest.remove(rest[nearest_ind])

            #print(len(self.node_list))
            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)
            if self.calc_dist_to_goal(self.node_list[-1].x,
                            self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                            self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list, self.robot_radius):
                    return self.generate_final_course(len(self.node_list) - 1)
            if animation:
                self.draw_graph(rnd_node)
            '''
                
        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_if_outside_play_area(node, play_area):

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
           node.y < play_area.ymin or node.y > play_area.ymax:
            return False  # outside - bad
        else:
            return True  # inside - ok

    @staticmethod
    def check_collision(node, obstacleList, robot_radius):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size+robot_radius)**2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def main(gx=6.0, gy=10.0):
    print("start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                    (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
    # Set Initial parameters
    rrt = RRT(
        start=[0, 0],
        goal=[gx, gy],
        rand_area=[-2, 15],
        obstacle_list=obstacleList,
        play_area=[-2.0, 15.0, -2.0, 15.0],
        robot_radius=0.8
        )
    start_t = time.time()
    path = rrt.planning(animation=show_animation)
    end_t = time.time()

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        path_len = 0.0
        for i in range(1,len(path)):
            path_len += math.hypot(path[i][0]- path[i-1][0],path[i][1]- path[i-1][1])
        print('Time: ' + str(end_t-start_t))
        print('length: ' + str(path_len))
        # Draw final path
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()


if __name__ == '__main__':
    main()