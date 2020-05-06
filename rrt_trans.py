"""
Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

author: tdy
"""

import math
import random
import time

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np

show_animation = True

from transducer import Once


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y, t):
            self.x = x
            self.y = y
            self.t = t
            self.path_x = []
            self.path_y = []
            self.path_t = []
            self.state_path = {'Once':{(0, 10): [], (10, 18): []}}
            self.parent = None
            self.child = []
            self.state = {'Once':{(0, 10): None, (10, 18): None}}
            self.cost = 0

    def __init__(self, start, obstacle_list, rand_area,
                 expand_dis = 3.0, path_resolution = 0.5, max_iter = 500, allow_speed = 3, cover_threshold = 35, goal_sample_rate = 5):
        """
        Setting Parameter

        start:Start Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [(min,min,min),(max,max,max)]

        """
        self.start = self.Node(start[0], start[1], 0)
        self.rand_area = rand_area
        #self.min_rand = rand_area[0]
        #self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.allow_speed = allow_speed
        self.cover_threshold = cover_threshold
        self.select_nd = []
        self.goal_sample_rate = goal_sample_rate
        self.task_point = {'Once':{(0, 10): [(17, 8, 3), (10, 16, 3)],  (10, 18): [(5, 6, 3)]}}
        self.start_node = self.Node(15, 15, 20)


    def planning(self, animation=True):
        """
        rrt path planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_node = self.get_nearest_node_index(self.node_list, rnd_node)

            if nearest_node == None:
                continue

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            for k, v in self.task_point['Once'].items():
                checker = Once(self.rand_area[1][2], nearest_node, new_node, k)
                new_node.state['Once'][k] = checker.trans()

            print("this new node")
            for k, v in new_node.state['Once'].items():
                print('bound:',k,'value',v)

            print('check result', self.check_propose(new_node))
            print('aval', self.check_aval(new_node))

            #time.sleep(2)

            if self.check_collision(new_node, self.obstacle_list) and self.check_propose(new_node):
                self.node_list.append(new_node)
                new_node.parent = nearest_node
                d_n, _ = self.calc_distance_and_angle(new_node, nearest_node)
                new_node.cost = new_node.parent.cost + d_n
                nearest_node.child.append(new_node)
                # add selectable points (t and coverage satisfy the conditions)
                #if self.calc_cover(new_node) <= self.cover_threshold and self.check_aval(new_node):
                #    self.select_nd.append(new_node)
                if self.rand_area[1][2] - new_node.t <= 5 and self.check_aval(new_node):
                    self.select_nd.append(new_node)

            #self.draw_graph(rnd_node)

        self.draw_rrt()
        self.draw_path_3D(self.start_node)
        self.draw_path_2D(self.start_node)

    def steer(self, from_node, to_node, extend_length=float('inf')):

        new_node = self.Node(from_node.x, from_node.y, from_node.t)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        speed = self.calc_speed(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]
        new_node.path_t = [new_node.t]


        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        #here need to transducer state

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.t += self.path_resolution / speed
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
            new_node.path_t.append(new_node.t)
            #construct the state path


        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.path_t.append(to_node.t)

        self.check_task_point(new_node)
        return new_node

    # def generate_final_course(self, goal_ind):
    #     path = [[self.end.x, self.end.y]]
    #     node = self.node_list[goal_ind]
    #     while node.parent is not None:
    #         path.append([node.x, node.y])
    #         node = node.parent
    #     path.append([node.x, node.y])
    #
    #     return path

    def check_task_point(self, node): #construct the state path
        for index, value in enumerate(node.path_t):
            for k, v in self.task_point['Once'].items():
                if (value > (self.rand_area[1][2] - k[1]) and value < (self.rand_area[1][2] - k[0])):
                    dist = [(node.path_x[index] - item[0]) ** 2 + (node.path_y[index] - item[1]) ** 2 < item[2] ** 2 for item in v]
                    if any(dist):
                        node.state_path['Once'][k].append(True)
                    else:
                        node.state_path['Once'][k].append(False)



    def check_propose(self, node):
        for k, v in node.state['Once'].items():
            if v is False:
                return False
        return True

    def check_aval(self, node):
        for k, v in node.state['Once'].items():
            if v is False or v is None:  #attention!!! can't write as if v is False or None. or lian jie 2 biao da shi
                return False
        return True

    def calc_cover(self, node):
        return 3.14*((self.rand_area[1][2]-node.t)*self.allow_speed)**2

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.rand_area[0][0], self.rand_area[1][0]),
                            random.uniform(self.rand_area[0][1], self.rand_area[1][1]),
                            self.rand_area[1][2])

            while not self.check_resident(rnd, self.obstacle_list):
                rnd = self.Node(random.uniform(self.rand_area[0][0], self.rand_area[1][0]),
                                random.uniform(self.rand_area[0][1], self.rand_area[1][1]),
                                self.rand_area[1][2])
        rnd = self.Node(random.uniform(self.rand_area[0][0], self.rand_area[1][0]),
                        random.uniform(self.rand_area[0][1], self.rand_area[1][1]),
                        random.uniform(self.rand_area[0][2], self.rand_area[1][2]))

        while not self.check_resident(rnd,self.obstacle_list):
            rnd = self.Node(random.uniform(self.rand_area[0][0], self.rand_area[1][0]),
                            random.uniform(self.rand_area[0][1], self.rand_area[1][1]),
                            random.uniform(self.rand_area[0][2], self.rand_area[1][2]))
        return rnd

    def draw_graph(self, rnd = None):
        plt.figure(3)
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        ax = plt.subplot(111, projection='3d')

        if rnd is not None:
            #plt.plot(rnd.x, rnd.y, rnd.t, "^k")
            ax.scatter(rnd.x, rnd.y, rnd.t, c='r', marker='1')
        # for node in self.node_list:
        #     if node.parent:
        #         #plt.plot(node.path_x, node.path_y, "-g")
        #         ax.scatter(node.path_x, node.path_y, node.path_t, c='g', marker='^')

        #draw the rrt
        for node in self.node_list:
            ax.scatter(node.x, node.y, node.t, c='g', marker='^')
            if node.parent:
                self.plot_line(node)

        for (ox, oy, size) in self.obstacle_list:
            x, y, z = self.plot_cylinder(ox, oy, size)
            ax.plot_surface(x, y, z, color = 'b', shade = False)

        for k, v in self.task_point['Once'].items():
            for item in v:
                x, y, z = self.plot_cylinder_cons(item[0], item[1] ,(self.rand_area[1][2] - k[1] ,self.rand_area[1][2] - k[0]), item[2])
                ax.plot_surface(x, y, z, color='y', shade=False)

        ax.scatter(self.start.x, self.start.y, 0,"xr")
        print("this is a new iteration!")
        #draw the cover of select nodes
        for node in self.select_nd:
            self.plot_circle(node.x, node.y, self.rand_area[1][2], (self.rand_area[1][2]-node.t)*self.allow_speed, color = 'r')
            #plt.text(node.x, node.y, node.t, node.cost, ha='center', va='bottom', fontsize=20)

            print(node.cost)

        plt.axis("equal")
        plt.axis([0, 20, 0, 20])
        plt.grid(True)
        plt.pause(0.01)

    #delete the other nodes
    def draw_rrt(self):
        plt.figure(2)
        plt.clf()
        ax3d = plt.axes(projection = '3d')

        #draw the rrt
        for node in self.select_nd:
            ax3d.scatter(node.x, node.y, node.t, c='k', marker='^')
            cn = node
            while(cn.parent is not None):
                self.plot_line(cn)
                cn = cn.parent

        for (ox, oy, size) in self.obstacle_list:
            x, y, z = self.plot_cylinder(ox, oy, size)
            ax3d.plot_surface(x, y, z, color = 'b', shade = False, alpha=0.5)

        for k, v in self.task_point['Once'].items():
            for item in v:
                x, y, z = self.plot_cylinder_cons(item[0], item[1] ,(self.rand_area[1][2] - k[1] ,self.rand_area[1][2] - k[0]), item[2])
                ax3d.plot_surface(x, y, z, color='y', shade=False, alpha=0.5)

        ax3d.scatter(self.start.x, self.start.y, 0,"xr")

        #draw the cover of select nodes
        # for node in self.select_nd:
        #     self.plot_circle(node.x, node.y, self.rand_area[1][2], (self.rand_area[1][2]-node.t)*self.allow_speed, color = 'r')
            #plt.text(node.x, node.y, node.t, node.cost, ha='center', va='bottom', fontsize=20)


        plt.axis("equal")
        plt.axis([0, 20, 0, 20])
        plt.grid(True)
        plt.pause(0.01)

    #construct the path in two dimensions, then can get the start point through ginput()
    def draw_path_2D(self, start_point):  # start point needs to be a node
        plt.figure(1)
        plt.clf()

        reach_list = [x for x in self.select_nd if self.calc_speed(x, start_point) < self.allow_speed]
        if len(reach_list) == 0:
            print('No path is available!')
            return None

        dlist = [math.hypot(node.x - start_point.x, node.y - start_point.y) + node.cost for node in reach_list]
        minind = dlist.index(min(dlist))
        nearest_select_node = reach_list[minind]

        start_point.parent = nearest_select_node
        cn = start_point
        while (cn.parent is not None):
            self.plot_line_2D(cn)
            plt.scatter(cn.x, cn.y, c='k', marker='^')
            plt.text(cn.x, cn.y, "{:.2f}".format(self.rand_area[1][2] - cn.t))
            cn = cn.parent

        plt.scatter(cn.x, cn.y, c='k', marker='^')
        plt.text(cn.x, cn.y, "{:.2f}".format(self.rand_area[1][2] - cn.t))


        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle_2D(ox, oy, size)


        for k, v in self.task_point['Once'].items():
            for item in v:
                self.plot_circle_2D(item[0], item[1], item[2], color = 'y')
                plt.text(item[0], item[1], str(k))


        # draw the cover of select nodes
        # for node in self.select_nd:
        #     self.plot_circle(node.x, node.y, self.rand_area[1][2], (self.rand_area[1][2]-node.t)*self.allow_speed, color = 'r')
        # plt.text(node.x, node.y, node.t, node.cost, ha='center', va='bottom', fontsize=20)

        plt.axis("equal")
        plt.axis([0, 20, 0, 20])
        plt.grid(True)
        plt.pause(0.01)

    #construct the path in three dimensions
    def draw_path_3D(self, start_point):  #start point needs to be a node
        plt.figure(4)
        plt.clf()
        ax3d = plt.axes(projection='3d')

        reach_list = [x for x in self.select_nd if self.calc_speed(x, start_point) < self.allow_speed]
        if len(reach_list) == 0:
            print('No path is available!')
            return None

        dlist = [math.hypot(node.x - start_point.x, node.y - start_point.y)+ node.cost for node in reach_list]
        minind = dlist.index(min(dlist))
        nearest_select_node = reach_list[minind]
        ax3d.scatter(start_point.x, start_point.y, start_point.t, c='k', marker='^')
        start_point.parent = nearest_select_node
        cn = start_point
        while (cn.parent is not None):
            self.plot_line(cn)
            cn = cn.parent

        for (ox, oy, size) in self.obstacle_list:
            x, y, z = self.plot_cylinder(ox, oy, size)
            ax3d.plot_surface(x, y, z, color='b', shade=False, alpha=0.5)

        for k, v in self.task_point['Once'].items():
            for item in v:
                x, y, z = self.plot_cylinder_cons(item[0], item[1],
                                                  (self.rand_area[1][2] - k[1], self.rand_area[1][2] - k[0]), item[2])
                ax3d.plot_surface(x, y, z, color='y', shade=False, alpha=0.5)

        ax3d.scatter(self.start.x, self.start.y, 0, "xr")

        # draw the cover of select nodes
        # for node in self.select_nd:
        #     self.plot_circle(node.x, node.y, self.rand_area[1][2], (self.rand_area[1][2]-node.t)*self.allow_speed, color = 'r')
        # plt.text(node.x, node.y, node.t, node.cost, ha='center', va='bottom', fontsize=20)

        plt.axis("equal")
        plt.axis([0, 20, 0, 20])
        plt.grid(True)
        plt.pause(0.01)

    def plot_circle(self, x, y, t, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, t, color)

    def plot_circle_2D(self, x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    def plot_line(self, node):
        if node.parent:
            lin_x = np.linspace(node.x, node.parent.x, 100)
            lin_y = np.linspace(node.y, node.parent.y, 100)
            lin_t = np.linspace(node.t, node.parent.t, 100)
            plt.plot(lin_x, lin_y, lin_t, c = 'b')

    def plot_line_2D(self, node):
        if node.parent:
            lin_x = np.linspace(node.x, node.parent.x, 100)
            lin_y = np.linspace(node.y, node.parent.y, 100)
            plt.plot(lin_x, lin_y, c = 'b')

    def plot_cylinder(self, x, y, size):
        h = np.linspace(0, self.rand_area[1][2], 100)  # 把高度均分为100�?
        h.shape = (100,1)
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        p = np.ones(len(xl))
        p.shape = (1,len(xl))
        z = p * h
        return np.array(xl), np.array(yl), z

    def plot_cylinder_cons(self, x, y, bound, size):
        h = np.linspace(bound[0], bound[1], 100)  # 把高度均分为100�?
        h.shape = (100,1)
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        p = np.ones(len(xl))
        p.shape = (1,len(xl))
        z = p * h
        return np.array(xl), np.array(yl), z



    def get_nearest_node_index(self, node_list, rnd_node):
        #judge time. the node with smaller time
        #past_list = list(filter(lambda x: x.t < rnd_node.t, node_list))
        past_list = [x for x in node_list if x.t < rnd_node.t]
        #judge the speed. smaller than the allow_speed
        reach_list = [x for x in past_list if self.calc_speed(x,rnd_node) < self.allow_speed]
        if len(reach_list) == 0:
            return None
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)
                 ** 2 + (node.t - rnd_node.t) ** 2 for node in reach_list]
        minind = dlist.index(min(dlist))
        nearest_node = reach_list[minind]
        return nearest_node


    def check_collision(self, node, obstacleList):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size ** 2:
                return False  # collision

        return True  # safe

    def check_resident(self, node, obstacleList):
        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy

            if d <= size ** 2:
                return False  # collision
        return True  # safe


    def calc_distance_and_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


    def calc_speed(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dt = to_node.t - from_node.t
        d = math.hypot(dx, dy)
        speed = d/dt
        return speed


def main(gx=6.0, gy=10.0):
    print("start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [
        (5, 16, 1),
        (15, 5, 1),
        (10, 10, 0.5),
    ]  # [x, y, radius]
    # Set Initial parameters
    rrt = RRT(start=[5.5, 6.5],
              rand_area=[(0,0,0),(20,20,20)],
              obstacle_list=obstacleList)
    path = rrt.planning(animation=show_animation)

    # Draw final path
    if show_animation:
        rrt.draw_graph()
        rrt.draw_rrt()

        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()

# if you want to show the rrt procedure, comment the draw_rrt in main() and planning(), and uncomment the draw_graph in main() and planning()
if __name__ == '__main__':
    main()
