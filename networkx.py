from matplotlib import pyplot as plt
import networkx as nx


class Maps:
    """
    创建地图
    """

    def __init__(self):
        self.nodes = ['S1', 'S2', 'A1', 'A2', 'B1', 'B2', 'C', 'G1', 'G2', "D"]
        self.edges1 = [
            ('S1', 'A1'), ('S1', 'A2'), ('A1', 'C'), ('A2', 'C'), ('B1', 'C'), ('D', 'E1'), ('E1', 'C'), ('D', 'E2'),
            ('E2', 'C'),
            ('B2', 'C'), ('S2', 'B1'), ('S2', 'B2'), ('G1', 'D'), ('G2', 'D'), ]
        self.edges = [
            ('S1', 'A'), ('S2', 'A'),
            ('A', 'B'), ('A', 'C'), ('C', 'D'), ('D', 'B'),
            ('G1', 'D'), ('G2', 'D')
        ]
        self.G = nx.Graph()
        self.G = nx.path_graph(0)
        for node in self.nodes:
            self.G.add_node(node)
        self.Gi = self.G.add_edges_from(self.edges)
        # nx.draw_networkx(self.G, node_color='y')
        # plt.show()


class Agentbase:
    """
    车类，包括车的id，当前位置crruent，终点目标goal
    """

    def __init__(self, id: str):
        self.agentid = id

class Agent(Agentbase):
    """
    车类的实例，继承车类Agentbase，添加车的信息
    """

    def __init__(self, id: str):
        super().__init__(id)
        self.goal=None  #目标站点
        self.crruent=None#当前站点

class Pathsrech(Maps):
    """
    寻找最短路径
    """

    def __init__(self):
        super().__init__()
        self.all_pathbest = None

    def pathserch(self, agent: Agent, start, goal, method='dijkstra'):
        self.method = method
        self.start = start
        self.goal = goal
        # self.path = nx.single_source_shortest_path(self.G, "G1")
        # self.length = nx.single_source_shortest_path_length(self.G, "G1")
        # print(self.path)
        # print(self.length)
        # 最短路算法 返回最短路的路径列表
        self.pathbest = {}
        self._pathbest = nx.shortest_path(self.G, self.start, self.goal, method=self.method)
        self.all_pathbest = nx.shortest_paths.all_shortest_paths(self.G, self.start, self.goal, method=self.method)
        # print(self.all_pathbest,"45678")
        self.pathbest[agent.agentid] = self._pathbest
        return self.pathbest

    def pathsrechall(self):
        pass


# class CT:
#     def __init__(self):
#         self.goalpathA={}
#         self.goalpathB = {}
#         self.cost={}
#     def newpath(self,pathA:dict,conflict:list):
#         #print(self.conflict)
#         self.conflict = conflict
#         self.conflicttime=self.conflict[3]
#         self.pathA=pathA
#         self.pathA_v=[]
#         self.pathA_k=""
#         (self.pathA_k, self.pathA_v), = self.pathA.items()
#         self.conflictbeforetime=self.conflicttime-1
#
#         pathA_conflictbefore=self.pathA_v[self.conflictbeforetime]
#         self.pathA_v.insert(self.conflicttime,pathA_conflictbefore)
#         #print("self.pathA_v",self.pathA_v)
#         self.newpathAB_A={self.pathA_k:self.pathA_v}
#         #self.newpathAB_A.update(pathB)
#         return self.newpathAB_A

# 有冲突后，更新路径函数，此方法：在冲突点前等待1个时间单位
#寻找最短路径
def newpath_svert(path: dict, conflict: list):
    (path_k, path_v), = path.items()
    pathA = {}
    pathA_v = []
    pathA_k = path_k

    for _path_v in path_v:
        pathA_v.append(_path_v)
    pathA[pathA_k] = pathA_v
    # print("==================长生新路径函数")
    # print("原始path", path)
    # print(conflict)
    conflicttime = conflict[3]
    (pathA_k, pathA_v), = pathA.items()
    conflictbeforetime = conflicttime - 1

    pathA_conflictbefore = pathA_v[conflictbeforetime]
    pathA_v.insert(conflicttime, pathA_conflictbefore)
    # print("self.pathA_v",self.pathA_v)
    newpathAB_A = {pathA_k: pathA_v}
    # print("pathA", newpathAB_A)
    # print("原始path", path)
    # print("==================长生新路径函数")
    return newpathAB_A


# class Sic:
#     def __init__(self):
#         pass
#
#     def sic(self, costA, costB):
#         """
#         计算cost，目前只根据len（var）的总和计算
#         """
#         self.costA = len(costA)
#         self.costB = len(costB)
#         self.cost = self.costA + self.costB
#         return self.cost


# class Conflict:
#     def __init__(self, agentA: dict, agentB: dict):
#         self.conflictlist = None
#         self.unconflict = []
#         self.conflicttime = 0
#         self.conflictvert = None
#         self.conflict = False
#         (self.agentA_k, self.agentA_v), = agentA.items()
#         (self.agentB_k, self.agentB_v), = agentB.items()
#         self.isconflict()
#
#     def isconflict(self):
#         self.minlen = min(len(self.agentA_v), len(self.agentB_v))
#         for _minlen in range(0, self.minlen):
#             # print(self.conflicttime)
#             self.conflicttime += 1
#             if self.agentA_v[self.conflicttime] == self.agentB_v[self.conflicttime]:
#                 self.conflictvert = self.agentA_v[self.conflicttime]
#                 for _conflicttime in range(0, self.conflicttime - 1):
#                     self.unconflict.append(self.agentB_v[_conflicttime])
#                 self.conflict = True
#                 # print(self.conflict)
#                 self.outconflict()
#                 return self.conflict
#         else:
#             return self.conflict
#
#     def outconflict(self):
#         """
#         查找冲突顶点,out[agentA，agentB，v,t]
#
#         """
#         if self.conflict == True:
#             print(self.agentA_k, " 和 ", self.agentB_k, "有冲突点", self.conflictvert)
#             self.conflictlist = [self.agentA_k, self.agentB_k, self.conflictvert, self.conflicttime]
#             return self.conflictlist


# class Constraints(Maps):
#     """
#     约束条件
#     """
#
#     def __init__(self):
#         super().__init__()
#         print("====")
#         self.uncfttoopenai = []
#         self.uncfttoopenaj = []
#         # nx.draw(G, with_labels=True, node_color='y', )
#         # G.add_nodes_from(nodes)
#         # G.add_weighted_edges_from(edges)
#         # path=nx.single_source_dijkstra_path(G,4)
#         # length=nx.single_source_dijkstra_path_length(G,4)
#         # print(path)
#         # print(length)
#
#     def Constraintscheck(self,a,b):
#         self.cftaj = None  # 冲突位置前置点i
#         self.cftaj = None  # 冲突位置前置点j
#         self.cftc = None  # 冲突位置
#         self.cfta=a
#         self.cftb=b
#         minlen=min(len(self.cfta),len(self.cftb))
#         for _minlen in range(1,minlen):
#             print()
#             if self.cfta[_minlen]==self.cftb[_minlen]:
#
#                 self.cftc=self.cfta[_minlen]
#                 self.cftai=self.cfta[_minlen-1]
#                 self.cftaj=self.cftb[_minlen-1]
#                 self.cftt=_minlen
#                 self.ConstraintsList=[self.cftai,self.cftaj,self.cftc,self.cftt]#['A1', 'B1', 'C', 2]
#                 for _uncfttoopenai in range(_minlen):
#                     self.uncfttoopenai.append(self.cfta[_uncfttoopenai])
#                 for _uncfttoopenaj in range(_minlen):
#                     self.uncfttoopenaj.append(self.cftb[_uncfttoopenaj])
#                 return self.ConstraintsList
#     def Constraintsdev(self,_ConstraintsList):
#         self.sftdevai=_ConstraintsList[0]
#         self.sftdevaj=_ConstraintsList[1]
#         self.sftdevc=_ConstraintsList[2]
#         self.sftdevt = _ConstraintsList[3]
#         self.sftdevailist=[self.sftdevai,self.sftdevc,self.sftdevt]
#         self.sftdevajlist=[self.sftdevaj,self.sftdevc,self.sftdevt]
#         return self.sftdevailist,self.sftdevajlist
#     def cbs(self):
#         pass
# 判断两路径是否冲突
def isconflict(a: dict, b: dict):
    conflict = False
    conflicttime = 0
    (ak, av), = a.items()
    (bk, bv), = b.items()
    minlen = min(len(av), len(bv))
    for _minlen in range(0, minlen):
        conflicttime += 1
        if av[conflicttime - 1] == bv[conflicttime - 1]:
            # print(av[conflicttime],bv[conflicttime])
            print("冲突触发isconflict")
            conflict = True
            return conflict
    return conflict


# 输出路径冲突的信息
def conflictlist(a: dict, b: dict):
    conflicttime = 0
    (ak, av), = a.items()
    (bk, bv), = b.items()
    minlen = min(len(av), len(bv))
    for _minlen in range(0, minlen):
        # print(av)
        conflicttime += 1
        # print(av[conflicttime], bv[conflicttime], conflicttime, minlen)
        if av[conflicttime] == bv[conflicttime]:
            conflictvert = av[conflicttime]
            print(ak, " 和 ", bk, "有冲突点", conflictvert)
            conflictlist = [ak, bk, conflictvert, conflicttime]
            return conflictlist


# 代价函数，只计算车经过站点数量作为代价
def SIC_Solution(pathA: dict, pathB: dict, opt: int = 1):
    (ak, av), = pathA.items()
    (bk, bv), = pathB.items()
    if opt == 1:
        cost = {}
        cost_v = len(av) + len(bv)
        cost["cost"] = cost_v
        return cost


if __name__ == '__main__':
    maps1 = Maps()  # 建图
    a1 = Agent("1号车")  # 创建车
    a2 = Agent("2号车")  # 创建车
    a3 = Agent("3号车")  # 创建车
    agentlist = [a1.agentid, a2.agentid, a3.agentid]
    print(agentlist)
    f = Pathsrech()  # 初始化路径搜索类

    goal_paths = {}  # 目标集合
    conflicts_set = {}  # 冲突集合
    Node_conflict = []
    unconflicts_set = {}
    Node_state_list = {}
    conflicts_state = True  # 冲突结束标志

    Node = 0  # 节点开始
    while_times = 0  # while循环次数
    a1path = f.pathserch(a1, "S1", "G1")  # 输入 起点 终点 搜索路径
    a2path = f.pathserch(a2, "S2", "G2")


    #主循环开始
    while conflicts_state:
        while_times += 1
        time = 0
        print(
            f"==============================================第{while_times}次循环================================================")

        # 开始首次预判没有站点冲突，退出
        if Node == 0:
            conflicts_state_sub = isconflict(a1path, a2path)  # 检查冲突
            if conflicts_state_sub:
                Node_conflict = conflictlist(a1path, a2path)  # 获得（a,b,v,t）保存
                print("节点产生", Node_conflict)
            else:
                print("没有站点冲突，退出！！！")
                conflicts_state = False
                break
            conflicts_set[str(Node)] = Node_conflict  # 获得（a,b,v,t）保存为dict，包含节点信息
            print("冲突集合", conflicts_set)

        # 站点冲突后，计算
        if Node_state_list != {}:
            for (Node_state_i, Node_state_j) in Node_state_list.items():
                # print("节点信息", Node_state_i, Node_state_j[3], Node_state_j[3]["conflict_state"], Node_state_j[0],Node_state_j[1])
                # print(type(Node_state_j[3]["conflict_state"]), Node_state_j[3]["conflict_state"])
                # print("===================conflict_state=================-----")
                if Node_state_j[3]["conflict_state"] == False:
                    goal_paths[str(Node_state_i)] = [Node_state_j[0], Node_state_j[1], Node_state_j[4]]
                    print("目标路径集合", goal_paths)
        if goal_paths != {}:
            min_cost = []
            for (goal_paths_i, goal_paths_j) in goal_paths.items():
                min_cost.append(goal_paths_j[2]["cost"])
            mincost_goal = min(min_cost)
            for (goal_paths_i, goal_paths_j) in goal_paths.items():
                if goal_paths_j[2]["cost"] == mincost_goal:
                    for _goal_paths_j in goal_paths_j:
                        print("CBS算法结果", goal_paths_j)
                        print("原路径", [a1path, a2path, SIC_Solution(a1path, a2path)])
                        # print("冲突---------",goal_paths_j[0], goal_paths_j[1])
                        conflicts_state_sub = isconflict(goal_paths_j[0], goal_paths_j[1])  # 检查冲突
                        if conflicts_state_sub:
                            Node += 1
                            Node_conflict = conflictlist(a1path, a2path)  # 获得（a,b,v,t）保存
                            conflicts_set[str(Node)] = Node_conflict  # 获得（a,b,v,t）保存为dict，包含节点信息
                            print("冲突集合", conflicts_set)
                        conflicts_state = False

                        break
        # 结束，并画图
        if not conflicts_state:
            """
            仅显示地图，不影响CBS核心，然后退出
            """
            print("结束CBS")
            nodes1 = ['S1', 'S2', 'A1', 'A2', 'B1', 'B2', 'C', 'G1', 'G2', "D"]
            nodes = ['S1', 'S2', 'A', 'B', 'C', 'D', 'G1', 'G2']
            edges1 = [
                ('S1', 'A1'), ('S1', 'A2'), ('A1', 'C'), ('A2', 'C'), ('B1', 'C'), ('D', 'E1'), ('E1', 'C'),
                ('D', 'E2'), ('E2', 'C'),
                ('B2', 'C'), ('S2', 'B1'), ('S2', 'B2'), ('G1', 'D'), ('G2', 'D'), ]
            edges = [
                ('S1', 'A'), ('S2', 'A'),
                ('A', 'B'), ('A', 'C'), ('C', 'D'), ('D', 'B'),
                ('G1', 'D'), ('G2', 'D')]
            edges3 = [
                ('S1', 'A1'), ('S1', 'A2'), ('A1', 'C'), ('A2', 'C'), ('B1', 'C'), ('D', 'E1'), ('E1', 'C'),
                ('D', 'E2'), ('E2', 'C'), ('S3', 'D1'), ('S3', 'D2'), ('D1', 'C'), ('D2', 'C'),
                ('B2', 'C'), ('S2', 'B1'), ('S2', 'B2'), ('G1', 'D'), ('G2', 'D'), ]

            G = nx.Graph()

            # G = nx.path_graph(0)
            for node in nodes:
                G.add_node(node,node_color="r")
            Gi = G.add_edges_from(edges)
            nx.draw_networkx(G, node_color=['y'])
            plt.show()
            break

        Node_state_list[str(Node)] = [a1path, a2path, conflicts_set, {"conflict_state": True}]  # 保存冲突节点信息;路径，冲突集合
        # print("节点列表", Node, Node_state_list)
        conflicts_set[str(Node)] = Node_conflict
        # print("冲突集合", conflicts_set)
        # 时间节点
        time += 1
        # 循环遍历

        for _agent in agentlist:
            Node += 1  # 节点加1
            print("for 节点", Node, _agent, "STAR-----------------------------------------------------", Node,
                  _agent)
            _Node_conflict = Node_conflict.copy()
            if _agent in _Node_conflict:
                # print(conflicts_set[str(time - 1)])
                if _agent == "1号车":
                    new1 = newpath_svert(a1path, conflicts_set[str(time - 1)])
                    # print("重新规划路径1", new1)
                    Node_state_list[str(Node)] = [new1, a2path, conflicts_set, {"conflict_state": False},
                                                  SIC_Solution(new1, a2path)]
                    _Node_conflict.remove(_agent)
                    # print("移除1号车", _Node_conflict)
                if _agent == "2号车":
                    new2 = newpath_svert(a2path, conflicts_set[str(time - 1)])
                    # print(ct.conflict)
                    # print("重新规划路径2", new2)
                    Node_state_list[str(Node)] = [a1path, new2, conflicts_set, {"conflict_state": False},
                                                  SIC_Solution(new2, a1path)]
                    _Node_conflict.remove(_agent)
                    # print("移除2号车", _Node_conflict)
                print(Node_conflict, "冲突结果", "节点", Node, Node_state_list)
