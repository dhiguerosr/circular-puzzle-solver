import numpy as np
import Queue as queue

# Up, dowm, left, right, Up-right, Left-down, Right-down, Up-left
valid_moves = [[0, 0, 0, 0, 0, 0, 0, 0], #Corner
               [0, 1, 0, 1, 0, 1, 0, 0],
               [0, 1, 1, 0, 0, 0, 1, 0],
               [0, 0, 0, 0, 0, 0, 0, 0], #Corner
               [0, 1, 0, 1, 1, 0, 0, 0],
               [1, 1, 1, 1, 0, 0, 0, 0],
               [1, 1, 1, 1, 0, 0, 0, 0],
               [0, 1, 1, 0, 0, 0, 0, 1],
               [1, 0, 0, 1, 0, 0, 1, 0],
               [1, 1, 1, 1, 0, 0, 0, 0],
               [1, 1, 1, 1, 0, 0, 0, 0],
               [1, 0, 1, 0, 0, 1, 0, 0],
               [0, 0, 0, 0, 0, 0, 0, 0], #Corner
               [1, 0, 0, 1, 0, 0, 0, 1],
               [1, 0, 1, 0, 1, 0, 0, 0],
               [0, 0, 0, 0, 0, 0, 0, 0]] #Corner
class Node:
    def __init__(self, val, goal, heuristic, parent = None):
        self.goal = goal
        self.val = val
        self.goal = goal
        self.parent = parent
        if self.parent == None:
            self.G = 0
        else:
            self.G = self.parent.G + 1
        self.H = 0
        for i in range(len(self.val)):
            for j in range(len(self.val[i])):
                if self.val[i][j] != None and self.val[i][j] != 12:
                    if(heuristic == 0):
                        gi, gj = np.where(self.goal == self.val[i][j])
                        self.H += self.manhattan_distance(i, j, gi[0], gj[0])
                    else:
                        self.H += 0 if self.val[i][j] == self.goal[i][j] else 1

    def score(self):
        return self.G + self.H

    def manhattan_distance(self, i, j, x, y):
        return abs(i - x) + abs(j - y)

    def rebuild_path(self):
        path = []
        curr = self
        while curr != None:
            path.append(curr)
            curr = curr.parent
        return path

    def neighbors(self):
        i, j = np.where(self.val == 12)
        blank = i[0] * len(self.val) + j[0]
        moves = valid_moves[blank]
        suc_list = []
        #Up
        if(moves[0] == 1):
            suc_array = self.val.copy()
            suc_array[i[0], j[0]], suc_array[i[0] -1, j[0]] = suc_array[i[0] - 1, j[0]], suc_array[i[0], j[0]]
            suc_list.append(Node(suc_array, self.goal, heuristic, self))
        #Down
        if(moves[1] == 1):
            suc_array = self.val.copy()
            suc_array[i[0], j[0]], suc_array[i[0] + 1, j[0]] = suc_array[i[0] + 1, j[0]], suc_array[i[0], j[0]]
            suc_list.append(Node(suc_array, self.goal, heuristic, self))
        #Left
        if(moves[2] == 1):
            suc_array = self.val.copy()
            suc_array[i[0], j[0]], suc_array[i[0], j[0] - 1] = suc_array[i[0], j[0] - 1], suc_array[i[0], j[0]]
            suc_list.append(Node(suc_array, self.goal, heuristic, self))
        #Right
        if(moves[3] == 1):
            suc_array = self.val.copy()
            suc_array[i[0], j[0]], suc_array[i[0], j[0] + 1] = suc_array[i[0], j[0] + 1], suc_array[i[0], j[0]]
            suc_list.append(Node(suc_array, self.goal, heuristic, self))
        #Up-right
        if(moves[4] == 1):
            suc_array = self.val.copy()
            suc_array[i[0], j[0]], suc_array[i[0] - 1, j[0] + 1] = suc_array[i[0] - 1, j[0] + 1], suc_array[i[0], j[0]]
            suc_list.append(Node(suc_array, self.goal, heuristic, self))
        #Left-down
        if(moves[5] == 1):
            suc_array = self.val.copy()
            suc_array[i[0], j[0]], suc_array[i[0] + 1, j[0] - 1] = suc_array[i[0] + 1, j[0] - 1], suc_array[i[0], j[0]]
            suc_list.append(Node(suc_array, self.goal, heuristic, self))
        #Right-down
        if(moves[6] == 1):
            suc_array = self.val.copy()
            suc_array[i[0], j[0]], suc_array[i[0] + 1, j[0] + 1] = suc_array[i[0] + 1, j[0] + 1], suc_array[i[0], j[0]]
            suc_list.append(Node(suc_array, self.goal, heuristic, self))
        #Up-left
        if(moves[7] == 1):
            suc_array = self.val.copy()
            suc_array[i[0], j[0]], suc_array[i[0] - 1, j[0] - 1] = suc_array[i[0] - 1, j[0] - 1], suc_array[i[0], j[0]]
            suc_list.append(Node(suc_array, self.goal, heuristic, self))
        return suc_list

def aStar(start, goal, heuristic, max_steps):
    counter = 0
    closedSet = {}
    openQueue = queue.PriorityQueue()
    solutionNode = None
    root = Node(start,goal, heuristic)
    openQueue.put((root.score(), counter, root))
    counter += 1
    while not openQueue.empty():
        current = openQueue.get()[2]
        closedSet[current.val.tostring()] = current
        for neighbor in current.neighbors():
            if neighbor.val.tostring() == goal.tostring():
                return neighbor
            if neighbor.val.tostring() in closedSet:
                continue
            if neighbor.G <= max_steps:
                openQueue.put((neighbor.score(), counter, neighbor))
                counter += 1
    return None

"""
MAIN
"""
f = open("parametros.txt", "r")
f1 = f.readlines()
p1 = int(f1[0].strip())
p2 = int(f1[1].strip())
p3 = int(f1[2].strip())
p4 = int(f1[3].strip())
p5 = int(f1[4].strip())
p6 = int(f1[5].strip())
p7 = int(f1[6].strip())
p8 = int(f1[7].strip())
p9 = int(f1[8].strip())
p10 = int(f1[9].strip())
p11 = int(f1[10].strip())
p12 = int(f1[11].strip())
max_steps = int(f1[12].strip()) # Numero maximo de pasos
heuristic = int(f1[13].strip()) # 0: Manhattan 1: Casillas fuera de lugar

start = np.array([[None,p8,p1,None],[p7,p12,p9,p2],[p6,p11,p10,p3], [None,p5,p4,None]])
goal = np.array([[None,8,1,None],[7,12,9,2],[6,11,10,3], [None,5,4,None]])
goalNode = aStar(start, goal, heuristic, max_steps)
if goalNode != None:
    result = goalNode.rebuild_path()
    
    while(result != []):
        i = result.pop()
        b = i.val.ravel()
        print "({}, g(x) = {}, h(x) = {})".format(b, i.G, i.H)
        file = open("resultado.txt", "a+")
        file.write("({}, g(x) = {}, h(x) = {})".format(b, i.G, i.H) + "\n")
        file.close()
else:
    print "No se encontro solucion factible"
