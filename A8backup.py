import matplotlib.pyplot as plt
import math
startpt = [0, 0]
goalpt = [100, 100]
plt.axes()
obs1 = plt.Rectangle((50, 35), 5, 25, fc='r')
obs2 = plt.Rectangle((30, 55), 25, 5, fc='r')
plt.gca().add_patch(obs1)
plt.gca().add_patch(obs2)
x = []
y = []
xl = []
yl = []

obstacles = {"obst1": {"xmin": 50, "xmax": 55, "ymin": 35, "ymax": 60},
             "obs2": {"xmin": 30, "xmax": 55, "ymin": 55, "ymax": 65}}


def check_obstruct(current):
    flag = True
    for key1 in obstacles:
        if current[0] >= obstacles[key1]["xmin"] and current[0] <= obstacles[key1]["xmax"]:
            if current[1] >= obstacles[key1]["ymin"] and current[1] <= obstacles[key1]["ymax"]:

                flag = False
    if flag:
        return True


def d(pt1, pt2):
    dist = math.sqrt((pt2[0] - pt1[0]) ** 2 + (pt1[1] - pt2[1]) ** 2)
    return dist


def hueristic(startpt, goalpt):
    dist = math.sqrt((goalpt[0] - startpt[0]) ** 2 + (goalpt[1] - startpt[1]) ** 2)
    dist = math.floor(dist)
    return dist


def reconstruct_path(camefrom, current):
    total_path = []
    total_path.append(current)
    while str(current) in camefrom:
        current = camefrom[str(current)]
        total_path.append(current)
        print(current)
    return total_path

# // A* finds a path from start to goal.
# // h is the heuristic function. h(n) estimates the cost to reach goal from node n.


def A_Star(startpt, goalpt, h):
    # // The set of discovered nodes that may need to be (re-)expanded.
    # // Initially, only the start node is known.
    # // This is usually implemented as a min-heap or priority queue rather than a hash-set.
    openset = [startpt]
    #  // For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start
    # // to n currently known.
    camefrom = {"[0, 0]": "done"}
    # // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gscore = {}

    tries = 0
    # for limiting the amount of tries
    gscore[str(startpt)] = 0
    # // For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
    # // how short a path from start to finish can be if it goes through n.
    fscore = {}
    fscore[str(startpt)] = h(startpt, goalpt)
    flag = True
    while len(openset) and tries > -1:
        current = min(fscore, key=fscore.get).strip('[]').split(', ')
        current = list(map(int, current))
        print(current)
        x.append(current[0])
        y.append(current[1])
        if current == goalpt:
            return reconstruct_path(camefrom, current)

        openset.remove(current)
        fscore.pop(str(current), None)
        neighborlist = [[current[0]+1, current[1]], [current[0]-1, current[1]], [current[0], current[1]+1], [current[0], current[1]-1],
                        [current[0]+1, current[1]+1], [current[0]-1, current[1]-1], [current[0]+1, current[1]-1], [current[0]-1, current[1]+1]]

        for neighbor in neighborlist:
            tentative_gscore = int(gscore[str(current)])+d(current, neighbor)
            if str(neighbor) not in camefrom:
                if check_obstruct(neighbor):

                    camefrom[str(neighbor)] = current
                    gscore[str(neighbor)] = tentative_gscore
                    fscore[str(neighbor)] = tentative_gscore+h(neighbor, goalpt)
            if neighbor not in openset:
                if check_obstruct(neighbor):
                    x.append(current[0])
                    y.append(current[1])
                    openset.append(neighbor)
        tries = tries + 1
    return False


total_path = A_Star(startpt, goalpt, hueristic)
# total_path = list(map(int, total_path))
for i in range(len(total_path)-1):
    xl.append(total_path[i][0])
    yl.append(total_path[i][1])

plt.scatter(x, y)
plt.plot(xl, yl, color='r')
plt.xlabel('x-axis')
plt.ylabel('y-axis')

plt.title("A*")

plt.show()
