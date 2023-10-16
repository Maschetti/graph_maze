import numpy as np
import matplotlib.pyplot as plt

class Maze():

    def __init__(self,nrow,ncol,start,exit,pobs=.3,pause=1):

        self.map = np.zeros((nrow,ncol))
        self.start = np.array(start)
        self.exit = np.array(exit)

        #add obstacles

        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                if np.random.rand() < pobs and ([i,j]!=self.start).any() and ([i,j]!=self.exit).any():
                    self.map[i][j] = 1

        ################## visualization ###################
        self.map[start[0]][start[1]] = 0.8 
        self.map[exit[0]][exit[1]] = 0.3
        self.pause = pause
        plt.ion()
        self.vis_map()
        plt.draw()
        plt.pause(pause)
        plt.clf()
        ###################################################
                
    def initial_percepts(self):

        return {'pos':self.start,
                'exit':self.exit,
                'neighbors':self.get_neighbors(self.start),
                'path':[]}
    
    def get_neighbors(self, pos):
        # mechi na funcao pois estava acessando posicoes que nao deveria
        directions = np.array([[1, 0], [-1, 0], [0, 1], [0, -1]])
        neighbors = []

        for dir in directions:
            candidate = pos + dir
            if (0 <= candidate[0] < self.map.shape[0]) and (0 <= candidate[1] < self.map.shape[1]) and (self.map[candidate[0]][candidate[1]] != 1):
                neighbors.append(candidate)

        return neighbors
    
    def state_transition(self,action):

        ################## visualization ###################
        plt.ion()
        self.plot_path(action['path'],self.pause)
        ####################################################

        return {'pos': action['move_to'],
                'exit':self.exit,
                'neighbors':self.get_neighbors(action['move_to']),
                'path':action['path']}
    
    # Visualization functions ###############################
    def plot_path(self, path, pause_time):
        plt.axes().invert_yaxis()
        plt.pcolormesh(self.map)
        for i in range(len(path)-1):
            plt.plot([path[i][1]+0.5,path[i+1][1]+0.5],[path[i][0]+0.5,path[i+1][0]+0.5],'-rs')
        plt.draw()
        plt.pause(pause_time)
        plt.clf()

    def vis_map(self):
        plt.axes().invert_yaxis()
        plt.pcolormesh(self.map)
        plt.plot(self.start[1]+0.5, self.start[0]+0.5,'rs')
        plt.show()
    ##########################################################

def DFS(env, pos, path=None):
    if path is None:
        path = []
    
    path.append(pos)
    
    if np.array_equal(pos, env.exit):
        return path
    
    n = env.get_neighbors(pos)
    for i in n:
        if not any(np.array_equal(i, p) for p in path):
            result = DFS(env, i, path.copy())
            if result is not None:
                return result
    return None

def BFS(env, pos):
    queue = []
    parent = {}
    v = []
    
    v.append(tuple(env.start))
    queue.append(tuple(env.start))
    parent[tuple(env.start)] = None
    
    while queue:
        pos = queue.pop(0)
        
        if np.array_equal(pos, env.exit):
            path = []
            while pos is not None:
                path.append(pos)
                pos = parent[pos]
            return list(reversed(path))
        
        n = env.get_neighbors(pos)
        for i in n:
            if not any(np.array_equal(i, p) for p in v):
                queue.append(tuple(i))
                v.append(tuple(i))
                parent[tuple(i)] = pos
    return None

if __name__ == '__main__':

    nrow = 5
    ncol = 5
    env = Maze(nrow,ncol,[0,0],[nrow-1,ncol-1])

    matriz = env.map
            
    print(env.map)

    actions = []
    actions.append({'move_to':[0,0],
                'path':[[0,0],[0,1],[0,2],[0,3],[1,3],[2,3]]})
    actions.append({'move_to':[0,0],
                'path':[[0,0],[1,1],[2,2],[3,3]]})
    actions.append({'move_to':[0,0],
                'path':[[0,0],[1,0],[2,0],[3,0]]})
    dfs = DFS(env, [0,0])
    if dfs is not None:
        actions.append({'move_to':[0,0],
                    'path': DFS(env, [0,0])})
    bfs = BFS(env, [0,0])
    if bfs is not None:
        actions.append({'move_to':[0,0],
                    'path': BFS(env, [0,0])})
    
    for a in actions:
        env.state_transition(a)