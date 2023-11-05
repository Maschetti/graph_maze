from environment import Maze
import numpy as np

class Agent():
    def __init__(self, env):
        self.env = env
        self.percepts = env.initial_percepts()
        pos = self.percepts['pos']
        self.F = [[pos]]
        self.V = [[pos]]
        self.C = [env.map[pos[0]][pos[1]]]

    def goal(self, pos):
        return (pos == self.percepts['exit']).all()

    def BFS(self):
        while self.F:
            path = self.F.pop(0)
            
            action = {
                'path': path,
                'move_to': path[-1]
            }        
            self.percepts = self.env.state_transition(action)
            
            if self.goal(path[-1]):
                return path
            
            for n in self.percepts['neighbors']:
                if not(any(np.array_equal(n,p) for p in self.V)):
                    self.V.append(n)
                    self.F.append(path + [n])
            
        return None   
    
    def DFS(self):
        while self.F:
            path = self.F.pop(-1)
            
            action = {
                'path': path,
                'move_to': path[-1]
            }
            self.percepts = self.env.state_transition(action)
            
            if self.goal(path[-1]):
                return path
            
            for n in self.percepts['neighbors']:
                if not(any(np.array_equal(n,p) for p in self.V)):
                    self.V.append(n)
                    self.F.append(path + [n])
                    
        return None
    # def AStar(self):
            
if __name__ == '__main__':
    nrow = 15
    ncol = 15
    env = Maze(nrow,ncol,[0,0],[nrow-1,ncol-1],pobs=0.2)

    ag = Agent(env)

    ag.DFS()
