import numpy as np
import heapq
from typing import Union

class Graph:

    def __init__(self, adjacency_mat: Union[np.ndarray, str]):
        """
    
        Unlike the BFS assignment, this Graph class takes an adjacency matrix as input. `adjacency_mat` 
        can either be a 2D numpy array of floats or a path to a CSV file containing a 2D numpy array of floats.

        In this project, we will assume `adjacency_mat` corresponds to the adjacency matrix of an undirected graph.
    
        """
        if type(adjacency_mat) == str:
            self.adj_mat = self._load_adjacency_matrix_from_csv(adjacency_mat)
        elif type(adjacency_mat) == np.ndarray:
            self.adj_mat = adjacency_mat
        else: 
            raise TypeError('Input must be a valid path or an adjacency matrix')
        self.mst = None

    def _load_adjacency_matrix_from_csv(self, path: str) -> np.ndarray:
        with open(path) as f:
            return np.loadtxt(f, delimiter=',')


# Method uses Prim's algorithm to construct the minimum spanning tree of the graph
    def construct_mst(self):

        '''
            Following psuedocode
            S ← ∅, T ← ∅.
            s ← any node in V.
            FOREACH v ≠ s : π[v] ← ∞, pred[v] ← null; π[s] ← 0.
            Create an empty priority queue pq.
            FOREACH v ∈ V : INSERT(pq, v, π[v]).
            WHILE (IS-NOT-EMPTY(pq))
            u ← DEL-MIN(pq).
            S ← S ∪ { u }, T ← T ∪ { pred[u] }.
            FOREACH edge e = (u, v) ∈ E with v ∉ S :
            IF (ce < π[v])
            DECREASE-KEY(pq, v, ce).
            π[v] ← ce; pred[v] ← e
        '''

        # S ← ∅, T ← ∅.
        #s ← any node in V.

        S = set()
        V = self.adj_mat.shape[0]

        s = 0

        #FOREACH v ≠ s : π[v] ← ∞, pred[v] ← null; π[s] ← 0.
        #Create an empty priority queue pq.
        pred = [None] * V  
        pi = [float('inf')] * V  
        pi[s] = 0
        pq = [(0, 0)]  

        self.mst = np.zeros_like(self.adj_mat)

    #WHILE (IS-NOT-EMPTY(pq))
        while pq:

            #u ← DEL-MIN(pq).
            cost, u = heapq.heappop(pq)
        
            #S ← S ∪ { u }, T ← T ∪ { pred[u] }.
            if u in S:
                continue
        
        
            S.add(u)
        
      
            if pred[u] is not None:
                self.mst[u][pred[u]] = self.adj_mat[u][pred[u]]  
                self.mst[pred[u]][u] = self.adj_mat[u][pred[u]]  
        
        #FOREACH edge e = (u, v) ∈ E with v ∉ S :
            for v in range(V):

               
                if v not in S and self.adj_mat[u][v] != 0:  
                    new_cost = self.adj_mat[u][v]
                 
                    #IF (ce < π[v])
                    if new_cost < pi[v]:
                        #DECREASE-KEY(pq, v, ce).
                        #π[v] ← ce; pred[v] ← e
                        pi[v] = new_cost
                        pred[v] = u
                        heapq.heappush(pq, (new_cost, v))   
                

