# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
import searchAgents

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).
    """

    """
    Returns the start state for the search problem.
    """
    def getStartState(self):
        util.raiseNotDefined()

    """
    state: Search state
    Returns True if and only if the state is a valid goal state.
    """
    def isGoalState(self, state):
        util.raiseNotDefined()

    """
    state: Search state
    For a given state, this should return a list of triples, (child,
    action, stepCost), where 'child' is a child to the current
    state, 'action' is the action required to get there, and 'stepCost' is
    the incremental cost of expanding to that child.
    """
    def expand(self, state):
        util.raiseNotDefined()

    """
    state: Search state
    For a given state, this should return a list of possible actions.
    """
    def getActions(self, state):
        util.raiseNotDefined()

    """
    state: Search state
    action: action taken at state.
    next_state: next Search state after taking action.
    For a given state, this should return the cost of the (s, a, s') transition.
    """
    def getActionCost(self, state, action, next_state):
        util.raiseNotDefined()

    """
    state: Search state
    action: action taken at state
    For a given state, this should return the next state after taking action from state.
    """
    def getNextState(self, state, action):
        util.raiseNotDefined()

    """
    actions: A list of actions to take
    This method returns the total cost of a particular sequence of actions.
    The sequence must be composed of legal moves.
    """
    def getCostOfActionSequence(self, actions):
        util.raiseNotDefined()


"""
Returns a sequence of moves that solves tinyMaze.  For any other maze, the
sequence of moves will be incorrect, so only use this for tinyMaze.
"""
def tinyMazeSearch(problem):
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """
    
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    #print("Start possible actions",problem.expand(problem.getStartState()))
    #print("Start possible actions",problem.getActions(problem.getStartState()))
    
    #if problem.isGoalState(node["state"]):
    #    return []
    node = {"state":problem.getStartState(),"actions":[]}
    
    frontier = util.Stack()
    frontier.push(node)

    explored = []

    while not frontier.isEmpty():
        
        node = frontier.pop()
        currentState = node["state"]
        actionsState = node["actions"]

        if currentState not in explored:
            explored.append(currentState)
        
            if problem.isGoalState(currentState):
                return actionsState

            else:
                successors = problem.expand(currentState)
                for successor in successors:
                    newActionState = actionsState + [successor[1]]

                    newNode = {"state":successor[0],"actions":newActionState}
                    frontier.push(newNode)
    
    return actionsState

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    #print("Start possible actions",problem.expand(problem.getStartState()))
    #print("Start possible actions",problem.getActions(problem.getStartState()))
    
    
    node = {"state":problem.getStartState(),"actions":[],"cost":0}
    
    frontier = util.Queue()
    frontier.push(node)

    explored = []    
    while not frontier.isEmpty():
        
        node = frontier.pop()
        currentState = node["state"]
        actionsState = node["actions"]
        costState    = node["cost"]

        if currentState not in explored:
            explored.append(currentState)
        
            if problem.isGoalState(currentState):
                return actionsState

            else:
                successors = problem.expand(currentState)
                for successor in successors:
                    newActionState = actionsState + [successor[1]]
                    newCostState   = costState + successor[2]

                    newNode = {"state":successor[0],"actions":newActionState,"cost":newCostState}
                    frontier.push(newNode)  
    
    return actionsState


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    #print("Start possible actions",problem.expand(problem.getStartState()))
    #print("Start possible actions",problem.getActions(problem.getStartState()))

    node = {"state":problem.getStartState(),"actions":[],"cost":0}

    frontier = util.PriorityQueue()
    frontier.push(node, 0)

    #holds (state, cost)
    explored = [] 

    while not frontier.isEmpty():

        node = frontier.pop()
        currentState = node["state"]
        actionsState = node["actions"]
        costState    = node["cost"]

        explored.append((currentState, costState))

        if problem.isGoalState(currentState):
            return actionsState

        else:
            successors = problem.expand(currentState)
            for successor in successors:
                newCurrentState = successor[0]
                newActionState  = actionsState + [successor[1]]
                newCostState    = problem.getCostOfActionSequence(newActionState)

                newNode = {"state":newCurrentState,"actions":newActionState,"cost":newCostState}

                visited = False
                for exploredNode in explored:
                    exploredState, exploredCost = exploredNode

                    if (exploredState == newCurrentState) and (exploredCost <= newCostState):
                        visited = True

                if not visited:
                    newPriority = newCostState + heuristic(newCurrentState,problem)

                    frontier.push(newNode,newPriority)
                    explored.append((newCurrentState,newCostState))

    return actionsState
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
