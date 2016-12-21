# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"


    if (problem.isGoalState(problem.getStartState())):
        return [];

    struct = util.Stack();
    start = (problem.getStartState(), [], [problem.getStartState()])
    struct.push(start)
    traceNodes = []

    while (not (struct.isEmpty())):
        current = struct.pop()
        node = current[0]
        actions = current[1]
        visited = current[2]

        if problem.isGoalState(node):
            return actions

        adjacentVertices = problem.getSuccessors(node)
        for element in adjacentVertices:
            coord = element[0]
            path = element[1]

            if not coord in visited:
                newPath = actions + [path]
                struct.push((coord, newPath, visited + [node]))
                traceNodes.append(coord)


    return traceNodes


def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    "*** YOUR CODE HERE ***"

    if (problem.isGoalState(problem.getStartState())):
        return [];

    struct = util.Queue()
    start = (problem.getStartState(), [], [problem.getStartState()])
    struct.push(start)
    traceNodes = []

    while (not (struct.isEmpty())):
        current = struct.pop()
        node = current[0]
        actions = current[1]
        visited = current[2]

        if problem.isGoalState(node):
            return actions

        adjacentVertices = problem.getSuccessors(node)
        for vertex in adjacentVertices:
            coord = vertex[0]
            path = vertex[1]

            if (not coord in visited) and (not coord in traceNodes):
                newPath = actions + [path]
                struct.push((coord, newPath, visited + [node]))
                traceNodes.append(coord)

    return traceNodes

    util.raiseNotDefined()

def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    "*** YOUR CODE HERE ***"

    if (problem.isGoalState(problem.getStartState())):
        return [problem.getStartState()];

    struct = util.PriorityQueue()
    start = (problem.getStartState(), [], 0)
    struct.push(start, 0)

    currentNode = start[0]
    actions = start[1]
    totalCost = start[2]

    visitedNodes = [(currentNode, 0)]
    while (not (struct.isEmpty())):
        nextNode = struct.pop()
        currentNode = nextNode[0]
        actions = nextNode[1]
        allcost = nextNode[2]
        if (problem.isGoalState(currentNode)):
            return actions

        adjacentVertices = problem.getSuccessors(currentNode)
        for vertex in adjacentVertices:
            possible = vertex[0]
            previousActions = vertex[1]
            cost = vertex[2]

            wasVertexVisited = False
            newCost = problem.getCostOfActions(actions + [previousActions])

            for element in visitedNodes:
                node = element[0]
                elementCost = element[1]

                if (possible == node) and (newCost >= elementCost):
                    wasVertexVisited = True

            if (not wasVertexVisited):
                newPath = actions + [previousActions]
                struct.push((possible, newPath, newCost), newCost)
                visitedNodes.append((possible,newCost))


    return [problem.getStartState()]

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"
    if (problem.isGoalState(problem.getStartState())):
        return [problem.getStartState()];

    struct = util.PriorityQueue()
    start = (problem.getStartState(), [], 0)
    struct.push(start, heuristic(problem.getStartState(), problem))
    closed = []

    currentNode = start[0]
    actions = start[1]
    totalCost = start[2]

    visitedNodes=[(currentNode, 0)]
    while (not (struct.isEmpty())):
        nextNode = struct.pop()
        currentNode = nextNode[0]
        actions = nextNode[1]

        if (problem.isGoalState(currentNode)):
            return actions

        closed.append(currentNode)

        adjacentVertices = problem.getSuccessors(currentNode)
        for vertex in adjacentVertices:
            possible = vertex[0]
            previousActions = vertex[1]
            cost = vertex[2]

            newPath = actions + [previousActions]

            wasVertexVisited = False
            newCost = problem.getCostOfActions(newPath) + heuristic(possible, problem)

            for element in visitedNodes:
                node = element[0]
                elementCost = element[1]

                if ((possible == node) and (newCost >= elementCost)):
                    wasVertexVisited = True

            if (not wasVertexVisited):
                struct.push((possible, newPath, newCost), newCost)
                visitedNodes.append((possible, newCost))

    return [problem.getStartState()]
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
