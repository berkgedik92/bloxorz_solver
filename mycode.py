# Solving Bloxorz using A* Search

# This is imported for priority queue implementation
import heapq


"""
Node class, for each node N, "costGuess" variable is basically g(N) + h(N)
where g(N) is the cost for coming to the node through "path" and h(N) is
the cost to go from N to goal node guessed by a heuristic function.
Path keeps the list of actions that are used to come to the "state" of node
from the initial state
"""
class Node:

    def __init__(self, costGuess, state, path):
        self.costGuess = costGuess
        self.state = state
        self.path = path

    '''
    __lt__ is used to inform the priority queue about how to compare nodes
    We say that node whose "costGuess" is smaller
    should become first in the priority queue
    '''
    def __lt__(self, other):
        return self.costGuess < other.costGuess


"""
State class, each state object keeps necessary information that represents a state.
Here "x_cor" is x coordinate of the top-leftmost part of the block, "y_cor" is
y coordinate of the top-leftmost part of the block, "orient" is 0 if the block stands vertically
1 if the block is standing horizontally and parallel to x-axis, 2 if the block is standing
horizontally and parallel to y-axis. This information is enough for representing
a state, under the assumption that positions of empty tiles and safe tiles will never change
and any empty or safe tile will not be removed or added during the game. 
"""
class State:

    """
    ySize and xSize are variables that are shared among all State objects, 
    they keep the size of input matrix, their values will be used in state hashing 
    function, and those values will be determined by Bloxorz class before solving 
    the problem.
    """
    ySize = 0
    xSize = 0

    def __init__(self, x_cor, y_cor, orient):
        self.x_cor = x_cor
        self.y_cor = y_cor
        self.orient = orient
    
    '''
    Hash function is used because we will keep states expanded, 
    and we need to have a way to easily detect if a state is already in the set.
    It is guaranteed that with this hashing function, same states will have 
    same hash values and different states will have different hash values.
    '''
    def __hash__(self):
        return (State.ySize * State.xSize * self.orient) + (State.xSize * self.y_cor) + self.x_cor

    def __eq__(self,other):
        return self.__hash__() == other.__hash__()


class PriorityQueue:
    
    def __init__(self):
        self.data = []
        self.counter = 0
    
    def push(self, item):
        heapq.heappush(self.data, item)
        self.counter += 1
    
    def pop(self):
        self.counter -= 1
        return heapq.heappop(self.data)
    
    def not_empty(self):
        return self.counter > 0


class Bloxorz:

    """
    This class is responsible from solving the problem, its constructor takes
    input matrix as parameter
    """

    def __init__(self, input_matrix):

        '''
        Map data will be kept here
        '''
        self.map = input_matrix
        
        '''
        Expanded states will be kept here so that we will not expand them again and
        we will not add them to queue again
        '''
        self.closed = set()

        '''
        States to be expanded will be kept here
        '''
        self.queue = PriorityQueue()
        
        '''
        Inform State class about the size of input matrix so that 
        State class will be able to use its hashing function
        '''
        State.xSize = len(self.map[0])
        State.ySize = len(self.map)

        '''
        Find goal position and keep this information, 
        because it will be used by goal test function
        '''
        for i in range(0,len(self.map)):
            res = self.map[i].find("G")
            if res != -1:
                self.goalX = res
                self.goalY = i
                break

        '''
        Find initial position and orientation of the block, create initial state and node according 
        to this information and put this node into queue. 
        Also in map, replace "S" with "O" because it is a safe tile
        '''
        for i in range(0,len(self.map)):
            res = self.map[i].find("S")
            if res != -1:
                orient = 0
                if res + 1 < len(self.map[i]) and self.map[i][res + 1] == 'S':
                    orient = 1
                    self.map[i] = self.map[i].replace("SS", "00")
                elif i + 1 < len(self.map) and self.map[i + 1][res] == 'S':
                    orient = 2
                    self.map[i] = self.map[i].replace("S", "0")
                    self.map[i + 1] = self.map[i + 1].replace("S", "0")
                else:
                    self.map[i] = self.map[i].replace("S", "0")
                self.initial = State(res, i, orient)
                self.queue.push(Node(self.heuristic(self.initial), self.initial, []))
                break

    def is_goal(self, state):
        return state.orient == 0 and state.x_cor == self.goalX and state.y_cor == self.goalY

    '''
    Guess the cost from "state" to goal state. 
    This heuristic function is admissible and monotonic
    '''
    def heuristic(self, state):
        return (abs(self.goalX - state.x_cor) / 3) + (abs(self.goalY - state.y_cor) / 3)

    # Check whether a tile is a safe tile or empty tile
    def is_safe(self, x, y):
        return (len(self.map) > y >= 0) and \
                (len(self.map[y]) > x >= 0) and \
               (self.map[y][x] == 'O' or self.map[y][x] == 'G')

    '''
    Display the board that is represented by "state", in other words, 
    merge the board information from "self.map" and block information 
    from "state" and display it to user
    '''
    def print_state(self, state):

        print_state = []
        for y in range(0, state.ySize):
            print_state.append([])
            for x in range(0, state.xSize):
                print_state[y].append(self.map[y][x])

        if not self.is_goal(state):
            print_state[state.y_cor][state.x_cor] = "S"

            if state.orient == 1:
                print_state[state.y_cor][state.x_cor + 1] = "S"

            if state.orient == 2:
                print_state[state.y_cor + 1][state.x_cor] = "S"

        for y in range(0, len(print_state)):
            print("".join(print_state[y]))

    '''
    Prints the shortest sequence of legal states and taken actions 
    that are used to navigate the block from its given initial 
    location into the goal
    '''
    def print_result(self, result):
        st = 1
        current_state = self.initial
            
        for action in result:
            print("State : ", st)
            self.print_state(current_state)
            print("Action : ", action)
            print(" ")
            st += 1
            current_state = self.successors(current_state)[action]

        print("State : ", st)
        self.print_state(current_state)
        if len(result) == 0:
            print("There is no solution for this problem!")
        else:
            print("This optimal solution requires", len(result), "moves.")

    '''
    Returns a hashtable where the values are possible states that can be reached from "state" and
    keys of values are the action that is required to go to this state
    '''
    def successors(self, state):
        result = {}
        
        if state.orient == 0:
            # Up
            if self.is_safe(state.x_cor, state.y_cor-2) and self.is_safe(state.x_cor, state.y_cor-1):
                result['U'] = State(state.x_cor, state.y_cor-2, 2)
            # Down
            if self.is_safe(state.x_cor, state.y_cor+1) and self.is_safe(state.x_cor, state.y_cor+2):
                result['D'] = State(state.x_cor, state.y_cor+1, 2)
            # Left
            if self.is_safe(state.x_cor-2, state.y_cor) and self.is_safe(state.x_cor-1, state.y_cor):
                result['L'] = State(state.x_cor-2, state.y_cor, 1)
            # Right
            if self.is_safe(state.x_cor+1, state.y_cor) and self.is_safe(state.x_cor+2, state.y_cor):
                result['R'] = State(state.x_cor+1, state.y_cor, 1)
                
        elif state.orient == 1:
            # Up
            if self.is_safe(state.x_cor, state.y_cor-1) and self.is_safe(state.x_cor+1, state.y_cor-1):
                result['U'] = (State(state.x_cor, state.y_cor-1, 1))
            # Down
            if self.is_safe(state.x_cor, state.y_cor+1) and self.is_safe(state.x_cor+1, state.y_cor+1):
                result['D'] = (State(state.x_cor, state.y_cor+1, 1))
            # Left
            if self.is_safe(state.x_cor-1, state.y_cor):
                result['L'] = (State(state.x_cor-1, state.y_cor, 0))
            # Right
            if self.is_safe(state.x_cor+2, state.y_cor):
                result['R'] = (State(state.x_cor+2, state.y_cor, 0))
                
        else:
            # Up
            if self.is_safe(state.x_cor, state.y_cor-1):
                result['U'] = State(state.x_cor, state.y_cor-1, 0)
            # Down
            if self.is_safe(state.x_cor, state.y_cor+2):
                result['D'] = State(state.x_cor, state.y_cor+2, 0)
            # Left
            if self.is_safe(state.x_cor-1, state.y_cor) and self.is_safe(state.x_cor-1, state.y_cor+1):
                result['L'] = State(state.x_cor-1, state.y_cor, 2)
            # Right
            if self.is_safe(state.x_cor+1, state.y_cor) and self.is_safe(state.x_cor+1, state.y_cor+1):
                result['R'] = State(state.x_cor+1, state.y_cor, 2)
                
        return result

    '''
    Finds the optimal solution and returns the action list that leads to solution
    from initial state
    '''
    def solve(self):
        while self.queue.not_empty():
            # get the node from queue whose guessedCost is the smallest
            current = self.queue.pop()

            '''
            if this state is expanded before, then do not take this into consideration
            because it cannot lead to an optimal solution as the heuristic function is monotonic
            (because it is sure that when a state is expanded, then the optimal path to this state is found)
            '''
            if current.state not in self.closed:
                # if the state is goal state, then return the path to this state...
                if self.is_goal(current.state):
                    return current.path
                # ...otherwise, find all successors of this state
                successors = self.successors(current.state)
                for key in successors:
                    new_state = successors[key]

                    '''
                    if the successor is not expanded before, add this to queue
                    (if there is same state in queue, we will expand the node with lower cost guess
                    and we will not take the other into consideration)
                    '''
                    if new_state not in self.closed:

                        '''
                        deep copy the path of parent state to a list and append the action, use
                        it in the creation of the new node
                        '''
                        templist = []
                        for element in current.path:
                            templist.append(element)
                        templist.append(key)
                        # costGuess is the current path length + cost guess for going from new state to goal state
                        self.queue.push(Node(len(templist) + self.heuristic(new_state), new_state, templist))
                # add expanded state to set
                self.closed.add(current.state)
        # if there is no solution, return an empty list
        return []
                

file = open('gameboard.txt', 'r')
inputData = []
while True:
    line = file.readline()
    if line == '':
        break
    inputData.append(line.strip())
file.close()
problem = Bloxorz(inputData)
solution = problem.solve()
problem.print_result(solution)
