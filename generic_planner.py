import heapq
import random   
import copy
import collections

"""
WATER JUG PROBLEM
2 water jugs with an initial amount of water: jugA and jugB (the initial amount can be defined by the user)
jugA's capacity is 70ml and jugB's capacity is 50ml (the jugs do not have markings on them)
The goal is to fill one of the jugs with 40ml of water
The actions we can take are the following:
    1) Fill the first jug
    2) Fill the second jug
    3) Empty the first jug
    4) Empty the second jug
    5) Pour the first jug into the second jug
    6) Pour the second jug into the first jug
"""

def choose():
    """
    Function that gives the user the ability to choose the algorithm he wants to implement in order to solve the problem
    """

    press = input("-----CHOOSE THE ALGORITHM YOU WANT TO USE. ENTER 'B' FOR BFS, 'D' FOR DFS: \n")
    # take only the first letter from the user's input
    press = press[0].lower()
    
    # check for invalid input
    while press != 'b' and press != 'd':
        press = input("INVALID INPUT! CHOOSE THE ALGORITHM YOU WANT TO USE. ENTER 'B' FOR BFS, 'D' FOR DFS: ")
        press = press[0].lower()

    if press == 'd':
        return True
    else: 
        return False

def alreadyVisited(state, visited):
    """
    Function to check whether a state has been visited or not
    """

    # if the state has been visited returns True, otherwise returns False
    return visited.get(str(state), False)

def goalReached(path, goal_amount):
    """
    Function to check whether the goal has been reached or no
    """

    if path[-1][0] == goal_amount or path[-1][1] == goal_amount:
        return True
    else:
        return False

def createChildren(jugs_capacities, path, visited):
    """
    Function that creates the possible next states based on the current state
    """

    possible_paths = []     # list that stores the possible paths followed based on the current state
    next_states = []        # list that stores all the possible next states based on the current state
    state = []

    a_max = 70
    b_max = 50

    a = path[-1][0]     # initial amount of water in the first jug 
    b = path[-1][1]     # initial amount of water in the second jug

    # 1) fill in the first jug
    state.append(a_max)
    state.append(b)
    if not alreadyVisited(state, visited):
        next_states.append(state)
    state = []

    # 2) fill in the second jug
    state.append(a)
    state.append(b_max)
    if not alreadyVisited(state, visited):
        next_states.append(state)
    state = []

    # 3) pour water from the second jug to the first jug
    state.append(min(a_max, a + b))
    state.append(b - (state[0] - a))
    if not alreadyVisited(state, visited):
        next_states.append(state)
    state = []

    # 4) pour water from the first jug to the second jug
    state.append(min(a + b, b_max))
    state.insert(0, a - (state[0] - b))
    if not alreadyVisited(state, visited):
        next_states.append(state)
    state = []

    # 5) empty the first jug
    state.append(0)
    state.append(b)
    if not alreadyVisited(state, visited):
        next_states.append(state)
    state = []

    # 6) empty the second jug  
    state.append(a)
    state.append(0)
    if not alreadyVisited(state, visited):
        next_states.append(state)

    for i in range(0, len(next_states)):
        temp = list(path)
        temp.append(next_states[i])
        possible_paths.append(temp)
    
    return possible_paths

def search(initial_state, jugs_capacities, goal_amount, visited, choice):
    """
    Function to search using either BFS or DFS for the wanted state and return the followed path
    """
    
    if choice:
        print("-----IMPLEMENTING DFS")
    else:
        print("-----IMPLEMENTING BFS")

    found = False

    # search_front represents metwpo anazhthshs
    search_front = collections.deque()
    search_front.appendleft(initial_state)

    while len(search_front) != 0:
        # path represents mikroskopio
        path = search_front.popleft()

        # mark the current state as visited
        visited[str(path[-1])] = True

        if goalReached(path, goal_amount):
            found = True
            goal = path
            break

        next_moves = createChildren(jugs_capacities, path, visited)
        for i in next_moves:
            if choice:
                # implementing DFS
                search_front.appendleft(i)
            else:
                # implementing BFS
                search_front.append(i)
    
    if found:
        print("-----THE GOAL HAS BEEN ACHIEVED, PRINTING THE PATH FOLLOWED TO THE GOAL STATE\n")
        for i in range(0, len(goal)):
            print(i, ". ", goal[i])
    else:
        print("-----THE PROBLEM CANNOT BE SOLVED, SORRY\n")

def waterJugs():
    jug1_initial = int(input("-----ENTER INITIAL AMOUNT OF WATER IN JUG 1: "))
    jug2_initial = int(input("-----ENTER INITIAL AMOUNT OF WATER IN JUG 2: "))

    initial_state = [[jug1_initial, jug2_initial]]        # initially both jugs are empty
    jugs_capacities = [70, 50]                            # list to store the max capacities of the jugs
    goal_amount = 40                                      # the wanted amount of water in a jug   
    visited = {}                                          # dictionary to store the visited states
    choice = choose()                                     # choice calls def choose, if the user chooses DFS choice = True, otherwise choice = False
    search(initial_state, jugs_capacities, goal_amount, visited, choice)











'''
BLOCKS WORLD PROBLEM
The blocks world is a planning domain in artificial intelligence. 
The algorithm is similar to a set of wooden blocks of various shapes and colors sitting on a table. 
The goal is to build one or more vertical stacks of blocks. Only one block may be moved at a time: 
it may either be placed on the table or placed atop another block. Because of this, any blocks that are under another block 
cannot be moved. Moreover, some kinds of blocks cannot have other blocks stacked on top of them.
'''

class PriorityQueue:
    '''
    Priotity queue is used to store all the possible children of a state in it, 
    based on distance to goal, closest first.
    '''
    def __init__(self):
        self.heap = []
        self.count = 0

    # function to push an item to the priority queue
    def push(self, item, priority):
        heapq.heappush(self.heap, (priority, self.count, item))
        self.count += 1

    # function to remove or retrieve an item from the priority queue
    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)

        return item

    def aStar(node):
        '''
        Implementation of the a star search algorithm. A star algorithm, is an algorithm 
        used to find the shortest possible path from start to end states. 
        We search the node that has the lowest combined cost and heuristic first.
        '''
        closed = []
        queue = PriorityQueue()                     # initialize a priority queue object
        queue.push(node, node.calculateCost())      # insert it in the priority queue along with its cost
        
        while True:
            if len(queue.heap) == 0:
                print ('-----FINAL STATE IS NOT ACHIEVABLE FROM GIVEN START STATE!')
            
            node = queue.pop()
            # check if the goal state has been achieved
            if node.goal():     
                return

            if node.state not in closed:
                closed.append(node.state)
                for childNode in node.createChildren():
                    queue.push(childNode, childNode.calculateCost())

class State:
    def __init__(self, start_state, final_state, parent = None) :
        self.state = start_state
        self.finalSt = final_state   
        self.parent = parent
        self.cost = 0
        if parent:
            self.cost = parent.cost + 1 
    
    # check if the goal state has been reached 
    def goal(self) :
        if self.state == self.finalSt:
            self.findPathFollowed()
            return True
        else:
            return False
    
    def calculateHeuristic(self):
        '''
        Heuristic function to help decide which path to follow next.
        The heuristic function provides an estimate of the minimum cost required 
        to reach from a given node to a target node.
        '''
        arg1 = arg2 = arg3 = 0

        arg1 = len(self.finalSt[0]) - len(self.state[0])
    
        for i in range(len(self.state[0])) :
            if self.state[0][i] != self.finalSt[0][i]:
                arg2 += 2
        
        for i in range(1, len(self.state)):
            for val in range(len(self.state[i]) - 1):
                if self.state[i][val] > self.state[i][val + 1]:
                    arg3 += 1
                
        heuristic_value = arg1 + 4 * arg2 - arg3

        return heuristic_value
    
    # function which generates all possible children of the current state
    def createChildren(self):
        children = []
        for i, stack in enumerate(self.state):
            for j, stack1 in enumerate(self.state):
                if i != j and len(stack1):
                    temp = copy.deepcopy(stack)
                    child = copy.deepcopy(self)
                    temp1 = copy.deepcopy(stack1)
                    temp.append(temp1[-1])
                    del temp1[-1]
                    child.state[i] = temp
                    child.state[j] = temp1
                    child.parent = copy.deepcopy(self)
                    children.append(child)

        return children

    # calculate and display the path followed to reach from the start to the goal state
    def findPathFollowed(self):
        node, path = self, []
        while node:
            path.append(node.state)
            node = node.parent
        
        print("\n-----SOLUTION FOUND, PRINTING THE PATH FOLLOWED TO THE GOAL STATE: ")
        move_nr = 0;
        for i in list(reversed(path)) :
            print(str(move_nr) + ". " + str(i))
            move_nr += 1

    def calculateCost(self):
        return self.calculateHeuristic() + self.cost

def calculateStates(stacks, blocks): 
    '''
    Function which calculates the starting and final states of the problem, 
    given the stacks and blocks which are retrieved from the users input.
    '''
    # CALCULATE THE STARTING STATE
    stacks_nr = stacks
    digits = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']
    block_nrs = digits[:blocks]     # if blocks = 5, blocks_nrs = ['0', '1', '2', '3', '4']    
    random.shuffle(block_nrs)
    
    startState = []
    while blocks:
        if stacks == 1:
            startState.append(block_nrs)
            break
        else:
            r = random.randint(1, blocks)
            s = block_nrs[:r]
            startState.append(s)
    
        blocks -= r
        stacks -= 1
        block_nrs = block_nrs[r:]
    
    while len(startState) < stacks_nr:
        startState += [[]]
    
    # randomize the stacks positions
    random.shuffle(startState)      

    # CALCULATE THE FINAL STATE
    finalState = []
    for stack in startState:
        finalState += stack

    finalState.sort()
    finalState = [finalState]
    
    # complete the final states stacks with empty stacks 
    for _ in range(len(startState) - 1):
        finalState += [[]]

    return startState, finalState

def blocksWorld():
    stacks = int(input("-----ENTER NUMBER OF STACKS: "))
    blocks = int(input("-----ENTER NUMBER OF BLOCKS: "))
    
    startState, finalState = calculateStates(stacks, blocks)
    print("\n-----START STATE: " + str(startState))
    print("-----FINAL STATE: " + str(finalState))

    PriorityQueue.aStar(State(startState, finalState))











def print_menu(): 
    print("\nCHOOSE ONE OF THE BELOW PROBLEMS TO SOLVE: ")
    print("1. WATER JUG")
    print("2. BLOCKS WORLD")

while True:          
    print_menu()    
    choice = int(input("ENTER YOUR CHOICE [1-2]: "))
     
    if choice == 1:  
        waterJugs()
    elif choice == 2:
        blocksWorld()
