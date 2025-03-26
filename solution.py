import sys
from constants import *
from environment import *
from state import State
import heapq


"""
solution.py

This file is a template you should use to implement your solution.

You should implement 

COMP3702 2024 Assignment 1 Support Code
"""

class StateNode:
    def __init__(self, env, state, parent, action_from_parent, path_steps, path_cost):
        """
        :param env: Environment 
        :param state: State instance representing the environment state
        :param parent: Parent StateNode
        :param action_from_parent: The action that led from the parent to this node (e.g., LEFT, RIGHT, UP, DOWN)
        :param path_steps: The number of steps taken to reach this node
        :param path_cost: The cumulative cost to reach this node
        """
        self.env = env
        self.state = state
        self.parent = parent
        self.action_from_parent = action_from_parent
        self.path_steps = path_steps
        self.path_cost = path_cost

    def get_path(self):
        """
        :return: A list of actions from the root node to this node
        """
        path = []
        cur = self
        while cur.action_from_parent is not None:
            path.append(cur.action_from_parent)
            cur = cur.parent
        path.reverse()
        return path

    def get_successors(self):
        """
        :return: A list of successor StateNodes
        """
        successors = []
        for action in BEE_ACTIONS:
            success, cost, next_state = self.state.environment.perform_action(self.state, action)
            if success:
                successors.append(StateNode(
                    env=self.env,
                    state=next_state,
                    parent=self,
                    action_from_parent=action,
                    path_steps=self.path_steps + 1,
                    path_cost=self.path_cost + cost
                ))
        return successors

    def __lt__(self, other):
        """
        Comparison based on path cost, useful for priority queues.
        """
        return self.path_cost < other.path_cost



class Solver:

    def __init__(self, environment, loop_counter):
        self.environment = environment
        self.loop_counter = loop_counter

        #
        # TODO: Define any class instance variables you require here.
        # NOTE: avoid performing any computationally expensive heuristic preprocessing operations here - use the preprocess_heuristic method below for this purpose
        #

    # === Uniform Cost Search ==========================================================================================
    def solve_ucs(self):
        """
        Find a path which solves the environment using Uniform Cost Search (UCS).
        :return: path (list of actions, where each action is an element of BEE_ACTIONS)
        """

        #
        #
        # TODO: Implement your UCS code here
        #
        # === Important ================================================================================================
        # To ensure your code works correctly with tester, you should include the following line of code in your main
        # search loop:
        #
        # self.loop_counter.inc()
        #
        # e.g.
        # while loop_condition(): // (While exploring frontier)
        #   self.loop_counter.inc()
        #   ...
        #
        # ==============================================================================================================
        #
        #
        
        #env.initial
        initial_node = self.process_state_node()

        frontier = []
        heapq.heappush(frontier, (initial_node.path_cost, initial_node))
        
        explored = set()
        
        best_cost = {}

        while frontier:
            self.loop_counter.inc()
            _, current_node = heapq.heappop(frontier)
            current_state = current_node.state
            
            if current_state in explored:
                continue
            
            explored.add(current_state)

            if self.environment.is_solved(current_state):
                path = current_node.get_path()
                return path
            
            for successor in current_node.get_successors():
                successor_state = successor.state
                
                if successor_state not in explored:
                    if successor_state not in best_cost or successor.path_cost < best_cost[successor_state]:
                        best_cost[successor_state] = successor.path_cost
                        heapq.heappush(frontier, (successor.path_cost, successor))
        
        return []

    # === A* Search ====================================================================================================

    def preprocess_heuristic(self):
        """
        Perform pre-processing (e.g. pre-computing repeatedly used values) necessary for your heuristic,
        """
        #
        #
        # TODO: (Optional) Implement code for any preprocessing required by your heuristic here (if your heuristic
        #  requires preprocessing).
        #
        # If you choose to implement code here, you should call this method from your solve_a_star method (e.g. once at
        # the beginning of your search).
        #
        #

        pass


    
    def compute_heuristic(self, state):
        """
        Compute a heuristic value h(n) for the given state.
        :param state: given state (GameState object)
        :return a real number h(n)
        """
        # passing in initial state
        #
        #
        # TODO: Implement your heuristic function for A* search here.
        #
        # You should call this method from your solve_a_star method (e.g. every time you need to compute a heuristic
        # value for a state).
        #

        total_distance = 0

        widgets = self.environment.widget_init_posits
        targets = self.environment.target_list

        for widget, target_position in zip(widgets, targets):
            total_distance += self.hex_distance(widget, target_position)

        return total_distance


    def solve_a_star(self):
        """
        Find a path which solves the environment using A* search.
        :return: path (list of actions, where each action is an element of BEE_ACTIONS)
        """

        #
        #
        # TODO: Implement your A* search code here
        #
        # === Important ================================================================================================
        # To ensure your code works correctly with tester, you should include the following line of code in your main
        # search loop:
        #
        # self.loop_counter.inc()
        #
        # e.g.
        # while loop_condition(): //
        #   self.loop_counter.inc()
        #   ...
        #
        # ==============================================================================================================
        #
        #

        initial_node = self.process_state_node()

        frontier = []
        heapq.heappush(frontier, (initial_node.path_cost + self.compute_heuristic(initial_node.state), initial_node))
        
        explored = set()
        
        best_cost = {}

        while frontier:
            self.loop_counter.inc()
            _, current_node = heapq.heappop(frontier)
            current_state = current_node.state

            if self.environment.is_solved(current_state):
                return current_node.get_path()
            
            if current_state in explored:
                continue

            explored.add(current_state)

            for successor in current_node.get_successors():
                successor_state = successor.state
                
                if successor_state not in explored:
                    total_cost = successor.path_cost + self.compute_heuristic(successor_state)
                    if successor_state not in best_cost or total_cost < best_cost[successor_state]:
                        best_cost[successor_state] = total_cost
                        heapq.heappush(frontier, (total_cost, successor))
        
        return []

    #
    #
    # TODO: Add any additional methods here
    #
    #

    def process_state(self):
        initial_state = State(
            environment=self.environment,
            BEE_posit=self.environment.BEE_init_posit,
            BEE_orient=self.environment.BEE_init_orient,
            widget_centres=self.environment.widget_init_posits,
            widget_orients=self.environment.widget_init_orients,
            force_valid=True
        )
        
        return initial_state


    def process_state_node(self):
        initial_node = StateNode(
            env=self.environment,
            state=self.process_state(),
            parent=None,
            action_from_parent=None,
            path_steps=0,
            path_cost=0
        )

        return initial_node

    def hex_distance(self, hex1, hex2):


        x1, y1 = hex1
        x2, y2 = hex2
        dx = abs(x1 - x2)
        dy = abs(y1 - y2)
        dz = abs((x1 + y1) - (x2 + y2))  
        return max(dx, dy, dz)

        
