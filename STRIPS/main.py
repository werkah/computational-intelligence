import time

from searchMPP import *
from stripsForwardPlanner import *
from stripsProblem import *


def execute_problem(problem, searcher_function):
    total_time = 0
    for _ in range(5):
        start = time.time()
        searcher_function(problem).search()
        end = time.time()
        total_time += (end - start)
    average_time = total_time / 5
    return average_time


blocks1_2domain = create_blocks_world({'a', 'b', 'c', 'd', 'e'})

problem1_state = {on('a'): 'table', clear('a'): False,
                  on('b'): 'a', clear('b'): False,
                  on('c'): 'b', clear('c'): False,
                  on('d'): 'c', clear('d'): True,
                  on('e'): 'table', clear('e'): True}  # initial state

problem1_goal = {on('c'): 'table',
                 on('a'): 'c',
                 on('e'): 'table',
                 on('d'): 'e',
                 on('b'): 'table'}  # goal

problem1 = Planning_problem(
    blocks1_2domain,
    problem1_state,
    problem1_goal
)

average_time1 = execute_problem(problem1, lambda p: SearcherMPP(Forward_STRIPS(p)))
print("Average for problem 1:", average_time1)

problem2_state = {on('a'): 'table', clear('a'): False,
                  on('b'): 'a', clear('b'): False,
                  on('c'): 'b', clear('c'): True,
                  on('d'): 'table', clear('d'): False,
                  on('e'): 'd', clear('e'): True}

problem2_goal = {on('a'): 'table',
                 on('e'): 'a',
                 on('b'): 'e',
                 on('c'): 'b',
                 on('d'): 'c'}  # goal

problem2 = Planning_problem(
    blocks1_2domain,
    problem2_state,
    problem2_goal
)

average_time2 = execute_problem(problem2, lambda p: SearcherMPP(Forward_STRIPS(p)))
print("Average for problem 2:", average_time2)

blocks3domain = create_blocks_world({'a', 'b', 'c', 'd', 'e', 'f'})

problem3_state = {on('d'): 'table', clear('d'): False,
                  on('f'): 'd', clear('f'): True,
                  on('b'): 'table', clear('b'): False,
                  on('c'): 'b', clear('c'): True,
                  on('e'): 'table', clear('e'): False,
                  on('a'): 'e', clear('a'): True}  # initial state

problem3_goal = {on('e'): 'table',
                 on('b'): 'e',
                 on('d'): 'b',
                 on('c'): 'table',
                 on('a'): 'c',
                 on('f'): 'table'}  # goal

problem3 = Planning_problem(
    blocks3domain,
    problem3_state,
    problem3_goal
)

average_time3 = execute_problem(problem3, lambda p: SearcherMPP(Forward_STRIPS(p)))
print("Average for problem 3:", average_time3)


def custom_heuristic(state, goal):
    heuristic_value = 0
    for key, value in state.items():
        if key in goal and value != goal[key]:
            heuristic_value += 1
    return heuristic_value


average_time_heuristic1 = execute_problem(problem1, lambda p: SearcherMPP(Forward_STRIPS(p, custom_heuristic)))
print("Average for problem 1 with custom heuristic:", average_time_heuristic1)
average_time_heuristic2 = execute_problem(problem2, lambda p: SearcherMPP(Forward_STRIPS(p, custom_heuristic)))
print("Average for problem 2 with custom heuristic:", average_time_heuristic2)
average_time_heuristic3 = execute_problem(problem3, lambda p: SearcherMPP(Forward_STRIPS(p, custom_heuristic)))
print("Average for problem 3 with custom heuristic:", average_time_heuristic3)
