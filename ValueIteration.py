import itertools
import copy


def main():
    input = {
        "map": [['P', 'P', 'P', 'P'],
                ['P', 'P', 'P', 'P'],
                ['P', 'I', 'P', 'P'],
                ['P', 'P', 'P', 'P'], ],
        "drones": {'drone 1': (3, 3)},
        "packages": {'package 1': (2, 2),
                     'package 2': (1, 1)},
        "clients": {'Alice': {"location": (0, 1),
                              "packages": ('package 1', 'package 2'),
                              "probabilities": (0.6, 0.1, 0.1, 0.1, 0.1)}},
        "turns to go": 100
    }
    value_iteration_output = value_iteration((0.6, 0.1, 0.1, 0.1, 0.1), input['map'], 50)
    best_action = cal_best_action(((3, 3), (0, 1)), 10, value_iteration_output, input['map'], (0.6, 0.1, 0.1, 0.1, 0.1))
    print(best_action)


def value_iteration(probabilities, map, T):
    """
    The function accepts:
        current location of drone with packages
        current locations of the packages owner
        probabilities of owner moving
        map
        T - number of remaining rounds
    """
    probabilities = list(probabilities)
    value_iteration_output = list(range(0, T))
    ############# Creating states ############################
    flat_map = []
    for index_1, sublist in enumerate(map):
        for index_2, item in enumerate(sublist):
            flat_map.append((index_1, index_2))

    list_for_cartesian_product = [flat_map, flat_map.copy()]
    states = list(itertools.product(*list_for_cartesian_product))

    ################# Value Iteration #########################
    Rewards = {}
    Value_per_State_t_minus_1 = {}
    Value_per_State_t = {}

    for state in states:
        drone_loc = state[0]
        client_loc = state[1]
        if drone_loc == client_loc:
            Rewards[state] = 10
            Value_per_State_t_minus_1[state] = 10
        else:
            Rewards[state] = 0
            Value_per_State_t_minus_1[state] = 0

    value_iteration_output[0] = Rewards
    for epoch in range(1, T):
        for state in states:
            drone_loc = state[0]
            client_loc = state[1]
            probabilities_and_locations = cal_probability(map, client_loc, probabilities)
            possible_actions = cal_possible_actions(map, drone_loc)
            max_value = 0
            for action in possible_actions:
                expectation = 0
                prev_expectation = 0
                for probability, drone_loc in probabilities_and_locations:
                    expectation += probability * Value_per_State_t_minus_1[(action, drone_loc)]
                if expectation > prev_expectation:
                    prev_expectation = expectation
            max_value = prev_expectation
            Value_per_State_t[state] = Rewards[state] + max_value
        value_iteration_output[epoch] = Value_per_State_t
        Value_per_State_t_minus_1 = Value_per_State_t

    return value_iteration_output


def cal_probability(map, client_loc, probabilities):
    probabilities = probabilities.copy()
    future_loc = [0, 0, 0, 0, client_loc]
    x = client_loc[0]
    y = client_loc[1]
    if x - 1 < 0:
        probabilities[0] = 0
        future_loc[0] = client_loc  # I assigned the current location for easing the programing - doesnt use it anyway
    else:
        future_loc[0] = (client_loc[0] - 1, client_loc[1])
    if y - 1 < 0:
        probabilities[2] = 0
        future_loc[2] = client_loc  # I assigned the current location for easing the programing - doesnt use it anyway
    else:
        future_loc[2] = (client_loc[0], client_loc[1] - 1)
    if x + 1 >= len(map):
        probabilities[1] = 0
        future_loc[1] = client_loc  # I assigned the current location for easing the programing - doesnt use it anyway
    else:
        future_loc[1] = (client_loc[0] + 1, client_loc[1])
    if y + 1 >= len(map[0]):
        probabilities[3] = 0
        future_loc[3] = client_loc  # I assigned the current location for easing the programing - doesnt use it anyway
    else:
        future_loc[3] = (client_loc[0], client_loc[1] + 1)

    probabilities = [[x / sum(probabilities), future_loc[index]] for index, x in enumerate(probabilities)]
    return probabilities


def cal_possible_actions(map, drone_loc):
    possible_actions = [drone_loc]
    index_1 = drone_loc[0]
    index_2 = drone_loc[1]
    if index_1 - 1 >= 0 and map[index_1 - 1][index_2] == 'P':
        possible_actions.append((index_1 - 1, index_2))
    if index_1 + 1 < len(map) and map[index_1 + 1][index_2] == 'P':
        possible_actions.append((index_1 + 1, index_2))
    if index_2 - 1 >= 0 and map[index_1][index_2 - 1] == 'P':
        possible_actions.append((index_1, index_2 - 1))
    if index_2 + 1 < len(map[0]) and map[index_1][index_2 + 1] == 'P':
        possible_actions.append((index_1, index_2 + 1))
    if index_1 - 1 >= 0 and index_2 - 1 >= 0 and map[index_1 - 1][index_2 - 1] == 'P':
        possible_actions.append((index_1 - 1, index_2 - 1))
    if index_1 - 1 >= 0 and index_2 + 1 < len(map[0]) and map[index_1 - 1][index_2 + 1] == 'P':
        possible_actions.append((index_1 - 1, index_2 + 1))
    if index_1 + 1 < len(map) and index_2 - 1 >= 0 and map[index_1 + 1][index_2 - 1] == 'P':
        possible_actions.append((index_1 + 1, index_2 - 1))
    if index_1 + 1 < len(map) and index_2 + 1 < len(map[0]) and map[index_1 + 1][index_2 + 1] == 'P':
        possible_actions.append((index_1 + 1, index_2 + 1))
    return possible_actions


def cal_best_action(state, t, value_iteration_output, map, probabilities):
    probabilities = list(probabilities)
    drone_loc = state[0]
    client_loc = state[1]
    possible_actions = cal_possible_actions(map, drone_loc)
    best_action = possible_actions[1]
    probabilities_and_locations = cal_probability(map, client_loc, probabilities)
    for action in possible_actions:
        expectation = 0
        prev_expectation = 0
        for probability, drone_loc in probabilities_and_locations:
            expectation += probability * value_iteration_output[t - 1][(action, drone_loc)]
        if expectation > prev_expectation:
            best_action = action
            prev_expectation = expectation
    return best_action


if __name__ == '__main__':
    main()
