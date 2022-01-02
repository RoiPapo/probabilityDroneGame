ids = ["316327451", "318295029"]
import copy
import itertools


def all_possible_matches(state):
    drone_lst = list(state["drones"].keys())
    packs_lst = list(state["packages"].keys())
    if len(drone_lst) < len(packs_lst):
        all_matches = [list(zip(drone_lst, x)) for x in itertools.permutations(packs_lst, len(drone_lst))]
        return all_matches
    all_matches = [list(zip(x, packs_lst)) for x in itertools.permutations(drone_lst, len(packs_lst))]
    return all_matches


class DroneAgent:
    def __init__(self, initial):
        self.map = initial['map']
        drone_init = {}
        package_init = {}
        clients_init = {}
        for key in initial['drones']:
            drone_init[key] = {'loc': initial['drones'][key], 'holding': ["null", "null"]}
        for key in initial['packages']:
            package_init[key] = {'loc': initial['packages'][key], 'belong': "null", 'holder': "null"}
        for key in initial['clients']:
            clients_init[key] = {'probabilities': initial['clients'][key]['probabilities'],
                                 'packages': initial['clients'][key]['packages'],
                                 'loc': initial['clients'][key]['location']}
        for client in clients_init:
            for package in clients_init[client]['packages']:
                package_init[package]['belong'] = client
        data = {
            'drones': drone_init,
            'packages': package_init,
            'clients': clients_init
        }
        self.d_num = len(drone_init.keys())
        self.p_num = len(package_init.keys())
        used_packages = {}
        for package, package_dict in data['packages'].items():
            if package_dict['belong'] != 'null':
                used_packages[package] = package_dict
        data['packages'] = used_packages
        ##### bfs distance dict ################################
        adj_dict = self.set_up_graph(self.map)
        dist_table = {}
        shortest_paths = {}
        for point, value in adj_dict.items():
            if self.map[point[0]][point[1]] == 'I':
                continue
            dist_table[point] = {}
            shortest_paths[point] = {}
        for index_1, y in enumerate(self.map):
            for index_2, x in enumerate(y):
                if self.map[index_1][index_2] == 'I':
                    continue
                root = (index_1, index_2)
                self.bfs(graph=adj_dict, root=root)
                self.fill_table(graph=adj_dict, node=root, dist_table=dist_table, shortest_paths=shortest_paths)
                self.reset(self.map, adj_dict)
        self.data = data
        self.shortest_paths = shortest_paths
        self.bfs_dist = dist_table
        self.best_match = self.find_best_match(data)
        self.value_iteration_tables = {}
        self.BZ_Drones = [item[0] for item in self.best_match[:-1]]
        self.idle_drones = set(self.data["drones"]) - set(self.BZ_Drones)

    def find_best_match(self, state):
        all_matches = all_possible_matches(state)
        evaluated_matches = self.evaluate_matches(all_matches, state)
        evaluated_matches.sort(key=lambda x: x[1])
        return evaluated_matches[0]

    def evaluate_matches(self, all_matches, state):
        for match in all_matches:
            counter = 0
            for dist in match:
                d = dist[0]
                p = dist[1]
                counter += self.bfs_dist[state['drones'][d]['loc']][state['packages'][p]['loc']]
            match.append(counter)
        return all_matches

    def check_drone_status(self, state):
        drones_before_taking = []
        drones_holding = []
        for package in state['packages'].keys():
            if package not in self.data['packages'].keys():
                continue  # makes sure that we dont try to give package that dosent belong to anyone
            relevant_match = [item for item in self.best_match[:-1] if item[1] == package]
            if len(relevant_match) == 0:
                continue

            if isinstance(state['packages'][package], str):
                drones_holding.append(relevant_match[0])
            else:
                drones_before_taking.append(relevant_match[0])
        return drones_before_taking, drones_holding

    def act(self, state):
        all_actions = []
        drones_before_taking, drones_holding = self.check_drone_status(state)
        for match in drones_before_taking:
            drone_loc = state['drones'][match[0]]
            pack_loc = state['packages'][match[1]]
            if drone_loc == pack_loc:
                all_actions.append(('pick up', match[0], match[1]))  ####### PICK UP ##########
                costumer = self.data["packages"][match[1]]["belong"]
                posebilities_of_client = self.data["clients"][costumer]['probabilities']
                self.value_iteration_tables[match[0]] = value_iteration(posebilities_of_client, self.map,
                                                                        state["turns to go"])
            else:
                # if self.shortest_paths[pack_loc][drone_loc][-1] == pack_loc:
                #     self.shortest_paths[pack_loc][drone_loc].pop()
                all_actions.append(('move', match[0], self.shortest_paths[pack_loc][drone_loc][0]))
                del self.shortest_paths[pack_loc][drone_loc][0]

        ############ part 2 ####################

        for drone in self.idle_drones:
            all_actions.append(('wait', str(drone)))

        steps_left = state["turns to go"]
        for (drone, package) in drones_holding:
            packages_owner = self.data["packages"][package]["belong"]
            current_drone_client_loc = (state['drones'][drone], state['clients'][packages_owner]['location'])
            future_drone_loc_VI = cal_best_action(current_drone_client_loc, steps_left,
                                                  self.value_iteration_tables[drone],
                                                  self.map,
                                                  state['clients'][packages_owner]['probabilities'])
            if current_drone_client_loc[0] == current_drone_client_loc[1]:
                all_actions.append(("deliver", str(drone), str(packages_owner), str(package)))
                self.idle_drones.add(drone)
                drones_holding.remove((drone, package))

            elif future_drone_loc_VI == state['drones'][drone]:
                all_actions.append(('wait', str(drone)))
            else:
                all_actions.append(('move', str(drone), future_drone_loc_VI))

        if len(state["packages"]) == 0:
            return "reset"

        # if len(all_actions) == 2:
        # print("WTF")

        if len(all_actions) != len(state["drones"].keys()):
            print("wtf")
            # all_actions.append(('wait', 'drone 1'))
        print(tuple(all_actions))

        return tuple(all_actions)

    def set_up_graph(self, map):
        adj_dict = {}

        for index_1, y in enumerate(map):
            for index_2, x in enumerate(y):
                if map[index_1][index_2] == 'P':
                    adj_dict[(index_1, index_2)] = ([], ['white', 1000000, 'null'])

        for index_1, y in enumerate(map):
            for index_2, x in enumerate(y):
                if map[index_1][index_2] == 'I':
                    continue
                if index_1 - 1 >= 0 and map[index_1 - 1][index_2] == 'P':
                    adj_dict[(index_1, index_2)][0].append((index_1 - 1, index_2))
                if index_1 + 1 < len(map) and map[index_1 + 1][index_2] == 'P':
                    adj_dict[(index_1, index_2)][0].append((index_1 + 1, index_2))
                if index_2 - 1 >= 0 and map[index_1][index_2 - 1] == 'P':
                    adj_dict[(index_1, index_2)][0].append((index_1, index_2 - 1))
                if index_2 + 1 < len(map[0]) and map[index_1][index_2 + 1] == 'P':
                    adj_dict[(index_1, index_2)][0].append((index_1, index_2 + 1))
                if index_1 - 1 >= 0 and index_2 - 1 >= 0 and map[index_1 - 1][index_2 - 1] == 'P':
                    adj_dict[(index_1, index_2)][0].append((index_1 - 1, index_2 - 1))
                if index_1 - 1 >= 0 and index_2 + 1 < len(map[0]) and map[index_1 - 1][index_2 + 1] == 'P':
                    adj_dict[(index_1, index_2)][0].append((index_1 - 1, index_2 + 1))
                if index_1 + 1 < len(map) and index_2 - 1 >= 0 and map[index_1 + 1][index_2 - 1] == 'P':
                    adj_dict[(index_1, index_2)][0].append((index_1 + 1, index_2 - 1))
                if index_1 + 1 < len(map) and index_2 + 1 < len(map[0]) and map[index_1 + 1][index_2 + 1] == 'P':
                    adj_dict[(index_1, index_2)][0].append((index_1 + 1, index_2 + 1))

        return adj_dict

    def reset(self, map, adj_dict):
        for index_1, y in enumerate(map):
            for index_2, x in enumerate(y):
                if map[index_1][index_2] == 'P':
                    adj_dict[(index_1, index_2)][1][0] = 'white'
                    adj_dict[(index_1, index_2)][1][1] = 1000000
                    adj_dict[(index_1, index_2)][1][2] = 'null'

    def bfs(self, graph, root):
        visited = []  # List to keep track of visited nodes.
        queue = []  # Initialize a queue
        graph[root][1][0] = 'grey'
        graph[root][1][1] = 0
        visited.append(root)
        queue.append(root)
        while queue:
            s = queue.pop(0)
            for neighbour in graph[s][0]:
                if graph[neighbour][1][0] == 'white':
                    graph[neighbour][1][0] = 'grey'
                    graph[neighbour][1][1] = graph[s][1][1] + 1
                    graph[neighbour][1][2] = s
                    queue.append(neighbour)

    def fill_table(self, graph, node, dist_table, shortest_paths):
        for vertix in graph:
            shortest_paths[node][vertix] = []
            dist_table[node][vertix] = graph[vertix][1][1]
            curr_vertix = vertix
            while curr_vertix != node:
                curr_vertix = graph[curr_vertix][1][2]
                shortest_paths[node][vertix].append(curr_vertix)
        # print("cat")

################################## value_iteration ########################################

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
            possible_drone_locations = cal_possible_actions(map, drone_loc)
            max_value = 0
            for possible_drone_location in possible_drone_locations:
                expectation = 0
                for probability, client_possible_loc in probabilities_and_locations:
                    expectation += probability * Value_per_State_t_minus_1[
                        (possible_drone_location, client_possible_loc)]
                if expectation > max_value:
                    max_value = expectation
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
        future_loc[
            0] = client_loc  # I assigned the current location for easing the programing - doesnt use it anyway
    else:
        future_loc[0] = (client_loc[0] - 1, client_loc[1])
    if y - 1 < 0:
        probabilities[2] = 0
        future_loc[
            2] = client_loc  # I assigned the current location for easing the programing - doesnt use it anyway
    else:
        future_loc[2] = (client_loc[0], client_loc[1] - 1)
    if x + 1 >= len(map):
        probabilities[1] = 0
        future_loc[
            1] = client_loc  # I assigned the current location for easing the programing - doesnt use it anyway
    else:
        future_loc[1] = (client_loc[0] + 1, client_loc[1])
    if y + 1 >= len(map[0]):
        probabilities[3] = 0
        future_loc[
            3] = client_loc  # I assigned the current location for easing the programing - doesnt use it anyway
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
    max_value = 0
    for action in possible_actions:
        expectation = 0
        for probability, drone_loc in probabilities_and_locations:
            expectation += probability * value_iteration_output[t - 1][(action, drone_loc)]
        if expectation > max_value:
            best_action = action
            max_value = expectation
    return best_action
