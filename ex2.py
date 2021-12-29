ids = ["316327451", "318295029"]
import copy
import itertools


def all_possible_matches(state):
    drone_lst = list(state["drones"].keys())
    packs_lst = list(state["packages"].keys())
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
        for point, value in adj_dict.items():
            if self.map[point[0]][point[1]] == 'I':
                continue
            dist_table[point] = {}
        for index_1, y in enumerate(self.map):
            for index_2, x in enumerate(y):
                if self.map[index_1][index_2] == 'I':
                    continue
                root = (index_1, index_2)
                self.bfs(graph=adj_dict, root=root)
                self.fill_table(graph=adj_dict, node=root, dist_table=dist_table)
                self.reset(self.map, adj_dict)

        self.bfs_dist = dist_table
        self.best_match = self.find_best_match(data)
        print("hi")

    def find_best_match(self, state):
        all_matches = all_possible_matches(state)
        evaluated_matches = self.evaluate_matches(all_matches, state)
        evaluated_matches.sort(key=lambda x: x[2])
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

    def act(self, state):
        print("hi")

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

    def fill_table(self, graph, node, dist_table):
        for vertix in graph:
            dist_table[node][vertix] = graph[vertix][1][1]
