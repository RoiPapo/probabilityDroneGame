import itertools
import random

ids = ["316327451", "318295029"]


class DroneAgent:
    def __init__(self, initial):
        self.init_state = initial
        self.map = initial['map']
        self.I_locations = []
        self.count_waits = 0
        for i, x in enumerate(self.map):
            for j, y in enumerate(x):
                if y == 'I':
                    self.I_locations.append((i, j))
        self.avg_run = 0
        self.turn_counter = 0
        self.number_of_resets = 0
        self.times = []

        ################ bfs distance dict ################################
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
        self.shortest_paths = shortest_paths
        self.bfs_dist = dist_table

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

    def send_reset(self, state):
        self.count_waits = 0
        self.times.append(self.turn_counter)
        self.turn_counter = 0
        self.number_of_resets += 1
        self.avg_run = sum(self.times) / len(self.times)
        if self.avg_run >= state["turns to go"]:
            # print("terminate")
            return "terminate"
        # print("reset")
        return 'reset'

    def act(self, state):
        self.turn_counter += 1
        final_actions = []
        if len(state['packages']) == 0:
            return self.send_reset(state)
        for drone_name, drone_loc in state['drones'].items():
            priority = 0
            # deliver
            priority = self.delivery(state, drone_name, drone_loc, final_actions)
            if priority == 1:
                continue
            # pick up
            priority = self.pick_up(state, drone_name, drone_loc, final_actions)
            if priority == 1:
                continue
            # move
            priority = self.move_drone(state, drone_name, drone_loc, final_actions)
        count_acts = 0
        for act in final_actions:
            if act[0] == 'wait':
                count_acts += 1
        if count_acts >= len(final_actions):
            self.count_waits += 1
        else:
            self.count_waits = 0
        if self.count_waits >= 5:
            return self.send_reset(state)
        return final_actions

    def delivery(self, state, drone_name, drone_loc, final_actions):
        for client_name, client_dict in state['clients'].items():
            if client_dict['location'] == drone_loc:
                for c_pack in client_dict['packages']:
                    if c_pack in self.pack_carry_drone(state, drone_name):
                        final_actions.append(['deliver', drone_name, client_name, c_pack])
                        return 1
        return 0

    def pick_up(self, state, drone_name, drone_loc, final_actions):
        packs_drone = 0
        for pack_name, pack_loc in state['packages'].items():
            if drone_name == pack_loc:
                packs_drone += 1
        for pack_name, pack_loc in state['packages'].items():
            picked_already = 0
            for act in final_actions:
                if act[0] == 'pick up' and act[2] == pack_name:
                    picked_already = 1
            if drone_loc == pack_loc and packs_drone < 2 and picked_already == 0:
                final_actions.append(['pick up', drone_name, pack_name])
                return 1
        return 0

    def pack_carry_drone(self, state, drone_name):
        out = []
        for d_name, d_loc in state['drones'].items():
            if drone_name == d_name:
                for pack_name, pack_loc in state['packages'].items():
                    if d_name == pack_loc:
                        out.append(pack_name)
        return out

    def package_per_client(self, state, package_name):
        for client_name, client_dict in state['clients'].items():
            if package_name in client_dict['packages']:
                return client_name
        return None

    def move_drone(self, state, drone_name, drone_loc, final_actions):
        packages_on_drone = self.pack_carry_drone(state, drone_name)
        if packages_on_drone:
            client_name = self.package_per_client(state, packages_on_drone[0])
            client_next_move = self.prob_client_next_move(state, client_name, state['clients'][client_name]['location'])
            next_move = self.make_best_move(drone_loc, client_next_move)
        elif self.no_products_to_carry(state):
            final_actions.append(['wait', drone_name])
            return 1
        else:
            pack_details = self.find_closest_package(state, drone_name, drone_loc)
            next_move = self.make_best_move(drone_loc, pack_details[2])
        if drone_loc == next_move:
            final_actions.append(['wait', drone_name])
        else:
            final_actions.append(['move', drone_name, next_move])
        return 1

    def no_products_to_carry(self, state):
        for pack_name, pack_loc in state['packages'].items():
            if type(pack_loc) != str:
                return False
        return True

    def prob_client_next_move(self, state, client_name, client_loc):
        probs_client = state['clients'][client_name]['probabilities']
        prob_client = self.cal_probability(client_loc, probs_client)
        next_loc = max(prob_client, key=lambda x: x[0])
        return next_loc[1]

    def cal_probability(self, client_loc, probabilities):
        probabilities = list(probabilities).copy()
        future_loc = [0, 0, 0, 0, client_loc]
        x = client_loc[0]
        y = client_loc[1]
        if x - 1 < 0:
            probabilities[0] = 0
            future_loc[0] = 'Null'
        else:
            future_loc[0] = (client_loc[0] - 1, client_loc[1])
        if y - 1 < 0:
            probabilities[2] = 0
            future_loc[2] = 'Null'
        else:
            future_loc[2] = (client_loc[0], client_loc[1] - 1)
        if x + 1 >= len(self.map):
            probabilities[1] = 0
            future_loc[1] = 'Null'
        else:
            future_loc[1] = (client_loc[0] + 1, client_loc[1])
        if y + 1 >= len(self.map[0]):
            probabilities[3] = 0
            future_loc[3] = 'Null'
        else:
            future_loc[3] = (client_loc[0], client_loc[1] + 1)

        probabilities = [[x / sum(probabilities), future_loc[index]] for index, x in enumerate(probabilities)]
        return probabilities

    def make_best_move(self, drone_loc, dest_loc):
        simple_moves = [[1, 0], [0, 1], [1, 1], [-1, 1], [-1, -1], [1, -1], [0, -1], [-1, 0], [0, 0]]
        valid_moves = []
        for s_m in simple_moves:
            checked_move = (drone_loc[0] + s_m[0], drone_loc[1] + s_m[1])
            if self.valid_move(checked_move):
                valid_moves.append(checked_move)
        v_m_distances = []
        for v_m in valid_moves:
            if self.map[dest_loc[0]][dest_loc[1]] == 'I':
                v_m_distances.append([v_m, self.euc_dist(v_m, dest_loc)])
            else:
                v_m_distances.append([v_m, self.bfs_dist[v_m][dest_loc]])
        best_move = min(v_m_distances, key=lambda x: x[1])
        return best_move[0]

    def valid_move(self, loc):
        if len(self.map) > loc[0] >= 0 and loc[1] < len(self.map[0]) and loc[
            1] >= 0 and loc not in self.I_locations:
            return True
        return False

    def find_closest_package(self, state, drone_name, drone_loc):
        out = []
        for pack_name, pack_loc in state['packages'].items():
            if type(pack_loc) != str:
                out.append((self.euc_dist(pack_loc, drone_loc), pack_name, pack_loc))
        return min(out, key=lambda x: x[0])

    def euc_dist(self, loc1, loc2):
        return ((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2) ** 0.5

    def best_next_move(self, loc1, loc2):
        Q = [(loc1, None)]
        Q_next = []
        visited = set()
        all_locs = []
        while Q:
            curr_loc = Q.pop(0)
            all_locs.append(curr_loc)
            visited.add(curr_loc[0])
            dist = 0
            curr_neigh = self.valid_moves(curr_loc[0])
            if curr_loc[0] == loc2:
                break
            for x in curr_neigh:
                if x not in visited:
                    Q_next.append((x, curr_loc))
            if not Q:
                Q = Q_next
                dist += 1

    def valid_moves(self, loc):
        simple_moves = [[1, 0], [0, 1], [1, 1], [-1, 1], [-1, -1], [1, -1], [0, -1], [-1, 0], [0, 0]]
        valid_moves = []
        for s_m in simple_moves:
            checked_move = (loc[0] + s_m[0], loc[1] + s_m[1])
            if self.valid_move(checked_move):
                valid_moves.append(checked_move)
        return valid_moves
