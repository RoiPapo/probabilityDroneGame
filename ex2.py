ids = ["316327451", "318295029"]


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


    def act(self, state):
        pass
