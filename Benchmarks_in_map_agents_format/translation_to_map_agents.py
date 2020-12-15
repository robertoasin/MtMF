import os
import shutil

from_folders = {"agents": "grid20_ag/Instances",
                "obstacles": "grid20_obs/Instances",
                "warehouses": "warehouse/Instances"}

to_folders = {"agents": "grid20_ag_map_agents_format/",
              "obstacles": "grid20_obs_map_agents_format/",
              "warehouses": "warehouse_map_agents_format/"}

for problem in from_folders.keys():
    if not os.path.exists(to_folders[problem]):
        os.mkdir(to_folders[problem])
        print('You created', to_folders[problem], 'directory')
    else:
        shutil.rmtree(to_folders[problem])
        os.mkdir(to_folders[problem])
        print('Creating', to_folders[problem], '...')

    for file in os.listdir(from_folders[problem]):
        with open(from_folders[problem] + '/' + file, 'r') as instance:
            data = instance.readlines()
            map_file = open(to_folders[problem] + file + '.map', 'w+')
            agents_file = open(to_folders[problem] + file + '.agents', 'w+')

            writing_grid = False
            writing_agents = False

            for line in data:
                if line == 'Grid:\n':
                    writing_grid = True
                    writing_agents = False
                elif line == 'Agents:\n':
                    writing_grid = False
                    writing_agents = True

                elif writing_grid:
                    map_file.write(line)
                elif writing_agents:
                    c = line.strip('\n').split(',')
                    if len(c) > 1:
                        agents_file.write(f'{c[3]},{c[4]},{c[1]},{c[2]}\n')
                    else:
                        agents_file.write(f'{c[0]}\n')

            map_file.close()
            agents_file.close()
                    