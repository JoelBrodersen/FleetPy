import pandas as pd
import pathlib
import subprocess
import multiprocessing
import os
import re

py_path = pathlib.Path(__file__)

SELECTED_SCENARIOS = list(range(72,73))
STUDY_NAME = "fleetpy_sumo_coupling_in"
PROCESS_COUNT = 1
SIM_NETWORK_NAME = "sumo_in"

def get_current_max_key(FP_path):
    filenames = os.listdir(FP_path/"studies"/STUDY_NAME/"scenarios")
    keys = [filename.split("_")[0] for filename in filenames]
    pattern = r'^\d+$' ## only digits
    filtered_keys = [int(key) for key in keys if re.match(pattern, key)]
    return max(filtered_keys)

class SimulationRunner:

    def __init__(self,selected_scenarios,study_name,process_count,sim_network_name):
        self.selected_scenarios = selected_scenarios
        self.study_name = study_name
        self.process_count = process_count
        self.py_path = pathlib.Path(__file__)
        sc_config = pd.read_csv(self.py_path.parent/"studies"/self.study_name/"simulation_parameters.csv")
        sc_config = sc_config.set_index('simulation_index')
        self.sc_config = sc_config
        self.sc_config_file_dict = {}
        self.sim_network_name = sim_network_name

    def create_sc_config_files(self):
        
        for sc_index, row in self.sc_config.iterrows():
            sc_df = pd.DataFrame()
            scenario_name = f'{str(sc_index).zfill(2)}_{row["network_name"]}_{row["SAV_demand_ratio"]}_{row["sim_env"]}'
            sc_df["scenario_name"] = [scenario_name]
            sc_df["op_module"] = ["PoolingIRSOnly"]
            sc_df['rq_file'] = [f'demand_in_{row["SAV_demand_ratio"]}.csv']
            sc_df['demand_name'] = [f"demand_in_{row['SAV_demand_ratio']}"]
            sc_df['op_fleet_composition'] = [f"{row['vehtype']}:{row['fleet_size']}"]
            sc_df['op_init_veh_distribution'] = ["init_veh_dist.csv"]
            sc_df['network_type'] = [row['network_type']]
            sc_df['op_vr_control_func_dict'] = [f"func_key:{row['objective_function']};vot:{row['vot']};vor:{row['vor']}"]
            sc_df['sim_env'] = [row['sim_env']]
            sc_df['network_name'] = [row['network_name']]
            sc_df['start_time'] = [row['start_time']]
            sc_df['end_time'] = [row['end_time']]
            sc_df['evaluation_int_start'] = [row['evaluation_int_start']]
            sc_df['evaluation_int_end'] = [row['evaluation_int_end']]
            sc_df['SAV_demand_ratio'] = [row['SAV_demand_ratio']]
            sc_df['sumo_statistics_interval'] = [row['sumo_statistics_interval']]
            sc_df['sumo_fco_vehicles'] = [row['sumo_fco_vehicles']]
            sc_df['op_routing_mode'] = [row['op_routing_mode']]

            self.sc_config_file_dict.update({sc_index:sc_df.squeeze()})
            sc_df.to_csv(py_path.parent/"studies"/STUDY_NAME/"scenarios"/f"{scenario_name}.csv", index=False)
        

    def run_sumo_command(self,sc_index):
        SAV_demand_ratio = float(self.sc_config_file_dict[sc_index].get("SAV_demand_ratio"))
        
        command = [
            "python",
            str(self.py_path.parent/"SUMO_TraciServer.py"),
            str(self.py_path.parent/"studies"/self.study_name/"scenarios"/"constant_config.csv"),
            str(self.py_path.parent/"studies"/self.study_name/"scenarios"/f"{self.sc_config_file_dict[sc_index]['scenario_name']}.csv"),
            str(self.py_path.parent.parent/"fleetpy_coupling"/"Simulation"/self.sim_network_name/f"{self.sim_network_name}_{str(1-SAV_demand_ratio)}.sumocfg"),
            "sumo",
            "info"
        ]
       # try:
        result = subprocess.run(command)
        #result = subprocess.run(command, capture_output=True, text=True)
            #print(f"Process ID {multiprocessing.current_process().pid} - Output:", result.stdout)
            #print(f"Process ID {multiprocessing.current_process().pid} - Errors:", result.stderr)
       # except subprocess.CalledProcessError as e:
           # print(f"Process ID {multiprocessing.current_process().pid} - Command failed with error:", e)

    def run_in_parallel(self):
        print(f"Running {sim_runner.selected_scenarios} on {sim_runner.process_count} processes in paralell.")

        # Create a pool of workers and execute the function in parallel
        with multiprocessing.Pool(self.process_count) as pool:
            pool.map(self.run_sumo_command,self.selected_scenarios)  # Mapping the function to run across multiple processes

if __name__ == "__main__":
    sim_runner = SimulationRunner(selected_scenarios=SELECTED_SCENARIOS,study_name=STUDY_NAME,process_count=PROCESS_COUNT,sim_network_name=SIM_NETWORK_NAME)
    sim_runner.create_sc_config_files()
    sim_runner.run_in_parallel()
