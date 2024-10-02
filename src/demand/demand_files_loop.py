import subprocess

NETWORK_NAME = "sumo_in"

def run_sumo_demand_script(demand_ratio):
    # Define the command and its arguments
    route_file = f"D:\\GitHub\\fleetpy_coupling\\Simulation\\sumo_in\\Routes\\routes_fcd_period_mo_thu_24h_det_calib.rou_rnd_00_subset_{str(demand_ratio)}.rou.xml.gz"
    if demand_ratio == 0.5:
        route_file = f"D:\\GitHub\\fleetpy_coupling\\Simulation\\sumo_in\\Routes\\routes_fcd_period_mo_thu_24h_det_calib.rou_rnd_00_subset_{str(demand_ratio)}_1.rou.xml.gz"


    command = [
        "python", 
        "D:\\GitHub\\Fleetpy\\src\\demand\\demand_from_sumo.py",
        route_file,           
        f"demand_in_{str(demand_ratio)}",
        "routes",
        NETWORK_NAME
    ]
    
    # Run the command and capture the output
    subprocess.run(command)


# Call the function
for demand_ratio in [0.02,0.05,0.1,0.2,0.3,0.4,0.6,0.7,0.8,0.9,1.0]:
    print(f"Creating FleetPy Demand for {NETWORK_NAME} and {demand_ratio*100} % of Routes served by SAVs")
    run_sumo_demand_script(demand_ratio)
