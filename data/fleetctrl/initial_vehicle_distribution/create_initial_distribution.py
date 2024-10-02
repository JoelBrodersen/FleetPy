import pandas as pd
import pathlib


py_path = pathlib.Path(__file__)
NET = "sumo_in"

file_path = py_path.parent / "in_6_locations.csv"
net_path = py_path.parent.parent.parent / "networks" / NET /"base"/ "edges.csv"
df = pd.read_csv(file_path)
df_net = pd.read_csv(net_path)

merged_df = pd.merge(df,df_net,left_on="SUMO_EDGE",right_on="source_edge_id")

init_veh_distribution_df  = merged_df[["from_node"]]
init_veh_distribution_df.rename(columns={"from_node":"node_index"})
init_veh_distribution_df["probability"] = round(float(1/len(init_veh_distribution_df["from_node"])),3)
init_veh_distribution_df.to_csv(py_path.parent/ "init_veh_dist.csv")

