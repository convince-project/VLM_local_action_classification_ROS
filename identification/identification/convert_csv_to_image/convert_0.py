import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def csv_to_image(csv_file,image_save_path):
    data = pd.read_csv(csv_file)
    gripper_data = data[data["name"] == "gripper_jaws"]
    gripper_position = gripper_data["position"].to_numpy(dtype=np.float64)
    gripper_position_time_stamps = gripper_data["timestamp"].to_numpy()
    gripper_position_time_stamps_no_offset = gripper_position_time_stamps - gripper_position_time_stamps[0]

    lower_bound = np.array([0.04 for _ in range(gripper_position.size)])

    plt.plot(gripper_position_time_stamps_no_offset,gripper_position)
    plt.plot(gripper_position_time_stamps_no_offset,lower_bound)
    plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
    plt.xlabel("time(seconds)")
    plt.ylabel("gripper jaws position")
    plt.title("gripper jaws position's evolution in time")
    plt.savefig(image_save_path)
    plt.close()
