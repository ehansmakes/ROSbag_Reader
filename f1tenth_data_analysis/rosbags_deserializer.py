# ▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂
# Import Required Modules
# ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔
from pathlib import Path
import matplotlib.pyplot as plt
import pandas as pd
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
# ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔


# ▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂
# Settings
# ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔
# Set the file path of your ROSbag file
# A ROSbag file should include a 'metadata.yaml' and 'bag_file_name.db3' 
bag_path = Path(r"sample_data\EME185_Run_3") 
# Set save path for .csv file
save_path = r"f1tenth_data_analysis\csv_files"
# Set .csv file name and the title of the plot.
file_name = 'test_run'

# To generate a data plot, set to True 
generate_plot = True
# ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔


# ▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂
# Functions
# ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔
def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)

# For data with a header, you can use the following to
# combine seconds with nanoseconds. This is helps with generating
# data plots with higher granularity than with using seconds. 
def combine_time_with_nanoseconds(seconds_list, nanoseconds_list):
    # Check if the lists have the same length
    if len(seconds_list) != len(nanoseconds_list):
        raise ValueError("Both lists must have the same length")
    
    # 1 nanosecond = 10^-9 seconds
    nano_to_sec = 1e-9
    
    # Combine the lists
    combined_times = [round(sec + (nano * nano_to_sec),8) for sec, nano in zip(seconds_list, nanoseconds_list)]
    
    return combined_times
# ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔


# ▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂
# Import Custom Messages 
# ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔
# Some messages used on the DARC are not standard to 
# ROS2. In this case, we need to include them to the 
# types.py directory for use to deserialize the data. 
add_types = {}

# Get the base typestore based on ROS2 Distro 
typestore = get_typestore(Stores.ROS2_FOXY)

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
VescImu_msgtype = 'vesc_msgs/msg/VescImu'
VescImu_msgdef = """
geometry_msgs/Vector3  ypr
geometry_msgs/Vector3  linear_acceleration
geometry_msgs/Vector3  angular_velocity

geometry_msgs/Vector3  compass
geometry_msgs/Quaternion orientation
"""
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
VescImuStamped_msgtype = 'vesc_msgs/msg/VescImuStamped'
VescImuStamped_msgdef  = """
std_msgs/Header  header
VescImu imu
"""
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	

add_types.update(get_types_from_msg(VescImu_msgdef, VescImu_msgtype))
add_types.update(get_types_from_msg(VescImuStamped_msgdef, VescImuStamped_msgtype))

# Add custom messages to types directory
typestore.register(add_types)
# ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔


# ▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂
# Lists Used For Analysis 
# ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔
time_header_sec = [] 
time_header_nanosec = []

LIDAR_velocity_x = []
LIDAR_velocity_y = []
LIDAR_velocity_z = []

imu_acceleration_x = []
imu_acceleration_y = []
imu_acceleration_z = []
# ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔


# ▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂
# Read and Record Your ROSbag Data
# ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔
# Create reader instance and open for reading.
with AnyReader([bag_path], default_typestore=typestore) as reader:
    connections = [x for x in reader.connections if x.topic == r"/sensors/imu"]
    for connection, timestamp, rawdata in reader.messages(connections=connections):
         msg = reader.deserialize(rawdata, connection.msgtype)
         time_header_sec.append(msg.header.stamp.sec)
         time_header_nanosec.append(msg.header.stamp.nanosec)
         imu_acceleration_x.append(msg.imu.linear_acceleration.x)

# Generating data for the trial_time list. This expresses trial time to the nanosecond
first_entry = time_header_sec[0]
time_header_sec = [num - first_entry for num in time_header_sec]
trial_time = combine_time_with_nanoseconds(time_header_sec,time_header_nanosec)


# Create a dictionary of the lists  
dict = { 'seconds' : trial_time, 'acceleration_x' : imu_acceleration_x
}
# Transfer the dictionary to a dataframe
df = pd.DataFrame(data=dict)
# Record dictionary as a CSV file
df = df.to_csv(f'{save_path}/{file_name}.csv', index=False)  
# ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔


# ▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂
# Generate Plot
# ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔
if generate_plot == True:
    
    # Add Plot Name 
    plt.figure(file_name)
    
    # Add Data 
    plt.plot(trial_time, imu_acceleration_x, label='imu_acceleration_x')

    # Add labels and title
    plt.xlabel('Time Step (seconds)')
    plt.ylabel('m/s^2')
    plt.title(file_name)

    # Add legend
    plt.legend()

    plt.grid()

    # Show plot
    plt.show()
    # ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔