import csv
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

def get_joint_info(sim, joint_handle):
    position = sim.getJointPosition(joint_handle)#Position: The current position or angle of the joint, obtained using sim.getJointPosition(joint_handle).
    velocity = sim.getObjectFloatParameter(joint_handle, sim.jointfloatparam_velocity)[1]#Velocity: The current velocity of the joint, retrieved with sim.getObjectFloatParameter(joint_handle, sim.jointfloatparam_velocity)[1].
    torque = sim.getJointForce(joint_handle)#Torque: The current torque or force being exerted by the joint, accessed using sim.getJointForce(joint_handle).
    
    # Debugging: Print if torque is None
    if torque is None:
        print(f"Warning: Torque for joint {joint_handle} is None.")
    
    return position, velocity, torque

def get_rectangle_mass(sim, rectangle_handle):
    return sim.getShapeMass(rectangle_handle)

# Initialize Remote API client
client = RemoteAPIClient()
sim = client.require('sim')

# Start simulation
sim.setStepping(True)
sim.startSimulation()

# Define joint names
joint_names = ['/axis1', '/axis2', '/axis3', '/axis4']  # Modify with correct joint names
joint_handles = [sim.getObject(joint_name) for joint_name in joint_names]

# Validate joint handles
for joint_name, joint_handle in zip(joint_names, joint_handles):
    if joint_handle == -1:
        raise ValueError(f"Failed to get handle for joint '{joint_name}'")

# Initialize target force or torque for each joint
target_forces = [100.0, 100.0, 100.0, 100.0]  # Example values, adjust as needed
for joint_handle, target_force in zip(joint_handles, target_forces):
    sim.setJointTargetForce(joint_handle, target_force)

# Get the handle for the rectangle object
rectangle_handle = sim.getObject('/Rectangle')
if rectangle_handle == -1:
    raise ValueError("Failed to get handle for rectangle object")

# List to store collected data
data = []
label = 'rectangle'

# Previous velocities for acceleration calculation
previous_velocities = [0.0] * len(joint_handles)
previous_time = 0.0

# Function to dynamically change mass
def change_mass(sim, handle, mass):
    sim.setShapeMass(handle, mass)
    print(f"Changed mass of object {handle} to {mass}")

# Run simulation and collect data
while (t := sim.getSimulationTime()) < 60:
    print(f"Simulation time: {t:.2f} [s], Label: {label}")  # Print simulation time for debugging
    
    # Change mass dynamically at certain time intervals
    if 20 <= t < 30:
        change_mass(sim, rectangle_handle, 10.0)  # Change to 10 kg
    elif 30 <= t < 40:
        change_mass(sim, rectangle_handle, 20.0)  # Change to 20 kg
    elif 40 <= t < 50:
        change_mass(sim, rectangle_handle, 5.0)   # Change to 5 kg
    else:
        change_mass(sim, rectangle_handle, 1.0)   # Change to 1 kg (default)
    
    current_mass = get_rectangle_mass(sim, rectangle_handle)
    print(f"Current mass of rectangle: {current_mass}")  # Print current mass for debugging
    
    joint_data = [t, current_mass]
    current_velocities = []
    for joint_handle in joint_handles:
        position, velocity, torque = get_joint_info(sim, joint_handle)
        current_velocities.append(velocity)
        # Calculate acceleration
        acceleration = (velocity - previous_velocities[joint_handles.index(joint_handle)]) / (t - previous_time) if t - previous_time > 0 else 0.0
        # Handle None torque values
        torque = torque if torque is not None else 0.0
        print(f"Joint {joint_handle} info: Position: {position}, Velocity: {velocity}, Acceleration: {acceleration}, Torque: {torque}")  # Print joint info for debugging
        joint_data.extend([position, velocity, acceleration, torque])
    joint_data.append(label)  # Add the label to the data
    data.append(joint_data)
    previous_velocities = current_velocities[:]
    previous_time = t
    sim.step()
    time.sleep(0.05)  # Small delay to ensure correct simulation stepping

# Stop simulation
sim.stopSimulation()

# Write data to CSV file
header = ['Time', 'Mass']
for i in range(len(joint_handles)):
    header.extend([f'Joint{i+1}_Position', f'Joint{i+1}_Velocity', f'Joint{i+1}_Acceleration', f'Joint{i+1}_Torque'])
header.append('Label')

with open('graph.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(header)
    writer.writerows(data)

print("Data saved to graph.csv")
