import subprocess
import time
import yaml
import shutil
import os
import signal
import math
import select
# Command to open a new terminal tab and run a command
NEW_TAB_CMD = 'gnome-terminal --tab -- {}'

# Builds a docker image from a image name
def build_image(image_name):
    docker_build_command = ["./dev_env.sh", "build", image_name]
    subprocess.run(docker_build_command, check=True)
    print(f"Docker image {image_name} has been built")


# Open a new terminal tab, run the docker image and run simulation
def simulation(image_name, world_value, mode_value, spawn_position, human_avoidance, drone_avoidance):
    # Command to run a Docker container that also execute simulation.sh inside the container
    DOCKER_RUN_CMD = "./dev_env.sh start {0} simulation.sh {1} {2} \"{3}\" {4} {5}".format(image_name, world_value, mode_value, spawn_position, human_avoidance, drone_avoidance)
    # Open a new terminal tab and start the Docker container, then execute the simulation command
    subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD), shell=True)


# Open a new terminal tab, open a new instance of the docker image and run exploration
def exploration(image_name, config_file):
    # Command to execute a command inside a running Docker container
    DOCKER_EXEC_CMD = "./dev_env.sh exec {0} exploration.sh {1}".format(image_name, config_file)
    # Open a new terminal tab and execute the exploration command inside the running Docker container
    subprocess.run(NEW_TAB_CMD.format(DOCKER_EXEC_CMD), shell=True)


# Open a new terminal tab, open a new instance of the docker image and run exploration
def exploration_dep(image_name, config_file, no_replan):
    # Command to execute a command inside a running Docker container
    DOCKER_EXEC_CMD_VOXBLOX = "./dev_env.sh exec {0} voxblox.sh".format(image_name)
    DOCKER_EXEC_CMD_PLANNER = "./dev_env.sh exec {0} exploration.sh {1} {2}".format(image_name, config_file, no_replan)
    # Open a new terminal tab and execute the exploration command inside the running Docker container
    subprocess.run(NEW_TAB_CMD.format(DOCKER_EXEC_CMD_VOXBLOX), shell=True)
    time.sleep(3)
    subprocess.run(NEW_TAB_CMD.format(DOCKER_EXEC_CMD_PLANNER), shell=True)


# Open a new terminal tab,  open a new instance of the docker image and run save_octomap
def save_octomap(image_name, octomap_name):
     # Command to execute a command inside a running Docker container
     DOCKER_EXEC_CMD_OCTOMAP = "./dev_env.sh exec {0} save_octomap.sh {1}".format(image_name, octomap_name)
     # Open a new terminal tab and execute the save_octomap command inside the running Docker container
     subprocess.run(NEW_TAB_CMD.format(DOCKER_EXEC_CMD_OCTOMAP), shell=True)


def kill_simulation(image_name):
    DOCKER_EXEC_CMD_SHUTDOWN = "./dev_env.sh exec {0} terminate.sh".format(image_name)
    subprocess.run(NEW_TAB_CMD.format(DOCKER_EXEC_CMD_SHUTDOWN), shell=True)


# Waits for timeout seconds or until the exploration is completed.
def check_planning_complete(timeout, image_name):
    command = ['./dev_env.sh', 'topic', image_name, 'exploration_completed.sh']
    process = subprocess.Popen(command, stdout=subprocess.PIPE, shell=False, preexec_fn=os.setsid)
    previous_minutes_remaining = None
    poll_obj = select.poll()
    poll_obj.register(process.stdout, select.POLLIN)
    while True:
        minutes_remaining = math.floor((timeout - time.time()) / 60)
        if minutes_remaining != previous_minutes_remaining:
            print("Minutes remaining: ", minutes_remaining)
            previous_minutes_remaining = minutes_remaining
        poll_result = poll_obj.poll(0)
        if poll_result:
            output = process.stdout.readline().decode("utf-8").strip()
            
            if "data:" in output:
                output = output.split("data: ")[1]  # extract the boolean value as a string
                output = True if output == "True" else False  # convert the string to a boolean
                # If planner is done with exploration, exit
                if output == True:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    return True     

        if time.time() > timeout:
            # If the timeout has been reached, terminate the process
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            return False

        # Wait for the subprocess to finish
        process.poll()
        if process.returncode is not None:
            return False


# Read the YAML file
with open('experiments.yaml', 'r') as file:
    try:
        data = yaml.safe_load(file)
    except yaml.YAMLError as e:
        print(e)


# Extract experiment parameters settings
print("Extracting data from experiments.yaml")
run_time = data['max_time']
N_simulations = data['simulation_runs']
planners = data['planners']
worlds = data['worlds']
modes = data['modes']
human_avoidance = data['human_avoidance']
drone_avoidance = data['drone_avoidance']
no_replan = data['no_replan']

print("\n--------- EXPERIMENT SETUP ---------")
print(f"Running every experiment for {run_time/60} Minutes ({run_time} Seconds)")
print(f"Running with the following planners: {' '.join(planners)}")
print(f"Running with the following worlds: {' '.join(worlds)}")
print(f"Running with the following modes: {' '.join(modes)}")
print(f"Moving objects tries to avoid the drone: {human_avoidance}")
print(f"Drone tries to avoid the moving objects: {drone_avoidance}")

print("------------------------------------")

# Run experiments
print("\n---------------- Starting experiments -----------------")

for planner in planners:
    print(f"--- Starting planning with {planner} ---")

    # Docker image name
    image_name = planners[planner]['image']

    # Build docker image
    build_image(image_name)
    

    for world in worlds:
        print(f"-Simulation world: {world}")
        # Config
        config_file = f"{world}_exploration.yaml"
        spawn_points = data["spawn_positions"][world]

        for mode in modes:
            print(f"-Environment mode: {mode}")
            for iter in range(N_simulations):
                print(f"------- Iteration {iter} -------")

                # Extract the spawn position for this iteration
                spawn_position = spawn_points[iter%5]
                print(f"Drone starts at position (x,y,z) : {spawn_position}")

                
                # Start simulation
                simulation(image_name, world, mode, spawn_position, human_avoidance, drone_avoidance)
                print("Waiting 20 seconds for gazebo simulation to open up")

                # Wait for the simulation to start up
                time.sleep(20)
                
                # Start Exploration
                if planner == "dep":
                    exploration_dep(image_name, config_file, no_replan)
                else:
                    exploration(image_name, config_file)
                
                # Wait for exploration planner to start
                print("Waiting 10 seconds for planner to start up")
                time.sleep(10)

                # Start timer
                print(f"Running experiment for {run_time} seconds")
                timeout = time.time() + run_time
                completed = check_planning_complete(timeout, image_name)
                if completed:
                    print("planner finished early")
                else:
                    print("Time is up")

                #time.sleep(run_time)
                
                # Save octomap
                octomap_name = f"{planner}_{world}_{mode}_{iter}.bt"
                save_octomap(image_name, octomap_name)
                print(f"Saving octomap inside {planner}/octomaps")
                
                time.sleep(5)

                # Kill the ROS nodes and shut down the simulation
                kill_simulation(image_name)

                print("Experiment Done!")

                # Save experiment data
                src_folder = f"./{image_name}/data/"
                dst_folder = f"./experiment_data/{planner}_{world}_{mode}_{iter}/"
                print(f"Saving saving data to {dst_folder}")
                
                # Create the destination directory if it doesn't exist
                if not os.path.exists(dst_folder):
                    os.makedirs(dst_folder)

                # Copy the files from the source directory to the destination directory
                for file_name in os.listdir(src_folder):
                    src_file = os.path.join(src_folder, file_name)
                    dst_file = os.path.join(dst_folder, file_name)
                    shutil.copy(src_file, dst_file)

                time.sleep(10)
                print("waiting 10 seconds for gazebo simulation to close properly")
        
