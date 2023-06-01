import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from tabulate import tabulate

# Variables
files = ["collision.csv", "coverage.csv", "logfile.csv", "path.csv"]
worlds = ["cafe", "maze", "apartment", "tunnel", "field", "auditorium", "crosswalks", "exhibition", "patrol", "granso", "granso2"]
modes = ["static", "dynamic"]
planners = ["aep", "depr", "dep", "nbvp", "daep"]

N = 5 #Should be the same as simulation_runs in experiments.yaml (Parametrize)

data_path = "visualized_data/"
logfile_path = data_path + "logfile.txt"
collision_path = data_path + "collision.txt"
coverage_path = data_path + "coverage.txt"
agg_path = data_path + "aggregated.txt"

# Store volume (%) plots to the following path
volume_p_path = data_path + "plots/coverage_p"

# Store volume (m3) plots to the following path
volume_v_path = data_path + "plots/coverage_v"

# Store average volume (m3) plots to the following path
volume_avg_path = data_path + "plots/coverage_avg"


#################################### LABELS ###########################################################

def get_legend_name(df_key):
    if "daep" in df_key:
        return "DAEP"
    elif "aep" in df_key:
        return "AEP"
    elif "depr" in df_key:
        return "DEP-R"
    elif "dep" in df_key:
        return "DEP"
    else:
        return "RH-NBVP"


#################################### COVERAGE ###########################################################

def plot_coverage_average(world, data_frames, max_time):
    N_interpolation_points = int(np.ceil(max_time*2))
    
    plt.figure()
    for key in data_frames.keys():
        # get all the simulation for certain world and planner
        dfs = data_frames[key]
        N_df = len(dfs)
        Y = np.zeros((N_df, N_interpolation_points))
        max_t = 0
        for i in range(len(dfs)):
            if dfs[i]['Time'].iloc[-1] > max_t:
                max_t = dfs[i]['Time'].iloc[-1]
        
        if "daep" in key:
            color = "cyan"
        elif "aep" in key:
            color = "blue"
        elif "depr" in key:
            color = "orange"
        elif "dep" in key:
            color = "red"
        else:
            color = "green" 
        for j, df in enumerate(dfs):
            y = df[' Coverage (m3)']
            
            x = np.linspace(0, max_t, N_interpolation_points)
            interpolate_df = np.interp(x, np.linspace(0, max_t, len(y)), y)
            Y[j, :] = interpolate_df

        mean_Y = np.mean(Y, axis=0)
        std_Y = np.std(Y, axis=0)

        plt.plot(x, mean_Y, color=color, label=get_legend_name(key), linewidth=2)
        plt.plot(x, mean_Y + std_Y, color=color, linestyle='--', linewidth=2)
        plt.plot(x, mean_Y - std_Y, color=color, linestyle='--', linewidth=2)
        # Fill the area between the lines
        plt.fill_between(x, mean_Y - std_Y, mean_Y + std_Y, color=color, alpha=0.2)

        plt.xlabel('Time [s]')
        plt.ylabel('Coverage [m3]')
        plt.title(f'World: {world} - Mean and Standard deviation')
        plt.legend()

    # create the subdirectory if it does not exist
    if not os.path.exists(volume_avg_path):
        os.makedirs(volume_avg_path)
        
    # save the plot in the subdirectory
    plt.savefig(volume_avg_path + f"/{world}")



def visualize_coverage_avg(show):
    #for file in files:
    file = "coverage.csv"
    for world in worlds:
        dfs = {}
        longest_exploration_time = None
        for mode in ["dynamic"]:
            for planner in planners:
                for iteration in range(N):
                    file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{iteration}/{file}"
                    if os.path.isfile(file_path): 
                        #Check if file exists
                        df = pd.read_csv(file_path)

                        if longest_exploration_time is None:
                            longest_exploration_time = df['Time'].iloc[-1]

                        elif df['Time'].iloc[-1] > longest_exploration_time:
                            longest_exploration_time = df['Time'].iloc[-1]
                        
                        key = f"{mode}_{planner}"
                        if key in dfs:
                            dfs[key].append(df)
                        else:
                            dfs[key] = [df]
        
        if longest_exploration_time is not None:
            
            plot_coverage_average(world, dfs, longest_exploration_time)
    if show:
        plt.show()



# Save plot of coverage (%) for the best simulation of every planner and mode for a certain world
def plot_coverage_percent(world, data_frames, interval_dfs):
    
    plt.figure()
    ax = plt.subplot(111)
    paintOnce = True
    colors = ["blue", "red", "green", "purple"]

    for i, df_key in enumerate(data_frames):
        if "daep" in df_key:
            color = "cyan"
        elif "aep" in df_key:
            color = "blue"
        elif "dep" in df_key:
            color = "red"
        else:
            color = "green"        

        if "static" in df_key:
            linestyle = '-'
        else:
            linestyle = '--'

        # Coverage data
        cov_key = data_frames[df_key].columns[2]
        coverage_col = data_frames[df_key][cov_key]
        time_col = data_frames[df_key]['Time']

        # Collision data
        start_col = interval_dfs[df_key]["start-time"]
        end_col = interval_dfs[df_key]["end-time"]
        
        plt.plot(time_col, coverage_col, label=get_legend_name(df_key),linestyle=linestyle, color=color)

        ## Uncomment these lines to show crosses at the start and end of collision for each planner in the plot ##
        #crosses_start = np.interp(start_col, time_col, coverage_col)
        #crosses_end = np.interp(end_col, time_col, coverage_col)
        #plt.plot(start_col, crosses_start, 'x', color="red")        
        #plt.plot(start_col, crosses_end, 'x', color="green") 

        for j in range(len(start_col)):
            duration = np.linspace(start_col[j], end_col[j], 500)
            duration_interpol = np.interp(duration, time_col, coverage_col)
            if(paintOnce):
                plt.plot(duration, duration_interpol, color="orange", linewidth=5, label="Collision-segment")
                paintOnce = False
            else:
                plt.plot(duration, duration_interpol, color="orange", linewidth=5)


    ax.axhline(y=100, color='purple', linestyle="--")
    plt.title(f"World: {world} - Coverage [%]")
    plt.xlabel('Time [s]')
    plt.ylabel('Coverage [%]')
    plt.ylim(0, 110)
    plt.legend()
    # create the subdirectory if it does not exist
    if not os.path.exists(volume_p_path):
        os.makedirs(volume_p_path)
        
    # save the plot in the subdirectory
    plt.savefig(volume_p_path + f"/{world}")
    

# Save plot of coverage (%) for the best simulation of every planner and mode for a certain world
def plot_volumes(world, planner, data_frames):
    line_styles = ['-', '--', '-.', ':']
    colors = ['blue', 'red']
    #plt.figure()
    fig, ax = plt.subplots(figsize=(10,6)) # larger figure size

    for i, df_key in enumerate(data_frames):
        cov_m3_key = data_frames[df_key].columns[1]
        cov_free_key = data_frames[df_key].columns[3]
        cov_occupied_key = data_frames[df_key].columns[4]
        cov_unmapped_key = data_frames[df_key].columns[5]

        cov_m3_col = data_frames[df_key][cov_m3_key]
        cov_free_col = data_frames[df_key][cov_free_key]
        cov_occupied_col = data_frames[df_key][cov_occupied_key]
        cov_unmapped_col = data_frames[df_key][cov_unmapped_key]

        time_col = data_frames[df_key]['Time']

        plt.plot(time_col, cov_m3_col, label="Mapped", linestyle=line_styles[0], color=colors[i])
        plt.plot(time_col, cov_free_col, label="Free", linestyle=line_styles[1], color=colors[i])
        plt.plot(time_col, cov_occupied_col, label="Occupied", linestyle=line_styles[2], color=colors[i])
        plt.plot(time_col, cov_unmapped_col, label="Unmapped", linestyle=line_styles[3], color=colors[i])

    plt.title(f"World: {world} - Coverage [m3]")
    plt.xlabel('Time [s]')
    plt.ylabel('Coverage [m3]')

    # Shrink current axis by 20%
    box = ax.get_position()
    ax.set_position([box.x0, box.y0, box.width * 0.7, box.height])

    # Put a legend to the right of the current axis
    ax.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    #plt.legend()
    # create the subdirectory if it does not exist
    if not os.path.exists(volume_v_path):
        os.makedirs(volume_v_path)
        
    # save the plot in the subdirectory
    plt.savefig(volume_v_path + f"/{world}_{planner}")


def visualize_coverage_p(show):
    #for file in files:
    file = "coverage.csv"
    for world in worlds:
        best_dfs = {}
        interval_dfs = {}
        for mode in modes:
            for planner in planners:
                best_coverage = 0
                best_df = None
                best_simulation = 0
                for iteration in range(N):
                    file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{iteration}/{file}"
                    if os.path.isfile(file_path): 
                        #Check if file exists
                        df = pd.read_csv(file_path)

                        if best_df is None:
                            best_df = df
                            best_coverage = df[' Coverage (%)'].iloc[-1]
                            best_simulation = iteration

                        elif df[' Coverage (%)'].iloc[-1] > best_coverage:
                            best_coverage = df[' Coverage (%)'].iloc[-1]
                            best_df = df
                            best_simulation = iteration

                if best_df is not None:
                    best_simulation_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{best_simulation}/intervals.csv"
                    key = f"{mode}_{planner}"
                    best_dfs[key] = best_df
                    interval_dfs[key] = pd.read_csv(best_simulation_path)

        if len(best_dfs) != 0:
            plot_coverage_percent(world, best_dfs, interval_dfs)
    if show:
        plt.show()


def visualize_coverage_v(show):
    file = "coverage.csv"
    for world in worlds:
        for planner in planners:
            best_dfs = {}
            for mode in modes:
                best_coverage = 0
                best_df = None
                for iteration in range(N):
                    file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{iteration}/{file}"
                    if os.path.isfile(file_path): 
                        #Check if file exists
                        df = pd.read_csv(file_path)
                        if best_df is None:
                            best_df = df
                            best_coverage = df[' Coverage (%)'].iloc[-1]
                        
                        elif df[' Coverage (%)'].iloc[-1] > best_coverage:
                                best_coverage = df[' Coverage (%)'].iloc[-1]
                                best_df = df

                if best_df is not None:
                    file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}"
                    key = f"{mode}_{planner}"
                    best_dfs[key] = best_df
            
            if len(best_dfs) != 0:
                plot_volumes(world, planner, best_dfs) 
    if show:
        plt.show()


#################################### LOGFILE ###########################################################


def visualize_logfile():
    # Create data frame and corresponding headers
    logfile_headers = ["World", "Planner", "Mode", "Total Time (mean)", "Total Time (std)", "Path Length (mean)", "Path Length (std)", "Total Planning time (mean)", "Total Planning time (std)", 
                     "Total Flying time (mean)", "Total Flying time (std)"]
    logfile_df = pd.DataFrame(columns=logfile_headers)
    file = files[2]

    for world in worlds:
        for mode in modes:
            for planner in planners:
                
                total_times = []
                total_planning_times = []
                total_flying_times = []
                total_path_lengths = []
                iteration_exists = False

                for iteration in range(N):
                    file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{iteration}/{file}"
                    if os.path.isfile(file_path): #Check if file exists

                        iteration_exists = True
                        df = pd.read_csv(file_path)

                        if planner != "depr" and planner != "dep":
                            #Add the total planning and flyging time for each iteration
                            total_times.append(df[" Time"].iloc[-1])
                            total_path_lengths.append(df[" Path length"].iloc[-1])
                            total_planning_times.append(df[" Planning"].iloc[-1])
                            total_flying_times.append(df[" Flying"].iloc[-1])
                        else:
                            #If we use the DEP planner, Flying is unavailable.
                            total_times.append(df[" Time"].iloc[-1])
                            total_path_lengths.append(df[" Path length"].iloc[-1])
                            total_planning_times.append(df[" Planning"].iloc[-1])
                            total_flying_times.append(0)
                    
                # If a certain experiment does not exist, overwrite all metrics with zeros
                if not iteration_exists:
                    total_times.append(0)
                    total_path_lengths.append(0)
                    total_planning_times.append(0)
                    total_flying_times.append(0)
                    
                # Extract metrics after all iterations
                total_time_mean = np.round(np.mean(total_times), 2)
                total_time_std = np.round(np.std(total_times), 2)
                total_path_lengths_mean = np.round(np.mean(total_path_lengths), 2)
                total_path_lengths_std = np.round(np.std(total_path_lengths), 2)
                total_planning_times_mean = np.round(np.mean(total_planning_times), 2)
                total_planning_times_std = np.round(np.std(total_planning_times), 2)
                total_flying_times_mean = np.round(np.mean(total_flying_times), 2)
                total_flying_times_std = np.round(np.std(total_flying_times), 2)


                # Create a row and insert at the end of the data frame
                list_row = [world, planner, mode, total_time_mean, total_time_std, total_path_lengths_mean, total_path_lengths_std, total_planning_times_mean, 
                            total_planning_times_std, total_flying_times_mean, total_flying_times_std]
                logfile_df.loc[len(logfile_df)] = list_row
                    

    ### Sort the data frame if needed by uncommenting and modifying the line below ###
    #collision_df = collision_df.sort_values(by=[logfile_headers[3]], ascending=False)

    #Convert data frame to table
    table = tabulate(logfile_df, headers=logfile_headers, tablefmt="fancy_grid")

    # Print table if needed, otherwise look in logfile.txt
    #print(table)
                
    with open(logfile_path, 'w') as f:
        f.write(table)


def visualize_coverage():
    # Create data frame and corresponding headers
    headers = ["World", "Planner", "Mode", "Coverage (%) mean", "Coverage (%) std"]
    coverage_df = pd.DataFrame(columns=headers)
    file = "coverage.csv"

    for world in worlds:
        for mode in modes:
            for planner in planners:
                
                total_coverages = []
                iteration_exists = False

                for iteration in range(N):
                    file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{iteration}/{file}"
                    if os.path.isfile(file_path): #Check if file exists

                        iteration_exists = True
                        df = pd.read_csv(file_path)
                        total_coverages.append(df[" Coverage (%)"].iloc[-1])
                    
                # If a certain experiment does not exist, overwrite all metrics with zeros
                if not iteration_exists:
                    total_coverages.append(0)
                    
                # Extract metrics after all iterations
                total_coverages_mean = np.round(np.mean(total_coverages), 2)
                total_coverages_std = np.round(np.std(total_coverages), 2)

                # Create a row and insert at the end of the data frame
                list_row = [world, planner, mode, total_coverages_mean, total_coverages_std]
                coverage_df.loc[len(coverage_df)] = list_row
                    

    ### Sort the data frame if needed by uncommenting and modifying the line below ###
    #coverage_df = coverage_df.sort_values(by=[headers[3]], ascending=False)

    #Convert data frame to table
    table = tabulate(coverage_df, headers=headers, tablefmt="fancy_grid")

    # Print table if needed, otherwise look in logfile.txt
    #print(table)
                
    with open(coverage_path, 'w') as f:
        f.write(table)


#################################### COLLISION ###########################################################

def visualize_collision():

    # Create data frame and corresponding headers
    collision_headers = ["World", "Planner", "Mode", "Number of Collision (mean)", "Number of Collision (std)", 
                        "Total collision duration (mean)", "Total collision duration (std)", "Average Collision Time"]
    collision_df = pd.DataFrame(columns=collision_headers)
    file = files[0]

    for world in worlds:
        for mode in modes:
            for planner in planners:
                
                collisions = []
                total_duration_times = []
                iteration_exists = False

                for iteration in range(N):
                    file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{iteration}/{file}"
                    if os.path.isfile(file_path): #Check if file exists
                        
                        iteration_exists = True
                        df = pd.read_csv(file_path)

                        #Add the current number of collisions and time from each iteration to each list
                        collisions.append(df["nr_of_collisions"][0])
                        total_duration_times.append(df["Total duration"][0])
                    
                # If a certain experiment does not exist, overwrite all metrics with zeros
                if not iteration_exists:
                    collisions.append(0)
                    total_duration_times.append(0)
                    
                # Extract metrics after all iterations                
                collision_mean = np.mean(collisions)
                collisions_std = np.std(collisions)
                total_duration_mean = np.mean(total_duration_times)
                total_duration_std = np.std(total_duration_times)
                
                if total_duration_mean != 0:
                    average_collision_time = collision_mean/total_duration_mean
                    
                else:
                    average_collision_time = 0
                

                # Create a row and insert at the end of the data frame
                list_row = [world, planner, mode, collision_mean, collisions_std, total_duration_mean, total_duration_std, average_collision_time]
                collision_df.loc[len(collision_df)] = list_row
                    

    ### Sort the data frame if needed by uncommenting and modifying the line below ###
    #collision_df = collision_df.sort_values(by=[collision_headers[3]], ascending=False)

    #Convert data frame to table
    table = tabulate(collision_df, headers=collision_headers, tablefmt="fancy_grid")

    # Print table if needed, otherwise look in table.txt
    #print(table)
                
    with open(collision_path, 'w') as f:
        f.write(table)


#################################### Aggregated results ###########################################################

def visualize_agg_results():
    agg_headers = ["Planner", "Coverage mean", "Coverage std", "Time mean", "Time std", "Path length mean", \
                    "Path length std","Planning time mean","Planning time std", "Collision mean", "Collision std", \
                        "Collision duration mean", "Collision duration std"]
    agg_df = pd.DataFrame(columns=agg_headers)
    for planner in planners:
        collisions = []
        total_duration_times = []
        total_coverages = []
        total_times = []
        total_planning_times = []
        total_path_lengths = []
        for world in worlds:
            for mode in ["dynamic"]:
                for iteration in range(N):
                    # Coverage data
                    coverage_file = files[1]
                    coverage_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{iteration}/{coverage_file}"
                    if os.path.isfile(coverage_path): #Check if file exists  
                        df = pd.read_csv(coverage_path)
                        total_coverages.append(df[" Coverage (%)"].iloc[-1])
                        
                    # Logfile data
                    log_file = files[2]
                    log_file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{iteration}/{log_file}"
                    if os.path.isfile(log_file_path): #Check if file exists 
                        df = pd.read_csv(log_file_path)         
                        #Add the total planning and flyging time for each iteration
                        total_times.append(df[" Time"].iloc[-1])
                        total_path_lengths.append(df[" Path length"].iloc[-1])
                        total_planning_times.append(df[" Planning"].iloc[-1])

                    # Collision data
                    collision_file = files[0]
                    collision_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{iteration}/{collision_file}"
                    
                    if os.path.isfile(collision_path): #Check if file exists          
                        df = pd.read_csv(collision_path)
                        #Add the current number of collisions and time from each iteration to each list
                        collisions.append(df["nr_of_collisions"][0])
                        total_duration_times.append(df["Total duration"][0])

        if total_coverages:
            total_coverages_mean = np.round(np.mean(total_coverages), 2)
            total_coverages_std = np.round(np.std(total_coverages), 2)
        else:
            total_coverages_mean = 0
            total_coverages_std = 0

        if collisions:
            collision_mean = np.mean(collisions)
            collisions_std = np.std(collisions)
        else:
            collision_mean = 0
            collisions_std = 0

        if total_duration_times:
            total_duration_mean = np.mean(total_duration_times)
            total_duration_std = np.std(total_duration_times)
        else:
            total_duration_mean = 0
            total_duration_std = 0

        if total_times:
            total_time_mean = np.round(np.mean(total_times), 2)
            total_time_std = np.round(np.std(total_times), 2)
        else:
            total_time_mean = 0
            total_time_std = 0

        if total_path_lengths:
            total_path_lengths_mean = np.round(np.mean(total_path_lengths), 2)
            total_path_lengths_std = np.round(np.std(total_path_lengths), 2)
        else:
            total_path_lengths_mean = 0
            total_path_lengths_std = 0

        if total_planning_times:
            total_planning_times_mean = np.round(np.mean(total_planning_times), 2)
            total_planning_times_std = np.round(np.std(total_planning_times), 2)
        else:
            total_planning_times_mean = 0
            total_planning_times_std = 0

        
        # Create a row and insert at the end of the data frame
        list_row = [planner, total_coverages_mean, total_coverages_std, total_time_mean, total_time_std, total_path_lengths_mean, \
                    total_path_lengths_std,total_planning_times_mean,total_planning_times_std, collision_mean, collisions_std, \
                        total_duration_mean, total_duration_std]
        agg_df.loc[len(agg_df)] = list_row
        
    table = tabulate(agg_df, headers=agg_headers, tablefmt="fancy_grid")
    with open(agg_path, 'w') as f:
        f.write(table)
            
                
    



if __name__ == '__main__':
    if not os.path.exists(data_path):
        os.makedirs(data_path)

    visualize_coverage_avg(False)
    visualize_coverage_p(False)
    visualize_coverage_v(False)
    visualize_collision()
    visualize_logfile()
    visualize_coverage()
    visualize_agg_results()

