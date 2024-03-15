import mujoco
import mujoco_viewer
import numpy as np
import gen_worm as gw
import genotype as g
import glob
import json
import os
import sys
import shutil
import matplotlib.pyplot as plt


average_top_10_fitness_per_generation = []

num_gens = 3

generation = 2

for k in range(num_gens):
    
    
    
#Read the existing genotypes
    with open(f"gen{generation-1}.json", "r") as f:
        og_genotypes = json.load(f)

    mutated_genotypes = []
    #Iterate and mutate each original genotype
    for og_genotype_dict in og_genotypes:
        og_genotype = g.Genotype.from_dict(og_genotype_dict)
        #Create two mutations of each original genotype, one big one small :)
        for mutation_rate in [1, 2]:
            mutated_genotypes.append(og_genotype.mutate(mutation_rate))

    #Now, leets get to work with mutated genotypes
    last_num = 0
    for g_n in mutated_genotypes:
        #Determine the new filename
        if last_num == 0:  #If it's the first mutation, find the last used number
            with open(f"gen{generation-1}.json", "r") as f:
                data = json.load(f)
                if data:  #Ensure there's existing data to read from
                    last_filename = data[-1]["filename"]
                    last_num = int(last_filename[len("model"):-len(".xml")])
        new_num = last_num + 1
        new_filename = f"model{new_num}.xml"
        last_num = new_num  #Update last_num for the next iteration

        #Generate the XML content for the mutated genotype
        xml_str = gw.generate_mujoco_xml(g_n)

        # Save the XML file
        with open(f'gen_{generation-1}/model{new_num}.xml', "w") as file:
            file.write(xml_str)

        #Append the new genotype data, including the filename, to the JSON
        g_n_data = g_n.to_dict()
        g_n_data['filename'] = new_filename  #Update the filename in the dictionary

        with open(f"gen{generation-1}.json", "r") as f:
            existing_data = json.load(f)  #Read the existing data
        existing_data.append(g_n_data)  #Append the new data
        with open(f"gen{generation-1}.json", "w") as f:
            json.dump(existing_data, f, indent=4)  #Write back the updated data
    
    #Load the genotypes from the JSON file
    with open(f"gen{generation-1}.json", "r") as f:
        genotypes = json.load(f)

    model_files = glob.glob(f'gen_{generation-1}/*.xml')
    
    model_files.sort(key=lambda x: int(os.path.splitext(os.path.basename(x))[0].split('model')[1]))
    
    
    
    fitness = []
    speeds = []
    devs = []
    
    j = 0
    
    for model_file in model_files:
        
        print("Processing file:", model_file)
        
        # Extract the model index from the file name
        model_index = j
        j+=1
        #print(model_index)
        genotype_dict = genotypes[model_index]
        
        #Convert the dictionary to a Genotype object
        genotype = g.Genotype.from_dict(genotype_dict)
        
        #sanity check
        
        #print(f"Genotype for {model_file}: {genotype}")
        
        model = mujoco.MjModel.from_xml_path(model_file)
        data = mujoco.MjData(model)
        
        prev_time = data.time
        prev_position = np.array([data.qpos[0], data.qpos[1]])
        
        actuators = model.nu
        
        moves = genotype.moves
        
        data.ctrl[:actuators] = moves
        
        fitness_scores = []

        for i in range(1500):
            
            current_time = data.time
            current_position = np.array([data.qpos[0], data.qpos[1]])
            
            if i % 40 == 0:
                # Switch movement direction back and forth every 40 steps
                moves *= -1
            # Print the timestep
            # print(f"Time Step| {i}")

            data.ctrl[:actuators] = moves
            mujoco.mj_step(model, data)
            
            if current_time - prev_time >= 0.1:
                #Calculate displacement
                displacement = current_position - prev_position
                #Calculate speed
                speed = np.linalg.norm(displacement) / (current_time - prev_time)
                # Append the calculated speed to the fitness scores list
                fitness_scores.append(speed)
                # print(f"Current speed: {speed} units/sec")
                # print(f"Average speed: {np.mean(fitness_scores)} units/sec")
                #Update previous position and time
                prev_position = current_position
                prev_time = current_time
        
        dev_score = np.std(fitness_scores)
        mean_score = np.mean(fitness_scores)
        fit_score = 0.6 * mean_score - (dev_score)
        print(f"Average speed for model{j}: {mean_score}\nStandard Deviation of speed for model{j}: {dev_score}\nFITNESS SCORE: {fit_score}")
        fitness.append(fit_score)
        speeds.append(mean_score)
        devs.append(dev_score)


    top_indices = np.argsort(fitness)[::-1][:10]  #Get indices of top 25 models
    
    
    top_10_fitness = [fitness[i] for i in top_indices]
    average_fitness = sum(top_10_fitness) / len(top_10_fitness)
    average_top_10_fitness_per_generation.append(average_fitness)

    
    gen_file_path = f"gen{generation}.json"
    target_directory = f"gen_{generation}"
    os.makedirs(target_directory, exist_ok=True)
    
    last_num = -1
    if os.path.exists(gen_file_path) and os.path.getsize(gen_file_path) > 0:
        with open(gen_file_path, 'r') as file:
            data = json.load(file)
            if data:
                last_filename = data[-1]['filename']
                last_num = int(last_filename.split('model')[1].split('.xml')[0])
    
    new_genotypes = []
    for index in top_indices:
        best_model_file = model_files[index]
        new_num = last_num + 1
        new_filename = f"model{new_num}.xml"
        new_file_path = os.path.join(target_directory, new_filename)
        shutil.copy2(best_model_file, new_file_path)
        
        #Update genotype with new filename and prepare for JSON
        best_genotype = genotypes[index]
        best_genotype['filename'] = new_filename
        new_genotypes.append(best_genotype)
        
        last_num = new_num  #Update for the next iteration
    
    #Save new genotypes to JSON
    with open(gen_file_path, "w") as file:
        json.dump(new_genotypes, file, indent=4)
    
    print(f"Top 10 genotypes saved to {gen_file_path}.")
    

   
    # user_response = input("Would you like to simulate the best model? (yes/no): ")
    user_response = "no"

    if user_response.lower() == 'yes':
        
        
        best_model_genotype_dict = genotypes[max_index]
        
        best_model_genotype = g.Genotype.from_dict(best_model_genotype_dict)
        
        
        model = mujoco.MjModel.from_xml_path(best_model_file)
        data = mujoco.MjData(model)
        viewer = mujoco_viewer.MujocoViewer(model, data)
        actuators = model.nu
        moves = best_model_genotype.moves
        data.ctrl[:actuators] = moves
        fitness_scores = []
        
        p_time = data.time
        p_position = np.array([data.qpos[0], data.qpos[1]])
        
        print(f"model:{best_model_file}\ndata:{data}\nmoves:{moves}\ngenotype:{best_model_genotype}")
        
        
        for i in range(1000):
            
            curr_time = data.time
            curr_position = np.array([data.qpos[0], data.qpos[1]])
            
            if viewer.is_alive:
                if i % 40 == 0:
                    
                    moves *= -1

                data.ctrl[:actuators] = moves
                mujoco.mj_step(model, data)
                viewer.render()
                
                if curr_time - p_time >= 0.1:  
                #Calculate displacement
                    disp = curr_position - p_position
                    #Calculate speed
                    speed = np.linalg.norm(disp) / (curr_time - p_time)
                    #Append the calculated speed to the fitness scores list
                    fitness_scores.append(speed)
                    print(f"Current speed: {speed} units/sec")
                    print(f"Average speed: {np.mean(fitness_scores)} units/sec")
                    #Update previous position and time for the next calculation
                    p_position = curr_position
                    p_time = curr_time
            
            else:
                break
            
        viewer.close()
            
    else:
        print("Simulation ended.")
        
        
    generation+=1
    
plt.plot(range(1, num_gens + 1), average_top_10_fitness_per_generation, marker='o', linestyle='-', color='b')
plt.title('Average Fitness Score of Top 10 Individuals per Generation')
plt.xlabel('Generation Number')
plt.ylabel('Average Fitness Score')
plt.xticks(range(1, num_gens + 1)) 
plt.grid(True)
plt.show()