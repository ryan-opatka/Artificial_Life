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


num_worms = 50

#Generate 100 random worms
gw.generate_generation(num_worms)

#Load the genotypes from the JSON file
with open("genotypes.json", "r") as f:
    genotypes = json.load(f)


#Specify "pattern" for files to simulate. Using glob
model_files = glob.glob('models/*.xml')
model_files.sort(key=lambda x: int(os.path.splitext(os.path.basename(x))[0].split('model')[1]))

fitness = []
speeds = []
devs = []

j = 0
for model_file in model_files:
    
    print("Processing file:", model_file)
    
    #Extract the model index from the file name
    model_index = j
    j+=1
    print(model_index)
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

    for i in range(1000):
        
        current_time = data.time
        current_position = np.array([data.qpos[0], data.qpos[1]])
        
        if i % 40 == 0:
            moves *= -1

        data.ctrl[:actuators] = moves
        mujoco.mj_step(model, data)
        
        if current_time - prev_time >= 0.1:
            #Calculate displacement
            displacement = current_position - prev_position
            #Calculate speed
            speed = np.linalg.norm(displacement) / (current_time - prev_time)
            #Append the calculated speed to the fitness scores list
            fitness_scores.append(speed)
            # print(f"Current speed: {speed} units/sec")
            # print(f"Average speed: {np.mean(fitness_scores)} units/sec")
            prev_position = current_position
            prev_time = current_time
    
    dev_score = np.std(fitness_scores)
    mean_score = np.mean(fitness_scores)
    fit_score = 0.6 * mean_score - (dev_score)
    print(f"Average speed for model{j}: {mean_score}\nStandard Deviation of speed for model{j}: {dev_score}\nFITNESS SCORE: {fit_score}")
    fitness.append(fit_score)
    speeds.append(mean_score)
    devs.append(dev_score)


scores = np.array(fitness)

max_value = np.max(scores)

max_index = np.argmax(scores)

speed_score = speeds[max_index]

deviation = devs[max_index]


print(f"Max fitness score was model {max_index} with a fitness score of {max_value} and average speed of {speed_score} and standard deviation in speeds of {deviation}")
    


best_model_file = model_files[max_index]

gen1_file_path = "gen1.json"
target_directory = "gen_1"

if os.path.exists(gen1_file_path):
    try:
        with open(gen1_file_path, 'r') as file:
            data = json.load(file)
            last_element = data[-1]       
            m_filename = last_element["filename"]
            num_part = m_filename[len("model"):-len(".xml")]
            num = int(num_part)
            
            os.makedirs(target_directory, exist_ok=True)
            
            new_filename = f"model{num+1}.xml"
            
            new_file_path = os.path.join(target_directory, new_filename)
            
            shutil.copy2(best_model_file, new_file_path)
            

            
    except FileNotFoundError:
        print(f"The file {gen1_file_path} does not exist.")
    except json.JSONDecodeError:
        print("Error decoding JSON")
else:
    os.makedirs(target_directory, exist_ok=True)
    
    new_filename = "model0.xml"

    new_file_path = os.path.join(target_directory, new_filename)
            
    shutil.copy2(best_model_file, new_file_path)







best_genotype = genotypes[max_index]

best_genotype['filename'] = new_filename

if os.path.exists(gen1_file_path):
    with open(gen1_file_path, "r") as file:
        existing_genotypes = json.load(file)
else:
    existing_genotypes = []
    
existing_genotypes.append(best_genotype)
with open(gen1_file_path, "w") as file:
    json.dump(existing_genotypes, file, indent=4)
    
print(f"Best genotype saved to {gen1_file_path}.")


#Ask the user if they want to simulate the best model
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
    
    for i in range(10000):
        
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
                p_position = curr_position
                p_time = curr_time
        
        else:
            break
        
    viewer.close()
        
else:
    print("Simulation ended.")
    
    

#Run simulation on all of the models generated, and evealuate fitness


# for i in range(1000):
    
    
    
