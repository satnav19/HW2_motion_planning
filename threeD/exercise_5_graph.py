
import numpy as np
from environment import Environment
from kinematics import UR5e_PARAMS, Transform
from building_blocks import BuildingBlocks3D
import time
import matplotlib.pyplot as plt

def main():
    inflation_factors = np.linspace(1.0, 1.8, 9)
    times = []
    is_collision_instances_false_negative = []
    is_collision_instances_false_positive = []

    # get ground truth collisions (inflation_factor = 1.0)
    ur_params = UR5e_PARAMS(inflation_factor=inflation_factors[0]) 
    env = Environment(env_idx=0)
    transform = Transform(ur_params)
    bb = BuildingBlocks3D(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.03)
    random_samples = np.load('C:\\Users\\user\\source\\repos\\HW2_motion_planning\\threeD\\random_samples_100k.npy')

    # Get ground truth collisions
    ground_truth_collisions = []
    start_time = time.time()
    for sample in random_samples:
        ground_truth_collisions.append(bb.is_in_collision(sample))
    end_time = time.time()
    times.append(end_time - start_time)
    is_collision_instances_false_negative.append(0)  # Ground truth has 0 false negatives
    is_collision_instances_false_positive.append(0)  # Ground truth has 0 false positives

    for inflation_factor in inflation_factors[1:]: 
        ur_params = UR5e_PARAMS(inflation_factor=inflation_factor)
        env = Environment(env_idx=0)
        transform = Transform(ur_params)
        bb = BuildingBlocks3D(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.03)
        
        false_negatives = 0
        false_positives = 0
        start_time = time.time()
        
        for j, sample in enumerate(random_samples):
            current_collision = bb.is_in_collision(sample)
            if ground_truth_collisions[j] and not current_collision:
                false_negatives += 1
            elif current_collision and not ground_truth_collisions[j]:
                false_positives += 1
                
        end_time = time.time()
        times.append(end_time - start_time)
        is_collision_instances_false_negative.append(false_negatives)
        is_collision_instances_false_positive.append(false_positives)



    fig1 = plt.figure()
    ax1 = fig1.add_subplot()
    ax1.set_xlabel('min radii factor')
    ax2 = ax1.twinx()
    ax1.set_ylabel('time (s)', color='blue')
    ax2.set_ylabel('False Negative Instances', color='red') 
    ax1.scatter(inflation_factors, times, c='blue')
    ax2.scatter(inflation_factors, is_collision_instances_false_negative, c='red')
    ax1.tick_params(axis='y', labelcolor='blue')
    ax2.tick_params(axis='y', labelcolor='red')
    fig1.tight_layout()
    plt.savefig('inflation_factor_analysis_false_negative.png')
    plt.show()

    
    fig2 = plt.figure()
    ax1 = fig2.add_subplot()
    ax1.set_xlabel('min radii factor')
    ax2 = ax1.twinx()
    ax1.set_ylabel('time (s)', color='blue')
    ax2.set_ylabel('False positive Instances', color='red') 
    ax1.scatter(inflation_factors, times, c='blue')
    ax2.scatter(inflation_factors, is_collision_instances_false_positive, c='red')
    ax1.tick_params(axis='y', labelcolor='blue')
    ax2.tick_params(axis='y', labelcolor='red')
    fig2.tight_layout()
    plt.savefig('inflation_factor_analysis_false_positive.png')
    plt.show()


if __name__ == '__main__':
    main()



