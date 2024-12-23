import numpy as np
from . import kinematics
import shapely
import math
from threeD.environment import Environment
class BuildingBlocks3D(object):
    '''
    @param resolution determines the resolution of the local planner(how many intermidiate configurations to check)
    @param p_bias determines the probability of the sample function to return the goal configuration
    '''

    def __init__(self, transform: kinematics.Transform, ur_params: kinematics.UR5e_PARAMS, env: Environment, resolution=0.1, p_bias=0.05):
        self.transform = transform
        self.ur_params = ur_params
        self.env = env
        self.resolution = resolution
        self.p_bias = p_bias
        self.cost_weights = np.array([0.4, 0.3, 0.2, 0.1, 0.07, 0.05])

        self.single_mechanical_limit = list(self.ur_params.mechamical_limits.values())[-1][-1]

        # pairs of links that can collide during sampling
        self.possible_link_collisions = [['shoulder_link', 'forearm_link'],
                                         ['shoulder_link', 'wrist_1_link'],
                                         ['shoulder_link', 'wrist_2_link'],
                                         ['shoulder_link', 'wrist_3_link'],
                                         ['upper_arm_link', 'wrist_1_link'],
                                         ['upper_arm_link', 'wrist_2_link'],
                                         ['upper_arm_link', 'wrist_3_link'],
                                         ['forearm_link', 'wrist_2_link'],
                                         ['forearm_link', 'wrist_3_link']]

    def sample(self, goal_conf) -> np.array:
        """
        sample random configuration
        @param goal_conf - the goal configuration
        """
        # TODO: HW2 5.2.1
        to_bias = np.random.uniform(low = 0, high = 1)
        if to_bias <= self.p_bias:
            return goal_conf
        config = []
        for joint_limit in self.ur_params.mechamical_limits.values():
            config.append(np.random.uniform(low = joint_limit[0], high = joint_limit[1]))
        return config
    
    
    
    def is_in_collision(self, conf) -> bool:
        """check for collision in given configuration, arm-arm and arm-obstacle
        return True if in collision
        @param conf - some configuration
        """
        # TODO: HW2 5.2.2
        coords = self.transform.conf2sphere_coords(conf)
        radii = self.ur_params.sphere_radius
        for collision in self.possible_link_collisions:
            joint1 = collision[0]
            joint2 = collision[1]
            rad_sum = radii[joint1] +  radii[joint2]
            joint1_centers = coords[joint1]
            joint2_centers = coords[joint2]
            for i in range(len(joint1_centers)):
                for j in range(i+1,len(joint2_centers)):
                    center1 = joint1_centers[i]
                    center2 = joint2_centers[j]
                    if center1[0] !=0 and center1[1] != 0 and np.abs(center1[2]) < radii[joint1]:
                        print(f'{joint1} hit floor at {center1}, dist should be more than {radii[joint1]}')
                        return True
                    if center2[0] !=0 and center2[1] != 0 and np.abs(center2[2]) < radii[joint2]:
                        print(f'{joint2} hit floor at {center2}, dist should be more than {radii[joint2]}')
                        return True
                    actual_distance = np.sqrt(np.sum((center1 - center2)**2))
                    if  actual_distance < rad_sum:
                        print(f'{joint1} is {joint1_centers} and {joint2} is {joint2_centers}')
                        print(f'center 1 is {center1} , center2 is {center2} , distance is {actual_distance} , should be more than {rad_sum}')
                        return True
                    obstacle_rad = self.env.radius
                    for obstacle in self.env.obstacles:
                        obstacle_dist1 = np.sqrt(np.sum((center1 - obstacle)**2))
                        obstacle_dist2 = np.sqrt(np.sum((center2 - obstacle)**2))
                        rad_sum1 = radii[joint1] + obstacle_rad
                        rad_sum2 = radii[joint2] + obstacle_rad
                        if  obstacle_dist1 < rad_sum1:
                            print(f'{joint1} is {joint1_centers} ')
                            print(f'center 1 is {center1} , obstacle is {obstacle} , distance is {obstacle_dist1} , should be more than {rad_sum1}')
                            return True
                        if  obstacle_dist2 < rad_sum2:
                            print(f'{joint2} is {joint2_centers} ')
                            print(f'center 2 is {center2} , obstacle is {obstacle} , distance is {obstacle_dist2} , should be more than {rad_sum2}')
                            return True        
        return False
    
    
    
    def local_planner(self, prev_conf, current_conf) -> bool:
        '''check for collisions between two configurations - return True if trasition is valid
        @param prev_conf - some configuration
        @param current_conf - current configuration
        '''
        # TODO: HW2 5.2.4
        pass


    def edge_cost(self, conf1, conf2):
        '''
        Returns the Edge cost- the cost of transition from configuration 1 to configuration 2
        @param conf1 - configuration 1
        @param conf2 - configuration 2
        '''
        return np.dot(self.cost_weights, np.power(conf1 - conf2, 2)) ** 0.5
