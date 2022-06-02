import math

class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = -1
        self.verbose = verbose

    def reward_function(self, params):

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def cal_projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the April_Open
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[0.00307, -3.35292, 4.0, 0.07534],
                        [0.26724, -3.49795, 4.0, 0.07534],
                        [0.53142, -3.64297, 4.0, 0.07534],
                        [0.79559, -3.788, 4.0, 0.07534],
                        [1.05976, -3.93303, 4.0, 0.07534],
                        [1.32393, -4.07807, 4.0, 0.07534],
                        [1.5881, -4.22309, 4.0, 0.07534],
                        [1.85229, -4.3681, 4.0, 0.07534],
                        [2.11648, -4.51309, 4.0, 0.07534],
                        [2.38068, -4.65806, 4.0, 0.07534],
                        [2.64484, -4.80313, 4.0, 0.07534],
                        [2.90894, -4.94831, 4.0, 0.07534],
                        [3.17307, -5.09342, 4.0, 0.07534],
                        [3.43718, -5.23859, 4.0, 0.07534],
                        [3.7015, -5.38128, 3.9197, 0.07663],
                        [3.96635, -5.51901, 3.65731, 0.08162],
                        [4.23187, -5.64937, 3.43864, 0.08602],
                        [4.49809, -5.77005, 3.25273, 0.08986],
                        [4.76492, -5.87893, 3.08116, 0.09353],
                        [5.03214, -5.97411, 2.93161, 0.09676],
                        [5.29942, -6.05383, 2.79604, 0.09975],
                        [5.56631, -6.11656, 2.6722, 0.1026],
                        [5.83227, -6.16088, 2.6722, 0.1009],
                        [6.09663, -6.1855, 2.6722, 0.09936],
                        [6.35857, -6.18905, 2.6722, 0.09803],
                        [6.61709, -6.17011, 2.6722, 0.097],
                        [6.87093, -6.12719, 2.6722, 0.09634],
                        [7.11844, -6.05858, 2.79672, 0.09184],
                        [7.359, -5.96684, 2.81752, 0.09138],
                        [7.59154, -5.8524, 2.83822, 0.09132],
                        [7.81492, -5.71552, 2.85579, 0.09174],
                        [8.02778, -5.55631, 2.85694, 0.09304],
                        [8.22854, -5.37478, 2.84321, 0.0952],
                        [8.41518, -5.17083, 2.82657, 0.09781],
                        [8.58524, -4.94459, 2.80827, 0.10078],
                        [8.73566, -4.69676, 2.79008, 0.10391],
                        [8.86276, -4.4294, 2.77016, 0.10686],
                        [8.96241, -4.14734, 2.74455, 0.109],
                        [9.03135, -3.85836, 2.71887, 0.10927],
                        [9.06881, -3.56969, 2.69095, 0.10817],
                        [9.07587, -3.28599, 2.65801, 0.10676],
                        [9.05432, -3.01023, 2.6197, 0.10559],
                        [9.00601, -2.74439, 2.57835, 0.10479],
                        [8.93253, -2.49001, 2.57835, 0.10269],
                        [8.83525, -2.24837, 2.57835, 0.10103],
                        [8.71528, -2.02068, 2.57835, 0.09982],
                        [8.57347, -1.80824, 2.57835, 0.09907],
                        [8.41038, -1.61256, 2.57835, 0.0988],
                        [8.22643, -1.43549, 2.71487, 0.09405],
                        [8.02449, -1.27647, 2.84254, 0.09043],
                        [7.80665, -1.13509, 2.98089, 0.08712],
                        [7.57471, -1.01088, 3.13877, 0.08382],
                        [7.33029, -0.90326, 3.32254, 0.08038],
                        [7.07492, -0.81145, 3.53799, 0.0767],
                        [6.81005, -0.73452, 3.80118, 0.07256],
                        [6.53709, -0.67127, 4.0, 0.07005],
                        [6.25743, -0.62025, 4.0, 0.07107],
                        [5.97241, -0.57975, 4.0, 0.07197],
                        [5.68332, -0.54781, 4.0, 0.07271],
                        [5.39138, -0.52232, 4.0, 0.07326],
                        [5.09571, -0.50087, 4.0, 0.07411],
                        [4.80098, -0.47386, 4.0, 0.07399],
                        [4.50748, -0.44014, 4.0, 0.07386],
                        [4.21556, -0.39852, 4.0, 0.07372],
                        [3.92561, -0.3479, 4.0, 0.07358],
                        [3.63813, -0.28729, 4.0, 0.07345],
                        [3.35368, -0.21579, 3.9713, 0.07386],
                        [3.07295, -0.13252, 3.94336, 0.07426],
                        [2.79673, -0.03683, 3.94066, 0.07418],
                        [2.52583, 0.07172, 3.94066, 0.07406],
                        [2.26118, 0.19348, 3.94066, 0.07393],
                        [2.00363, 0.32847, 3.94066, 0.07379],
                        [1.754, 0.47655, 3.94066, 0.07365],
                        [1.51305, 0.6374, 3.94066, 0.07352],
                        [1.2814, 0.81057, 3.95561, 0.07312],
                        [1.0595, 0.99545, 4.0, 0.07221],
                        [0.84762, 1.19131, 4.0, 0.07213],
                        [0.64583, 1.39735, 4.0, 0.0721],
                        [0.45413, 1.61282, 4.0, 0.0721],
                        [0.27236, 1.83693, 4.0, 0.07214],
                        [0.10021, 2.06892, 4.0, 0.07222],
                        [-0.06274, 2.308, 4.0, 0.07233],
                        [-0.21692, 2.55351, 4.0, 0.07248],
                        [-0.36288, 2.80474, 4.0, 0.07264],
                        [-0.50116, 3.06108, 4.0, 0.07282],
                        [-0.63237, 3.32195, 4.0, 0.073],
                        [-0.75711, 3.58678, 4.0, 0.07318],
                        [-0.86608, 3.83214, 3.77984, 0.07103],
                        [-0.97788, 4.06918, 3.57507, 0.07331],
                        [-1.09383, 4.29812, 3.3885, 0.07573],
                        [-1.21523, 4.51925, 3.22257, 0.07828],
                        [-1.34338, 4.73276, 3.076, 0.08095],
                        [-1.4795, 4.93857, 3.076, 0.08022],
                        [-1.62475, 5.13632, 3.076, 0.07977],
                        [-1.78022, 5.32535, 3.076, 0.07957],
                        [-1.94701, 5.50468, 3.076, 0.07961],
                        [-2.12622, 5.67301, 3.076, 0.07993],
                        [-2.31899, 5.82876, 3.1956, 0.07755],
                        [-2.52426, 5.97219, 3.26434, 0.07671],
                        [-2.74148, 6.10308, 3.33003, 0.07616],
                        [-2.97018, 6.22113, 3.39408, 0.07583],
                        [-3.20992, 6.32594, 3.46026, 0.07562],
                        [-3.46025, 6.41708, 3.52617, 0.07555],
                        [-3.72068, 6.49405, 3.59278, 0.07559],
                        [-3.99065, 6.55631, 3.66318, 0.07563],
                        [-4.2695, 6.60334, 3.72541, 0.07591],
                        [-4.55643, 6.63453, 3.78539, 0.07625],
                        [-4.85036, 6.64928, 3.68265, 0.07991],
                        [-5.14942, 6.64713, 3.58075, 0.08352],
                        [-5.44651, 6.62832, 3.48294, 0.08547],
                        [-5.73769, 6.59395, 3.3886, 0.08653],
                        [-6.02223, 6.54451, 3.30048, 0.0875],
                        [-6.29954, 6.47986, 3.21263, 0.08864],
                        [-6.56908, 6.39982, 3.13831, 0.08959],
                        [-6.83025, 6.3042, 3.07896, 0.09033],
                        [-7.08244, 6.19273, 3.03135, 0.09096],
                        [-7.3249, 6.0651, 2.99093, 0.09161],
                        [-7.55677, 5.92092, 2.96393, 0.09212],
                        [-7.777, 5.75973, 2.93779, 0.0929],
                        [-7.98431, 5.58114, 2.9237, 0.09359],
                        [-8.17721, 5.38497, 2.90828, 0.0946],
                        [-8.35389, 5.1714, 2.89535, 0.09573],
                        [-8.5122, 4.94111, 2.88507, 0.09686],
                        [-8.64983, 4.69568, 2.87148, 0.09799],
                        [-8.7644, 4.43753, 2.84706, 0.0992],
                        [-8.85406, 4.17001, 2.82299, 0.09995],
                        [-8.91754, 3.89686, 2.79252, 0.10042],
                        [-8.95442, 3.6218, 2.75932, 0.10057],
                        [-8.965, 3.34808, 2.72053, 0.10069],
                        [-8.94997, 3.0783, 2.72053, 0.09932],
                        [-8.91002, 2.81449, 2.72053, 0.09807],
                        [-8.84582, 2.55826, 2.72053, 0.09709],
                        [-8.75781, 2.31098, 2.72053, 0.09648],
                        [-8.64612, 2.07397, 2.72053, 0.09631],
                        [-8.51055, 1.84866, 3.03166, 0.08674],
                        [-8.35585, 1.63367, 3.18006, 0.08329],
                        [-8.18366, 1.4288, 3.35055, 0.07987],
                        [-7.99549, 1.23376, 3.54307, 0.07649],
                        [-7.79273, 1.04817, 3.7724, 0.07287],
                        [-7.57679, 0.87153, 4.0, 0.06974],
                        [-7.34912, 0.70321, 4.0, 0.07078],
                        [-7.1112, 0.54242, 4.0, 0.07179],
                        [-6.86456, 0.38819, 4.0, 0.07272],
                        [-6.61079, 0.23936, 4.0, 0.07355],
                        [-6.35149, 0.0946, 4.0, 0.07424],
                        [-6.08823, -0.04757, 4.0, 0.0748],
                        [-5.8222, -0.18894, 4.0, 0.07531],
                        [-5.55632, -0.33058, 4.0, 0.07531],
                        [-5.29056, -0.47246, 4.0, 0.07532],
                        [-5.02492, -0.61459, 4.0, 0.07532],
                        [-4.75939, -0.75694, 4.0, 0.07532],
                        [-4.49398, -0.89951, 4.0, 0.07532],
                        [-4.22868, -1.04232, 4.0, 0.07532],
                        [-3.9635, -1.18536, 4.0, 0.07532],
                        [-3.69845, -1.32863, 4.0, 0.07533],
                        [-3.43351, -1.47215, 4.0, 0.07533],
                        [-3.16865, -1.61582, 4.0, 0.07533],
                        [-2.90391, -1.75973, 4.0, 0.07533],
                        [-2.63929, -1.90388, 4.0, 0.07533],
                        [-2.37479, -2.04826, 4.0, 0.07534],
                        [-2.11041, -2.19287, 4.0, 0.07534],
                        [-1.84615, -2.33773, 4.0, 0.07534],
                        [-1.58198, -2.48276, 4.0, 0.07534],
                        [-1.3178, -2.62779, 4.0, 0.07534],
                        [-1.05363, -2.77282, 4.0, 0.07534],
                        [-0.78946, -2.91785, 4.0, 0.07534],
                        [-0.52528, -3.06287, 4.0, 0.07534],
                        [-0.26111, -3.2079, 4.0, 0.07534]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 2
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 11 # seconds (time that is easily done by model)
        FASTEST_TIME = 7.6  # seconds (best time of 1st place on the track)
        times_list = [row[3] for row in racing_track]
        projected_time = cal_projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 15:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
