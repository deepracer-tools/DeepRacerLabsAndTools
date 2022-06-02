def reward_function(params):

    #flexible raceline guidance
    left_lane = [*range(20,30),*range(45,50)]#Fill in the waypoints 填入弯道航路点
    right_lane = [*range(63,72)]#Fill in the waypoints 填入弯道航路点
    
    #speed guidance
    high_speed_part = [*range(1,15),*range(55,85)] #Fill in the waypoints 填入高速赛段航路点
    #medium_speed_part = []

    #Read input parameters
    all_wheels_on_track = params["all_wheels_on_track"]
    #distance_from_center = params["distance_from_center"]
    #track_width = params["track_width"]
    speed = params["speed"]
    closest_waypoints = params["closest_waypoints"]
    is_left_of_center = params["is_left_of_center"]
    is_offtrack = params["is_offtrack"]
    progress = params["progress"]
    steps = params["steps"]  

    position = closest_waypoints[0]
    PROGRESS_STEPS_FACTOR = 2.5
    
    #设定高速阈值
    HighSpeed = 3
    #MediumSpeed = 2.5
    
    #Give a very low reward by default. 默认给予低奖励
    reward = 1e-3

    #Give a high reward if not all wheels go off the track and the agent is somewhere in between the track borders.
    #如果没有车轮在赛道外，即车子在赛道上，则给予奖励

    if position in left_lane and is_left_of_center and not all_wheels_on_track:
        reward = 1.0
    elif position in right_lane and not is_left_of_center:
        reward = 1.0
    
    #add speed reward
    if position in high_speed_part and speed > HighSpeed:
    #Give a high reward if speed up and all wheels on track
        reward  += speed/HighSpeed
    
    ####
    #add progress reward
    if is_offtrack:
        reward = 1e-3
    elif progress:
        reward += progress/steps * PROGRESS_STEPS_FACTOR
    
    
    return float(reward)
