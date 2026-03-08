from Odysseus import *
import time
ody = odysseus('COM10')
print("starting...")
number_of_obstacles = 0
turning_time = 0
object_start = -1
obstacle_time = -1
total_moving_time = 0
total_obstacle_time = 0
total_forward_time = 0
current_move = "forward"
start_forward_time = time.time()
while number_of_obstacles <= 5:
    if ody.ultrasonic_distance() >=30:
        if current_move == "turn":
            obstacle_time = time.time() - object_start
            print("number of obstacle : ", number_of_obstacles)
            print("duration of obstacle : ", obstacle_time)
            total_obstacle_time = total_obstacle_time+obstacle_time
            ody.print("forward")
            current_move = "forward"
            start_forward_time = time.time()
    else:
        if current_move == "forward":
            forward_time = time.time()-start_forward_time
            total_forward_time = total_forward_time+forward_time
            print("χρόνος κίνησης : ", forward_time)
            current_move = "turn"
            object_start = time.time()
            ody.print("left")
            number_of_obstacles = number_of_obstacles+1;
        else:
            if time.time()-object_start >5:
                ody.print("odysseus")
                total_obstacle_time = total_obstacle_time+5
                break;
    time.sleep(0.1)
print("συνολικός χρόνος κίνησης ", total_forward_time)
print("συνολικός χρόνος στασης ", total_obstacle_time)
ody.print("odysseus")
   # print(current_move)



