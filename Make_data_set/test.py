

iterate = 0
move_distance=20

for delta_x in range(-move_distance, move_distance+1, 10):
    for delta_y in range(-(move_distance-abs(delta_x)), (move_distance-abs(delta_x))+1, 10):
        for delta_z in range(-(move_distance-abs(delta_x)-abs(delta_y)), (move_distance-abs(delta_x)-abs(delta_y))+1, 10):
            print(delta_x,delta_y,delta_z)
            iterate += 1

print iterate