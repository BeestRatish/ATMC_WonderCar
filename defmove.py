def move():
    global car_stop
    global __isRunning
    global detect_color
    global stop_timer

    stop_timer = 0

    while True:
        if __isRunning:
            if detect_color != 'red':
                set_rgb(detect_color)  # Set LED color based on detected color
                sensor_data = line.readData()  # Read data from 4 infrared sensors

                # Line following logic based on sensor readings
                if not sensor_data[0] and sensor_data[1] and sensor_data[2] and not sensor_data[3]:
                    car.set_velocity(35, 90, 0)  # Move forward
                    car_stop = True
                elif not sensor_data[0] and not sensor_data[1] and sensor_data[2] and not sensor_data[3]:
                    car.set_velocity(35, 90, 0.03)  # Small right turn
                    car_stop = True
                elif not sensor_data[0] and sensor_data[1] and not sensor_data[2] and not sensor_data[3]:
                    car.set_velocity(35, 90, -0.03)  # Small left turn
                    car_stop = True
                elif not sensor_data[0] and not sensor_data[1] and not sensor_data[2] and sensor_data[3]:
                    car.set_velocity(35, 90, 0.3)  # Big right turn
                    car_stop = True
                elif sensor_data[0] and not sensor_data[1] and not sensor_data[2] and not sensor_data[3]:
                    car.set_velocity(35, 90, -0.3)  # Big left turn

                # Slow down if a cross line is detected
                elif sensor_data[0] and sensor_data[1] and sensor_data[2] and sensor_data[3]:
                    if not car_stop:
                        # Slow down the car gradually
                        for vel in range(35, 5, -1):
                            car.set_velocity(vel, 90, 0)
                            time.sleep(0.1)
                        car_stop = True
                        stop_timer = 0.5

                    if stop_timer > 0:
                        stop_timer -= 0.01
                    elif car_stop:
                        car_stop = False
                        car.set_velocity(35, 90, 0)

                if detect_color == 'green':  # Detected green color
                    if not car_stop:
                        car.set_velocity(30, 90, 0)  # Move forward
                        car_stop = True

            else:  # Detected red color
                if car_stop:
                    setBuzzer(0.1)  # Sound buzzer for 0.1 seconds
                    set_rgb(detect_color)  # Set LED color to red
                    car.set_velocity(0, 90, 0)  # Stop the robot
                    time.sleep(3)  # Wait for 3 seconds
                    car_stop = False
                time.sleep(0.01)

        else:
            if car_stop:
                car.set_velocity(0, 90, 0)  # Stop the robot
                time.sleep(3)  # Wait for 3 seconds
                car_stop = False
            time.sleep(0.01)
