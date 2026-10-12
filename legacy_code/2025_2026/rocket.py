import time
from multiprocessing import Process, Pipe
import json
import numpy as np
import csv
import os
import math

# Global Variables
filename = "flight_log.csv"
log_buffer = []
buffer_size = 25
columns = [
    "t_from_launch", "dt", 
    "a_x", "a_y", "a_z",
    "g_x", "g_y", "g_z",
    "m_x", "m_y", "m_z",
    "temp", "pressure",
    "theta", "fused roll"
]

# Logging Functions
def log_data(row):
    global log_buffer

    log_buffer.append(row)

    # Check for max buffer size to flush to CSV
    if len(log_buffer) >= buffer_size:
        flush_buffer_to_csv(log_buffer)

def flush_buffer_to_csv(log_buffer):
    # Check if file exists to determine if we need to write headers
    file_exists = os.path.isfile(filename)
    with open(filename, "a", newline="") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(columns)

        # Write/append 250 rows from the buffer to the CSV file
        writer.writerows(log_buffer)
    
    # Clear the buffer after flushing to free up memory
    log_buffer.clear()

# Subprocess functions
def sensor_read_process(pipe):
    import adafruit_bno055 as BNO
    from modules import bmpfuncs
    from modules import fusion
    from gpiozero import OutputDevice
    from gpiozero import AngularServo
    from gpiozero.pins.rpigpio import RPiGPIOFactory
    from smbus2 import SMBus
    from adafruit_extended_bus import ExtendedI2C as I2C

    # Register Addresses
    BARO_ADDR   = 0x76
    PRESS_ADDR  = 0x04
    CHIP_ID     = 0x00
    ERR_REG     = 0x02
    STATUS      = 0x03
    EVENT       = 0x10
    INT_CTRL    = 0x19
    PWR_CTRL    = 0x1B
    OSR         = 0x1C
    ODR         = 0x1D
    CMD         = 0x7E

    # Register Values(s)
    SOFT_RESET = 0xB6
    PWR_SLEEP_EN = 0x03
    PWR_FORCED_EN = 0x13
    PWR_NORMAL_EN = 0x33
    INT_CTRL_DRDY = 0x40
    OSR_VAL = 0b00001001
    ODR_VAL = 0x01
    
    # Phases (treat as enum vals)
    BOOST = 20
    BREAKING = 30
    RECOVERY = 40

    i2c = I2C(8)
    bno_sensor = BNO.BNO055_I2C(i2c, 0x28)

    time.sleep(0.5)

    bno_sensor.mode = BNO.CONFIG_MODE
    time.sleep(0.05)
    bno_sensor.set_normal_mode()
    time.sleep(0.05)

    # Sensor configuration
    bno_sensor.accel_bandwidth = BNO.ACCEL_125HZ
    bno_sensor.gyro_bandwidth = BNO.GYRO_116HZ
    bno_sensor.mag_bandwidth = BNO.MAGNET_30HZ
    bno_sensor.accel_range = BNO.ACCEL_16G
    time.sleep(0.05)

    bno_sensor.mode = BNO.AMG_MODE

    time.sleep(1)

    # Grab compensation data for gyro and accel and mag
    # You will need to recalibrate for your new sensors
    with open("gyro_calib.json", "r") as f:
        calibration_data = json.load(f)
    b = np.array(calibration_data["bias"])

    with open("accel_calib.json", "r") as f:
        accel_data = json.load(f)

    with open("mag_calibration.json", "r") as f:
        mag_data = json.load(f)
    
    hi_offset = mag_data["hard_iron_offset"]
    si_scale = mag_data["soft_iron_scale"]
    
    acc_b = np.array(accel_data["bias"])
    M = np.array(accel_data["M"])

    # Extract Gyro Biases as independent floats
    # Same with any bias data you will need to recollect for your new sensors
    bg_x, bg_y, bg_z = float(b[0]), float(b[1]), float(b[2])

    # Extract Accel Biases as independent floats
    ba_x, ba_y, ba_z = float(acc_b[0]), float(acc_b[1]), float(acc_b[2])

    # Unpack Scale Factor/Cross-Axis Matrix M
    # M is a 3x3 matrix. We pull out each coefficient explicitly
    M00, M01, M02 = float(M[0,0]), float(M[0,1]), float(M[0,2])
    M10, M11, M12 = float(M[1,0]), float(M[1,1]), float(M[1,2])
    M20, M21, M22 = float(M[2,0]), float(M[2,1]), float(M[2,2])

    # These variables used a library to access certain GPIO pins and trigger then high or low
    # in order to release parachutes, the servo is just access to a pin connected to a servo
    # in order to rotate it these values will need to be changed in your implementation
    servo = AngularServo(21, min_angle=-0.5, max_angle=2.5)
    mosfet_drogue = OutputDevice("BOARD22", pin_factory=RPiGPIOFactory())
    mosfet_main = OutputDevice("BOARD11", pin_factory=RPiGPIOFactory())

    # In order to increase the output of our barometer I needed to use direct
    # register read and writes in order to change the specific bit values.
    # Ideally your barometer will not have this issue, MAKE SURE TO RESEARCH THOROUGHLY :)
    with SMBus(8) as bus:
        bus.write_byte_data(BARO_ADDR, CMD, SOFT_RESET)
        time.sleep(0.02)

        # Put the baro in sleep mode
        bus.write_byte_data(BARO_ADDR, PWR_CTRL, PWR_SLEEP_EN)

        # Configure our interrupt, sampling, and rate while in sleep mode
        bus.write_byte_data(BARO_ADDR, INT_CTRL, INT_CTRL_DRDY)
        bus.write_byte_data(BARO_ADDR, OSR, OSR_VAL)
        bus.write_byte_data(BARO_ADDR, ODR, ODR_VAL)

        # Enter normal mode for barometer
        bus.write_byte_data(BARO_ADDR, PWR_CTRL, PWR_NORMAL_EN)
        
        raw = bmpfuncs.read_nvm_coefs(bus, BARO_ADDR)
        data = bmpfuncs.convert_nvm_coefs(raw)

        # Variables for loop
        phase = BOOST
        test_z_acc = [0.0] * 50
        launch_times = [0.0] * 50
        t_idx = 0
        i = 0
        min_acc_criteria = 35.0
        t0 = time.perf_counter()
        t_prev = t0
        samples = 0
        
        # Pre Launch Output and Pedal Actuation
        # This was to verify before the actual launch
        print("Beginning Pedal Actuation Test")
        servo.angle = 1.7832
        time.sleep(1)
        servo.angle = 0.0
        time.sleep(1)
        
        print("Petal Actuation Complete!")
        print("Entering Initialization Loop")

        '''
        Initialization Loop
        The way this loop works is by continuously reading in vertical acceleration
        data from the IMU and collecting 50 values in an array and having the array index
        jump back to the start and continue collection, once the average value of the array is 
        above a certain threshold it enters our main loop.
        ''' 
        while True:
            t_curr = time.perf_counter()
            time_elapsed = t_curr - t0
            
            # Store IMU input and time data
            launch_times[i] = time_elapsed
            test_z_acc[i] = bno_sensor.acceleration[0]

            i = (i + 1) % 50

            # Average of z accelerations (rocket vertical)
            acc_z_avg = np.mean(test_z_acc)

            # When avg z is >= min_acc_criteria find max difference and get idx in time arr
            if acc_z_avg >= min_acc_criteria:
                max_diff = float("-inf")
                ordered_acc = test_z_acc[i:] + test_z_acc[:i]
                ordered_times = launch_times[i:] + launch_times[:i]

                for k in range(49):
                    diff = ordered_acc[k+1] - ordered_acc[k]
                    if diff > max_diff:
                        max_diff = diff
                        t_idx = k+1
                break

        # Using the determined time value from the loop above we can
        # determine the offset needed to know when the launch occurred and that
        # we should treat that as t0.
        launch_time = ordered_times[t_idx]
        t_from_launch = (time.perf_counter() - t0) - launch_time
        t_prev = t_from_launch

        samples = 0
        recovered = False
        pressures = []
        times = []
        conf_intvs = []
        descent = 0.0
        init_alt = 0.0
        act_angle = 0.0

        fused_roll = 0.0
        gyro_filtered_x = 0.0
        gyro_filtered_y = 0.0
        gyro_filtered_z = 0.0
        gyro_bias_x = 0.0

        pitch_global_rad = 0.0
        yaw_global_rad = 0.0

        last_mx = None
        last_my = None
        last_mz = None

        # Current pos and vel as well as stored previous values for ODE solver
        pos_x, pos_y, pos_z = 0.0, 0.0, 0.0
        vel_x, vel_y, vel_z = 0.0, 0.0, 0.0
        kinematic_initialized = False

        pos_x_prev, pos_y_prev, pos_z_prev = 0.0, 0.0, 0.0
        vel_x_prev, vel_y_prev, vel_z_prev = 0.0, 0.0, 0.0

        # Vertical fusion weights (Alpha-Beta complementary gains)
        ALPHA = 0.08
        BETA  = 0.03

        # Main Loop
        # All the math used to correct for drift in here was not very good
        # your team should focus heavily on trying to get something better than a
        # complimentary filter (Kalman filter would be ideal but its not easy lol)
        while True:
            t_from_launch = (time.perf_counter() - t0) - launch_time
            dt = t_from_launch - t_prev

            block = bus.read_i2c_block_data(BARO_ADDR, PRESS_ADDR, 6)

            # Uncompensated temp/press values
            u_temp = bmpfuncs.uncomp_temp(block)
            u_press = bmpfuncs.uncomp_press(block)

            # Compensated temp/press values
            c_temp = bmpfuncs.temp_compensation(u_temp, data)
            c_press = bmpfuncs.press_compensation(u_press, 30.0, data)
            c_press = c_press / 100

            # Calculate barometric altitude
            baro_alt = 44330.0 * (1.0 - (c_press / 1013.25) ** 0.1903)

            # Raw IMU data
            acc_raw = bno_sensor.acceleration
            gyro_raw = bno_sensor.gyro
            mag_raw = bno_sensor.magnetic
            
            mx_raw = mag_raw[0]
            my_raw = mag_raw[1]
            mz_raw = mag_raw[2]

            # Correct gyro data
            gx_corr = gyro_raw[0] - bg_x
            gy_corr = gyro_raw[1] - bg_y
            gz_corr = gyro_raw[2] - bg_z

            # Correct accel data via bias shift
            ax_s = acc_raw[0] - ba_x
            ay_s = acc_raw[1] - ba_y
            az_s = acc_raw[2] - ba_z

            # Correct accel data via scale factor and cross-axis correction
            ax_corr = (M00 * ax_s) + (M01 * ay_s) + (M02 * az_s)
            ay_corr = (M10 * ax_s) + (M11 * ay_s) + (M12 * az_s)
            az_corr = (M20 * ax_s) + (M21 * ay_s) + (M22 * az_s)

            # Filter high frequency vibrations from the X-axis gyro
            alpha = fusion.calculate_rt_alpha(dt, cutoff_freq=15.0)
            gyro_filtered_x = fusion.low_pass_filter(gx_corr, gyro_filtered_x, alpha)
            gyro_filtered_y = fusion.low_pass_filter(gy_corr, gyro_filtered_y, alpha)
            gyro_filtered_z = fusion.low_pass_filter(gz_corr, gyro_filtered_z, alpha)

            # Subtract bias and integrate to get roll angle
            fused_roll += (gyro_filtered_x - gyro_bias_x) * dt

            # Check for updated mag values to correct gyro bias estimation
            if mx_raw != last_mx or my_raw != last_my or mz_raw != last_mz:
                last_mx = mx_raw
                last_my = my_raw
                last_mz = mz_raw

                # Calibrate magnetometer data only when it is fresh
                mx_centered = mx_raw - hi_offset[0]
                my_centered = my_raw - hi_offset[1]
                mz_centered = mz_raw - hi_offset[2]

                mx_corr = (si_scale[0][0] * mx_centered) + (si_scale[0][1] * my_centered) + (si_scale[0][2] * mz_centered)
                my_corr = (si_scale[1][0] * mx_centered) + (si_scale[1][1] * my_centered) + (si_scale[1][2] * mz_centered)
                mz_corr = (si_scale[2][0] * mx_centered) + (si_scale[2][1] * my_centered) + (si_scale[2][2] * mz_centered)

                fused_roll, gyro_bias_x = fusion.update_gyro_bias_complementary(
                    fused_roll, 
                    gyro_bias_x, 
                    my_corr, 
                    mz_corr,
                    dt
                )
            
            # Map body gyro rates to global frame using the current fused roll angle
            gx_global, gy_global = fusion.unspin_body_rates_to_global(
                gyro_filtered_y,
                gyro_filtered_z,
                fused_roll
            )
            
            # Accumulate global rates over time
            pitch_global_rad += gy_global * dt
            yaw_global_rad += gx_global * dt

            # Calculate total tilt away from vertical (theta)
            theta = fusion.resolve_total_vertical_tilt(pitch_global_rad, yaw_global_rad)

            # Avoid division by zero if rocket is perfectly vertical and theta is small
            if theta > 0.0001:
                heading_global_rad = math.atan2(yaw_global_rad, pitch_global_rad)
            else:
                heading_global_rad = 0.0
            
            # Rotate body accelerations to global frame and subtract gravity
            ax_g, ay_g, az_g = fusion.rotate_body_accel_to_global(
                ax_corr,
                ay_corr,
                az_corr, 
                theta,
                heading_global_rad
            )
            # Kinematic 3D state estimation (ENU Global Frame)
            # x: East, y: North, z: Up (In global frame)
            # these values will of course be different in your implementation
            if not kinematic_initialized:
                init_alt = baro_alt
                pos_z = baro_alt
                pos_x = 0.0
                pos_y = 0.0

                kinematic_initialized = True
            else:
                # Kinematic prediction step
                pos_x += vel_x * dt + 0.5 * ax_g * (dt ** 2)
                pos_y += vel_y * dt + 0.5 * ay_g * (dt ** 2)
                pos_z += vel_z * dt + 0.5 * az_g * (dt ** 2)

                # Update velocity
                vel_x += ax_g * dt
                vel_y += ay_g * dt
                vel_z += az_g * dt

                # Complementary Correction Step (Vertical Baro Handoff)
                alt_error_z = baro_alt - pos_z
                pos_z += ALPHA * alt_error_z
                vel_z += (BETA * alt_error_z) / dt

            # This is where each phase becomes important as it allows us to isolate
            # our desired behavior. In the breaking phase which we determined would
            # take place after our max acceleration around 4.4s we send our data
            # over to a parallel process which is running our ODE solver using these supplied
            # values
            if phase == BOOST and t_from_launch >= 4.4:
                phase = BREAKING


            if phase == BREAKING or recovered == True:
                if pipe.poll():
                    # Take act percent output and apply to servo
                    act_angle = pipe.recv()

                if samples % 50 == 0:
                    pipe.send([
                        dt,
                        theta,
                        pos_x, vel_x,
                        pos_y, vel_y,
                        pos_z, vel_z,
                        pos_x_prev, vel_x_prev,
                        pos_y_prev, vel_y_prev,
                        pos_z_prev, vel_z_prev,
                        recovered
                    ])

                if theta > 0.524:
                    act_angle = 0.0

                recovered = False
                servo.angle = act_angle
            

            # Log raw data to buffer for future team analysis
            log_row = [
                t_from_launch, dt, 
                acc_raw[0], acc_raw[1], acc_raw[2],
                gyro_raw[0], gyro_raw[1], gyro_raw[2],
                mag_raw[0], mag_raw[1], mag_raw[2],
                c_temp, c_press,
                theta, fused_roll
            ]

            log_data(log_row)

            if phase == RECOVERY:
                # This checked our determined altitude to make sure we triggered the chute
                # At a specific altitude
                if baro_alt - init_alt <= 540.0:
                    # Trigger main chute
                    mosfet_main.on()
                    time.sleep(0.5)
                    mosfet_main.off()
            
            if phase == RECOVERY and (t_from_launch - descent) >= 1200:
                # Main has definitely gone off and we dont need to record anymore
                break

            '''
            This is our recovery logic, there should be a flowchart somewhere in the teams
            which will make it clearer to understand. It essentially uses a linear regression and
            confidence interval to determine the likelyhood we have begun to desecend based on our
            time and pressure values checking for trends where our pressure begins to increase again
            and we are therefore descending.
            ''' 
            if phase == BREAKING and t_from_launch >= 8.5:
                pressures.append(c_press)
                times.append(t_from_launch)

                # Grab 20 samples at a time
                if (len(pressures) == 20):
                    a, b, SE = bmpfuncs.slinear_regression(times, pressures)
                    ci_upper, ci_lower = bmpfuncs.conf_interval(a, b, SE, 20)

                    #Determine if CI is positive or not
                    if ci_lower > 0:
                        conf_intvs.append(1)
                    else:
                        conf_intvs.clear()
                    pressures.clear()
                    times.clear()
                
                if (len(conf_intvs) == 5):
                    servo.angle = 0.0
                    mosfet_drogue.on()
                    time.sleep(0.5)
                    mosfet_drogue.off()
                    phase = RECOVERY
                    recovered = True
                    descent = t_from_launch
            
            # Update previous state for next loop iteration
            pos_x_prev, pos_y_prev, pos_z_prev = pos_x, pos_y, pos_z
            vel_x_prev, vel_y_prev, vel_z_prev = vel_x, vel_y, vel_z
            t_prev = t_from_launch

            samples += 1

# This is a process running in parallel in order to save on time complexity given
# the amount of time it takes to run the ODE solver which we optimized to the best
# of our ability using Cython. (Hopefully you get to write yours in C but who knows)
# Contact Jordan Howard for questions about the ODE solver I did not write that portion.
def actuation_solver_process(pipe):
    import pandas as pd
    from modules import actuation
    from ode_funcs import ODE_sol, actuation_perc
    
    # 2D Bilinear Interpolation for Drag Coefficient
    CFD_Table = pd.read_csv(r"CD_Table.csv", on_bad_lines='skip')
    CFD_Table = np.array(CFD_Table.iloc[:, :].to_numpy(dtype=float))

    A_tgt = 3048
    max_iter = 5
    tol = 0.5
    act_up = 1.0
    act_dn = 0.0

    while True:
        if pipe.poll():
            # Pull pre-fused 3D tracking telemetry maps out of the OS buffer
            data = pipe.recv()
            
            dt             = data[0]
            theta_vertical = data[1]
            pos_x          = data[2]
            vel_x          = data[3]
            pos_y          = data[4]
            vel_y          = data[5]
            pos_z          = data[6]
            vel_z          = data[7]
            pos_x_prev     = data[8]
            vel_x_prev     = data[9]
            pos_y_prev     = data[10]
            vel_y_prev     = data[11]
            pos_z_prev     = data[12]
            vel_z_prev     = data[13]
            recovered      = data[14]

            if recovered == True:
                # We have recovered and no longer need to solve for actuation
                break

            x_n = math.sqrt(pow(pos_x, 2) + pow(pos_y, 2))
            x_prev = math.sqrt(pow(pos_x_prev, 2) + pow(pos_y_prev, 2))

            vel_x_n = math.sqrt(pow(vel_x, 2) + pow(vel_y, 2))            

            act_struct = actuation_perc(
                A_tgt,
                act_up,
                act_dn,
                tol,
                max_iter,
                x_n,
                x_prev,
                pos_z,
                pos_z_prev,
                vel_x_n,
                vel_z,
                dt,
                theta_vertical,
                CFD_Table
            )

            percentage = act_struct["act_k"]
            
            angle = actuation.angle_solver(percentage * 2.175385049)

            pipe.send(angle)

        else:
            time.sleep(0.001)

        

# The main function simply triggers both processes to start in parallel
# Look at the python docs for Pipe() and Multiprocessing for more info
def main():
    ode_conn, collect_conn = Pipe()

    p1 = Process(target=sensor_read_process, args=(collect_conn,))
    p2 = Process(target=actuation_solver_process, args=(ode_conn,))

    p1.start()
    p2.start()

    p2.join()
    p1.join()

if __name__ == "__main__":
    main()

# GOOD LUCK!
