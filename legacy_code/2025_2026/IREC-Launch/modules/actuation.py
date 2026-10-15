import math

def act_fn(act_angle):
    return  0.65 * math.tan(act_angle - 0.97) - 1.70 * (
            math.sin(math.pi / 2 - act_angle + 0.97 + math.asin(0.382352941176 - 0.811764705882* math.sin(
            act_angle + math.pi / 2 - 0.97) )) ) / (math.sin(act_angle + math.pi / 2 - 0.97)) + 2.8336

def angle_solver(act_desired):
    fin_diff = (act_fn(0.900000001) - act_fn(0.899999999) ) * (5e8)
    act_guess = 0.9 + (act_desired - act_fn(0.9)) / fin_diff
    fin_diff = (act_fn(act_guess + 1e-9) - act_fn(act_guess - 1e-9) ) * (5e8)
    act_guess = act_guess + (act_desired - act_fn(act_guess)) / fin_diff
    return act_guess
