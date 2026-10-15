from libc.math cimport sqrt
from libc.math cimport pow
from libc.math cimport sin, cos
from libc.math cimport M_PI
from libc.math cimport fabs

# Global Variables
cdef double P0 = 100000.5
cdef double T0 = 307.03889
cdef double molar_mass = 0.0289652
cdef double ideal_gas_const = 8.31446261815324
cdef double lapse_rate = 0.0092
cdef double rho_const = P0 * molar_mass / (ideal_gas_const * T0)
cdef double T0_inv = 1 / T0
cdef double power_const = 9.80665 * molar_mass / (ideal_gas_const * lapse_rate) - 1
cdef double rho_const2 = 1 - T0_inv * lapse_rate * 848.0
cdef double A_ref = 0.019
cdef double mass = 23.010
cdef double h = 0.004
cdef double h_inv = 1.0 / h
cdef double g = 9.80665
cdef double acc_const = A_ref / (2 * mass)
cdef double theta_n = 0.10471975512 # This can be dynamically found via launch pad theta

cdef struct act_out:
    double act_k
    double A_k
    int iteration

cdef inline double clamp(double x):
    if x < 0.0:
        x = 0.0
    elif x > 1.0:
        x = 1.0

    return x

def rho(double z):
    if z < 1524:
        rho = -7.13208137e-5 * z + 0.867271457
    else:
        rho = -6.54056354e-5 * z + 0.858279361
    return rho

def SoS(double z):
    if z < 1524:
        SoS = -5.39413093e-3 * z + 346.763396
    else:
        SoS = -5.52840869e-3 * z + 346.969064
    return SoS

cdef inline double CD(
    double mach,
    double act,
    double[:, :] CFD_Table
):
    cdef double m_inv_step = 100.0
    cdef double a_inv_step = 10.0

    cdef int idx_m1
    cdef int idx_a1

    cdef double u
    cdef double v

    cdef double f11
    cdef double f12
    cdef double f21
    cdef double f22
   
    if act < 0.0:
        act = 0.0

    idx_m1 = <int>(mach * m_inv_step)
    u = (mach * m_inv_step) - idx_m1

    idx_a1 = <int>(act * a_inv_step)  +1
    v = (act * a_inv_step) - (idx_a1 - 1)

    if act >= 1.0:
        return CFD_Table[idx_m1, 11] + u * (
            CFD_Table[idx_m1 + 1, 11] - CFD_Table[idx_m1, 11]
        )
    elif act < 0.0:
        act = 0.0

    if mach < 0.1:
        return CFD_Table[0, idx_a1] + v * (
            CFD_Table[0, idx_a1 + 1] - CFD_Table[0, idx_a1]
        )

    f11 = CFD_Table[idx_m1, idx_a1]
    f12 = CFD_Table[idx_m1, idx_a1 + 1]
    f21 = CFD_Table[idx_m1 + 1, idx_a1]
    f22 = CFD_Table[idx_m1 + 1, idx_a1 + 1]

    return (1.0 - u) * (f11 + v * (f12 - f11)) + u * (f21 + v * (f22 - f21))

cpdef double ODE_sol(
        double x_n,
        double x_n_prev,
        double z_n,
        double z_n_prev,
        double vel_x_n,
        double vel_z_n,
        double dt,
        double theta,
        double act,
        double[:, :] CFD_Table
):
    cdef double h_frac
    cdef double h2

    cdef double vel_sqr_n
    cdef double mach_n
    cdef double rho_n
    cdef double CD_n

    cdef double acc_x
    cdef double acc_z
    cdef double x_fwd
    cdef double z_fwd

    cdef double sin_theta
    cdef double cos_theta
    cdef double coeff

    cdef double x_curr
    cdef double x_old
    cdef double z_curr
    cdef double z_old
    cdef double vx_curr
    cdef double vz_curr

    h_frac = h / dt
    h2 = h * h

    x_curr = x_n
    z_curr = z_n
    vx_curr = vel_x_n
    vz_curr = vel_z_n

    x_old = x_curr - (x_curr - x_n_prev) * h_frac
    z_old = z_curr - (z_curr - z_n_prev) * h_frac

 
    sin_theta = sin(theta)
    cos_theta = cos(theta)

    while (z_curr - z_old) >= 0.0:
        vel_sqr_n = vx_curr * vx_curr + vz_curr * vz_curr
        mach_n = sqrt(vel_sqr_n) / SoS(z_curr)

        rho_n = rho(z_curr)
        CD_n = CD(mach_n, act, CFD_Table)

        coeff = -rho_n * acc_const * vel_sqr_n * CD_n

        acc_x = coeff * sin_theta
        acc_z = coeff * cos_theta - g

        x_fwd = acc_x * h2 + 2.0 * x_curr - x_old
        z_fwd = acc_z * h2 + 2.0 * z_curr - z_old

        x_old = x_curr
        z_old = z_curr
        x_curr = x_fwd
        z_curr = z_fwd

        vx_curr = (x_curr - x_old) * h_inv
        vz_curr = (z_curr - z_old) * h_inv

    return z_curr

cpdef act_out actuation_perc(
    double A_tgt,
    double act_up,
    double act_dn,
    double tol,
    int max_iter,
    double x_n,
    double x_n_prev,
    double z_n,
    double z_n_prev,
    double vel_x_n,
    double vel_z_n,
    double dt,
    double theta,
    double[:, :] CFD_Table
):
    cdef act_out result
    cdef double A_dn = ODE_sol(
        x_n,
        x_n_prev,
        z_n,
        z_n_prev,
        vel_x_n,
        vel_z_n,
        dt,
        theta,
        act_up,
        CFD_Table
    )
    
    cdef double A_up = ODE_sol(
        x_n,
        x_n_prev,
        z_n,
        z_n_prev,
        vel_x_n,
        vel_z_n,
        dt,
        theta,
        act_dn,
        CFD_Table
    )
    cdef double act_k
    
    if fabs(A_up - A_dn) <= 1e-9:
        result.act_k = 0.0
        result.A_k = A_up
        result.iteration = 0
        return result

    act_k = (A_tgt - A_up) / (A_up - A_dn) * (act_dn - act_up) + act_dn
    act_k = clamp(act_k)

    cdef double A_k = ODE_sol(
        x_n,
        x_n_prev,
        z_n,
        z_n_prev,
        vel_x_n,
        vel_z_n,
        dt,
        theta,
        act_k,
        CFD_Table
    )

    cdef int iteration = 1

    while (fabs(A_k - A_tgt) > tol) and (iteration <= max_iter):
        if A_k <= A_tgt:
            act_up = act_k
            A_dn = A_k
        else:
            act_dn = act_k
            A_up = A_k

        if fabs(A_up - A_dn) > 0:
            act_k = (A_tgt - A_up) / (A_up - A_dn) * (act_dn - act_up) + act_dn
            act_k = clamp(act_k)
            A_k = ODE_sol(
                x_n,
                x_n_prev,
                z_n,
                z_n_prev,
                vel_x_n,
                vel_z_n,
                dt,
                theta,
                act_k,
                CFD_Table
            )
        else:
            act_k = 1.0

        
        A_k = ODE_sol(
            x_n,
            x_n_prev,
            z_n,
            z_n_prev,
            vel_x_n,
            vel_z_n,
            dt,
            theta,
            act_k,
            CFD_Table
        )

        if (act_k < 0.0):
            act_k = 0.0

        iteration += 1

    if (iteration > max_iter):
        act_k = 0.0
        
    result.act_k = act_k
    result.A_k = A_k
    result.iteration = iteration
    
    return result
    

