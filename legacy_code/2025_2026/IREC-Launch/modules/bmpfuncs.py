import time
import math

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

# Register Value(s)
SOFT_RESET      = 0xB6
PWR_SLEEP_EN    = 0x03
PWR_FORCED_EN   = 0x13
PWR_NORMAL_EN   = 0x33
INT_CTRL_DRDY   = 0x40
OSR_VAL         = 0b00001001
ODR_VAL         = 0x00

# Helper functions
def to_signed_8(x: int) -> int:
    if x & 0x80:
        return x - 256
    return x

def to_signed_16(x: int) -> int:
    if x & 0x8000:
        return x - 65536
    return x
def u16_from_bytes(lsb: int, msb: int) -> int:
    return (msb << 8) | lsb

def read_nvm_coefs(bus, address: int) -> dict[int]:
    # Read BMP388 trimming coefficients and return raw nvm int values
    coefs = bus.read_i2c_block_data(address, 0x31, 21)

    nvm_par_t1 = u16_from_bytes(coefs[0], coefs[1])
    nvm_par_t2 = u16_from_bytes(coefs[2], coefs[3])
    nvm_par_t3 = to_signed_8(coefs[4])

    nvm_par_p1 = to_signed_16(u16_from_bytes(coefs[5], coefs[6]))
    nvm_par_p2 = to_signed_16(u16_from_bytes(coefs[7], coefs[8]))
    nvm_par_p3 = to_signed_8(coefs[9])
    nvm_par_p4 = to_signed_8(coefs[10])
    nvm_par_p5 = u16_from_bytes(coefs[11], coefs[12])
    nvm_par_p6 = u16_from_bytes(coefs[13], coefs[14])
    nvm_par_p7 = to_signed_8(coefs[15])
    nvm_par_p8 = to_signed_8(coefs[16])
    nvm_par_p9 = to_signed_16(u16_from_bytes(coefs[17], coefs[18]))
    nvm_par_p10 = to_signed_8(coefs[19])
    nvm_par_p11 = to_signed_8(coefs[20])

    return {
    "nvm_par_t1": nvm_par_t1,
    "nvm_par_t2": nvm_par_t2,
    "nvm_par_t3": nvm_par_t3,
    "nvm_par_p1": nvm_par_p1,
    "nvm_par_p2": nvm_par_p2,
    "nvm_par_p3": nvm_par_p3,
    "nvm_par_p4": nvm_par_p4,
    "nvm_par_p5": nvm_par_p5,
    "nvm_par_p6": nvm_par_p6,
    "nvm_par_p7": nvm_par_p7,
    "nvm_par_p8": nvm_par_p8,
    "nvm_par_p9": nvm_par_p9,
    "nvm_par_p10": nvm_par_p10,
    "nvm_par_p11": nvm_par_p11,
    }

def convert_nvm_coefs(raw_data: dict[int]) -> dict[float]:
    # Convert values to floating point based on datasheet
    par_t1 = raw_data["nvm_par_t1"] / (2 ** -8)
    par_t2 = raw_data["nvm_par_t2"] / (2 ** 30)
    par_t3 = raw_data["nvm_par_t3"] / (2 ** 48)

    par_p1 = (raw_data["nvm_par_p1"] - (2 ** 14)) / (2 ** 20)
    par_p2 = (raw_data["nvm_par_p2"] - (2 ** 14)) / (2 ** 29)
    par_p3 = raw_data["nvm_par_p3"] / (2 ** 32)
    par_p4 = raw_data["nvm_par_p4"] / (2 ** 37)
    par_p5 = raw_data["nvm_par_p5"] / (2 ** -3)
    par_p6 = raw_data["nvm_par_p6"] / (2 ** 6)
    par_p7 = raw_data["nvm_par_p7"] / (2 ** 8)
    par_p8 = raw_data["nvm_par_p8"] / (2 ** 15)
    par_p9 = raw_data["nvm_par_p9"] / (2 ** 48)
    par_p10 = raw_data["nvm_par_p10"] / (2 ** 48)
    par_p11 = raw_data["nvm_par_p11"] / (2 ** 65)

    return {
        "par_t1": par_t1,
        "par_t2": par_t2,
        "par_t3": par_t3,
        "par_p1": par_p1,
        "par_p2": par_p2,
        "par_p3": par_p3,
        "par_p4": par_p4,
        "par_p5": par_p5,
        "par_p6": par_p6,
        "par_p7": par_p7,
        "par_p8": par_p8,
        "par_p9": par_p9,
        "par_p10": par_p10,
        "par_p11": par_p11,
    }

# uncompensated temperature and pressure functions
def uncomp_temp(block: list[int]) -> int:
    return block[5] << 16 | block[4] << 8 | block[3]

def uncomp_press(block: list[int]) -> int:
    return block[2] << 16 | block[1] << 8 | block[0]

# Compensation functions
def temp_compensation(temp: int, calib_data: dict[float]) -> float:
    partial_data1: float
    partial_data2: float

    partial_data1 = float(temp - calib_data["par_t1"])
    partial_data2 = float(partial_data1 * calib_data["par_t2"])

    return float(partial_data2 + (partial_data1 ** 2) * calib_data["par_t3"])

def press_compensation(press: int, temp_comp: float, calib_data: dict[float]) -> float:
    partial_data1: float
    partial_data2: float
    partial_data3: float
    partial_data4: float
    partial_out1: float
    partial_out2: float

    partial_data1 = calib_data["par_p6"] * temp_comp
    partial_data2 = calib_data["par_p7"] * (temp_comp ** 2)
    partial_data3 = calib_data["par_p8"] * (temp_comp ** 3)
    partial_out1 = float(
        calib_data["par_p5"]
        + partial_data1
        + partial_data2
        + partial_data3
    )

    partial_data1 = calib_data["par_p2"] * temp_comp
    partial_data2 = calib_data["par_p3"] * (temp_comp ** 2)
    partial_data3 = calib_data["par_p4"] * (temp_comp ** 3)
    partial_out2 = float(press * (
        calib_data["par_p1"]
        + partial_data1
        + partial_data2
        + partial_data3
    ))

    partial_data1 = float(press ** 2)
    partial_data2 = calib_data["par_p9"] + calib_data["par_p10"] * temp_comp
    partial_data3 = partial_data1 * partial_data2
    partial_data4 = partial_data3 + float(press **3) * calib_data["par_p11"]

    return partial_out1 + partial_out2 + partial_data4

# Confidence Level functions
def slinear_regression(times: list, pressures: list):
    n = len(times)

    t_bar = sum(times) / n
    p_bar = sum(pressures) / n

    # Bounded memory calculation (Zero list allocations)
    sxx = 0.0
    sxy = 0.0
    for t, p in zip(times, pressures):
        t_diff = t - t_bar
        sxx += t_diff ** 2
        sxy += t_diff * (p - p_bar)

    # Get regression line and intercept
    b = sxy / sxx
    a = p_bar - b * t_bar

    # Predicted values
    p_hat = [a + b*t for t in times]

    # Get standard error
    SE = math.sqrt(
        sum((p - ph) ** 2 for p, ph in zip(pressures, p_hat)) / (n - 2)
        ) / math.sqrt(sxx)

    return a, b, SE

def conf_interval(a, b, SE, n, clvl=0.95):
    alpha = 1 - clvl
    c_prob = 1 - alpha / 2
    df = n - 2
    t_score = 2.101

    # Compute critical value
    ME = t_score * SE

    # Use margin of error to get the upper and lower confidence intervals
    ci_upper = b + ME
    ci_lower = b - ME

    return ci_upper, ci_lower
