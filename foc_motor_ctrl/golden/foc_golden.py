"""
foc_golden.py
============
Golden reference model for the 3-loop PMSM Field-Oriented Controller.

Architecture
------------

        omega_ref
            |
        Speed PI
            |
         iq_ref
            |
      +-----+-----+
      |           |
    id_ref       iq_ref
      |           |
    Id PI       Iq PI
      |           |
      +-----+-----+
            |
          Vd/Vq
            |
        Inv Park
            |
          Va/Vb
            |
          SVPWM
            |
       PMSM Plant
            |
       Id/Iq/omega

The PMSM plant is driven directly with averaged dq voltage Vd/Vq.
SVPWM is evaluated in parallel to generate the corresponding PWM duties.

Field weakening
---------------
Below the field-weakening speed:
    Id_ref = 0 A

Above the field-weakening speed:
    A negative Id reference is generated from the PMSM voltage equation
    so that the demanded steady-state dq voltage remains within a safe
    fraction of the available DC-bus voltage.

This allows the golden model to reach the required 1000 RPM target with
the existing 24 V voltage base.

Fixed-point conventions used by the project:

    Current:
        Q1.15
        1.0 = 20 A

    Voltage:
        Q1.15
        1.0 = 24 V

    Electrical angle:
        Q2.14
        16384 counts = pi rad

    Speed:
        Q10.6
        64 counts = 1 rad/s

Simulation:
    Control frequency = 20 kHz
    Control period    = 50 us
    Step 1            = 0 -> 500 RPM
    Step 2            = 500 -> 1000 RPM
"""

import csv
import inspect
import math
import os

import matplotlib.pyplot as plt

from pmsm_plant import PMSMPlant
from transforms import clarke, park, inv_park
from pi_controller import PIControllerFloat
from speed_pi import SpeedPIControllerFloat
from svpwm import svpwm_float


# ============================================================================
# PROJECT CONSTANTS
# ============================================================================

DT = 50e-6
CONTROL_FREQ = 1.0 / DT

I_BASE = 20.0
V_BASE = 24.0

OMEGA_TO_RPM = 60.0 / (2.0 * math.pi)
RPM_TO_OMEGA = 2.0 * math.pi / 60.0

T_STEP_1 = 0.50
T_STEP_2 = 0.85
T_END = 2.00

RPM_REF_1 = 500.0
RPM_REF_2 = 1000.0


# ============================================================================
# PMSM PARAMETERS
#
# These match pmsm_plant.py.
# ============================================================================

RS = 1.5
LD = 8.5e-3
LQ = 8.5e-3
LAM_M = 0.175
POLE_PAIRS = 2


# ============================================================================
# CURRENT-LOOP PI GAINS
# ============================================================================

CURRENT_KP = 3.0
CURRENT_KI = 0.025

CURRENT_OUTPUT_MIN = -1.0
CURRENT_OUTPUT_MAX = +1.0


# ============================================================================
# SPEED PI
# ============================================================================

SPEED_KP = 0.04
SPEED_KI = 0.5

SPEED_OUTPUT_MIN = -1.0
SPEED_OUTPUT_MAX = +1.0


# ============================================================================
# FIELD WEAKENING
# ============================================================================

# Start field weakening above this mechanical speed.
FIELD_WEAKENING_START_RPM = 600.0

# Maximum allowed negative d-axis current.
FIELD_WEAKENING_ID_MIN_A = -14.0

# Keep steady-state voltage demand below this fraction of V_BASE.
FIELD_WEAKENING_VOLTAGE_MARGIN = 0.90


# ============================================================================
# GENERAL HELPERS
# ============================================================================

def clamp(value, minimum, maximum):
    """Clamp a floating-point value."""
    return max(minimum, min(maximum, value))


def wrap_angle(angle):
    """Wrap angle to [0, 2*pi)."""
    angle = angle % (2.0 * math.pi)

    if angle < 0.0:
        angle += 2.0 * math.pi

    return angle


def pu_current_to_amp(pu):
    return pu * I_BASE


def amp_to_pu_current(amps):
    return amps / I_BASE


def pu_voltage_to_volt(pu):
    return pu * V_BASE


def volt_to_pu_voltage(volts):
    return volts / V_BASE


def omega_to_rpm(omega):
    return omega * OMEGA_TO_RPM


def rpm_to_omega(rpm):
    return rpm * RPM_TO_OMEGA


# ============================================================================
# FIELD-WEAKENING CALCULATION
# ============================================================================

def required_steady_state_voltage(id_current, iq_current, omega_mech):
    """
    Calculate the steady-state dq voltage magnitude required by the PMSM.

    PMSM equations for Ld = Lq:

        Vd = Rs*Id - omega_e*Lq*Iq

        Vq = Rs*Iq + omega_e*(Ld*Id + lambda_m)

    where:

        omega_e = pole_pairs * omega_mech
    """

    omega_e = POLE_PAIRS * omega_mech

    vd = (
        RS * id_current
        - omega_e * LQ * iq_current
    )

    vq = (
        RS * iq_current
        + omega_e * (LD * id_current + LAM_M)
    )

    voltage_magnitude = math.hypot(vd, vq)

    return voltage_magnitude


def calculate_field_weakening_id(
    omega_mech,
    iq_ref,
):
    """
    Calculate the required negative Id reference.

    Below FIELD_WEAKENING_START_RPM:
        Id_ref = 0

    Above the threshold:
        Find the least-negative Id that brings the estimated steady-state
        voltage requirement below the configured voltage margin.

    The result is limited by FIELD_WEAKENING_ID_MIN_A.
    """

    speed_rpm = omega_to_rpm(omega_mech)

    if speed_rpm <= FIELD_WEAKENING_START_RPM:
        return 0.0

    voltage_limit = (
        FIELD_WEAKENING_VOLTAGE_MARGIN * V_BASE
    )

    # At Id = 0, check whether field weakening is actually necessary.
    voltage_at_zero_id = required_steady_state_voltage(
        0.0,
        iq_ref,
        omega_mech,
    )

    if voltage_at_zero_id <= voltage_limit:
        return 0.0

    # Check the configured maximum field weakening first.
    voltage_at_min_id = required_steady_state_voltage(
        FIELD_WEAKENING_ID_MIN_A,
        iq_ref,
        omega_mech,
    )

    if voltage_at_min_id > voltage_limit:
        return FIELD_WEAKENING_ID_MIN_A

    # Binary search for the least-negative Id that satisfies the
    # voltage constraint.
    low = FIELD_WEAKENING_ID_MIN_A
    high = 0.0

    for _ in range(40):
        mid = 0.5 * (low + high)

        voltage = required_steady_state_voltage(
            mid,
            iq_ref,
            omega_mech,
        )

        if voltage > voltage_limit:
            # Need more negative Id.
            high = mid
        else:
            low = mid

    return low


# ============================================================================
# PI API ADAPTERS
# ============================================================================

def _construct_current_pi(
    kp,
    ki,
    output_min,
    output_max,
):
    """
    Construct PIControllerFloat using the actual project API.

    The current PI is an update-based controller and does not receive dt.
    """

    signature = inspect.signature(
        PIControllerFloat.__init__
    )

    parameters = signature.parameters
    kwargs = {}

    if "kp" in parameters:
        kwargs["kp"] = kp
    elif "Kp" in parameters:
        kwargs["Kp"] = kp

    if "ki" in parameters:
        kwargs["ki"] = ki
    elif "Ki" in parameters:
        kwargs["Ki"] = ki

    for name in (
        "output_min",
        "out_min",
        "min_output",
        "output_lower",
    ):
        if name in parameters:
            kwargs[name] = output_min
            break

    for name in (
        "output_max",
        "out_max",
        "max_output",
        "output_upper",
    ):
        if name in parameters:
            kwargs[name] = output_max
            break

    for name in (
        "integrator_min",
        "integral_min",
        "i_min",
        "integrator_lower",
    ):
        if name in parameters:
            kwargs[name] = output_min
            break

    for name in (
        "integrator_max",
        "integral_max",
        "i_max",
        "integrator_upper",
    ):
        if name in parameters:
            kwargs[name] = output_max
            break

    return PIControllerFloat(**kwargs)


def _call_current_pi(controller, setpoint, measurement):
    """Call the project current PI update API."""

    update_method = controller.update
    signature = inspect.signature(update_method)
    parameters = signature.parameters

    if "setpoint" in parameters and "measurement" in parameters:
        return update_method(
            setpoint=setpoint,
            measurement=measurement,
        )

    if "reference" in parameters and "measurement" in parameters:
        return update_method(
            reference=setpoint,
            measurement=measurement,
        )

    if "target" in parameters and "measurement" in parameters:
        return update_method(
            target=setpoint,
            measurement=measurement,
        )

    error = setpoint - measurement

    if "error" in parameters:
        return update_method(error=error)

    return update_method(setpoint, measurement)


def _construct_speed_pi():
    """Construct the project speed PI."""

    return SpeedPIControllerFloat(
        kp=SPEED_KP,
        ki=SPEED_KI,
        dt=DT,
        output_min=SPEED_OUTPUT_MIN,
        output_max=SPEED_OUTPUT_MAX,
    )


def _call_speed_pi(controller, setpoint, measurement):
    """Call the project speed PI update API."""

    update_method = controller.update
    signature = inspect.signature(update_method)
    parameters = signature.parameters

    if "setpoint" in parameters and "measurement" in parameters:
        return update_method(
            setpoint=setpoint,
            measurement=measurement,
        )

    if "reference" in parameters and "measurement" in parameters:
        return update_method(
            reference=setpoint,
            measurement=measurement,
        )

    if "target" in parameters and "measurement" in parameters:
        return update_method(
            target=setpoint,
            measurement=measurement,
        )

    error = setpoint - measurement

    if "error" in parameters:
        return update_method(error=error)

    return update_method(setpoint, measurement)


# ============================================================================
# PMSM CURRENT RECONSTRUCTION
# ============================================================================

def phase_currents_from_dq(id_current, iq_current, theta_e):
    """
    Reconstruct phase currents Ia/Ib/Ic from dq currents.
    """

    cos_theta = math.cos(theta_e)
    sin_theta = math.sin(theta_e)

    i_alpha = (
        id_current * cos_theta
        - iq_current * sin_theta
    )

    i_beta = (
        id_current * sin_theta
        + iq_current * cos_theta
    )

    ia = i_alpha

    ib = (
        -0.5 * i_alpha
        + (math.sqrt(3.0) / 2.0) * i_beta
    )

    ic = (
        -0.5 * i_alpha
        - (math.sqrt(3.0) / 2.0) * i_beta
    )

    return ia, ib, ic


# ============================================================================
# FOC CONTROLLER
# ============================================================================

class FOCController:
    """
    Floating-point golden FOC controller with field weakening.
    """

    def __init__(self):

        self.speed_pi = _construct_speed_pi()

        self.id_pi = _construct_current_pi(
            kp=CURRENT_KP,
            ki=CURRENT_KI,
            output_min=CURRENT_OUTPUT_MIN,
            output_max=CURRENT_OUTPUT_MAX,
        )

        self.iq_pi = _construct_current_pi(
            kp=CURRENT_KP,
            ki=CURRENT_KI,
            output_min=CURRENT_OUTPUT_MIN,
            output_max=CURRENT_OUTPUT_MAX,
        )

        self.id_ref_pu = 0.0

        self.last_iq_ref_pu = 0.0
        self.last_id_meas_pu = 0.0
        self.last_iq_meas_pu = 0.0

        self.last_vd_pu = 0.0
        self.last_vq_pu = 0.0

        self.last_valpha_pu = 0.0
        self.last_vbeta_pu = 0.0

        self.last_duty_a = 0.5
        self.last_duty_b = 0.5
        self.last_duty_c = 0.5

        self.last_sector = 1

    def update(
        self,
        omega_ref,
        omega_mech,
        ia,
        ib,
        ic,
        theta_e,
    ):
        # ------------------------------------------------------------------
        # 1. SPEED PI
        # ------------------------------------------------------------------

        iq_ref_pu = _call_speed_pi(
            self.speed_pi,
            omega_ref,
            omega_mech,
        )

        iq_ref_pu = clamp(
            iq_ref_pu,
            SPEED_OUTPUT_MIN,
            SPEED_OUTPUT_MAX,
        )

        iq_ref = pu_current_to_amp(iq_ref_pu)

        # ------------------------------------------------------------------
        # 2. FIELD WEAKENING
        # ------------------------------------------------------------------

        id_ref = calculate_field_weakening_id(
            omega_mech=omega_mech,
            iq_ref=iq_ref,
        )

        id_ref_pu = clamp(
            amp_to_pu_current(id_ref),
            FIELD_WEAKENING_ID_MIN_A / I_BASE,
            0.0,
        )

        self.id_ref_pu = id_ref_pu

        # ------------------------------------------------------------------
        # 3. CLARKE
        # ------------------------------------------------------------------

        i_alpha, i_beta = clarke(
            ia,
            ib,
            ic,
        )

        # ------------------------------------------------------------------
        # 4. PARK
        # ------------------------------------------------------------------

        id_meas, iq_meas = park(
            i_alpha,
            i_beta,
            theta_e,
        )

        id_meas_pu = amp_to_pu_current(id_meas)
        iq_meas_pu = amp_to_pu_current(iq_meas)

        # ------------------------------------------------------------------
        # 5. CURRENT PI LOOPS
        # ------------------------------------------------------------------

        vd_pu = _call_current_pi(
            self.id_pi,
            id_ref_pu,
            id_meas_pu,
        )

        vq_pu = _call_current_pi(
            self.iq_pi,
            iq_ref_pu,
            iq_meas_pu,
        )

        vd_pu = clamp(
            vd_pu,
            CURRENT_OUTPUT_MIN,
            CURRENT_OUTPUT_MAX,
        )

        vq_pu = clamp(
            vq_pu,
            CURRENT_OUTPUT_MIN,
            CURRENT_OUTPUT_MAX,
        )

        vd = pu_voltage_to_volt(vd_pu)
        vq = pu_voltage_to_volt(vq_pu)

        # ------------------------------------------------------------------
        # 6. INVERSE PARK
        # ------------------------------------------------------------------

        valpha, vbeta = inv_park(
            vd,
            vq,
            theta_e,
        )

        valpha_pu = volt_to_pu_voltage(valpha)
        vbeta_pu = volt_to_pu_voltage(vbeta)

        # ------------------------------------------------------------------
        # 7. SVPWM
        # ------------------------------------------------------------------

        svpwm_result = svpwm_float(
            valpha_pu,
            vbeta_pu,
        )

        duty_a, duty_b, duty_c, sector = (
            extract_svpwm_result(svpwm_result)
        )

        # ------------------------------------------------------------------
        # Save state.
        # ------------------------------------------------------------------

        self.last_iq_ref_pu = iq_ref_pu

        self.last_id_meas_pu = id_meas_pu
        self.last_iq_meas_pu = iq_meas_pu

        self.last_vd_pu = vd_pu
        self.last_vq_pu = vq_pu

        self.last_valpha_pu = valpha_pu
        self.last_vbeta_pu = vbeta_pu

        self.last_duty_a = duty_a
        self.last_duty_b = duty_b
        self.last_duty_c = duty_c

        self.last_sector = sector

        return {
            "omega_ref": omega_ref,
            "omega_mech": omega_mech,

            "ia": ia,
            "ib": ib,
            "ic": ic,

            "i_alpha": i_alpha,
            "i_beta": i_beta,

            "id_ref": id_ref,
            "iq_ref": iq_ref,

            "id_ref_pu": id_ref_pu,
            "iq_ref_pu": iq_ref_pu,

            "id": id_meas,
            "iq": iq_meas,

            "id_pu": id_meas_pu,
            "iq_pu": iq_meas_pu,

            "vd": vd,
            "vq": vq,

            "vd_pu": vd_pu,
            "vq_pu": vq_pu,

            "valpha": valpha,
            "vbeta": vbeta,

            "valpha_pu": valpha_pu,
            "vbeta_pu": vbeta_pu,

            "duty_a": duty_a,
            "duty_b": duty_b,
            "duty_c": duty_c,

            "sector": sector,

            "theta_e": theta_e,
        }


# ============================================================================
# SVPWM RESULT ADAPTER
# ============================================================================

def extract_svpwm_result(result):

    if isinstance(result, dict):

        if all(
            key in result
            for key in ("Ta", "Tb", "Tc")
        ):
            return (
                clamp(float(result["Ta"]), 0.0, 1.0),
                clamp(float(result["Tb"]), 0.0, 1.0),
                clamp(float(result["Tc"]), 0.0, 1.0),
                int(result.get("sector", 1)),
            )

        if all(
            key in result
            for key in ("duty_a", "duty_b", "duty_c")
        ):
            return (
                clamp(float(result["duty_a"]), 0.0, 1.0),
                clamp(float(result["duty_b"]), 0.0, 1.0),
                clamp(float(result["duty_c"]), 0.0, 1.0),
                int(result.get("sector", 1)),
            )

        if all(
            key in result
            for key in ("ta", "tb", "tc")
        ):
            return (
                clamp(float(result["ta"]), 0.0, 1.0),
                clamp(float(result["tb"]), 0.0, 1.0),
                clamp(float(result["tc"]), 0.0, 1.0),
                int(result.get("sector", 1)),
            )

    if hasattr(result, "Ta"):
        return (
            clamp(float(result.Ta), 0.0, 1.0),
            clamp(float(result.Tb), 0.0, 1.0),
            clamp(float(result.Tc), 0.0, 1.0),
            int(getattr(result, "sector", 1)),
        )

    if hasattr(result, "duty_a"):
        return (
            clamp(float(result.duty_a), 0.0, 1.0),
            clamp(float(result.duty_b), 0.0, 1.0),
            clamp(float(result.duty_c), 0.0, 1.0),
            int(getattr(result, "sector", 1)),
        )

    if isinstance(result, (tuple, list)):

        values = list(result)

        if len(values) >= 4:
            try:
                duty_a = float(values[0])
                duty_b = float(values[1])
                duty_c = float(values[2])
                sector = int(values[3])

                if (
                    0.0 <= duty_a <= 1.0
                    and 0.0 <= duty_b <= 1.0
                    and 0.0 <= duty_c <= 1.0
                    and 1 <= sector <= 6
                ):
                    return (
                        duty_a,
                        duty_b,
                        duty_c,
                        sector,
                    )

            except (TypeError, ValueError):
                pass

        if len(values) >= 3:
            try:
                duty_a = float(values[0])
                duty_b = float(values[1])
                duty_c = float(values[2])

                if (
                    0.0 <= duty_a <= 1.0
                    and 0.0 <= duty_b <= 1.0
                    and 0.0 <= duty_c <= 1.0
                ):
                    return (
                        duty_a,
                        duty_b,
                        duty_c,
                        1,
                    )

            except (TypeError, ValueError):
                pass

    raise TypeError(
        "Could not decode the return value from svpwm_float(). "
        f"Received type={type(result).__name__}, "
        f"value={result!r}"
    )


# ============================================================================
# REFERENCE SPEED PROFILE
# ============================================================================

def reference_speed_rpm(time_s):

    if time_s < T_STEP_1:
        return 0.0

    if time_s < T_STEP_2:
        return RPM_REF_1

    return RPM_REF_2


# ============================================================================
# SIMULATION
# ============================================================================

def run_simulation():

    number_of_steps = int(round(T_END / DT))

    print()
    print("=" * 72)
    print("FOC GOLDEN MODEL")
    print("=" * 72)
    print(f"Control frequency : {CONTROL_FREQ:,.0f} Hz")
    print(f"Control period    : {DT * 1e6:.1f} us")
    print(f"Simulation time   : {T_END:.3f} s")
    print("Step 1            : 0 -> 500 RPM")
    print("Step 2            : 500 -> 1000 RPM")
    print(f"Simulation steps  : {number_of_steps:,}")
    print()
    print(
        "Field weakening   : "
        f"starts at {FIELD_WEAKENING_START_RPM:.0f} RPM"
    )
    print(
        "Id minimum        : "
        f"{FIELD_WEAKENING_ID_MIN_A:.2f} A"
    )
    print(
        "Voltage margin    : "
        f"{FIELD_WEAKENING_VOLTAGE_MARGIN * 100.0:.1f}%"
    )
    print("=" * 72)
    print()

    plant = PMSMPlant(dt=DT)
    controller = FOCController()

    plant.reset()

    records = []

    omega_mech = plant.omega_mech
    theta_e = plant.theta_e

    for step in range(number_of_steps):

        time_s = step * DT

        speed_ref_rpm = reference_speed_rpm(time_s)
        omega_ref = rpm_to_omega(speed_ref_rpm)

        # --------------------------------------------------------------
        # Plant dq currents -> physical phase currents.
        # --------------------------------------------------------------

        id_plant = plant.Id
        iq_plant = plant.Iq

        ia, ib, ic = phase_currents_from_dq(
            id_plant,
            iq_plant,
            theta_e,
        )

        # --------------------------------------------------------------
        # FOC controller.
        # --------------------------------------------------------------

        control = controller.update(
            omega_ref=omega_ref,
            omega_mech=omega_mech,
            ia=ia,
            ib=ib,
            ic=ic,
            theta_e=theta_e,
        )

        # --------------------------------------------------------------
        # Apply averaged dq voltage to plant.
        # --------------------------------------------------------------

        vd = control["vd"]
        vq = control["vq"]

        id_new, iq_new, omega_new, theta_new = plant.step(
            vd,
            vq,
        )

        # --------------------------------------------------------------
        # Record.
        # --------------------------------------------------------------

        records.append(
            {
                "step": step,
                "time_s": time_s,

                "speed_ref_rpm": speed_ref_rpm,
                "omega_ref": omega_ref,

                "omega_mech": omega_mech,
                "speed_rpm": omega_to_rpm(omega_mech),

                "theta_e": theta_e,

                "ia": ia,
                "ib": ib,
                "ic": ic,

                "i_alpha": control["i_alpha"],
                "i_beta": control["i_beta"],

                "id_ref": control["id_ref"],
                "iq_ref": control["iq_ref"],

                "id_ref_pu": control["id_ref_pu"],
                "iq_ref_pu": control["iq_ref_pu"],

                "id": control["id"],
                "iq": control["iq"],

                "id_pu": control["id_pu"],
                "iq_pu": control["iq_pu"],

                "vd": vd,
                "vq": vq,

                "vd_pu": control["vd_pu"],
                "vq_pu": control["vq_pu"],

                "valpha": control["valpha"],
                "vbeta": control["vbeta"],

                "valpha_pu": control["valpha_pu"],
                "vbeta_pu": control["vbeta_pu"],

                "duty_a": control["duty_a"],
                "duty_b": control["duty_b"],
                "duty_c": control["duty_c"],

                "sector": control["sector"],

                "plant_id": id_new,
                "plant_iq": iq_new,
                "plant_omega": omega_new,
                "plant_theta_e": theta_new,
            }
        )

        omega_mech = omega_new
        theta_e = theta_new

    print("Simulation complete.")
    print(f"Samples generated : {len(records):,}")
    print()

    return records


# ============================================================================
# CSV OUTPUT
# ============================================================================

def save_csv(records, filename):

    if not records:
        raise ValueError("No simulation records to save.")

    os.makedirs(
        os.path.dirname(filename) or ".",
        exist_ok=True,
    )

    fieldnames = list(records[0].keys())

    with open(
        filename,
        "w",
        newline="",
        encoding="utf-8",
    ) as file:

        writer = csv.DictWriter(
            file,
            fieldnames=fieldnames,
        )

        writer.writeheader()
        writer.writerows(records)

    print(f"CSV saved         : {filename}")


# ============================================================================
# STEP RESPONSE METRICS
# ============================================================================

def calculate_step_metrics(records):

    if not records:
        return {}

    # --------------------------------------------------------------
    # First step: 0 -> 500 RPM
    # --------------------------------------------------------------

    first_step_records = [
        r
        for r in records
        if T_STEP_1 <= r["time_s"] < T_STEP_2
    ]

    t10_500 = None
    t90_500 = None

    for record in first_step_records:

        if (
            t10_500 is None
            and record["speed_rpm"] >= 50.0
        ):
            t10_500 = record["time_s"]

        if (
            t90_500 is None
            and record["speed_rpm"] >= 450.0
        ):
            t90_500 = record["time_s"]

    rise_500 = None

    if t10_500 is not None and t90_500 is not None:
        rise_500 = t90_500 - t10_500

    max_speed_500 = max(
        (r["speed_rpm"] for r in first_step_records),
        default=0.0,
    )

    overshoot_500 = max(
        0.0,
        (max_speed_500 - 500.0) / 500.0 * 100.0,
    )

    # --------------------------------------------------------------
    # Second step: 500 -> 1000 RPM
    # --------------------------------------------------------------

    second_step_records = [
        r
        for r in records
        if r["time_s"] >= T_STEP_2
    ]

    t10_1000 = None
    t90_1000 = None

    for record in second_step_records:

        if (
            t10_1000 is None
            and record["speed_rpm"] >= 550.0
        ):
            t10_1000 = record["time_s"]

        if (
            t90_1000 is None
            and record["speed_rpm"] >= 950.0
        ):
            t90_1000 = record["time_s"]

    rise_1000 = None

    if t10_1000 is not None and t90_1000 is not None:
        rise_1000 = t90_1000 - t10_1000

    max_speed_1000 = max(
        (r["speed_rpm"] for r in second_step_records),
        default=0.0,
    )

    overshoot_1000 = max(
        0.0,
        (max_speed_1000 - 1000.0) / 1000.0 * 100.0,
    )

    # --------------------------------------------------------------
    # Current metrics.
    # --------------------------------------------------------------

    peak_abs_iq = max(
        (abs(r["iq"]) for r in records),
        default=0.0,
    )

    peak_abs_id = max(
        (abs(r["id"]) for r in records),
        default=0.0,
    )

    min_id_ref = min(
        (r["id_ref"] for r in records),
        default=0.0,
    )

    return {
        "rise_time_0_to_500_s": rise_500,
        "overshoot_500_percent": overshoot_500,

        "rise_time_500_to_1000_s": rise_1000,
        "overshoot_1000_percent": overshoot_1000,

        "peak_abs_id_A": peak_abs_id,
        "peak_abs_iq_A": peak_abs_iq,

        "minimum_id_ref_A": min_id_ref,

        "final_speed_rpm": records[-1]["speed_rpm"],
        "final_id_A": records[-1]["id"],
        "final_iq_A": records[-1]["iq"],
    }


def print_metrics(metrics):

    print()
    print("=" * 72)
    print("GOLDEN MODEL METRICS")
    print("=" * 72)

    def fmt(value, suffix=""):
        if value is None:
            return "N/A"
        return f"{value:.6f}{suffix}"

    print(
        "0 -> 500 RPM rise time       : "
        + fmt(metrics["rise_time_0_to_500_s"], " s")
    )

    print(
        "0 -> 500 RPM overshoot       : "
        + fmt(metrics["overshoot_500_percent"], " %")
    )

    print(
        "500 -> 1000 RPM rise time    : "
        + fmt(metrics["rise_time_500_to_1000_s"], " s")
    )

    print(
        "500 -> 1000 RPM overshoot    : "
        + fmt(metrics["overshoot_1000_percent"], " %")
    )

    print(
        "Peak |Id|                    : "
        + fmt(metrics["peak_abs_id_A"], " A")
    )

    print(
        "Peak |Iq|                    : "
        + fmt(metrics["peak_abs_iq_A"], " A")
    )

    print(
        "Minimum Id reference         : "
        + fmt(metrics["minimum_id_ref_A"], " A")
    )

    print(
        "Final speed                  : "
        + fmt(metrics["final_speed_rpm"], " RPM")
    )

    print(
        "Final Id                     : "
        + fmt(metrics["final_id_A"], " A")
    )

    print(
        "Final Iq                     : "
        + fmt(metrics["final_iq_A"], " A")
    )

    print("=" * 72)


# ============================================================================
# MEASUREMENT-CHAIN TEST
# ============================================================================

def test_measurement_chain(records):

    max_id_error = 0.0
    max_iq_error = 0.0

    for record in records:

        i_alpha, i_beta = clarke(
            record["ia"],
            record["ib"],
            record["ic"],
        )

        id_reconstructed, iq_reconstructed = park(
            i_alpha,
            i_beta,
            record["theta_e"],
        )

        max_id_error = max(
            max_id_error,
            abs(id_reconstructed - record["id"]),
        )

        max_iq_error = max(
            max_iq_error,
            abs(iq_reconstructed - record["iq"]),
        )

    passed = (
        max_id_error < 1e-9
        and max_iq_error < 1e-9
    )

    if passed:
        print(
            "Measurement-chain test : PASS "
            f"(max Id error={max_id_error:.3e} A, "
            f"max Iq error={max_iq_error:.3e} A)"
        )
    else:
        print(
            "Measurement-chain test : FAIL "
            f"(max Id error={max_id_error:.3e} A, "
            f"max Iq error={max_iq_error:.3e} A)"
        )

    return passed


# ============================================================================
# SVPWM OUTPUT TEST
# ============================================================================

def test_svpwm_outputs(records):

    max_duty = 0.0
    min_duty = 1.0
    invalid_sector_count = 0

    for record in records:

        duties = (
            record["duty_a"],
            record["duty_b"],
            record["duty_c"],
        )

        max_duty = max(
            max_duty,
            max(duties),
        )

        min_duty = min(
            min_duty,
            min(duties),
        )

        if record["sector"] not in range(1, 7):
            invalid_sector_count += 1

    passed = (
        min_duty >= 0.0
        and max_duty <= 1.0
        and invalid_sector_count == 0
    )

    if passed:
        print(
            "SVPWM output test      : PASS "
            f"(duty range={min_duty:.6f}..{max_duty:.6f})"
        )
    else:
        print(
            "SVPWM output test      : FAIL "
            f"(duty range={min_duty:.6f}..{max_duty:.6f}, "
            f"invalid sectors={invalid_sector_count})"
        )

    return passed


# ============================================================================
# FIELD-WEAKENING TEST
# ============================================================================

def test_field_weakening(records):

    below_threshold = [
        r
        for r in records
        if r["time_s"] >= T_STEP_1
        and r["speed_rpm"] < FIELD_WEAKENING_START_RPM
    ]

    above_threshold = [
        r
        for r in records
        if r["speed_rpm"] > FIELD_WEAKENING_START_RPM + 20.0
    ]

    zero_id_ok = all(
        abs(r["id_ref"]) < 1e-9
        for r in below_threshold
    )

    negative_id_present = any(
        r["id_ref"] < -0.1
        for r in above_threshold
    )

    id_limit_ok = all(
        r["id_ref"] >= FIELD_WEAKENING_ID_MIN_A - 1e-9
        for r in records
    )

    passed = (
        zero_id_ok
        and negative_id_present
        and id_limit_ok
    )

    if passed:
        minimum_id_ref = min(
            r["id_ref"]
            for r in records
        )

        print(
            "Field-weakening test  : PASS "
            f"(minimum Id_ref={minimum_id_ref:.6f} A)"
        )
    else:
        print(
            "Field-weakening test  : FAIL"
        )

    return passed


# ============================================================================
# PLOTS
# ============================================================================

def make_plots(records, output_directory):

    os.makedirs(
        output_directory,
        exist_ok=True,
    )

    time_ms = [
        r["time_s"] * 1000.0
        for r in records
    ]

    speed_ref = [
        r["speed_ref_rpm"]
        for r in records
    ]

    speed = [
        r["speed_rpm"]
        for r in records
    ]

    iq_ref = [
        r["iq_ref"]
        for r in records
    ]

    iq = [
        r["iq"]
        for r in records
    ]

    id_ref = [
        r["id_ref"]
        for r in records
    ]

    id_values = [
        r["id"]
        for r in records
    ]

    duty_a = [
        r["duty_a"]
        for r in records
    ]

    duty_b = [
        r["duty_b"]
        for r in records
    ]

    duty_c = [
        r["duty_c"]
        for r in records
    ]

    # --------------------------------------------------------------
    # Speed response.
    # --------------------------------------------------------------

    plt.figure(figsize=(11, 5))

    plt.plot(
        time_ms,
        speed_ref,
        label="Speed reference",
    )

    plt.plot(
        time_ms,
        speed,
        label="Motor speed",
    )

    plt.xlabel("Time (ms)")
    plt.ylabel("Speed (RPM)")
    plt.title("PMSM FOC Golden Model - Speed Response")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    speed_filename = os.path.join(
        output_directory,
        "foc_speed_response.png",
    )

    plt.savefig(
        speed_filename,
        dpi=150,
    )

    plt.close()

    # --------------------------------------------------------------
    # Current response.
    # --------------------------------------------------------------

    plt.figure(figsize=(11, 5))

    plt.plot(
        time_ms,
        iq_ref,
        label="Iq reference",
    )

    plt.plot(
        time_ms,
        iq,
        label="Iq",
    )

    plt.plot(
        time_ms,
        id_ref,
        label="Id reference",
    )

    plt.plot(
        time_ms,
        id_values,
        label="Id",
    )

    plt.xlabel("Time (ms)")
    plt.ylabel("Current (A)")
    plt.title("PMSM FOC Golden Model - Current Response")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    current_filename = os.path.join(
        output_directory,
        "foc_current_response.png",
    )

    plt.savefig(
        current_filename,
        dpi=150,
    )

    plt.close()

    # --------------------------------------------------------------
    # PWM duties.
    # --------------------------------------------------------------

    plt.figure(figsize=(11, 5))

    plt.plot(
        time_ms,
        duty_a,
        label="Duty A",
    )

    plt.plot(
        time_ms,
        duty_b,
        label="Duty B",
    )

    plt.plot(
        time_ms,
        duty_c,
        label="Duty C",
    )

    plt.xlabel("Time (ms)")
    plt.ylabel("Duty")
    plt.title("PMSM FOC Golden Model - SVPWM Duties")
    plt.ylim(-0.05, 1.05)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    pwm_filename = os.path.join(
        output_directory,
        "foc_svpwm_duties.png",
    )

    plt.savefig(
        pwm_filename,
        dpi=150,
    )

    plt.close()

    print()
    print("Plots generated:")
    print(f"  {speed_filename}")
    print(f"  {current_filename}")
    print(f"  {pwm_filename}")


# ============================================================================
# CONFIGURATION
# ============================================================================

def print_configuration():

    print()
    print("=" * 72)
    print("FOC GOLDEN MODEL CONFIGURATION")
    print("=" * 72)

    print(f"DT                  = {DT:.8f} s")
    print(f"Control frequency   = {CONTROL_FREQ:.0f} Hz")

    print()
    print("Current base:")
    print(f"  I_BASE            = {I_BASE:.3f} A")

    print()
    print("Voltage base:")
    print(f"  V_BASE            = {V_BASE:.3f} V")

    print()
    print("Current PI:")
    print(f"  Kp                = {CURRENT_KP:.6f}")
    print(f"  Ki/update         = {CURRENT_KI:.6f}")
    print(f"  Output min        = {CURRENT_OUTPUT_MIN:.3f}")
    print(f"  Output max        = {CURRENT_OUTPUT_MAX:.3f}")

    print()
    print("Speed PI:")
    print(f"  Kp                = {SPEED_KP:.6f}")
    print(f"  Ki                = {SPEED_KI:.6f}")
    print(f"  Output min        = {SPEED_OUTPUT_MIN:.3f}")
    print(f"  Output max        = {SPEED_OUTPUT_MAX:.3f}")

    print()
    print("Field weakening:")
    print(
        f"  Start speed       = "
        f"{FIELD_WEAKENING_START_RPM:.1f} RPM"
    )
    print(
        f"  Minimum Id        = "
        f"{FIELD_WEAKENING_ID_MIN_A:.3f} A"
    )
    print(
        f"  Voltage margin    = "
        f"{FIELD_WEAKENING_VOLTAGE_MARGIN * 100.0:.1f}%"
    )

    print("=" * 72)


# ============================================================================
# MAIN
# ============================================================================

def main():

    print_configuration()

    try:
        records = run_simulation()

    except Exception as exc:
        print()
        print("=" * 72)
        print("FOC GOLDEN MODEL ERROR")
        print("=" * 72)
        print(f"{type(exc).__name__}: {exc}")
        print("=" * 72)
        raise

    print()
    print("GOLDEN MODEL VALIDATION")
    print("-" * 72)

    measurement_pass = test_measurement_chain(
        records
    )

    svpwm_pass = test_svpwm_outputs(
        records
    )

    field_weakening_pass = test_field_weakening(
        records
    )

    metrics = calculate_step_metrics(
        records
    )

    print_metrics(
        metrics
    )

    output_directory = os.path.dirname(
        os.path.abspath(__file__)
    )

    csv_filename = os.path.join(
        output_directory,
        "foc_golden_results.csv",
    )

    save_csv(
        records,
        csv_filename,
    )

    make_plots(
        records,
        output_directory,
    )

    print()
    print("=" * 72)

    if (
        measurement_pass
        and svpwm_pass
        and field_weakening_pass
    ):
        print("FOC GOLDEN MODEL : PASS")
    else:
        print("FOC GOLDEN MODEL : FAIL")

    print("=" * 72)
    print()

    return records


if __name__ == "__main__":
    main()