"""
pmsm_plant.py
=============
Golden-reference ODE plant model for a Permanent Magnet Synchronous Motor (PMSM)
operating in the synchronously rotating d-q reference frame.

This module is part of the FOC (Field-Oriented Control) motor controller
verification suite.  It provides a numerically exact (within Euler-integration
error) software model of the motor physics that the RTL controller is expected
to match.

Theory background
-----------------
A surface-mounted PMSM (Ld = Lq) in the d-q frame is described by four coupled
first-order ODEs:

  1. d-axis current:
         dId/dt = (Vd - Rs·Id + ω_e·Lq·Iq) / Ld

  2. q-axis current:
         dIq/dt = (Vq - Rs·Iq - ω_e·(Ld·Id + λm)) / Lq

  3. Mechanical speed (Newton's second law on the rotor):
         dω_mech/dt = (Te - TL) / J
     where the electromagnetic torque is
         Te = (3/2)·(P/2)·[λm·Iq + (Ld - Lq)·Id·Iq]

  4. Electrical rotor angle:
         dθ_e/dt = ω_e = (P/2)·ω_mech

Variable glossary
-----------------
  Id, Iq      d- and q-axis stator currents          [A]
  Vd, Vq      d- and q-axis stator voltages          [V]
  ω_e         electrical angular velocity             [rad/s]
  ω_mech      mechanical angular velocity             [rad/s]
  θ_e         electrical rotor angle                  [rad]
  Rs          stator resistance                       [Ω]
  Ld, Lq      d- and q-axis stator inductances        [H]
  λm          permanent-magnet flux linkage           [Wb]
  P           number of poles
  J           rotor moment of inertia                 [kg·m²]
  TL          load (friction) torque                  [N·m]
  Te          electromagnetic torque                  [N·m]

Author : Golden-reference generator
Units  : SI throughout (no per-unit normalisation)
"""

import csv
import math
import os

import numpy as np


# ---------------------------------------------------------------------------
# Motor nameplate / physical constants  (typical small surface-mounted PMSM)
# ---------------------------------------------------------------------------

RS   = 1.5      # Stator resistance                  [Ω]
LD   = 8.5e-3   # d-axis synchronous inductance       [H]
LQ   = 8.5e-3   # q-axis synchronous inductance       [H]  (= Ld for SPMSM)
LAM  = 0.175    # Permanent-magnet flux linkage       [Wb]
P    = 4        # Number of poles
J    = 0.001    # Rotor moment of inertia             [kg·m²]
TL   = 0.5      # Constant load torque                [N·m]

# Pole-pair count (appears in every torque / angle equation)
PP   = P // 2   # = 2 for a 4-pole machine


class PMSMPlant:
    """
    Discrete-time PMSM plant model using first-order (Euler-forward) integration.

    The plant accepts d-q voltage commands each control cycle, advances the
    motor ODEs by one time step, and returns the updated d-q currents,
    mechanical speed, and electrical angle.

    Parameters
    ----------
    dt : float, optional
        Integration time step in seconds.  Default is 50 µs (= 1 / 20 kHz),
        which matches a typical FOC PWM carrier frequency.

    Attributes
    ----------
    Id        : float  – d-axis current [A]
    Iq        : float  – q-axis current [A]
    omega_mech: float  – mechanical angular velocity [rad/s]
    theta_e   : float  – electrical rotor angle [rad], wrapped to [0, 2π)

    Notes
    -----
    Euler-forward integration introduces truncation error of O(dt).  For
    verification purposes this is acceptable as long as the RTL implements the
    *same* numerical scheme.  If higher accuracy is needed, replace the Euler
    update in `step()` with a 4th-order Runge-Kutta integrator.
    """

    # ------------------------------------------------------------------
    # Motor parameters exposed as class-level constants for easy access
    # ------------------------------------------------------------------
    Rs  = RS
    Ld  = LD
    Lq  = LQ
    lam = LAM   # λm
    n_p = PP    # pole pairs

    def __init__(self, dt: float = 50e-6) -> None:
        """
        Initialise the plant model.

        Parameters
        ----------
        dt : float
            Control / integration time step [s].  Must be positive.
        """
        if dt <= 0:
            raise ValueError(f"Time step dt must be positive, got {dt}")

        self.dt = dt  # [s]  integration step size

        # ---- State variables (initialised to zero = motor at standstill) ----
        self._Id         = 0.0   # d-axis current          [A]
        self._Iq         = 0.0   # q-axis current          [A]
        self._omega_mech = 0.0   # mechanical speed        [rad/s]
        self._theta_e    = 0.0   # electrical angle        [rad]

    # ------------------------------------------------------------------
    # Read-only state properties
    # ------------------------------------------------------------------

    @property
    def Id(self) -> float:
        """d-axis current [A]."""
        return self._Id

    @property
    def Iq(self) -> float:
        """q-axis current [A]."""
        return self._Iq

    @property
    def omega_mech(self) -> float:
        """Mechanical angular velocity [rad/s]."""
        return self._omega_mech

    @property
    def theta_e(self) -> float:
        """Electrical rotor angle [rad], in [0, 2π)."""
        return self._theta_e

    # ------------------------------------------------------------------
    # Public methods
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """
        Reset all state variables to zero.

        Call this before starting a new simulation episode to return the
        motor to the initial (standstill, un-magnetised) condition.
        """
        self._Id         = 0.0
        self._Iq         = 0.0
        self._omega_mech = 0.0
        self._theta_e    = 0.0

    def step(self, Vd: float, Vq: float):
        """
        Advance the motor model by one time step.

        Applies Euler-forward integration to the four PMSM ODEs using the
        supplied d-q voltage commands.

        Parameters
        ----------
        Vd : float
            d-axis voltage command [V].
        Vq : float
            q-axis voltage command [V].

        Returns
        -------
        tuple of float
            (Id, Iq, omega_mech, theta_e) after the update:
              Id         – d-axis current          [A]
              Iq         – q-axis current          [A]
              omega_mech – mechanical speed        [rad/s]
              theta_e    – electrical angle        [rad]

        Implementation notes
        --------------------
        All derivatives are evaluated at the *beginning* of the interval
        (Euler-forward), so the update reads:

            x[k+1] = x[k] + dt · f(x[k], u[k])

        This matches common RTL fixed-point implementations.
        """
        # ---- Retrieve current states ----------------------------------
        Id    = self._Id
        Iq    = self._Iq
        w_m   = self._omega_mech   # ω_mech  [rad/s]
        th_e  = self._theta_e      # θ_e     [rad]

        # ---- Derived electrical quantities ---------------------------
        # Electrical angular velocity: ω_e = (P/2) · ω_mech
        # For a 4-pole machine PP = 2, so ω_e = 2 · ω_mech.
        w_e = PP * w_m             # [rad/s]  electrical speed

        # ---- ODE 1: d-axis current -----------------------------------
        # The d-axis voltage equation (in rotor-fixed d-q frame):
        #
        #   Vd = Rs·Id + Ld·(dId/dt) - ω_e·Lq·Iq
        #
        # Solving for dId/dt:
        #   dId/dt = (Vd - Rs·Id + ω_e·Lq·Iq) / Ld
        #
        # Terms:
        #   Vd        – applied d-axis voltage
        #   Rs·Id     – resistive voltage drop on d-axis
        #  +ω_e·Lq·Iq – cross-coupling (q-axis back-EMF on d-axis)
        dId_dt = (Vd - RS * Id + w_e * LQ * Iq) / LD

        # ---- ODE 2: q-axis current -----------------------------------
        # The q-axis voltage equation:
        #
        #   Vq = Rs·Iq + Lq·(dIq/dt) + ω_e·(Ld·Id + λm)
        #
        # Solving for dIq/dt:
        #   dIq/dt = (Vq - Rs·Iq - ω_e·(Ld·Id + λm)) / Lq
        #
        # Terms:
        #   Vq              – applied q-axis voltage
        #   Rs·Iq           – resistive voltage drop on q-axis
        #  -ω_e·Ld·Id       – cross-coupling (d-axis back-EMF on q-axis)
        #  -ω_e·λm          – permanent-magnet back-EMF (main torque-producing term)
        dIq_dt = (Vq - RS * Iq - w_e * (LD * Id + LAM)) / LQ

        # ---- Electromagnetic torque ----------------------------------
        # General expression for a salient-pole PMSM:
        #
        #   Te = (3/2)·(P/2)·[λm·Iq + (Ld - Lq)·Id·Iq]
        #
        # For a surface PMSM (Ld = Lq) the reluctance term (Ld-Lq)·Id·Iq = 0,
        # leaving only the magnet torque λm·Iq.  The term is kept in full for
        # generality (and to handle slight numerical asymmetry).
        Te = 1.5 * PP * (LAM * Iq + (LD - LQ) * Id * Iq)   # [N·m]

        # ---- ODE 3: mechanical speed ---------------------------------
        # Newton's second law on the rotor shaft:
        #
        #   J·(dω_mech/dt) = Te - TL
        #
        # where TL is the (constant) load torque opposing rotation.
        # dω_mech/dt = (Te - TL) / J
        dw_m_dt = (Te - TL) / J

        # ---- ODE 4: electrical rotor angle ---------------------------
        # The electrical angle advances at the electrical speed:
        #
        #   dθ_e/dt = ω_e = (P/2)·ω_mech
        #
        # (Already computed as w_e above; written again for clarity.)
        dth_e_dt = w_e                                       # [rad/s]

        # ---- Euler-forward integration --------------------------------
        # x[k+1] = x[k] + dt · dx/dt|[k]
        self._Id         = Id   + self.dt * dId_dt
        self._Iq         = Iq   + self.dt * dIq_dt
        self._omega_mech = w_m  + self.dt * dw_m_dt
        # Wrap θ_e to [0, 2π) to prevent unbounded growth
        self._theta_e    = (th_e + self.dt * dth_e_dt) % (2.0 * math.pi)

        return (self._Id, self._Iq, self._omega_mech, self._theta_e)


# ---------------------------------------------------------------------------
# Open-loop smoke test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    """
    Simple open-loop validation of the PMSMPlant model.

    Applies constant voltages Vd = 0 V, Vq = 10 V for 0.1 s and records the
    motor response.  This exercises the start-up acceleration phase where the
    q-axis voltage drives current and torque to overcome the load.

    Output
    ------
    * Console: final Id, Iq, omega_mech values.
    * File   : golden/pmsm_plant_openloop.csv
                 Columns: t, Vd, Vq, Id, Iq, omega_mech, theta_e
    """

    # ---- Simulation parameters ----------------------------------------
    DT          = 50e-6     # 50 µs  (20 kHz control rate)
    T_END       = 0.1       # 100 ms simulation duration   [s]
    VD_CMD      = 0.0       # d-axis voltage command        [V]
    VQ_CMD      = 10.0      # q-axis voltage command        [V]

    N_STEPS     = int(round(T_END / DT))

    # ---- Create plant ----------------------------------------------------
    plant = PMSMPlant(dt=DT)
    plant.reset()

    # ---- Storage for CSV output ------------------------------------------
    # Pre-allocate numpy arrays for efficiency
    t_arr     = np.empty(N_STEPS + 1)
    Id_arr    = np.empty(N_STEPS + 1)
    Iq_arr    = np.empty(N_STEPS + 1)
    wm_arr    = np.empty(N_STEPS + 1)
    the_arr   = np.empty(N_STEPS + 1)
    Vd_arr    = np.full(N_STEPS + 1, VD_CMD)
    Vq_arr    = np.full(N_STEPS + 1, VQ_CMD)

    # Record initial (t = 0) state before any stepping
    t_arr[0]   = 0.0
    Id_arr[0]  = plant.Id
    Iq_arr[0]  = plant.Iq
    wm_arr[0]  = plant.omega_mech
    the_arr[0] = plant.theta_e

    # ---- Run simulation --------------------------------------------------
    print("Running open-loop PMSM simulation ...")
    print(f"  Vd = {VD_CMD} V, Vq = {VQ_CMD} V, duration = {T_END*1e3:.1f} ms")
    print(f"  Steps = {N_STEPS:,},  dt = {DT*1e6:.1f} µs")
    print()

    for k in range(N_STEPS):
        t = (k + 1) * DT                           # time after this step
        Id, Iq, wm, th_e = plant.step(VD_CMD, VQ_CMD)

        t_arr[k + 1]   = t
        Id_arr[k + 1]  = Id
        Iq_arr[k + 1]  = Iq
        wm_arr[k + 1]  = wm
        the_arr[k + 1] = th_e

    # ---- Print final state -----------------------------------------------
    print("=== Final state (t = {:.4f} s) ===".format(T_END))
    print(f"  Id         = {plant.Id:+.6f}  A")
    print(f"  Iq         = {plant.Iq:+.6f}  A")
    print(f"  omega_mech = {plant.omega_mech:+.4f}  rad/s"
          f"  ({plant.omega_mech * 60 / (2 * math.pi):.2f} RPM)")
    print(f"  theta_e    = {plant.theta_e:.6f}  rad")
    print()

    # ---- Save CSV --------------------------------------------------------
    # Determine output path relative to *this* file so the CSV lands in
    # foc_motor_ctrl/golden/ regardless of the working directory.
    script_dir  = os.path.dirname(os.path.abspath(__file__))
    csv_path    = os.path.join(script_dir, "pmsm_plant_openloop.csv")

    print(f"Saving results to: {csv_path}")

    with open(csv_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        # Header row
        writer.writerow(["t", "Vd", "Vq", "Id", "Iq", "omega_mech", "theta_e"])
        # Data rows — iterate over all recorded time steps
        for k in range(N_STEPS + 1):
            writer.writerow([
                f"{t_arr[k]:.9f}",
                f"{Vd_arr[k]:.6f}",
                f"{Vq_arr[k]:.6f}",
                f"{Id_arr[k]:.9f}",
                f"{Iq_arr[k]:.9f}",
                f"{wm_arr[k]:.9f}",
                f"{the_arr[k]:.9f}",
            ])

    print("Done.")
