"""Bang-coast-bang landing guidance (1D vertical).

Phase 1: free fall (T=0). Phase 2: full thrust (T_max) until touchdown.
Ignition altitude: h = -(vz*t + 0.5*a_net*t²), where t = (vz_target - vz) / a_net.
"""

import math
import logging


def computeIgnitionAltitude(
    vz: float,
    mass: float,
    T_max: float,
    g: float,
    vz_target: float = -5.0,
) -> dict:
    """Compute minimum ignition altitude for braking at full thrust T_max."""
    a_net = T_max / mass - g

    if a_net <= 0:
        return {"feasible": False, "h_ignite": None, "t_burn": None, "a_net": a_net}

    delta_v = vz_target - vz

    if delta_v <= 0:
        return {"feasible": True, "h_ignite": 0.0, "t_burn": 0.0, "a_net": a_net}

    t_burn = delta_v / a_net
    delta_h = vz * t_burn + 0.5 * a_net * t_burn**2
    h_ignite = -delta_h

    return {
        "feasible": h_ignite >= 0,
        "h_ignite": max(h_ignite, 0.0),
        "t_burn": t_burn,
        "a_net": a_net,
    }


def shouldStartLanding(
    h: float,
    vz: float,
    mass: float,
    T_max: float,
    g: float,
    vz_target: float = -5.0,
    safety_margin: float = 1.05,
    bellyflop_time: float = 1.5,
    drag_acc: float = 0.0,
) -> dict:
    """Determine whether to start the bellyflop and engine ignition sequence."""
    state = computeIgnitionAltitude(vz, mass, T_max, g, vz_target)

    if not state["feasible"]:
        logging.warning(f"Landing infeasible — a_net={state['a_net']:.2f} m/s² (insufficient thrust)")
        return {
            "bellyflop_now": True,
            "h_ignite": 0.0,
            "h_bellyflop": 0.0,
            "margin": 0.0,
            "t_burn": None,
        }

    h_ignite = state["h_ignite"] * safety_margin

    # Altitude lost during bellyflop rotation; drag reduces effective downward acceleration.
    a_eff = max(0.0, g - drag_acc)
    h_lost_during_flip = abs(vz) * bellyflop_time + 0.5 * a_eff * bellyflop_time**2
    h_bellyflop = h_ignite + h_lost_during_flip

    margin = h / h_ignite if h_ignite > 0 else math.inf

    return {
        "bellyflop_now": h <= h_bellyflop,
        "h_ignite": h_ignite,
        "h_bellyflop": h_bellyflop,
        "margin": margin,
        "t_burn": state["t_burn"],
    }
