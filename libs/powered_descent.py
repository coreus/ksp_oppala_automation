"""
Guidance bang-coast-bang pour le déclenchement de la descente propulsée.

Stratégie optimale en carburant (1D vertical) :
  Phase 1 — chute libre (coast, T=0)
  Phase 2 — pleine poussée (T_max) jusqu'au touchdown

La question clé : à quelle altitude démarrer la phase 2 ?

Formulation 1D (axe vertical) :
  - a_net  = T_max/m − g          (décélération nette en pleine poussée)
  - t_burn = (vz_cible − vz) / a_net
  - h_igni = −(vz·t_burn + ½·a_net·t_burn²)

Le bellyflop doit commencer avant l'allumage : on ajoute la chute libre
estimée pendant la durée de la rotation.
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
    """
    Altitude minimale pour démarrer la phase de freinage *maintenant*,
    en supposant une poussée constante à T_max.

    Paramètres
    ----------
    vz        : vitesse verticale actuelle (m/s, négatif = descente)
    mass      : masse du vaisseau (kg)
    T_max     : poussée maximale (N)
    g         : gravité de surface (m/s², valeur positive)
    vz_target : vitesse cible au toucher (m/s, ex. -5)

    Retour
    ------
    dict :
        feasible  (bool)  — la poussée est suffisante
        h_ignite  (float) — altitude minimale d'allumage (m)
        t_burn    (float) — durée du freinage (s)
        a_net     (float) — décélération nette (m/s²)
    """
    a_net = T_max / mass - g

    if a_net <= 0:
        return {"feasible": False, "h_ignite": None, "t_burn": None, "a_net": a_net}

    delta_v = vz_target - vz  # positif : on doit ralentir

    if delta_v <= 0:
        return {"feasible": True, "h_ignite": 0.0, "t_burn": 0.0, "a_net": a_net}

    t_burn = delta_v / a_net
    # Variation d'altitude pendant le freinage (négative : on descend encore)
    delta_h = vz * t_burn + 0.5 * a_net * t_burn**2
    h_ignite = -delta_h  # toujours positif si delta_v > 0 et a_net > 0

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
    """
    Détermine si le bellyflop (et l'allumage moteur) doit commencer.

    Paramètres
    ----------
    h              : altitude actuelle (m)
    vz             : vitesse verticale (m/s, négatif = descente)
    mass           : masse du vaisseau (kg)
    T_max          : poussée maximale (N)
    g              : gravité de surface (m/s²)
    vz_target      : vitesse cible au toucher (m/s)
    safety_margin  : marge de sécurité sur h_ignite (ex. 1.05 = +5 %)
    bellyflop_time : durée estimée de la rotation bellyflop (s) — ~3 s sur KSP
    drag_acc       : décélération aérodynamique actuelle = |F_drag| / m  (m/s²).
                     Réduit l'accélération effective pendant le flip :
                     a_eff = max(0, g − drag_acc).

    Retour
    ------
    dict :
        bellyflop_now  (bool)  — démarrer le bellyflop maintenant
        h_ignite       (float) — altitude d'allumage avec marge (m)
        h_bellyflop    (float) — altitude de déclenchement du bellyflop (m)
        margin         (float) — h / h_ignite  (> 1 = encore au-dessus)
        t_burn         (float) — durée du freinage (s)
    """
    state = computeIgnitionAltitude(vz, mass, T_max, g, vz_target)

    if not state["feasible"]:
        logging.warning(
            f"GFOLD atterrissage infaisable — "
            f"a_net={state['a_net']:.2f} m/s² (poussée insuffisante)"
        )
        return {
            "bellyflop_now": True,
            "h_ignite": 0.0,
            "h_bellyflop": 0.0,
            "margin": 0.0,
            "t_burn": None,
        }

    h_ignite = state["h_ignite"] * safety_margin

    # Altitude perdue pendant le bellyflop.
    # L'accélération effective vers le bas est réduite par la traînée :
    #   a_eff = max(0, g − drag_acc)
    # La vitesse initiale |vz| est aussi freinée, mais on l'intègre
    # de façon conservative (on garde |vz|·t comme borne haute).
    a_eff = max(0.0, g - drag_acc)
    h_lost_during_flip = abs(vz) * bellyflop_time + 0.5 * a_eff * bellyflop_time**2  #on est à vitesse terminale donc a_eff proche de 0
    h_bellyflop = h_ignite + h_lost_during_flip

    margin = h / h_ignite if h_ignite > 0 else math.inf

    return {
        "bellyflop_now": h <= h_bellyflop,
        "h_ignite": h_ignite,
        "h_bellyflop": h_bellyflop,
        "margin": margin,
        "t_burn": state["t_burn"],
    }
