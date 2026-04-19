"""
Guidance cinématique pour le déclenchement du bellyflop.

Équations de mouvement originales (code de base) :
    a_net        = T_max / m − g
    h_anticipated = h − 2.5 × speed
    v_safe²     = 2 × h_anticipated × a_net
    trigger      = speed > v_safe  AND  h_anticipated < 10000

Même interface que powered_descent.shouldStartLanding pour pouvoir les interchanger
en changeant uniquement l'import dans flightplan.py.
"""

import math


def shouldStartLanding(
    h: float,
    vz: float,
    mass: float,
    T_max: float,
    g: float,
    speed: float = None,
    vz_target: float = -5.0,
    anticipation_factor: float = 2.5,
    max_altitude: float = 10000.0,
    min_altitude: float = 0.0,
    drag_acc: float = 0.0,
) -> dict:
    """
    Détermine si le bellyflop doit commencer selon les équations cinématiques
    originales.

    Paramètres
    ----------
    h                  : altitude actuelle (m)
    vz                 : vitesse verticale (m/s, négatif = descente)
    mass               : masse du vaisseau (kg)
    T_max              : poussée maximale (N)
    g                  : gravité de surface (m/s²)
    speed              : vitesse totale scalaire (m/s). Si None, utilise abs(vz).
    vz_target          : non utilisé, conservé pour compatibilité d'interface
    anticipation_factor: facteur k sur speed pour anticiper la descente (défaut 2.5)
    max_altitude       : altitude max au-delà de laquelle on n'allume pas
    min_altitude       : altitude de sécurité absolue (allumage forcé en dessous)
    drag_acc           : non utilisé, conservé pour compatibilité d'interface

    Retour
    ------
    dict :
        bellyflop_now  (bool)  — démarrer le bellyflop maintenant
        h_ignite       (float) — altitude anticipée utilisée pour le calcul (m)
        h_bellyflop    (float) — identique à h_ignite
        margin         (float) — v_safe / speed  (> 1 = encore en sécurité)
        t_burn         (float) — None (non calculé dans cette approche)
    """
    if speed is None:
        speed = abs(vz)

    a_net = T_max / mass - g

    if a_net <= 0:
        return {
            "bellyflop_now": True,
            "h_ignite": 0.0,
            "h_bellyflop": 0.0,
            "margin": 0.0,
            "t_burn": None,
        }

    h_anticipated = h - anticipation_factor * speed
    square_speed = 2.0 * h_anticipated * a_net

    if square_speed < 0:
        bellyflop_now = True
        v_safe = 0.0
    elif h < min_altitude:
        bellyflop_now = True
        v_safe = math.sqrt(square_speed)
    else:
        v_safe = math.sqrt(square_speed)
        bellyflop_now = (speed > v_safe and h_anticipated < max_altitude)

    margin = v_safe / speed if speed > 0 else math.inf

    return {
        "bellyflop_now": bellyflop_now,
        "h_ignite": max(h_anticipated, 0.0),
        "h_bellyflop": max(h_anticipated, 0.0),
        "margin": margin,
        "t_burn": None,
    }
