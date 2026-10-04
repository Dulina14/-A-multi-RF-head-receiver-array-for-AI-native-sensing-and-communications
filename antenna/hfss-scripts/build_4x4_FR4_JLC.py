# -*- coding: utf-8 -*-
"""HFSS 4x4 FR-4 array: measured-impedance wide-band matching candidate.

Corrected build script.  Two defects in the previous revision are fixed:

  Problem 1  Copper cracks at  Q(70.71 ohm) -> Q50 taper -> 50 ohm  interfaces.
             Adjacent solids met on a single mathematical plane (zero overlap
             area), so Unite depended on exact face coincidence.  Every copper
             piece is now extended by CONNECT_OVERLAP_MM past each interior
             joint, so each joint has a real overlap volume before Unite.
             The overlap is NOT added to any electrical length.

  Problem 2  The stage-2 clearance route left the feed bus too close to the
             inner patches of the row below.  The 45 degree rise now starts
             immediately at the end of the stage-2 quarter-wave section (the
             Q50 taper sits on the diagonal instead of on the bus), and the
             pitch is increased by the minimum amount that satisfies
             MIN_FEED_PATCH_CLEARANCE_MM at the binding corner.

  Flat junctions  The round tee discs (radius W50/2) stuck out past the
             narrower W70 arms as half-circle bumps at every branch point.
             They are replaced by flat rectangles that sit fully inside the
             T (FLAT_TEES), and the stage-2 bend discs by flat chamfer
             triangles (FLAT_BEND_FILL).  Line lengths, widths, tapers and
             the path/phase budget are unchanged.

  JLCPCB build  (1) HFSS substrate er = 4.5 (JLC's value) while the copper
             keeps the verified er = 4.4 sizing; (2) all copper pulled back
             0.2 mm from the board edge, lumped port moved onto the new end of
             the input line; (3) top ground pads + vias for an edge-mount SMA;
             (4) four M3 corner mounting holes; (5) JLC-ready Gerber and drill
             files written from the same geometry.

Run inside AEDT 2018 or AEDT Student 2025 R2: Tools > Run Script.
Gerbers only (no HFSS), from a normal Python 3 or 2 prompt:

    python build_4x4_FR4_JLC.py --gerber

writes JLC_Gerbers/ and Array4x4_5p85GHz_JLC_Gerbers.zip beside the script.
Native IronPython/AEDT COM API.  PyAEDT is not required, no third-party
packages are used.  A NEW uniquely named project is created; existing
projects are untouched.  No solve is started.

This candidate preserves the original radiator and corporate tree. Its one
input stub is moved along the straight 50-ohm line and points into the open
lower board area. Nominal stubD and stubL were selected by de-embedding the
measured HFSS port impedance and minimizing the worst predicted return loss
over 5.7-6.0 GHz. The estimate is approximate and must be verified by HFSS.
It cannot guarantee
agreement with a fabricated board: actual FR-4 properties, copper roughness,
manufacturing tolerance and the SMA launch must be measured or modelled.

The geometry planner also runs under ordinary CPython without HFSS:

    python build_4x4_FR4_band_candidate.py --preview

which writes the geometry JSON, the annotated SVG preview and the full
numerical report beside this script.

All lengths are millimetres.  Every dimension below comes from impedance
matching, a guided quarter wavelength, the patch pitch, board geometry or a
symmetry constraint.  Nothing is scaled from image pixels.

IMPORTANT: this is an analytically corrected STARTING geometry.  No claim is
made about S11, gain, coupling or efficiency until HFSS has solved it.
"""
from __future__ import division, print_function
import json
import math
import os
import sys
import time
import traceback

# ----------------------------------------------------------------------------
# 1. USER SETTINGS
# ----------------------------------------------------------------------------
F0_GHZ = 5.85                 # design centre frequency
BAND_GHZ = (5.7, 6.0)         # band of interest
ER = 4.4                      # FR-4 permittivity used to SIZE the copper (keep:
                              # this is the geometry you already verified)
# JLCPCB states er = 4.5 for its 2-layer FR-4.  This value is used ONLY for the
# HFSS substrate material, so the simulation shows how the SAME copper behaves
# on the real JLC board.  Set it back to ER to reproduce the old result.
ER_BOARD = 4.5
TAND = 0.02                   # FR-4 loss tangent
H_MM = 1.6                    # substrate thickness
COPPER_MM = 0.035             # 1 oz copper, explicitly modelled
COPPER_CONDUCTIVITY_S_PER_M = 58000000.0
COPPER_MATERIAL_NAME = "Copper_5p8e7"
C_MM_GHZ = 299.792458         # speed of light, mm*GHz

# --- Full-wave calibration from the supplied 4x4 HFSS result ---------------
# The existing array has Lp = 10.94947009 mm and its well-matched lower S11
# minimum is at approximately 5.55 GHz.  First-order resonant-length scaling
# therefore gives 10.94947009*(5.55/5.85) = 10.38795880 mm for 5.85 GHz.
# Only this resonant patch dimension is scaled; the verified array pitch and
# top-layer routing envelope are deliberately kept unchanged below.
LP_BASE_MM = 10.38795880
# LP_BASE_MM = 10.94947009
MEASURED_RESONANCE_GHZ = 5.55

# The solved design shown in the supplied screenshots uses WpT = 1.0 mm.
# On this 1.6 mm FR-4 stackup that width is about 87.465 ohms, so the matched
# real edge load is Zpt^2/50 = 153.002 ohms.  Using that value makes the script
# regenerate the same 1.0 mm transformer width instead of 0.3215 mm.
EDGE_R_OHM = 153.00215359680516

# Preserve the accepted physical array placement while tuning resonance.
LOCK_ARRAY_ENVELOPE = True
LOCKED_DX_MM = 40.25491051
LOCKED_DY_MM = 35.86851091
LOCKED_BOARD_X_MM = 161.0
LOCKED_BOARD_Y_MM = 152.0

# --- Measured-array input match ---------------------------------------------
# Baseline full-wave port impedance has been de-embedded approximately through
# its existing stub. A new position and length were selected by a minimax
# 5.7-6.0 GHz circuit sweep. HFSS must confirm the candidate.
ENABLE_ARRAY_INPUT_STUB = True
MEASURED_ARRAY_R_OHM = 37.43
MEASURED_ARRAY_X_OHM = -34.03
MATCH_STUB_FROM_PORT_MM = 21.3
MATCH_STUB_ELECTRICAL_MM = 3.04639353
MATCH_STUB_OPEN_END_MM = 0.62575602
# Band-coverage candidate. Keep the old 7.6861672/3.05 mm design as baseline.
MATCH_STUB_PHYSICAL_MM = 2.8
STUB_DIRECTION = -1.0  # Away from the upper patch row at the new tee location.

# Dispersion.  Kirschning-Jansen is applied to every line at F0_GHZ.  Set
# USE_DISPERSIVE_QUARTER_WAVE = False to fall back on the quasi-static values.
USE_DISPERSIVE_QUARTER_WAVE = True

# Feed rise: vertical run of the patch transformer into the radiating edge.
# Kept at 3.0 mm because the user's verified single-patch model used exactly
# this geometry (horizontal arm + 3 mm rise); changing it would invalidate the
# tuned Lp.
FEED_RISE_MM = 3.0

# --- Problem 1 ---------------------------------------------------------------
# Controlled CAD overlap at every interior copper joint.  0.05 mm is 0.7 % of a
# quarter-wave section and ~1/32 of the narrowest trace, i.e. electrically
# insignificant, but it gives Unite a real intersection volume.
CONNECT_OVERLAP_MM = 0.05
# A joint where the two pieces run in different directions leaves a wedge of
# bare substrate on the OUTSIDE of the turn that no amount of collinear
# overlap can close, because the two end faces are not parallel.  That wedge
# is the notch still visible at the four stage-2 bends.  A copper disc at the
# corner closes it and rounds the outside of the bend at the same time.
CORNER_FILL = True
CORNER_FILL_FACTOR = 0.62      # disc radius / widest trace at that corner

# --- Problem 2 ---------------------------------------------------------------
# Minimum copper-edge to copper-edge distance between any feed conductor and a
# patch it does not feed.  5.0 mm = 3.1 * h = 0.098 * lambda0.  Microstrip
# fringing on a 1.6 mm substrate is essentially spent by ~3h, and 0.1 lambda0
# is the usual spacing at which edge-to-edge coupling between coplanar
# printed conductors drops below about -30 dB.  Parameterised: raise it if the
# HFSS coupling check is unsatisfactory.
MIN_FEED_PATCH_CLEARANCE_MM = 5.0
# Minimum spacing between feed conductors that are not meant to touch.
MIN_FEED_FEED_CLEARANCE_MM = 3.0
# Board edge margin, copper to substrate edge.
BOARD_MARGIN_MM = 12.0
# Pitch floor.  The solver never returns less than this.
DX_MIN_MM = 38.386399606
DY_MIN_MM = 34.0
# Upper guard: 0.85*lambda0.  Broadside grating lobes need d >= lambda0, but
# sidelobe growth is not worth the routing convenience beyond this.
PITCH_MAX_FRACTION_LAMBDA0 = 0.85

# Routing constants
INPUT_Y_MM = 0.0              # common input bus runs along the array centreline
TEE_APPROACH_MM = 4.0         # straight landing before every rerouted tee
Q50_TAPER_MM = 3.0            # linear taper, 70.71 ohm width -> 50 ohm width
FIFTY_TO_PATCH_TAPER_MM = 3.0 # linear taper, 50 ohm width -> patch-transformer width
ROUND_JUNCTIONS = True
# Tee discs are inscribed in the 50 ohm trace width so they round the junction
# without adding copper outside the nominal conductor outline.
JUNCTION_RADIUS_FACTOR = 0.5

# --- Flat junctions (removes the half-circle bumps) --------------------------
# The old tee disc had radius W50/2, but the two quarter-wave arms leaving a
# tee are only W70 wide, so half of the disc stuck out past the arms as a
# half-circle bump at every branch point.
#   FLAT_TEES = True    -> each tee disc is replaced by a flat rectangle that
#                          lies completely inside the T (width W50 along the
#                          arms, W70 across them).  The tee is a clean T with
#                          straight edges.
#   FLAT_BEND_FILL=True -> the 0.62*W70 disc at each stage-2 bend (it bulged
#                          slightly past the trace edge) is replaced by a flat
#                          triangle that closes the outside of the bend with a
#                          straight chamfer.
# Set both to False to get the original rounded geometry back.
# The lens-shaped widenings on the lines are NOT changed: they are the
# 70.7 ohm -> 50 ohm -> patch-transformer impedance tapers of the feed.
FLAT_TEES = True
FLAT_BEND_FILL = True

# --- JLCPCB manufacturing settings ------------------------------------------
# Copper pull-back: JLC needs >= 0.2 mm between copper and the board edge.
# The ground plane is pulled back on all four sides, and the 50 ohm input line
# now stops this far from the right-hand edge.  The HFSS lumped port moves
# with it (it sits on the new end of the line).  The matching stub keeps its
# old physical position, measured from the board edge.
EDGE_PULLBACK_MM = 0.2

# Top-side ground pads for an edge-mount (end-launch) SMA connector.  Many
# edge SMAs have two legs on the top side and two on the bottom.  The top legs
# are soldered to these pads; vias (plated holes) join the pads to the bottom
# ground plane.  CHECK THE DIMENSIONS AGAINST YOUR CONNECTOR'S DATASHEET: the
# top legs must land on the pads.  Set False if your connector has no top
# legs (or you will cut them off); the bottom legs then solder to the ground.
ENABLE_SMA_GROUND_PADS = True
SMA_PAD_LENGTH_MM = 4.0       # along the input line, from the pulled-back edge
SMA_PAD_WIDTH_MM = 2.0        # across the line
SMA_PAD_GAP_MM = 1.0          # gap between the 50 ohm line edge and each pad
SMA_VIA_DRILL_MM = 0.4        # JLC standard via drill
SMA_VIAS_PER_PAD = 2

# Corner mounting holes: non-plated (NPTH), sized for M3 screws.  The ground
# plane is cleared in a ring around each hole so a screw cannot short to it.
ENABLE_MOUNT_HOLES = True
MOUNT_HOLE_DRILL_MM = 3.2     # M3 clearance hole
MOUNT_HOLE_EDGE_MM = 5.0      # hole centre to each of the two nearest edges
MOUNT_HOLE_COPPER_CLEAR_MM = 0.5   # copper-free ring width around each hole

# Gerber output (JLC-ready).  Written by  python <this script> --gerber  and,
# when WRITE_GERBERS_WITH_HFSS_BUILD is True, also by the HFSS run.
GERBER_FOLDER_NAME = "JLC_Gerbers"
GERBER_BASENAME = "Array4x4_5p85GHz"
WRITE_GERBERS_WITH_HFSS_BUILD = True
TOP_SOLDER_MASK_OFF = True    # no mask on the antenna side (HFSS has no mask)
JLC_ORDER_NUMBER_ON_BOTTOM = True  # prints JLCJLCJLCJLC on the bottom so JLC
                                   # puts its order number there

AIR_PAD_MM = 16.0             # > lambda0/4 = 14.14 mm at 5.3 GHz sweep start
BUILD_DESIGNS = ("Array4x4_FR4_BandCandidate",)
UNITE_TOP_COPPER = True
OUTPUT_DIR = None
VALIDATE_AFTER_BUILD = True
STRICT = True                 # abort the build if any geometry check fails

# Cross-version comparison: retain the existing 5.3-6.4 GHz interpolating
# overview and add full-wave points through the sensitive 5.88 GHz dip.
ADAPTIVE_MAX_DELTA_S = 0.02
ADAPTIVE_MAX_PASSES = 18

# --- Ansys Student mesh limit (64,000 tetrahedra) ---------------------------
# With True, the HFSS model stays as close as possible to the version that
# solved before; the manufacturing-only details go to the Gerbers only:
#   * ground plane and input line reach the board edge in HFSS (the 0.2 mm
#     pull-back is kept in the Gerbers) and the port sits at the board edge
#   * mounting holes, SMA ground pads and vias are NOT drawn in HFSS
#   * adaptive passes are capped so the mesh cannot pass the student limit;
#     if Delta S has not reached its target by then, HFSS still finishes
#     the frequency sweeps (with a "not converged" warning) instead of
#     stopping with an error.
# Set False with a full (non-student) licence to simulate every detail.
STUDENT_MESH_SAFE = True
STUDENT_MAX_PASSES = 14
CHECK_SWEEP_START_GHZ = 5.7
CHECK_SWEEP_STOP_GHZ = 6.0
CHECK_SWEEP_STEP_GHZ = 0.025


# ----------------------------------------------------------------------------
# 2. MICROSTRIP SYNTHESIS AND ANALYSIS
# ----------------------------------------------------------------------------
def eeff_static(w, h=H_MM, er=ER):
    """Hammerstad quasi-static effective permittivity, zero metal thickness."""
    u = w / h
    correction = 0.04 * (1.0 - u) ** 2 if u < 1.0 else 0.0
    return (er + 1) / 2.0 + (er - 1) / 2.0 * (1.0 / math.sqrt(1 + 12.0 / u) + correction)


def z0_static(w, h=H_MM, er=ER):
    """Hammerstad-Jensen characteristic impedance, zero metal thickness."""
    u, ee = w / h, eeff_static(w, h, er)
    if u <= 1.0:
        return 60.0 / math.sqrt(ee) * math.log(8.0 / u + u / 4.0)
    return 120.0 * math.pi / (math.sqrt(ee) * (u + 1.393 + 0.667 * math.log(u + 1.444)))


def eeff_dispersive(w, f=F0_GHZ, h=H_MM, er=ER):
    """Kirschning-Jansen microstrip dispersion, f in GHz, h and w in mm."""
    u = w / h
    ee0 = eeff_static(w, h, er)
    fn = f * h                                   # GHz*mm
    p1 = (0.27488 + (0.6315 + 0.525 / (1 + 0.0157 * fn) ** 20) * u
          - 0.065683 * math.exp(-8.7513 * u))
    p2 = 0.33622 * (1 - math.exp(-0.03442 * er))
    p3 = 0.0363 * math.exp(-4.6 * u) * (1 - math.exp(-(fn / 38.7) ** 4.97))
    p4 = 1 + 2.751 * (1 - math.exp(-(er / 15.916) ** 8))
    p = p1 * p2 * ((0.1844 + p3 * p4) * fn) ** 1.5763
    return er - (er - ee0) / (1 + p)


def width_for_impedance(target, h=H_MM, er=ER):
    """Numerically invert z0_static() for the width giving `target` ohms."""
    low, high = 0.001 * h, 100.0 * h
    for _unused in range(200):
        mid = 0.5 * (low + high)
        if z0_static(mid, h, er) > target:
            low = mid
        else:
            high = mid
    return 0.5 * (low + high)


def line_properties(target_z, f=F0_GHZ, h=H_MM, er=ER, width=None):
    """Full description of one microstrip line: width, Z0, eeff, lambda_g."""
    w = width_for_impedance(target_z, h, er) if width is None else width
    ee_s = eeff_static(w, h, er)
    ee_d = eeff_dispersive(w, f, h, er)
    lam0 = C_MM_GHZ / f
    ee_use = ee_d if USE_DISPERSIVE_QUARTER_WAVE else ee_s
    return {"Z_target_ohm": target_z, "W_mm": w, "Z_actual_ohm": z0_static(w, h, er),
            "eeff_static": ee_s, "eeff_dispersive": ee_d, "eeff_used": ee_use,
            "lambda0_mm": lam0,
            "lambda_g_static_mm": lam0 / math.sqrt(ee_s),
            "lambda_g_mm": lam0 / math.sqrt(ee_use),
            "Lquarter_static_mm": lam0 / (4 * math.sqrt(ee_s)),
            "Lquarter_mm": lam0 / (4 * math.sqrt(ee_use))}


def patch_edge_conductance(w_mm, l_mm, f=F0_GHZ):
    """Balanis self and mutual slot conductance; lossless, for comparison."""
    lam0 = C_MM_GHZ / f
    k0 = 2 * math.pi / lam0
    steps = 4000

    def integrand(theta, mutual):
        s = math.sin(theta)
        c = math.cos(theta)
        if abs(c) < 1e-9:
            base = (k0 * w_mm / 2.0) ** 2
        else:
            base = (math.sin(k0 * w_mm / 2.0 * c) / c) ** 2
        val = base * s ** 3
        if mutual:
            val *= bessel_j0(k0 * l_mm * s)
        return val

    def integrate(mutual):
        total = 0.0
        for i in range(steps):
            a = math.pi * i / steps
            b = math.pi * (i + 1) / steps
            m = 0.5 * (a + b)
            total += (integrand(a, mutual) + 4 * integrand(m, mutual)
                      + integrand(b, mutual)) * (b - a) / 6.0
        return total

    g1 = integrate(False) / (120.0 * math.pi ** 2)
    g12 = integrate(True) / (120.0 * math.pi ** 2)
    return g1, g12, 1.0 / (2.0 * (g1 + g12))


def bessel_j0(x):
    """Abramowitz-Stegun polynomial approximation to J0, adequate here."""
    ax = abs(x)
    if ax < 8.0:
        y = x * x
        num = (57568490574.0 + y * (-13362590354.0 + y * (651619640.7 + y *
               (-11214424.18 + y * (77392.33017 + y * (-184.9052456))))))
        den = (57568490411.0 + y * (1029532985.0 + y * (9494680.718 + y *
               (59272.64853 + y * (267.8532712 + y)))))
        return num / den
    z = 8.0 / ax
    y = z * z
    xx = ax - 0.785398164
    p0 = (1.0 + y * (-0.1098628627e-2 + y * (0.2734510407e-4 + y *
          (-0.2073370639e-5 + y * 0.2093887211e-6))))
    q0 = (-0.1562499995e-1 + y * (0.1430488765e-3 + y * (-0.6911147651e-5 + y *
          (0.7621095161e-6 + y * (-0.934935152e-7)))))
    return math.sqrt(0.636619772 / ax) * (math.cos(xx) * p0 - z * math.sin(xx) * q0)


def synthesize():
    """All electrical quantities the layout needs, derived from first principles."""
    lam0 = C_MM_GHZ / F0_GHZ
    # --- radiating element ---------------------------------------------------
    wp_analytic = C_MM_GHZ / (2 * F0_GHZ) * math.sqrt(2.0 / (ER + 1))
    ep = eeff_static(wp_analytic)
    dl = (0.412 * H_MM * (ep + 0.3) * (wp_analytic / H_MM + 0.264) /
          ((ep - 0.258) * (wp_analytic / H_MM + 0.8)))
    lp_analytic = C_MM_GHZ / (2 * F0_GHZ * math.sqrt(ep)) - 2 * dl

    # Wp matches the analytic value to 4 decimal places and is kept.
    # Lp is rescaled from the measured array resonance (see MEASURED_RESONANCE_GHZ).
    wp = 15.594
    lp = LP_BASE_MM * MEASURED_RESONANCE_GHZ / F0_GHZ

    g1, g12, r_edge_analytic = patch_edge_conductance(wp, lp)

    s = {"lambda0_mm": lam0,
         "Wp_analytic_mm": wp_analytic, "Lp_analytic_mm": lp_analytic,
         "deltaL_mm": dl, "eeff_patch": ep,
         "Wp_mm": wp, "Lp_mm": lp, "Lp_base_mm": LP_BASE_MM,
         "Lp_scale": MEASURED_RESONANCE_GHZ / F0_GHZ,
         "G1_S": g1, "G12_S": g12,
         "R_edge_analytic_ohm": r_edge_analytic,
         "R_edge_used_ohm": EDGE_R_OHM,
         "element_source": "User-verified single-patch HFSS model"}

    # --- transmission lines --------------------------------------------------
    s["line50"] = line_properties(50.0)
    s["divider"] = line_properties(math.sqrt(50.0 * 100.0))
    s["patch_transformer"] = line_properties(math.sqrt(50.0 * EDGE_R_OHM))
    # Round the two widths that the previous revision used, then re-analyse so
    # the printed Z0 is the impedance of the width actually drawn.
    s["line50"] = line_properties(50.0, width=round(s["line50"]["W_mm"], 3))
    s["divider"] = line_properties(math.sqrt(50.0 * 100.0),
                                   width=round(s["divider"]["W_mm"], 4))
    s["patch_transformer"] = line_properties(math.sqrt(50.0 * EDGE_R_OHM),
                                             width=round(s["patch_transformer"]["W_mm"], 4))
    s["Zin_predicted_ohm"] = (s["patch_transformer"]["Z_actual_ohm"] ** 2
                              / EDGE_R_OHM)
    return s


# ----------------------------------------------------------------------------
# 3. PARAMETRIC EXPRESSION HELPER
# ----------------------------------------------------------------------------
class E(object):
    """A nominal millimetre value paired with its HFSS parameter expression."""

    def __init__(self, value, expr=None):
        self.v = float(value)
        self.s = expr if expr is not None else "%.10gmm" % value

    def __add__(self, other):
        other = as_e(other)
        return E(self.v + other.v, "(%s+%s)" % (self.s, other.s))

    __radd__ = __add__

    def __sub__(self, other):
        other = as_e(other)
        return E(self.v - other.v, "(%s-%s)" % (self.s, other.s))

    def __rsub__(self, other):
        return as_e(other).__sub__(self)

    def __mul__(self, scalar):
        return E(self.v * scalar, "(%s*%.10g)" % (self.s, scalar))

    __rmul__ = __mul__

    def __neg__(self):
        return self * -1


def as_e(value):
    return value if isinstance(value, E) else E(value)


# ----------------------------------------------------------------------------
# 4. PURE-PYTHON PLANAR GEOMETRY (clearance and continuity checks)
# ----------------------------------------------------------------------------
def point_in_polygon(pt, poly):
    """Ray casting; returns True for strictly interior or on-edge points."""
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if abs((x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)) < 1e-9:
            if min(x1, x2) - 1e-9 <= x <= max(x1, x2) + 1e-9 and \
               min(y1, y2) - 1e-9 <= y <= max(y1, y2) + 1e-9:
                return True
        if (y1 > y) != (y2 > y):
            xint = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if xint > x:
                inside = not inside
    return inside


def point_segment_distance(p, a, b):
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom < 1e-18:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / denom
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def segments_intersect(a, b, c, d):
    def cross(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
    d1, d2 = cross(c, d, a), cross(c, d, b)
    d3, d4 = cross(a, b, c), cross(a, b, d)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return True
    return False


def polygon_distance(pa, pb):
    """Minimum edge-to-edge distance; 0.0 if the polygons touch or overlap."""
    na, nb = len(pa), len(pb)
    for i in range(na):
        a1, a2 = pa[i], pa[(i + 1) % na]
        for j in range(nb):
            b1, b2 = pb[j], pb[(j + 1) % nb]
            if segments_intersect(a1, a2, b1, b2):
                return 0.0
    if point_in_polygon(pa[0], pb) or point_in_polygon(pb[0], pa):
        return 0.0
    best = 1e18
    for i in range(na):
        a1, a2 = pa[i], pa[(i + 1) % na]
        for j in range(nb):
            b1, b2 = pb[j], pb[(j + 1) % nb]
            best = min(best,
                       point_segment_distance(a1, b1, b2),
                       point_segment_distance(a2, b1, b2),
                       point_segment_distance(b1, a1, a2),
                       point_segment_distance(b2, a1, a2))
    return best


def stroke_polyline(points, width):
    """Return one quadrilateral per segment plus a square cap at each vertex.

    This is a conservative outer approximation of the AEDT "Curved" bend, which
    only removes copper at the inside of a corner, never adds it.  Using it for
    the clearance test therefore never reports a clearance that is too large.
    """
    polys = []
    half = width / 2.0
    for i in range(len(points) - 1):
        (x1, y1), (x2, y2) = points[i], points[i + 1]
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        if length < 1e-12:
            continue
        nx, ny = -dy / length * half, dx / length * half
        polys.append([(x1 + nx, y1 + ny), (x2 + nx, y2 + ny),
                      (x2 - nx, y2 - ny), (x1 - nx, y1 - ny)])
    for (x, y) in points[1:-1]:
        polys.append([(x - half, y - half), (x + half, y - half),
                      (x + half, y + half), (x - half, y + half)])
    return polys


def extend_point(p, q, distance):
    """Point `distance` beyond p, on the line q->p (i.e. away from q)."""
    dx, dy = p[0] - q[0], p[1] - q[1]
    length = math.hypot(dx, dy)
    if length < 1e-12:
        return p
    return (p[0] + dx / length * distance, p[1] + dy / length * distance)


def extend_point_e(p, q, distance):
    """Parametric version of extend_point for axis-aligned or 45 degree runs."""
    dxv, dyv = p[0].v - q[0].v, p[1].v - q[1].v
    length = math.hypot(dxv, dyv)
    if length < 1e-12:
        return p
    ux, uy = dxv / length, dyv / length
    return (p[0] + E(distance * ux, "(%s*%.10g)" % ("cOver", ux)),
            p[1] + E(distance * uy, "(%s*%.10g)" % ("cOver", uy)))


# ----------------------------------------------------------------------------
# 5. LAYOUT MODEL
# ----------------------------------------------------------------------------
class Layout(object):
    """Holds every copper piece with both its HFSS expression and its nominal
    polygon, plus the list of joints that must be electrically continuous."""

    def __init__(self, name, synthesis, dx_mm, dy_mm, board_x, board_y):
        self.name = name
        self.synthesis = synthesis
        self.vars = []
        self.pieces = []          # every copper solid
        self.joints = []          # intended connections
        self.routes = []          # per-branch electrical bookkeeping
        self.leaf_paths = []
        self.over = CONNECT_OVERLAP_MM

        s = synthesis
        self.Wp = self.var("Wp", s["Wp_mm"])
        self.Lp = self.var("Lp", s["Lp_mm"])
        self.h = self.var("h", H_MM)
        self.t = self.var("tCu", COPPER_MM)
        self.dx = self.var("dx", dx_mm)
        self.dy = self.var("dy", dy_mm)
        self.rise = self.var("feedRise", FEED_RISE_MM)
        self.bx = self.var("boardX", board_x)
        self.by = self.var("boardY", board_y)
        self.pad = self.var("airPad", AIR_PAD_MM)
        self.junction_r = self.var("junctionR",
                                   JUNCTION_RADIUS_FACTOR * s["line50"]["W_mm"])
        self.q50_taper = self.var("Lq50Taper", Q50_TAPER_MM)
        self.pt_taper = self.var("L50PatchTaper", FIFTY_TO_PATCH_TAPER_MM)
        self.tee_approach = self.var("teeApproach", TEE_APPROACH_MM)
        self.input_y = self.var("inputY", INPUT_Y_MM)
        self.w50 = self.var("W50", s["line50"]["W_mm"])
        self.wq = self.var("W70", s["divider"]["W_mm"])
        self.wpT = self.var("WpT", s["patch_transformer"]["W_mm"])
        self.lpT = self.var("LpT", s["patch_transformer"]["Lquarter_mm"])
        # Exposed as HFSS local variables so Optimetrics can tune the input
        # match without rebuilding or changing the corporate-feed geometry.
        self.stub_d = self.var("stubD", MATCH_STUB_FROM_PORT_MM)
        self.stub_l = self.var("stubL", MATCH_STUB_PHYSICAL_MM)
        self.cover = self.var("cOver", CONNECT_OVERLAP_MM)
        self.edge_pb = self.var("edgePullback", EDGE_PULLBACK_MM)
        self.sma_pads = []        # top ground pads for the SMA (not feed copper)
        self.vias = []            # plated vias, pad -> bottom ground
        self.mount_holes = []     # non-plated corner holes
        self.lq = {}
        for stage in (1, 2, 3, 4):
            # Same width and same quarter-wave length at every stage: see the
            # impedance induction proof in the report.
            self.lq[stage] = self.var("Lq%d" % stage, s["divider"]["Lquarter_mm"])
        self.bus_y = -(self.Lp * 0.5) - self.rise
        self.port = None

    # -- variables ------------------------------------------------------------
    def var(self, name, value):
        self.vars.append((name, "%.10gmm" % value))
        return E(value, name)

    # -- piece constructors ---------------------------------------------------
    def _add(self, piece):
        self.pieces.append(piece)
        return piece

    def add_rect(self, name, x, y, w, h, kind, owner):
        vals = [as_e(v) for v in (x, y, w, h)]
        if vals[2].v <= 0 or vals[3].v <= 0:
            raise ValueError("Non-positive geometry: " + name)
        x0, y0, wv, hv = [v.v for v in vals]
        return self._add({"name": name, "cls": "box", "kind": kind, "owner": owner,
                          "expr": [v.s for v in vals], "xywh": [x0, y0, wv, hv],
                          "w_ref": min(wv, hv),
                          "polys": [[(x0, y0), (x0 + wv, y0),
                                     (x0 + wv, y0 + hv), (x0, y0 + hv)]]})

    def patch(self, name, x, y):
        piece = self.add_rect(name, x - self.Wp * 0.5, y - self.Lp * 0.5,
                              self.Wp, self.Lp, "patch", name)
        piece["w_ref"] = 1e6          # never limits a joint coverage radius
        return piece

    def segment(self, name, p, q, width, kind, owner,
                grow_start=True, grow_end=True):
        """Axis-aligned constant-width run.  Electrical length is |q-p|; the
        controlled overlap is added outside that length at the chosen ends."""
        p = tuple(as_e(v) for v in p)
        q = tuple(as_e(v) for v in q)
        width = as_e(width)
        dxv, dyv = q[0].v - p[0].v, q[1].v - p[1].v
        if abs(dxv) > 1e-9 and abs(dyv) > 1e-9:
            raise ValueError("segment() needs an axis-aligned run: " + name)
        length = math.hypot(dxv, dyv)
        pe = extend_point_e(p, q, self.over) if grow_start else p
        qe = extend_point_e(q, p, self.over) if grow_end else q
        if abs(dyv) <= 1e-9:
            lo, hi = (pe[0], qe[0]) if pe[0].v < qe[0].v else (qe[0], pe[0])
            self.add_rect(name, lo, p[1] - width * 0.5, hi - lo, width, kind, owner)
        else:
            lo, hi = (pe[1], qe[1]) if pe[1].v < qe[1].v else (qe[1], pe[1])
            self.add_rect(name, p[0] - width * 0.5, lo, width, hi - lo, kind, owner)
        return length

    def taper(self, name, p, q, w_start, w_end, kind, owner,
              grow_start=True, grow_end=True):
        """Linear-width transition along any direction (axis-aligned or 45 deg).

        The controlled overlap is realised as a constant-width stub outside each
        reference plane, so the tapered part keeps its designed length."""
        p = tuple(as_e(v) for v in p)
        q = tuple(as_e(v) for v in q)
        w1, w2 = as_e(w_start), as_e(w_end)
        dxv, dyv = q[0].v - p[0].v, q[1].v - p[1].v
        length = math.hypot(dxv, dyv)
        if length < 1e-9:
            raise ValueError("Zero-length taper: " + name)
        ux, uy = dxv / length, dyv / length
        nx, ny = -uy, ux
        pe = extend_point_e(p, q, self.over) if grow_start else p
        qe = extend_point_e(q, p, self.over) if grow_end else q
        corners = [(pe[0] + w1 * (0.5 * nx), pe[1] + w1 * (0.5 * ny)),
                   (qe[0] + w2 * (0.5 * nx), qe[1] + w2 * (0.5 * ny)),
                   (qe[0] - w2 * (0.5 * nx), qe[1] - w2 * (0.5 * ny)),
                   (pe[0] - w1 * (0.5 * nx), pe[1] - w1 * (0.5 * ny))]
        poly = [(c[0].v, c[1].v) for c in corners]
        # keep the polygon counter-clockwise for ThickenSheet
        area = 0.0
        for i in range(4):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % 4]
            area += x1 * y2 - x2 * y1
        if area < 0:
            corners.reverse()
            poly.reverse()
        self._add({"name": name, "cls": "sheet", "kind": kind, "owner": owner,
                   "expr": [[c[0].s, c[1].s] for c in corners],
                   "w_ref": min(w1.v, w2.v), "polys": [poly]})
        return length

    def trace(self, name, points, width, kind, owner,
              grow_start=True, grow_end=True):
        """Multi-segment constant-width trace, built by AEDT as a rectangular
        polyline with Curved bends.  Electrical length is the true centreline
        length, diagonals included."""
        pts = [tuple(as_e(v) for v in pt) for pt in points]
        width = as_e(width)
        length = 0.0
        for i in range(len(pts) - 1):
            length += math.hypot(pts[i + 1][0].v - pts[i][0].v,
                                 pts[i + 1][1].v - pts[i][1].v)
        drawn = list(pts)
        if grow_start:
            drawn[0] = extend_point_e(pts[0], pts[1], self.over)
        if grow_end:
            drawn[-1] = extend_point_e(pts[-1], pts[-2], self.over)
        nominal = [(p[0].v, p[1].v) for p in drawn]
        self._add({"name": name, "cls": "trace", "kind": kind, "owner": owner,
                   "pts": nominal,
                   "expr": [[p[0].s, p[1].s] for p in drawn],
                   "width_expr": width.s, "width_mm": width.v,
                   "w_ref": width.v, "polys": stroke_polyline(nominal, width.v)})
        return length

    def junction(self, name, point, radius=None, kind="divider",
                 owner=None):
        """Copper disc at a tee or at a change of direction.

        At a tee it rounds the junction.  At a bend between two pieces whose
        end faces are not parallel it fills the wedge of bare substrate on the
        outside of the turn, which no amount of collinear overlap can close.
        """
        x, y = [as_e(v) for v in point]
        radius = self.junction_r if radius is None else as_e(radius)
        r = radius.v
        poly = [(x.v + r * math.cos(2 * math.pi * i / 32),
                 y.v + r * math.sin(2 * math.pi * i / 32)) for i in range(32)]
        return self._add({"name": name, "cls": "disc", "kind": kind,
                          "owner": owner or name,
                          "expr": [x.s, y.s], "xy": [x.v, y.v],
                          "radius_expr": radius.s, "radius_mm": r,
                          "w_ref": 2.0 * r, "polys": [poly]})

    def tee(self, name, point, bar_axis, bar_width, stem_width,
            kind="divider", owner=None):
        """Tee junction.  With FLAT_TEES it is a rectangle lying fully inside
        the T: `stem_width` long along the bar and `bar_width` across it, so
        nothing sticks out past the conductor edges.  Otherwise the original
        round disc is used."""
        if not FLAT_TEES:
            return self.junction(name, point, stem_width * 0.5, kind, owner)
        x, y = [as_e(v) for v in point]
        along, across = as_e(stem_width), as_e(bar_width)
        if bar_axis == "x":
            piece = self.add_rect(name, x - along * 0.5, y - across * 0.5,
                                  along, across, kind, owner or name)
        else:
            piece = self.add_rect(name, x - across * 0.5, y - along * 0.5,
                                  across, along, kind, owner or name)
        piece["tee"] = True
        piece["xy"] = [x.v, y.v]
        return piece

    def flat_fill(self, name, corners, kind, owner, w_ref):
        """Closed flat copper polygon (used for the chamfered bend fill)."""
        corners = [(as_e(c[0]), as_e(c[1])) for c in corners]
        poly = [(c[0].v, c[1].v) for c in corners]
        area = 0.0
        for i in range(len(poly)):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % len(poly)]
            area += x1 * y2 - x2 * y1
        if area < 0:
            corners.reverse()
            poly.reverse()
        return self._add({"name": name, "cls": "sheet", "kind": kind,
                          "owner": owner,
                          "expr": [[c[0].s, c[1].s] for c in corners],
                          "w_ref": w_ref, "polys": [poly]})

    def join(self, a, b, point, label):
        self.joints.append({"a": a, "b": b, "label": label,
                            "xy": [as_e(point[0]).v, as_e(point[1]).v]})


    # -- one corporate-feed branch -------------------------------------------
    def branch(self, name, p, q, stage, parent_piece, leaf=None, reroute=False):
        """Build one branch of the binary corporate divider.

        Every branch starts with a quarter-wave 70.71 ohm section at the parent
        tee, then a linear taper to 50 ohm, then 50 ohm routing.  `reroute`
        starts the 45 degree rise into the inter-row channel immediately at the
        end of the quarter-wave section, which is what buys the patch
        clearance; the taper then lives on the diagonal.
        """
        p = tuple(as_e(v) for v in p)
        q = tuple(as_e(v) for v in q)
        axis = 0 if abs(p[0].v - q[0].v) > 1e-9 else 1
        sign = 1 if q[axis].v > p[axis].v else -1
        span = abs(q[axis].v - p[axis].v)

        lq = self.lq[stage]
        a_pt = list(p)
        a_pt[axis] = p[axis] + lq * sign
        a_pt = tuple(a_pt)

        q_name = name + "_Q"
        self.segment(q_name, p, a_pt, self.wq, "divider", name)
        self.join(parent_piece, q_name, p, "tee -> Q(70.71ohm)")

        taper_name = name + "_Q50_Taper"
        parts = {"Lq": lq.v, "Ltrans": self.q50_taper.v, "L50": 0.0, "Lpt": 0.0}

        if reroute:
            co = self.Lp * 0.5 + self.rise        # rise into the clear channel
            inv = 1.0 / math.sqrt(2.0)
            t_pt = (a_pt[0] + self.q50_taper * (sign * inv),
                    a_pt[1] + self.q50_taper * inv)
            self.taper(taper_name, a_pt, t_pt, self.wq, self.w50, "transition", name)
            self.join(q_name, taper_name, a_pt, "Q(70.71ohm) -> Q50 taper")
            if CORNER_FILL and FLAT_BEND_FILL:
                # Same wedge, closed with a flat triangle instead of a disc:
                # corners = bend centre, outer edge of the horizontal section,
                # outer edge of the 45 degree taper.  The outer edge becomes a
                # straight chamfer; nothing extends past either trace edge.
                # The chamfer is 0.5*cos(22.5deg) = 0.462*W70 from the bend
                # centre, above the 0.45*W70 continuity-check radius.
                half = self.wq * 0.5
                outer_q = (a_pt[0], a_pt[1] - half)
                outer_t = (a_pt[0] + half * (sign * inv), a_pt[1] - half * inv)
                self.flat_fill(name + "_BendFill", [a_pt, outer_q, outer_t],
                               "transition", name, self.wq.v)
            elif CORNER_FILL:
                # The horizontal quarter-wave section and the 45 degree taper
                # have non-parallel end faces, so their union leaves a wedge of
                # bare substrate on the outside of the turn.  This disc fills
                # it and rounds the outside of the bend.  Radius factor 0.62
                # exceeds 0.5/cos(45deg/2) = 0.541, the exact requirement.
                self.junction(name + "_BendFill", a_pt,
                              self.wq * CORNER_FILL_FACTOR, "transition",
                              owner=name)

            d1 = (a_pt[0] + co * sign, p[1] + co)
            d3 = (q[0] - self.tee_approach * sign, p[1])
            d2 = (d3[0] - co * sign, p[1] + co)
            channel = (d2[0].v - d1[0].v) * sign
            if channel <= 0:
                raise ValueError(name + ": stage-2 span too short for both "
                                        "45 degree clearance approaches")
            trace_name = name + "_50_Clearance"
            l50 = self.trace(trace_name, [t_pt, d1, d2, d3, q],
                             self.w50, "line50", name)
            self.join(taper_name, trace_name, t_pt, "Q50 taper -> 50 ohm route")
            parts["L50"] = l50
            total = lq.v + self.q50_taper.v + l50
            self.routes.append({"name": name, "stage": stage, "kind": "reroute",
                                "start": [p[0].v, p[1].v], "end": [q[0].v, q[1].v],
                                "channel_mm": channel, "parts": parts,
                                "length_mm": total, "leaf": None,
                                "end_piece": trace_name})
            return total, trace_name

        b_pt = list(a_pt)
        b_pt[axis] = a_pt[axis] + self.q50_taper * sign
        b_pt = tuple(b_pt)
        self.taper(taper_name, a_pt, b_pt, self.wq, self.w50, "transition", name)
        self.join(q_name, taper_name, a_pt, "Q(70.71ohm) -> Q50 taper")

        if leaf is None:
            if span <= lq.v + self.q50_taper.v:
                raise ValueError(name + ": branch shorter than its transformer")
            line_name = name + "_50"
            l50 = self.segment(line_name, b_pt, q, self.w50, "line50", name)
            self.join(taper_name, line_name, b_pt, "Q50 taper -> 50 ohm line")
            parts["L50"] = l50
            total = lq.v + self.q50_taper.v + l50
            self.routes.append({"name": name, "stage": stage, "kind": "straight",
                                "start": [p[0].v, p[1].v], "end": [q[0].v, q[1].v],
                                "parts": parts, "length_mm": total, "leaf": None,
                                "end_piece": line_name})
            return total, line_name

        # ---- leaf branch: 50 ohm run, 50->WpT taper, patch transformer ------
        if axis != 0:
            raise ValueError(name + ": the final branch must run horizontally")
        patch_name, patch_y = leaf
        fifty_end = list(q)
        fifty_end[0] = q[0] - (self.lpT - self.rise) * sign
        fifty_end = tuple(fifty_end)
        pt_start = list(fifty_end)
        pt_start[0] = fifty_end[0] - self.pt_taper * sign
        pt_start = tuple(pt_start)
        if (pt_start[0].v - b_pt[0].v) * sign <= 0:
            raise ValueError(name + ": leaf branch too short for its transformers")

        line_name = name + "_50"
        l50 = self.segment(line_name, b_pt, pt_start, self.w50, "line50", name)
        self.join(taper_name, line_name, b_pt, "Q50 taper -> 50 ohm line")

        pt_taper_name = name + "_50PT_Taper"
        l_trans2 = self.taper(pt_taper_name, pt_start, fifty_end,
                              self.w50, self.wpT, "transition", name)
        self.join(line_name, pt_taper_name, pt_start, "50 ohm line -> patch taper")

        edge_y = patch_y - self.Lp * 0.5
        pt_name = name + "_PT"
        lpt = self.trace(pt_name, [fifty_end, (q[0], p[1]), (q[0], edge_y)],
                         self.wpT, "patchT", name)
        self.join(pt_taper_name, pt_name, fifty_end, "patch taper -> patch transformer")
        self.join(pt_name, patch_name, (q[0], edge_y), "patch transformer -> patch")

        parts["L50"] = l50
        parts["Ltrans"] = self.q50_taper.v + l_trans2
        parts["Lpt"] = lpt
        total = lq.v + self.q50_taper.v + l50 + l_trans2 + lpt
        self.routes.append({"name": name, "stage": stage, "kind": "leaf",
                            "start": [p[0].v, p[1].v], "end": [q[0].v, q[1].v],
                            "parts": parts, "length_mm": total, "leaf": patch_name,
                            "end_piece": pt_name})
        return total, pt_name

    # -- serialisation --------------------------------------------------------
    def data(self):
        return {"name": self.name, "parameters": dict(self.vars),
                "board_mm": [self.bx.v, self.by.v],
                "pieces": [{"name": p["name"], "cls": p["cls"], "kind": p["kind"],
                            "owner": p["owner"],
                            "xywh": p.get("xywh"), "polys": p["polys"]}
                           for p in self.pieces],
                "joints": self.joints, "branches": self.routes,
                "leaf_paths": self.leaf_paths,
                "port": [self.port[0].v, self.port[1].v],
                "sma_pads": self.sma_pads, "vias": self.vias,
                "mount_holes": self.mount_holes,
                "synthesis": self.synthesis}


# ----------------------------------------------------------------------------
# 6. PITCH AND BOARD SOLVERS
# ----------------------------------------------------------------------------
def solve_pitch(s):
    """Smallest pitch that satisfies MIN_FEED_PATCH_CLEARANCE_MM.

    Binding constraint: the stage-2 45 degree rise leaves the feed bus at the
    end of the quarter-wave section, at local (Lq2, 0) with the bus at y = 0.
    The nearest non-owning copper is the inner top corner of the patch in the
    row below, at local (dx/2 - Wp/2, -(dy/2 - Lp - feedRise)).  The distance
    from that corner to the lower edge of the 45 degree band is

        C = [ dx/2 + dy/2 - Wp/2 - Lq2 - Lp - feedRise - W50/sqrt(2) ] / sqrt(2)

    so the requirement is a constraint on (dx + dy), not on either pitch alone.
    The shortfall is therefore split equally between dx and dy, which preserves
    the original dx - dy difference and keeps both pitches as small as possible.
    """
    lq = s["divider"]["Lquarter_mm"]
    need_half_sum = (MIN_FEED_PATCH_CLEARANCE_MM * math.sqrt(2.0)
                     + s["Wp_mm"] / 2.0 + lq + s["Lp_mm"] + FEED_RISE_MM
                     + s["line50"]["W_mm"] / math.sqrt(2.0))
    need_sum = 2.0 * need_half_sum
    base_sum = DX_MIN_MM + DY_MIN_MM
    delta = max(0.0, (need_sum - base_sum) / 2.0)
    dx = DX_MIN_MM + delta
    dy = DY_MIN_MM + delta
    cap = PITCH_MAX_FRACTION_LAMBDA0 * s["lambda0_mm"]
    info = {"delta_mm": delta, "required_sum_mm": need_sum,
            "base_sum_mm": base_sum, "cap_mm": cap,
            "capped": dx > cap or dy > cap}
    return min(dx, cap), min(dy, cap), info


def solve_board(s, dx, dy):
    """Board size from the final copper extent plus BOARD_MARGIN_MM."""
    half_x_copper = 1.5 * dx + s["Wp_mm"] / 2.0
    half_y_copper = max(1.5 * dy + s["Lp_mm"] / 2.0,
                        1.5 * dy + s["Lp_mm"] / 2.0 + FEED_RISE_MM
                        + s["line50"]["W_mm"] / 2.0)
    half_x = math.ceil((half_x_copper + BOARD_MARGIN_MM) * 2.0) / 2.0
    half_y = math.ceil((half_y_copper + BOARD_MARGIN_MM) * 2.0) / 2.0
    return 2.0 * half_x, 2.0 * half_y, half_x_copper, half_y_copper


# ----------------------------------------------------------------------------
# 7. ARRAY ASSEMBLY
# ----------------------------------------------------------------------------
def make_layout(name):
    """Dispatch to the single-element probe or to the full 4x4 array."""
    if name == "SinglePatch_Probe":
        return make_probe_layout(name)
    return make_array_layout(name)


def make_probe_layout(name):
    """One patch, one transformer, one 50 ohm line, one port.

    This design exists to MEASURE the patch edge resistance instead of
    assuming it.  Its input reflection at the patch-transformer reference
    plane is Gamma = (Zpt^2/R - 50) / (Zpt^2/R + 50), and a 50 ohm line does
    not change |Gamma|, so the probe's |S11| at resonance gives R directly:

        R = Zpt^2 / Zin      with  Zin = 50*(1 -/+ |S11|)/(1 +/- |S11|)

    Solve this design first, read the resonant frequency and |S11|, feed the
    result back into LP_BASE_MM / MEASURED_RESONANCE_GHZ and EDGE_R_OHM, then
    rebuild.  It solves in a fraction of the time the full array needs.
    """
    s = synthesize()
    board_x = 4.0 * s["Wp_mm"]
    board_y = 2.0 * (s["Lp_mm"] / 2.0 + FEED_RISE_MM + s["line50"]["W_mm"] / 2.0
                     + BOARD_MARGIN_MM)
    m = Layout(name, s, DX_MIN_MM, DY_MIN_MM, round(board_x), round(board_y))
    m.pitch_info = {"delta_mm": 0.0, "required_sum_mm": 0.0, "base_sum_mm": 0.0,
                    "cap_mm": 0.0, "capped": False}
    m.copper_half_extent = (s["Wp_mm"] / 2.0,
                            s["Lp_mm"] / 2.0 + FEED_RISE_MM
                            + s["line50"]["W_mm"] / 2.0)
    m.patch("Patch_Probe", E(0), E(0))
    bus = m.bus_y
    m.port = (-m.bx * 0.5 + m.edge_pb, bus)
    fifty_end = (-(m.lpT - m.rise), bus)
    pt_start = (fifty_end[0] - m.pt_taper, bus)
    l50 = m.segment("Probe_50", m.port, pt_start, m.w50, "line50", "probe",
                    grow_start=False)
    l_tr = m.taper("Probe_50PT_Taper", pt_start, fifty_end, m.w50, m.wpT,
                   "transition", "probe")
    m.join("Probe_50", "Probe_50PT_Taper", pt_start, "50 ohm line -> patch taper")
    lpt = m.trace("Probe_PT", [fifty_end, (E(0), bus), (E(0), -m.Lp * 0.5)],
                  m.wpT, "patchT", "probe")
    m.join("Probe_50PT_Taper", "Probe_PT", fifty_end,
           "patch taper -> patch transformer")
    m.join("Probe_PT", "Patch_Probe", (E(0), -m.Lp * 0.5),
           "patch transformer -> patch")
    m.input_length_mm = l50
    m.routes.append({"name": "probe", "stage": 4, "kind": "leaf",
                     "start": [m.port[0].v, bus.v], "end": [0.0, bus.v],
                     "parts": {"Lq": 0.0, "Ltrans": l_tr, "L50": l50,
                               "Lpt": lpt},
                     "length_mm": l50 + l_tr + lpt, "leaf": "Patch_Probe",
                     "end_piece": "Probe_PT"})
    m.leaf_paths.append({"patch": "Patch_Probe", "branch": "probe",
                         "path_mm": l50 + l_tr + lpt,
                         "parts": {"Lq": 0.0, "Ltrans": l_tr, "L50": l50,
                                   "Lpt": lpt},
                         "R_edge_assumed_ohm": EDGE_R_OHM})
    return m


def make_array_layout(name):
    """Build the 4x4 array: 1 -> 2 -> 4 -> 8 -> 16 binary corporate divider."""
    s = synthesize()
    if LOCK_ARRAY_ENVELOPE:
        dx_mm, dy_mm = LOCKED_DX_MM, LOCKED_DY_MM
        solved_dx, solved_dy, pitch_info = solve_pitch(s)
        pitch_info["locked"] = True
        pitch_info["solver_dx_mm"] = solved_dx
        pitch_info["solver_dy_mm"] = solved_dy
        board_x, board_y = LOCKED_BOARD_X_MM, LOCKED_BOARD_Y_MM
        hx = 1.5 * dx_mm + s["Wp_mm"] / 2.0
        hy = (1.5 * dy_mm + s["Lp_mm"] / 2.0 + FEED_RISE_MM
              + s["line50"]["W_mm"] / 2.0)
    else:
        dx_mm, dy_mm, pitch_info = solve_pitch(s)
        board_x, board_y, hx, hy = solve_board(s, dx_mm, dy_mm)
    m = Layout(name, s, dx_mm, dy_mm, board_x, board_y)
    m.pitch_info = pitch_info
    m.copper_half_extent = (hx, hy)

    n = 4
    stages = [1, 2, 3, 4]
    xs = [(i - (n - 1) / 2.0) * m.dx for i in range(n)]
    ys = [(j - (n - 1) / 2.0) * m.dy for j in range(n)]

    # ---- common input: port at the right board edge, along y = 0 ------------
    # y = 0 is the centre channel between patch rows 2 and 3, so the long
    # common run is (dy - Lp)/2 - W50/2 from both of them.
    # The port sits on the end of the input line, EDGE_PULLBACK_MM inside the
    # right-hand board edge (JLC copper-to-edge rule).
    m.port = (m.bx * 0.5 - m.edge_pb, m.input_y)
    drop = m.input_y - m.bus_y                     # = Lp/2 + feedRise
    diag_end_x = m.tee_approach
    diag_start_x = diag_end_x + drop
    root = (E(0), m.bus_y)
    tee_prefix = "Tee_" if FLAT_TEES else "RoundTee_"
    root_piece = tee_prefix + "root"
    # Input arrives horizontally (W50); stage-1 arms leave vertically (W70).
    m.tee(root_piece, root, "y", m.wq, m.w50)
    input_len = m.trace("Input50_Rerouted",
                        [m.port, (diag_start_x, m.input_y),
                         (diag_end_x, m.bus_y), root],
                        m.w50, "line50", "input", grow_start=False)
    m.join("Input50_Rerouted", root_piece, root, "input 50 ohm -> root tee")
    m.input_length_mm = input_len

    # Measured-array single-open-stub match.  Distance is measured from the
    # existing wave-port plane toward the array along the straight input run.
    # grow_end=False preserves the calculated open-circuit physical length.
    if ENABLE_ARRAY_INPUT_STUB:
        # Measured from the board edge, so the stub keeps its old position.
        stub_junction = (m.bx * 0.5 - m.stub_d, m.input_y)
        stub_end = (stub_junction[0],
                    stub_junction[1] + m.stub_l * STUB_DIRECTION)
        m.trace("InputMatchStub", [stub_junction, stub_end], m.w50,
                "line50", "input", grow_start=True, grow_end=False)
        stub_tee = tee_prefix + "InputMatch"
        # Bar = horizontal 50 ohm input line; stem = vertical 50 ohm stub.
        m.tee(stub_tee, stub_junction, "x", m.w50, m.w50, "divider",
              owner="input")
        m.join("Input50_Rerouted", stub_tee, stub_junction,
               "input line -> matching-stub tee")
        m.join(stub_tee, "InputMatchStub", stub_junction,
               "matching-stub tee -> open stub")

    def mean(values):
        return sum(values, E(0)) * (1.0 / len(values))

    def divide(ixs, iys, depth, path_length, path_parts, prefix, parent_piece):
        axis = "y" if depth % 2 == 0 else "x"
        indices = iys if axis == "y" else ixs
        half = len(indices) // 2
        for side, child_indices in enumerate((indices[:half], indices[half:])):
            cx = ixs if axis == "y" else child_indices
            cy = child_indices if axis == "y" else iys
            parent = (mean([xs[i] for i in ixs]), mean([ys[j] for j in iys]) + m.bus_y)
            child = (mean([xs[i] for i in cx]), mean([ys[j] for j in cy]) + m.bus_y)
            label = prefix + str(side)
            last = depth == len(stages) - 1
            leaf = None
            if last:
                patch_name = "Patch_r%d_c%d" % (cy[0] + 1, cx[0] + 1)
                leaf = (patch_name, ys[cy[0]])
                m.patch(patch_name, xs[cx[0]], ys[cy[0]])
            step, end_piece = m.branch("Branch_" + label, parent, child,
                                       stages[depth], parent_piece, leaf,
                                       reroute=(depth == 1))
            parts = dict(path_parts)
            for key, value in m.routes[-1]["parts"].items():
                parts[key] = parts.get(key, 0.0) + value
            if last:
                m.leaf_paths.append({"patch": leaf[0], "branch": label,
                                     "path_mm": path_length + step,
                                     "parts": parts,
                                     "R_edge_assumed_ohm": EDGE_R_OHM})
            else:
                child_piece = tee_prefix + label
                if ROUND_JUNCTIONS or FLAT_TEES:
                    # The next stage's arms run along this axis (W70 wide);
                    # the incoming 50 ohm line is the stem.
                    next_axis = "y" if (depth + 1) % 2 == 0 else "x"
                    m.tee(child_piece, child, next_axis, m.wq, m.w50)
                    m.join(end_piece, child_piece, child, "50 ohm line -> tee")
                else:
                    child_piece = end_piece
                divide(cx, cy, depth + 1, path_length + step, parts, label,
                       child_piece)

    divide(list(range(n)), list(range(n)), 0, input_len, {"Lq": 0.0, "Ltrans": 0.0,
           "L50": input_len, "Lpt": 0.0}, "", root_piece)
    add_fabrication_features(m, side=+1)
    return m


def add_fabrication_features(m, side=+1):
    """SMA top ground pads + vias, and corner mounting holes.

    These are NOT part of the feed network, so they are kept out of
    m.pieces (the feed checks would otherwise flag the pads as crowding the
    input line).  They are drawn in HFSS and written to the Gerbers."""
    px, py = m.port
    if ENABLE_SMA_GROUND_PADS:
        m.sma_l = m.var("smaPadL", SMA_PAD_LENGTH_MM)
        m.sma_w = m.var("smaPadW", SMA_PAD_WIDTH_MM)
        m.sma_gap = m.var("smaPadGap", SMA_PAD_GAP_MM)
        m.via_d = m.var("viaDrill", SMA_VIA_DRILL_MM)
        # pad x range: from the pulled-back edge, SMA_PAD_LENGTH_MM inward
        x0 = px - m.sma_l if side > 0 else px
        inner = m.w50 * 0.5 + m.sma_gap
        for tag, y0 in (("Upper", py + inner), ("Lower", py - inner - m.sma_w)):
            name = "SMA_GndPad_" + tag
            m.sma_pads.append({
                "name": name, "expr": [x0.s, y0.s, m.sma_l.s, m.sma_w.s],
                "xywh": [x0.v, y0.v, m.sma_l.v, m.sma_w.v]})
            yc = y0 + m.sma_w * 0.5
            nv = max(1, int(SMA_VIAS_PER_PAD))
            for i in range(nv):
                xc = x0 + m.sma_l * ((i + 1.0) / (nv + 1.0))
                m.vias.append({"name": "SMA_Via_%s_%d" % (tag, i + 1),
                               "expr": [xc.s, yc.s], "xy": [xc.v, yc.v],
                               "drill_expr": m.via_d.s,
                               "drill_mm": m.via_d.v})
    if ENABLE_MOUNT_HOLES:
        m.mh_d = m.var("mountHoleD", MOUNT_HOLE_DRILL_MM)
        m.mh_edge = m.var("mountHoleEdge", MOUNT_HOLE_EDGE_MM)
        m.mh_clear = m.var("mountHoleClear", MOUNT_HOLE_COPPER_CLEAR_MM)
        k = 0
        for sx in (-1, 1):
            for sy in (-1, 1):
                k += 1
                x = (m.bx * 0.5 - m.mh_edge) * sx
                y = (m.by * 0.5 - m.mh_edge) * sy
                m.mount_holes.append({"name": "MountHole_%d" % k,
                                      "expr": [x.s, y.s], "xy": [x.v, y.v],
                                      "drill_mm": m.mh_d.v,
                                      "clear_mm": m.mh_clear.v})


def check_fabrication(model, report):
    """JLC-oriented checks on the extra fabrication features."""
    report.append("")
    report.append("JLCPCB fabrication checks")
    ok = True
    hx, hy = model.bx.v / 2.0, model.by.v / 2.0
    # narrowest drawn copper
    widths = []
    for p in model.pieces:
        if p["cls"] == "trace":
            widths.append(p["width_mm"])
        elif p["cls"] == "box":
            widths.append(min(p["xywh"][2], p["xywh"][3]))
    wmin = min(widths)
    good = wmin >= 0.10
    ok = ok and good
    report.append("  Narrowest copper %.3f mm (JLC minimum 0.10 mm): %s"
                  % (wmin, "PASS" if good else "FAIL"))
    top_polys = [poly for p in model.pieces for poly in p["polys"]]
    for pad in model.sma_pads:
        x, y, w, h = pad["xywh"]
        poly = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
        gap = min(polygon_distance(poly, q) for q in top_polys)
        margin = min(hx - abs(x), hx - abs(x + w), hy - abs(y), hy - abs(y + h))
        good = gap >= 0.10 - 1e-9 and margin >= EDGE_PULLBACK_MM - 1e-6
        ok = ok and good
        report.append("  %-18s gap to feed copper %.3f mm, edge margin %.3f mm: %s"
                      % (pad["name"], gap, margin, "PASS" if good else "FAIL"))
    for via in model.vias:
        inside = any(
            p["xywh"][0] + via["drill_mm"] / 2 + 0.15 <= via["xy"][0] <=
            p["xywh"][0] + p["xywh"][2] - via["drill_mm"] / 2 - 0.15 and
            p["xywh"][1] + via["drill_mm"] / 2 + 0.15 <= via["xy"][1] <=
            p["xywh"][1] + p["xywh"][3] - via["drill_mm"] / 2 - 0.15
            for p in model.sma_pads)
        ok = ok and inside
        if not inside:
            report.append("  %s is not inside its pad with 0.15 mm ring: FAIL"
                          % via["name"])
    if model.vias:
        report.append("  %d vias, %.2f mm drill, each inside a ground pad: %s"
                      % (len(model.vias), model.vias[0]["drill_mm"],
                         "PASS" if ok else "FAIL"))
    for hole in model.mount_holes:
        r = hole["drill_mm"] / 2.0 + hole["clear_mm"]
        x, y = hole["xy"]
        ring = [(x + r * math.cos(2 * math.pi * i / 32),
                 y + r * math.sin(2 * math.pi * i / 32)) for i in range(32)]
        near = min(polygon_distance(ring, q) for q in top_polys)
        good = near >= 1.0
        ok = ok and good
        report.append("  %-12s at (%.2f, %.2f) mm, %.1f mm NPTH, nearest top "
                      "copper %.2f mm beyond its clear ring: %s"
                      % (hole["name"], x, y, hole["drill_mm"], near,
                         "PASS" if good else "FAIL"))
    report.append("  STATUS = %s" % ("PASS" if ok else "FAIL"))
    return ok


# ----------------------------------------------------------------------------
# 8. GEOMETRY VERIFICATION
# ----------------------------------------------------------------------------
def piece_map(model):
    return dict((p["name"], p) for p in model.pieces)


def _bbox(poly):
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return (min(xs), min(ys), max(xs), max(ys))


def _covered(pt, pieces_bboxes):
    for poly, (x0, y0, x1, y1) in pieces_bboxes:
        if x0 - 1e-9 <= pt[0] <= x1 + 1e-9 and y0 - 1e-9 <= pt[1] <= y1 + 1e-9:
            if point_in_polygon(pt, poly):
                return True
    return False


def check_continuity(model, report):
    """Every intended joint must be solid copper in a disc around the joint.

    Testing only that the two solids touch is not enough: two runs that meet at
    an angle have non-parallel end faces, so their union leaves a wedge of bare
    substrate on the outside of the turn even when they overlap generously on
    the inside.  That wedge is a visible crack and a mesh hazard.  The test
    here is therefore a COVERAGE test: a disc of radius 0.45 * min(width) about
    the joint point must be entirely copper, from any piece, not just the two.
    """
    pieces = piece_map(model)
    indexed = []
    for piece in model.pieces:
        for poly in piece["polys"]:
            indexed.append((poly, _bbox(poly)))
    report.append("")
    report.append("Checking copper continuity (overlap %.3f mm per side, "
                  "corner fill %s)..."
                  % (model.over, "ON" if CORNER_FILL else "OFF"))
    failures = []
    angles = [2 * math.pi * k / 48.0 for k in range(48)]
    for joint in model.joints:
        a = pieces.get(joint["a"])
        b = pieces.get(joint["b"])
        if a is None or b is None:
            failures.append(joint)
            report.append("  %-30s %-44s MISSING PIECE"
                          % (joint["a"], joint["label"]))
            continue
        pt = tuple(joint["xy"])
        r_test = 0.45 * min(a.get("w_ref", 1.0), b.get("w_ref", 1.0))
        local = [(poly, bb) for poly, bb in indexed
                 if bb[0] - r_test <= pt[0] <= bb[2] + r_test
                 and bb[1] - r_test <= pt[1] <= bb[3] + r_test]
        worst_r = 0.0
        holes = 0
        for step in range(1, 7):
            rad = r_test * step / 6.0
            for ang in angles:
                probe = (pt[0] + rad * math.cos(ang), pt[1] + rad * math.sin(ang))
                if not _covered(probe, local):
                    holes += 1
                    worst_r = max(worst_r, rad)
        inside_a = any(point_in_polygon(pt, poly) for poly in a["polys"])
        inside_b = any(point_in_polygon(pt, poly) for poly in b["polys"])
        ok = inside_a and inside_b and holes == 0
        if not ok:
            failures.append(joint)
        report.append("  %-30s %-42s %s"
                      % (a["name"], joint["label"],
                         "PASS (solid to %.3f mm)" % r_test if ok else
                         "FAIL (%d voids out to %.3f mm)" % (holes, worst_r)))
    report.append("  Joints checked: %d   failures: %d" % (len(model.joints),
                                                           len(failures)))
    if not failures:
        report.append("  All feed paths geometrically continuous, no corner "
                      "voids: PASS")
    return failures


def check_patch_clearance(model, report):
    """Edge-to-edge distance from every feed conductor to every patch it does
    not feed.  Centreline distances are never used."""
    patches = [p for p in model.pieces if p["kind"] == "patch"]
    feeds = [p for p in model.pieces if p["kind"] != "patch"]
    worst = (1e18, None, None)
    rows = []
    for f in feeds:
        owner_patch = None
        for route in model.routes:
            if route["name"] == f["owner"] and route["leaf"]:
                owner_patch = route["leaf"]
        local = (1e18, None)
        for patch in patches:
            if patch["name"] == owner_patch:
                continue
            d = min(polygon_distance(fp, pp)
                    for fp in f["polys"] for pp in patch["polys"])
            if d < local[0]:
                local = (d, patch["name"])
        if local[1] is not None:
            rows.append((f["name"], local[1], local[0]))
            if local[0] < worst[0]:
                worst = (local[0], f["name"], local[1])
    rows.sort(key=lambda r: r[2])
    report.append("")
    report.append("Feed-to-patch clearance verification (non-owning patches only)")
    if not rows:
        report.append("  Single-element design: no non-owning patch exists. "
                      "SKIPPED")
        return True, (float("inf"), None, None)
    report.append("")
    report.append("  %-34s %-16s %s" % ("Route piece", "Nearest patch", "Clearance"))
    for name, patch, d in rows[:14]:
        report.append("  %-34s %-16s %7.3f mm" % (name, patch, d))
    report.append("  ... %d further pieces have larger clearance" % max(0, len(rows) - 14))
    report.append("")
    report.append("  Minimum non-owning patch clearance = %.3f mm (%s vs %s)"
                  % (worst[0], worst[1], worst[2]))
    report.append("  Required clearance                 = %.3f mm"
                  % MIN_FEED_PATCH_CLEARANCE_MM)
    ok = worst[0] >= MIN_FEED_PATCH_CLEARANCE_MM - 1e-6
    report.append("  STATUS = %s" % ("PASS" if ok else "FAIL"))
    return ok, worst


def check_feed_feed(model, report):
    """Feed pieces that are not joined must not touch or crowd each other."""
    joined = set()
    for joint in model.joints:
        joined.add((joint["a"], joint["b"]))
        joined.add((joint["b"], joint["a"]))
    feeds = [p for p in model.pieces if p["kind"] != "patch"]
    tee_pts = [(p["xy"][0], p["xy"][1]) for p in model.pieces
               if p["cls"] == "disc" or p.get("tee")]
    worst = (1e18, None, None)
    for i in range(len(feeds)):
        for j in range(i + 1, len(feeds)):
            a, b = feeds[i], feeds[j]
            if (a["name"], b["name"]) in joined:
                continue
            if a["owner"] == b["owner"]:
                continue
            d = min(polygon_distance(pa, pb)
                    for pa in a["polys"] for pb in b["polys"])
            if d > 1e-9 and d >= worst[0]:
                continue
            near_tee = False
            for tx, ty in tee_pts:
                da = min(polygon_distance([(tx, ty), (tx + 1e-6, ty),
                                           (tx, ty + 1e-6)], pa) for pa in a["polys"])
                db = min(polygon_distance([(tx, ty), (tx + 1e-6, ty),
                                           (tx, ty + 1e-6)], pb) for pb in b["polys"])
                if da <= model.junction_r.v + 1e-6 and db <= model.junction_r.v + 1e-6:
                    near_tee = True
                    break
            if near_tee:
                continue
            if d < worst[0]:
                worst = (d, a["name"], b["name"])
    report.append("")
    report.append("Feed-to-feed clearance (unconnected conductors)")
    if worst[1] is None:
        report.append("  No unconnected conductor pair exists. SKIPPED")
        return True, worst
    report.append("  Minimum = %.3f mm (%s vs %s); required %.3f mm"
                  % (worst[0], worst[1], worst[2], MIN_FEED_FEED_CLEARANCE_MM))
    ok = worst[0] >= MIN_FEED_FEED_CLEARANCE_MM - 1e-6
    report.append("  STATUS = %s" % ("PASS" if ok else "FAIL"))
    return ok, worst


def check_board_containment(model, report):
    """Every copper piece must sit inside the board with the design margin."""
    hx, hy = model.bx.v / 2.0, model.by.v / 2.0
    worst = (1e18, None)
    for p in model.pieces:
        for poly in p["polys"]:
            for (x, y) in poly:
                d = min(hx - abs(x), hy - abs(y))
                if d < worst[0]:
                    worst = (d, p["name"])
    report.append("")
    report.append("Board containment")
    report.append("  Board %.1f x %.1f mm; tightest copper-to-edge margin "
                  "%.3f mm (%s)" % (model.bx.v, model.by.v, worst[0], worst[1]))
    # every top-copper piece must respect the JLC copper-to-edge pull-back
    ok = worst[0] >= EDGE_PULLBACK_MM - 1e-6
    report.append("  Required copper-to-edge pull-back %.3f mm: %s"
                  % (EDGE_PULLBACK_MM, "PASS" if ok else "FAIL"))
    return ok, worst


def check_symmetry(model, report):
    """Geometrically equivalent branches must have identical lengths."""
    groups = {}
    for route in model.routes:
        groups.setdefault(route["stage"], []).append(route)
    report.append("")
    report.append("Branch symmetry by stage")
    ok = True
    for stage in sorted(groups):
        lengths = [r["length_mm"] for r in groups[stage]]
        spread = max(lengths) - min(lengths)
        ok = ok and spread < 1e-6
        report.append("  Stage %d: %2d branches, length %9.4f mm, spread %.2e mm  %s"
                      % (stage, len(lengths), lengths[0], spread,
                         "PASS" if spread < 1e-6 else "FAIL"))
    return ok


def path_table(model, report):
    """Per-patch length and phase budget at F0."""
    s = model.synthesis
    b50 = 360.0 * math.sqrt(s["line50"]["eeff_used"]) * F0_GHZ / C_MM_GHZ
    b70 = 360.0 * math.sqrt(s["divider"]["eeff_used"]) * F0_GHZ / C_MM_GHZ
    bpt = 360.0 * math.sqrt(s["patch_transformer"]["eeff_used"]) * F0_GHZ / C_MM_GHZ
    btr = 0.5 * (b50 + b70)
    rows = []
    for leaf in model.leaf_paths:
        p = leaf["parts"]
        phase = p["L50"] * b50 + p["Lq"] * b70 + p["Ltrans"] * btr + p["Lpt"] * bpt
        rows.append((leaf["patch"], p["L50"], p["Lq"], p["Ltrans"], p["Lpt"],
                     leaf["path_mm"], phase))
    rows.sort()
    ref = rows[0][6]
    report.append("")
    report.append("Port-to-patch budget (all lengths mm, centreline, diagonals true length)")
    report.append("")
    report.append("  %-12s %9s %9s %9s %9s %10s %10s %9s" %
                  ("Element", "50ohm", "70.71ohm", "tapers", "patchTx",
                   "total", "phase[deg]", "dphi"))
    for name, l50, lq, lt, lpt, total, phase in rows:
        report.append("  %-12s %9.4f %9.4f %9.4f %9.4f %10.4f %10.2f %9.2e"
                      % (name.replace("Patch_", ""), l50, lq, lt, lpt, total,
                         phase, phase - ref))
    totals = [r[5] for r in rows]
    phases = [r[6] for r in rows]
    report.append("")
    report.append("  dL_max  = %.3e mm   dphi_max = %.3e deg"
                  % (max(totals) - min(totals), max(phases) - min(phases)))
    report.append("  Per-class lengths identical across all 16 paths: %s"
                  % ("YES" if max(totals) - min(totals) < 1e-6 else "NO"))
    return max(phases) - min(phases)


# ----------------------------------------------------------------------------
# 9. NUMERICAL REPORT
# ----------------------------------------------------------------------------
def electrical_report(model, report):
    s = model.synthesis
    report.append("=" * 100)
    report.append("4x4 CORPORATE-FED PATCH ARRAY  -  ANALYTICAL STARTING GEOMETRY")
    report.append("f0 = %.3f GHz   band %.2f - %.2f GHz   FR-4 er = %.2f (copper "
                  "sizing), %.2f (HFSS board material)  tand = %.3f  h = %.2f mm  "
                  "tCu = %.3f mm"
                  % (F0_GHZ, BAND_GHZ[0], BAND_GHZ[1], ER, ER_BOARD, TAND, H_MM,
                     COPPER_MM))
    report.append("lambda0 = %.3f mm" % s["lambda0_mm"])
    report.append("=" * 100)

    report.append("")
    report.append("Microstrip lines (Hammerstad-Jensen Z0, Kirschning-Jansen dispersion)")
    report.append("")
    report.append("  %-14s %10s %10s %9s %9s %12s %12s"
                  % ("Line", "Width(mm)", "Z0(ohm)", "Eeff_dc", "Eeff(f0)",
                     "Lambda_g(mm)", "Lambda_g/4"))
    for label, key in (("50 ohm", "line50"), ("70.71 ohm", "divider"),
                       ("Patch Tx", "patch_transformer")):
        d = s[key]
        report.append("  %-14s %10.4f %10.3f %9.4f %9.4f %12.4f %12.4f"
                      % (label, d["W_mm"], d["Z_actual_ohm"], d["eeff_static"],
                         d["eeff_dispersive"], d["lambda_g_mm"], d["Lquarter_mm"]))
    report.append("")
    report.append("  Quasi-static quarter waves for comparison: 50 ohm %.4f, "
                  "70.71 ohm %.4f, patch Tx %.4f mm"
                  % (s["line50"]["Lquarter_static_mm"],
                     s["divider"]["Lquarter_static_mm"],
                     s["patch_transformer"]["Lquarter_static_mm"]))
    report.append("  Dispersion is applied at f0 (USE_DISPERSIVE_QUARTER_WAVE = %s)."
                  % USE_DISPERSIVE_QUARTER_WAVE)
    report.append("  Note: dispersion shortens lambda_g while a lower real FR-4 "
                  "er near 6 GHz lengthens it; the two partly cancel, which is")
    report.append("  why the quasi-static numbers sit close to the user's "
                  "full-wave tuned 7.25 mm.  Set ER to the laminate's measured "
                  "value at 6 GHz if known.")

    report.append("")
    report.append("Radiating element")
    report.append("  Wp analytic = %.4f mm, Wp used = %.4f mm (full-wave verified)"
                  % (s["Wp_analytic_mm"], s["Wp_mm"]))
    report.append("  Lp analytic = %.4f mm (deltaL = %.4f, eeff = %.4f), "
                  "Lp used = %.4f mm (full-wave verified, feed-loaded)"
                  % (s["Lp_analytic_mm"], s["deltaL_mm"], s["eeff_patch"], s["Lp_mm"]))
    report.append("  Slot conductances G1 = %.4e S, G12 = %.4e S -> lossless "
                  "edge R = %.1f ohm" % (s["G1_S"], s["G12_S"],
                                         s["R_edge_analytic_ohm"]))
    report.append("  Edge R used = %.2f ohm -> patch transformer %.2f ohm, "
                  "%.4f mm wide, %.4f mm long."
                  % (s["R_edge_used_ohm"], s["patch_transformer"]["Z_actual_ohm"],
                     s["patch_transformer"]["W_mm"],
                     s["patch_transformer"]["Lquarter_mm"]))
    report.append("  Lp = %.4f mm = %.4f mm scaled by %.4f (measured resonance "
                  "%.3f GHz / f0)." % (s["Lp_mm"], s["Lp_base_mm"],
                                       s["Lp_scale"], MEASURED_RESONANCE_GHZ))
    report.append("")
    report.append("HFSS calibration applied")
    report.append("  The supplied single-patch validation gives S11 = -12.905 dB "
                  "and Z = 37.15 - j15.30 ohm at 5.85 GHz; its S11 minimum is "
                  "near 5.93 GHz.")
    report.append("  The supplied full array gives S11 = -8.26 dB and "
                  "Z = %.2f %+.2fj ohm at 5.85 GHz."
                  % (MEASURED_ARRAY_R_OHM, MEASURED_ARRAY_X_OHM))
    report.append("  Patch and corporate-tree geometry are retained.  The array "
                  "input is corrected with a measured-impedance open-stub tuner.")
    if ENABLE_ARRAY_INPUT_STUB and "Array4x4" in model.name:
        report.append("  Matching-stub tee: %.4f mm inward from the existing port."
                      % MATCH_STUB_FROM_PORT_MM)
        report.append("  Open 50-ohm stub: %.4f mm physical (band candidate)."
                      % MATCH_STUB_PHYSICAL_MM)
        report.append("  Stub direction along y: %+.0f; pointing toward lower board area."
                      % STUB_DIRECTION)

    report.append("")
    report.append("Corporate divider impedance proof (1 -> 2 -> 4 -> 8 -> 16)")
    report.append("  Claim: every branch presents 50 ohm at its own input.")
    report.append("  Base case (stage 4): patch edge %.2f ohm -> %.2f ohm "
                  "quarter wave gives %.2f^2/%.2f = %.2f ohm;"
                  % (EDGE_R_OHM, s["patch_transformer"]["Z_actual_ohm"],
                     s["patch_transformer"]["Z_actual_ohm"], EDGE_R_OHM,
                     s["patch_transformer"]["Z_actual_ohm"] ** 2 / EDGE_R_OHM))
    report.append("    the 50 ohm run is then matched, so the stage-4 branch "
                  "input (before its own 70.71 ohm section) is 50 ohm.")
    report.append("  Induction step: a branch whose subtree presents 50 ohm is "
                  "raised by its %.3f ohm quarter wave to %.3f^2/50 = %.2f ohm;"
                  % (s["divider"]["Z_actual_ohm"], s["divider"]["Z_actual_ohm"],
                     s["divider"]["Z_actual_ohm"] ** 2 / 50.0))
    report.append("    two identical branches in parallel at the tee give "
                  "%.2f ohm, and the parent 50 ohm line is matched."
                  % (s["divider"]["Z_actual_ohm"] ** 2 / 100.0))
    report.append("  Therefore the SAME 70.71 ohm quarter wave is correct at "
                  "every one of the four stages, provided it is placed on the")
    report.append("  CHILD side of each tee, immediately at the junction.  It is "
                  "placed there in this script, so there is no stray 50 ohm")
    report.append("  stub between a tee and its transformer.  A 35.36 ohm "
                  "section would only be correct for the opposite placement")
    report.append("  (transformer on the parent side, feeding two 50 ohm "
                  "branches directly); that variant is not used here.")

    report.append("")
    report.append("Pitch and board")
    report.append("  dx = %.4f mm = %.4f lambda0     dy = %.4f mm = %.4f lambda0"
                  % (model.dx.v, model.dx.v / s["lambda0_mm"],
                     model.dy.v, model.dy.v / s["lambda0_mm"]))
    if model.pitch_info.get("locked"):
        report.append("  Array envelope locked to the supplied solved model: "
                      "dx = %.4f mm, dy = %.4f mm, board = %.1f x %.1f mm."
                      % (model.dx.v, model.dy.v,
                         model.bx.v, model.by.v))
    else:
        report.append("  Pitch increase applied to both axes: %.4f mm "
                      "(clearance-driven, see solve_pitch)."
                      % model.pitch_info["delta_mm"])
    report.append("  Broadside grating lobes require d >= lambda0; both pitches "
                  "are below %.2f lambda0, so no grating lobe exists in real"
                  % PITCH_MAX_FRACTION_LAMBDA0)
    report.append("  space, at the cost of a slightly narrower beam and "
                  "somewhat higher sidelobes than the previous spacing.")
    report.append("  Original board 155.0 x 145.0 mm; corrected board %.1f x %.1f mm"
                  % (model.bx.v, model.by.v))
    report.append("  Copper half-extent %.3f x %.3f mm plus %.1f mm margin."
                  % (model.copper_half_extent[0], model.copper_half_extent[1],
                     BOARD_MARGIN_MM))
    report.append("  Physical aperture A_board = %.1f mm^2 (was 22475.0 mm^2)."
                  % (model.bx.v * model.by.v))


def run_checks(model):
    report = []
    electrical_report(model, report)
    minimum_air_pad = C_MM_GHZ / (4.0 * 5.3)
    physical_ok = (model.pad.v >= minimum_air_pad and
                   abs(model.stub_l.v - MATCH_STUB_PHYSICAL_MM) < 1e-9 and
                   abs(model.stub_d.v - MATCH_STUB_FROM_PORT_MM) < 1e-9 and
                   STUB_DIRECTION == -1.0 and
                   H_MM > 0 and COPPER_MM > 0 and
                   ER > 1.0 and ER_BOARD > 1.0 and TAND >= 0 and
                   COPPER_CONDUCTIVITY_S_PER_M > 0)
    report.append("")
    report.append("Nominal physical-input checks")
    report.append("  airPad %.3f mm >= lambda0/4 at 5.3 GHz (%.3f mm): %s"
                  % (model.pad.v, minimum_air_pad,
                     "PASS" if model.pad.v >= minimum_air_pad else "FAIL"))
    report.append("  stubL=%.3f mm, stubD=%.3f mm, downward route: %s" %
                  (MATCH_STUB_PHYSICAL_MM, MATCH_STUB_FROM_PORT_MM,
                   "PASS" if (abs(model.stub_l.v - MATCH_STUB_PHYSICAL_MM) < 1e-9 and
                              abs(model.stub_d.v - MATCH_STUB_FROM_PORT_MM) < 1e-9 and
                              STUB_DIRECTION == -1.0) else "FAIL"))
    report.append("  ER, tanD, thickness and copper conductivity valid: %s" %
                  ("PASS" if physical_ok else "FAIL"))
    report.append("  These are assumed properties, not measurements of a board.")
    failures = check_continuity(model, report)
    patch_ok, patch_worst = check_patch_clearance(model, report)
    feed_ok, _feed_worst = check_feed_feed(model, report)
    board_ok, _board_worst = check_board_containment(model, report)
    sym_ok = check_symmetry(model, report)
    fab_ok = check_fabrication(model, report)
    dphi = path_table(model, report)
    ok = (physical_ok and not failures and patch_ok and feed_ok and
          board_ok and sym_ok and fab_ok)
    report.append("")
    report.append("-" * 100)
    report.append("OVERALL GEOMETRY VALIDATION: %s" % ("PASS" if ok else "FAIL"))
    report.append("This candidate geometry is not yet full-wave solved.  No claim is made "
                  "about S11, VSWR, gain, coupling or efficiency.")
    report.append("-" * 100)
    model.min_patch_clearance = patch_worst
    model.max_dphi = dphi
    return ok, report


# ----------------------------------------------------------------------------
# 10. SVG PREVIEW
# ----------------------------------------------------------------------------
COLORS = {"patch": "#f5be23", "line50": "#f5be23", "divider": "#f5be23",
          "transition": "#f5be23", "patchT": "#f5be23"}


def write_preview(model, out):
    data = model.data()
    with open(os.path.join(out, model.name + "_geometry.json"), "w") as stream:
        json.dump(data, stream, indent=2)

    bx, by = data["board_mm"]
    scale, margin = 5.0, 60.0
    width = max(bx * scale + 2 * margin, 980.0)
    height = by * scale + 2 * margin + 116

    def sx(x):
        return margin + (x + bx / 2.0) * scale

    def sy(y):
        return margin + (by / 2.0 - y) * scale

    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="%g" height="%g" '
             'viewBox="0 0 %g %g">' % (width, height, width, height),
             '<rect width="100%" height="100%" fill="#ffffff"/>',
             '<g font-family="Arial, sans-serif" fill="#152838">',
             '<text x="%g" y="26" font-size="19">%s | %.2f GHz FR-4 corrected '
             'starting geometry</text>' % (margin, model.name, F0_GHZ),
             '<text x="%g" y="46" font-size="12">Nominal dimensions in mm. '
             'Unoptimized; no simulated performance is shown.</text>' % margin,
             '<rect x="%g" y="%g" width="%g" height="%g" fill="#32914b" '
             'stroke="#87949d"/>' % (margin, margin, bx * scale, by * scale)]

    # clearance envelopes around every patch
    c = MIN_FEED_PATCH_CLEARANCE_MM
    for piece in model.pieces:
        if piece["kind"] != "patch":
            continue
        x, y, w, h = piece["xywh"]
        parts.append('<rect x="%.3f" y="%.3f" width="%.3f" height="%.3f" '
                     'fill="none" stroke="#c03a3a" stroke-width="0.8" '
                     'stroke-dasharray="4 3"/>'
                     % (sx(x - c), sy(y + h + c), (w + 2 * c) * scale,
                        (h + 2 * c) * scale))

    for piece in model.pieces:
        color = COLORS.get(piece["kind"], "#777777")
        if piece["cls"] == "disc":
            parts.append('<circle cx="%.3f" cy="%.3f" r="%.3f" fill="%s"><title>'
                         '%s</title></circle>'
                         % (sx(piece["xy"][0]), sy(piece["xy"][1]),
                            model.junction_r.v * scale, color, piece["name"]))
            continue
        for poly in piece["polys"]:
            pts = " ".join("%.3f,%.3f" % (sx(px), sy(py)) for px, py in poly)
            parts.append('<polygon points="%s" fill="%s" stroke="%s" '
                         'stroke-width="0.25"><title>%s</title></polygon>'
                         % (pts, color, color, piece["name"]))
        if piece["kind"] == "patch":
            x, y, w, h = piece["xywh"]
            parts.append('<text x="%.3f" y="%.3f" text-anchor="middle" '
                         'font-size="11">%s</text>'
                         % (sx(x + w / 2.0), sy(y + h / 2.0) + 4,
                            piece["name"].replace("Patch_", "")))

    # marker at the binding clearance
    d, fname, pname = model.min_patch_clearance
    if fname is None or pname is None:
        d, fname, pname = 0.0, None, None
    pieces = piece_map(model)
    best = (1e18, None, None)
    fa = pieces.get(fname)
    pa = pieces.get(pname)
    if fa is None or pa is None:
        fa = pa = {"polys": []}
    for poly_a in fa["polys"]:
        for poly_b in pa["polys"]:
            for (ax, ay) in poly_a:
                for i in range(len(poly_b)):
                    b1, b2 = poly_b[i], poly_b[(i + 1) % len(poly_b)]
                    dd = point_segment_distance((ax, ay), b1, b2)
                    if dd < best[0]:
                        ux, uy = b2[0] - b1[0], b2[1] - b1[1]
                        den = ux * ux + uy * uy
                        tt = 0.0 if den < 1e-18 else max(0.0, min(1.0, (
                            (ax - b1[0]) * ux + (ay - b1[1]) * uy) / den))
                        best = (dd, (ax, ay), (b1[0] + tt * ux, b1[1] + tt * uy))
    if best[1] is not None:
        parts.append('<line x1="%.3f" y1="%.3f" x2="%.3f" y2="%.3f" '
                     'stroke="#c03a3a" stroke-width="1.6"/>'
                     % (sx(best[1][0]), sy(best[1][1]),
                        sx(best[2][0]), sy(best[2][1])))
        parts.append('<circle cx="%.3f" cy="%.3f" r="4" fill="none" '
                     'stroke="#c03a3a" stroke-width="1.6"/>'
                     % (sx(best[1][0]), sy(best[1][1])))
        parts.append('<text x="%.3f" y="%.3f" font-size="11" fill="#c03a3a">min '
                     '%.2f mm</text>' % (sx(best[1][0]) + 7,
                                         sy(best[1][1]) - 6, d))

    yy = margin + by * scale + 24
    parts.append('<text x="%g" y="%g" font-size="13">Board %.1f x %.1f mm | '
                 'Patch %.3f x %.3f | dx = %.3f mm (%.3f lambda0) | dy = %.3f mm '
                 '(%.3f lambda0)</text>'
                 % (margin, yy, bx, by, model.Wp.v, model.Lp.v, model.dx.v,
                    model.dx.v / model.synthesis["lambda0_mm"], model.dy.v,
                    model.dy.v / model.synthesis["lambda0_mm"]))
    parts.append('<text x="%g" y="%g" font-size="12">Yellow: all copper | '
                 'Green: FR-4 substrate | Red dashed: clearance envelopes'
                 '</text>' % (margin, yy + 20))
    parts.append('<text x="%g" y="%g" font-size="12">Dashed red: %.1f mm '
                 'feed-to-patch clearance envelope; no unrelated feed '
                 'conductor may enter it.</text>' % (margin, yy + 38, c))
    status = ("PASS" if (best[1] is None or d >= MIN_FEED_PATCH_CLEARANCE_MM - 1e-6)
              else "FAIL")
    fill = "#1d7a3c" if status == "PASS" else "#c03a3a"
    parts.append('<text x="%g" y="%g" font-size="12" fill="%s">Minimum '
                 'non-owning feed-to-patch clearance %.3f mm (required %.2f mm) '
                 '- %s | joint overlap %.3f mm | max feed phase spread %.2e deg'
                 '</text>' % (margin, yy + 58, fill, d,
                              MIN_FEED_PATCH_CLEARANCE_MM, status,
                              CONNECT_OVERLAP_MM, model.max_dphi))
    parts.append('<text x="%g" y="%g" font-size="11" fill="#5a6672">Unsimulated '
                 'geometry. No S11, gain, efficiency or coupling result is '
                 'implied.</text>' % (margin, yy + 76))
    parts.append('</g></svg>')
    with open(os.path.join(out, model.name + "_layout.svg"), "w") as stream:
        stream.write("\n".join(parts))


# ----------------------------------------------------------------------------
# 11. NATIVE AEDT PRIMITIVES
# ----------------------------------------------------------------------------
AEDT_COPPER_YELLOW = "(245 190 35)"
AEDT_SUBSTRATE_GREEN = "(50 145 75)"
AEDT_COLORS = {"patch": AEDT_COPPER_YELLOW,
               "line50": AEDT_COPPER_YELLOW,
               "divider": AEDT_COPPER_YELLOW,
               "transition": AEDT_COPPER_YELLOW,
               "patchT": AEDT_COPPER_YELLOW}


def attributes(name, material="vacuum", solve_inside=True,
               color="(160 160 160)", transparency=0):
    return ["NAME:Attributes", "Name:=", name, "Flags:=", "",
            "Color:=", color, "Transparency:=", transparency,
            "PartCoordinateSystem:=", "Global", "MaterialValue:=", '"' + material + '"',
            "SolveInside:=", solve_inside]


def create_box(editor, name, xyz, sizes, material, solve_inside, color,
               transparency=0):
    editor.CreateBox(["NAME:BoxParameters",
                      "XPosition:=", xyz[0], "YPosition:=", xyz[1],
                      "ZPosition:=", xyz[2], "XSize:=", sizes[0],
                      "YSize:=", sizes[1], "ZSize:=", sizes[2]],
                     attributes(name, material, solve_inside, color, transparency))


def create_trace(editor, name, points, width, color):
    """Flat rectangular copper trace; AEDT rounds the bends (Curved)."""
    pl_points = ["NAME:PolylinePoints"]
    for x, y in points:
        pl_points.append(["NAME:PLPoint", "X:=", x, "Y:=", y, "Z:=", "h+tCu/2"])
    pl_segments = ["NAME:PolylineSegments"]
    for index in range(len(points) - 1):
        pl_segments.append(["NAME:PLSegment", "SegmentType:=", "Line",
                            "StartIndex:=", index, "NoOfPoints:=", 2])
    editor.CreatePolyline(
        ["NAME:PolylineParameters", "IsPolylineCovered:=", True,
         "IsPolylineClosed:=", False, pl_points, pl_segments,
         ["NAME:PolylineXSection", "XSectionType:=", "Rectangle",
          "XSectionOrient:=", "Auto", "XSectionWidth:=", width,
          "XSectionTopWidth:=", width, "XSectionHeight:=", "tCu",
          "XSectionNumSegments:=", "0", "XSectionBendType:=", "Curved"]],
        attributes(name, COPPER_MATERIAL_NAME, False, color))


def create_sheet_solid(editor, name, points, color):
    """Closed covered polygon at z = h, thickened upward by tCu."""
    pl_points = ["NAME:PolylinePoints"]
    for x, y in points:
        pl_points.append(["NAME:PLPoint", "X:=", x, "Y:=", y, "Z:=", "h"])
    pl_segments = ["NAME:PolylineSegments"]
    for index in range(len(points)):
        pl_segments.append(["NAME:PLSegment", "SegmentType:=", "Line",
                            "StartIndex:=", index, "NoOfPoints:=", 2])
    editor.CreatePolyline(
        ["NAME:PolylineParameters", "IsPolylineCovered:=", True,
         "IsPolylineClosed:=", True, pl_points, pl_segments,
         ["NAME:PolylineXSection", "XSectionType:=", "None",
          "XSectionOrient:=", "Auto", "XSectionWidth:=", "0mm",
          "XSectionTopWidth:=", "0mm", "XSectionHeight:=", "0mm",
          "XSectionNumSegments:=", "0", "XSectionBendType:=", "Corner"]],
        attributes(name, COPPER_MATERIAL_NAME, False, color))
    editor.ThickenSheet(["NAME:Selections", "Selections:=", name,
                         "NewPartsModelFlag:=", "Model"],
                        ["NAME:SheetThickenParameters", "Thickness:=", "tCu",
                         "BothSides:=", False])


def create_disc(editor, name, xy, radius, color):
    editor.CreateCylinder(["NAME:CylinderParameters", "XCenter:=", xy[0],
                           "YCenter:=", xy[1], "ZCenter:=", "h",
                           "Radius:=", radius, "Height:=", "tCu",
                           "WhichAxis:=", "Z", "NumSides:=", "0"],
                          attributes(name, COPPER_MATERIAL_NAME, False, color))


def create_cylinder(editor, name, xy, z, radius, height, material, color):
    # HFSS rule: a non-conducting material (vacuum, dielectric) must have
    # "Solve Inside" ON; a metal (copper) has it OFF.
    solve_inside = (material != COPPER_MATERIAL_NAME)
    editor.CreateCylinder(["NAME:CylinderParameters", "XCenter:=", xy[0],
                           "YCenter:=", xy[1], "ZCenter:=", z,
                           "Radius:=", radius, "Height:=", height,
                           "WhichAxis:=", "Z", "NumSides:=", "0"],
                          attributes(name, material, solve_inside, color))


def build_fabrication_features(editor, model, log):
    """SMA ground pads, vias and mounting holes in the HFSS model."""
    if model.sma_pads:
        for pad in model.sma_pads:
            x, y, w, h = pad["expr"]
            create_box(editor, pad["name"], [x, y, "h"], [w, h, "tCu"],
                       COPPER_MATERIAL_NAME, False, AEDT_COPPER_YELLOW)
        via_names = []
        for via in model.vias:
            # Solid copper cylinder from the ground top face (z=0) to the pad
            # bottom face (z=h).  A plated via is modelled as filled; at
            # 5.85 GHz the difference is negligible.
            create_cylinder(editor, via["name"], via["expr"], "0mm",
                            "(%s)/2" % via["drill_expr"], "h",
                            COPPER_MATERIAL_NAME, AEDT_COPPER_YELLOW)
            via_names.append(via["name"])
        # Cut the via holes out of the substrate (keep the vias).
        editor.Subtract(["NAME:Selections", "Blank Parts:=", "Substrate",
                         "Tool Parts:=", ",".join(via_names)],
                        ["NAME:SubtractParameters", "KeepOriginals:=", True])
        log.write("%s: %d SMA ground pads and %d vias created.\n"
                  % (model.name, len(model.sma_pads), len(via_names)))
    if model.mount_holes:
        drill_tools, clear_tools = [], []
        for hole in model.mount_holes:
            # hole through the substrate
            create_cylinder(editor, hole["name"], hole["expr"], "0mm",
                            "mountHoleD/2", "h", "vacuum", "(200 200 200)")
            # larger copper-free ring in the ground plane
            create_cylinder(editor, hole["name"] + "_GndClear", hole["expr"],
                            "-tCu", "mountHoleD/2+mountHoleClear", "tCu",
                            "vacuum", "(200 200 200)")
            drill_tools.append(hole["name"])
            clear_tools.append(hole["name"] + "_GndClear")
        editor.Subtract(["NAME:Selections", "Blank Parts:=", "Substrate",
                         "Tool Parts:=", ",".join(drill_tools)],
                        ["NAME:SubtractParameters", "KeepOriginals:=", False])
        editor.Subtract(["NAME:Selections", "Blank Parts:=", "Ground",
                         "Tool Parts:=", ",".join(clear_tools)],
                        ["NAME:SubtractParameters", "KeepOriginals:=", False])
        log.write("%s: %d corner mounting holes cut.\n"
                  % (model.name, len(model.mount_holes)))


def set_variables(design, variables):
    props = ["NAME:NewProps"]
    for name, value in variables:
        props.append(["NAME:" + name, "PropType:=", "VariableProp",
                      "UserDef:=", True, "Value:=", value])
    design.ChangeProperty(["NAME:AllTabs", ["NAME:LocalVariableTab",
                          ["NAME:PropServers", "LocalVariables"], props]])


# ----------------------------------------------------------------------------
# 12. HFSS SETUP, FAR FIELD AND REPORTS
# ----------------------------------------------------------------------------
def create_setup(design):
    analysis = design.GetModule("AnalysisSetup")
    analysis.InsertSetup("HfssDriven", [
        "NAME:Setup1", "SolveType:=", "Single",
        "Frequency:=", "%gGHz" % F0_GHZ,
        "MaxDeltaS:=", ADAPTIVE_MAX_DELTA_S,
        "PortsOnly:=", False, "UseMatrixConv:=", False,
        "MaximumPasses:=", (min(ADAPTIVE_MAX_PASSES, STUDENT_MAX_PASSES)
                            if STUDENT_MESH_SAFE else ADAPTIVE_MAX_PASSES),
        "MinimumPasses:=", 3,
        "MinimumConvergedPasses:=", 2, "PercentRefinement:=", 8,
        "IsEnabled:=", True, "BasisOrder:=", 1, "DoLambdaRefine:=", True,
        "DoMaterialLambda:=", True, "SetLambdaTarget:=", False,
        "PortAccuracy:=", 2, "SaveAnyFields:=", True, "SaveRadFieldsOnly:=", False])
    analysis.InsertFrequencySweep("Setup1", [
        "NAME:WideSweep", "IsEnabled:=", True, "RangeType:=", "LinearStep",
        "RangeStart:=", "5.3GHz", "RangeEnd:=", "6.4GHz", "RangeStep:=", "0.005GHz",
        "Type:=", "Interpolating", "SaveFields:=", False, "SaveRadFields:=", False,
        "InterpTolerance:=", 0.5, "InterpMaxSolns:=", 250, "InterpMinSolns:=", 0,
        "InterpMinSubranges:=", 1])
    # Discrete points with fields for far-field evaluation at 5.70/5.85/6.00 GHz
    analysis.InsertFrequencySweep("Setup1", [
        "NAME:BandFields", "IsEnabled:=", True, "RangeType:=", "LinearStep",
        "RangeStart:=", "5.7GHz", "RangeEnd:=", "6GHz", "RangeStep:=", "0.15GHz",
        "Type:=", "Discrete", "SaveFields:=", True, "SaveRadFields:=", True])
    # Direct solves verify target-band coverage independently of interpolation.
    analysis.InsertFrequencySweep("Setup1", [
        "NAME:ResonanceCheck", "IsEnabled:=", True,
        "RangeType:=", "LinearStep",
        "RangeStart:=", "%gGHz" % CHECK_SWEEP_START_GHZ,
        "RangeEnd:=", "%gGHz" % CHECK_SWEEP_STOP_GHZ,
        "RangeStep:=", "%gGHz" % CHECK_SWEEP_STEP_GHZ,
        "Type:=", "Discrete", "SaveFields:=", False,
        "SaveRadFields:=", False])


def create_farfield(design):
    radiation = design.GetModule("RadField")
    radiation.InsertInfiniteSphereSetup([
        "NAME:Sphere", "UseCustomRadiationSurface:=", False,
        "CSDefinition:=", "Theta-Phi", "Polarization:=", "Linear",
        "ThetaStart:=", "0deg", "ThetaStop:=", "180deg", "ThetaStep:=", "2deg",
        "PhiStart:=", "-180deg", "PhiStop:=", "180deg", "PhiStep:=", "2deg",
        "UseLocalCS:=", False])


def create_reports(design, log):
    """Best-effort report creation; a failure here never stops the build."""
    reports = design.GetModule("ReportSetup")

    def guard(label, fn):
        try:
            fn()
            log.write("  report OK: %s\n" % label)
        except Exception:
            log.write("  report SKIPPED (%s): %s\n"
                      % (label, traceback.format_exc().splitlines()[-1]))

    band = ["Freq:=", ["RangeStart:=", "%gGHz" % BAND_GHZ[0],
                       "RangeEnd:=", "%gGHz" % BAND_GHZ[1]]]
    guard("dB(S11)", lambda: reports.CreateReport(
        "S11_dB", "Modal Solution Data", "Rectangular Plot",
        "Setup1 : WideSweep", [], band,
        ["X Component:=", "Freq", "Y Component:=", ["dB(S(P1,P1))"]], []))
    guard("dB(S11) direct check", lambda: reports.CreateReport(
        "S11_ResonanceCheck_dB", "Modal Solution Data", "Rectangular Plot",
        "Setup1 : ResonanceCheck", [],
        ["Freq:=", ["All"]],
        ["X Component:=", "Freq", "Y Component:=", ["dB(S(P1,P1))"]], []))
    guard("VSWR", lambda: reports.CreateReport(
        "VSWR_P1", "Modal Solution Data", "Rectangular Plot",
        "Setup1 : WideSweep", [], band,
        ["X Component:=", "Freq", "Y Component:=", ["VSWR(P1)"]], []))

    cuts = ["Context:=", "Sphere", "Freq:=", ["5.85GHz"],
            "Phi:=", ["0deg", "90deg"], "Theta:=", ["All"]]
    guard("Realized gain cuts phi=0/90", lambda: reports.CreateReport(
        "RealizedGain_Cuts_5p85GHz", "Far Fields", "Radiation Pattern",
        "Setup1 : BandFields", ["Context:=", "Sphere"], cuts,
        ["Ang Component:=", "Theta",
         "Mag Component:=", ["dB(RealizedGainTotal)"]], []))
    band_cuts = ["Context:=", "Sphere",
                 "Freq:=", ["5.7GHz", "5.85GHz", "6GHz"],
                 "Phi:=", ["0deg"], "Theta:=", ["All"]]
    guard("Realized gain vs band, phi=0", lambda: reports.CreateReport(
        "RealizedGain_Eplane_Band", "Far Fields", "Radiation Pattern",
        "Setup1 : BandFields", ["Context:=", "Sphere"], band_cuts,
        ["Ang Component:=", "Theta",
         "Mag Component:=", ["dB(RealizedGainTotal)"]], []))

    params = ["Context:=", "Sphere", "Freq:=", ["5.7GHz", "5.85GHz", "6GHz"],
              "Phi:=", ["All"], "Theta:=", ["All"]]
    guard("Antenna parameters table", lambda: reports.CreateReport(
        "AntennaParameters", "Antenna Parameters", "Data Table",
        "Setup1 : BandFields", ["Context:=", "Sphere"], params,
        ["X Component:=", "Freq",
         "Y Component:=", ["dB(PeakRealizedGain)", "dB(PeakGain)",
                           "dB(PeakDirectivity)", "dB(FrontToBackRatio)",
                           "RadiationEfficiency", "TotalEfficiency",
                           "RadiatedPower", "AcceptedPower", "BeamArea"]], []))


# ----------------------------------------------------------------------------
# 13. NATIVE AEDT BUILD
# ----------------------------------------------------------------------------
def build_native(desktop, models, out):
    project = desktop.NewProject()
    project_path = os.path.join(
        out, "FR4_5850_4x4_BandCandidate_" +
        time.strftime("%Y%m%d_%H%M%S") + ".aedt")
    definition = project.GetDefinitionManager()
    material = "FR4_JLC_er%s" % (("%g" % ER_BOARD).replace(".", "p"))
    definition.AddMaterial(["NAME:" + material, "CoordinateSystemType:=", "Cartesian",
                            "BulkOrSurfaceType:=", 1,
                            ["NAME:PhysicsTypes", "set:=", ["Electromagnetic"]],
                            "permittivity:=", str(ER_BOARD),
                            "dielectric_loss_tangent:=", str(TAND),
                            "permeability:=", "1", "conductivity:=", "0"])
    definition.AddMaterial(["NAME:" + COPPER_MATERIAL_NAME,
                            "CoordinateSystemType:=", "Cartesian",
                            "BulkOrSurfaceType:=", 1,
                            ["NAME:PhysicsTypes", "set:=", ["Electromagnetic"]],
                            "permittivity:=", "1", "permeability:=", "1",
                            "conductivity:=", str(COPPER_CONDUCTIVITY_S_PER_M)])
    log_path = os.path.join(out, "hfss_4x4_band_candidate_build_log.txt")
    with open(log_path, "w") as log:
        log.write("4x4 FR-4 array; matching candidate stubL=2.8mm; "
                  "stubD=21.3mm, pointing toward negative y. "
                  "Original patch and corporate-tree dimensions retained. "
                  "FR-4 copper sizing er=%g, HFSS board er=%g, tanD=0.02, h=1.6mm; " % (ER, ER_BOARD) +
                  "copper=0.035mm "
                  "with conductivity 5.8e7 S/m. "
                  "Adaptive maxDeltaS=0.02, maxPasses=18; "
                  "ResonanceCheck is a discrete 5.7-6.0GHz sweep. "
                  "No simulation was started.\n")
        log.write("Flat tees=%s, flat bend fill=%s.\n"
                  % (FLAT_TEES, FLAT_BEND_FILL))
        log.write("Physical limitation: nominal FR-4 and ideal lumped input "
                  "are not measured substrate data or an SMA/coax launch.\n")
        for model in models:
            project.InsertDesign("HFSS", model.name, "DrivenModal", "")
            design = project.SetActiveDesign(model.name)
            # 2025 maps legacy DrivenModal to Hybrid Modal Network.  With no
            # hybrid regions, request the plain modal network where available.
            # AEDT 2018 retains its original DrivenModal setting.
            try:
                solution_type = str(design.GetSolutionType())
                if "Hybrid" in solution_type:
                    design.SetSolutionType("HFSS Modal Network", [
                        "NAME:Options", "EnableAutoOpen:=", False])
                    solution_type = str(design.GetSolutionType())
                log.write("Solution type: %s\n" % solution_type)
            except Exception:
                log.write("Solution type normalization unavailable; "
                          "check HFSS > Solution Type in this version.\n")
            set_variables(design, model.vars + [("f0", "%gGHz" % F0_GHZ)])
            editor = design.SetActiveEditor("3D Modeler")
            editor.SetModelUnits(["NAME:Units Parameter", "Units:=", "mm",
                                  "Rescale:=", False])

            create_box(editor, "Substrate", ["-boardX/2", "-boardY/2", "0mm"],
                       ["boardX", "boardY", "h"], material, True,
                       AEDT_SUBSTRATE_GREEN, 0.6)
            # Ground plane pulled back EDGE_PULLBACK_MM from every board edge.
            if STUDENT_MESH_SAFE:
                # full-size ground in HFSS (pull-back only in the Gerbers)
                create_box(editor, "Ground", ["-boardX/2", "-boardY/2", "-tCu"],
                           ["boardX", "boardY", "tCu"],
                           COPPER_MATERIAL_NAME, False, AEDT_COPPER_YELLOW)
            else:
                create_box(editor, "Ground",
                           ["-boardX/2+edgePullback", "-boardY/2+edgePullback",
                            "-tCu"],
                           ["boardX-2*edgePullback", "boardY-2*edgePullback",
                            "tCu"],
                           COPPER_MATERIAL_NAME, False, AEDT_COPPER_YELLOW)

            copper_names = []
            for piece in model.pieces:
                color = AEDT_COLORS.get(piece["kind"], AEDT_COPPER_YELLOW)
                if piece["cls"] == "box":
                    x, y, w, h = piece["expr"]
                    create_box(editor, piece["name"], [x, y, "h"],
                               [w, h, "tCu"], COPPER_MATERIAL_NAME, False, color)
                elif piece["cls"] == "sheet":
                    create_sheet_solid(editor, piece["name"],
                                       [(p[0], p[1]) for p in piece["expr"]], color)
                elif piece["cls"] == "trace":
                    create_trace(editor, piece["name"],
                                 [(p[0], p[1]) for p in piece["expr"]],
                                 piece["width_expr"], color)
                elif piece["cls"] == "disc":
                    create_disc(editor, piece["name"], piece["expr"],
                                piece["radius_expr"], color)
                copper_names.append(piece["name"])
            if STUDENT_MESH_SAFE and "Array4x4" in model.name:
                # extend the input line over the 0.2 mm pull-back so the
                # port sits on the board edge, as in the solved version
                px0, py0 = model.port
                create_box(editor, "Input50_EdgeExt",
                           [(px0 - model.cover).s, (py0 - model.w50 * 0.5).s, "h"],
                           [(model.edge_pb + model.cover).s, "W50", "tCu"],
                           COPPER_MATERIAL_NAME, False, AEDT_COPPER_YELLOW)
                copper_names.append("Input50_EdgeExt")
            log.write("%s: %d copper solids created.\n"
                      % (model.name, len(copper_names)))

            if UNITE_TOP_COPPER:
                editor.Unite(["NAME:Selections",
                              "Selections:=", ",".join(copper_names)],
                             ["NAME:UniteParameters", "KeepOriginals:=", False])
                survivor = copper_names[0]
                editor.ChangeProperty(["NAME:AllTabs",
                    ["NAME:Geometry3DAttributeTab",
                     ["NAME:PropServers", survivor],
                     ["NAME:ChangedProps",
                      ["NAME:Name", "Value:=", "TopCopper"]]]])
                remaining = list(editor.GetMatchedObjectName("Branch_*")) + \
                    list(editor.GetMatchedObjectName("Patch_*")) + \
                    list(editor.GetMatchedObjectName("RoundTee_*")) + \
                    list(editor.GetMatchedObjectName("Tee_*")) + \
                    list(editor.GetMatchedObjectName("Probe_*")) + \
                    list(editor.GetMatchedObjectName("Input50*"))
                remaining = [n for n in remaining if n != "TopCopper"]
                log.write("%s: Unite left %d separate top-copper bodies "
                          "(0 expected).\n" % (model.name, len(remaining)))
                if remaining:
                    log.write("  WARNING, not united: %s\n" % ", ".join(remaining))
                    raise RuntimeError("Top copper is not one connected body.")

            if STUDENT_MESH_SAFE:
                log.write("%s: STUDENT_MESH_SAFE - SMA pads, vias and mounting "
                          "holes are in the Gerbers only, not in HFSS.\n"
                          % model.name)
            else:
                build_fabrication_features(editor, model, log)

            create_box(editor, "Air",
                       ["-boardX/2-airPad", "-boardY/2-airPad", "-tCu-airPad"],
                       ["boardX+2*airPad", "boardY+2*airPad",
                        "h+2*tCu+2*airPad"], "vacuum", True, "(180 210 230)", 0.92)
            boundaries = design.GetModule("BoundarySetup")
            boundaries.AssignRadiation(["NAME:Radiation", "Objects:=", ["Air"],
                                        "IsFssReference:=", False,
                                        "IsForPML:=", False])

            px, py = model.port
            if STUDENT_MESH_SAFE and "Array4x4" in model.name:
                px = model.bx * 0.5          # port on the board edge
            editor.CreateRectangle(["NAME:RectangleParameters", "IsCovered:=", True,
                                    "XStart:=", px.s,
                                    "YStart:=", (py - model.w50 * 0.5).s,
                                    "ZStart:=", "0mm", "Width:=", "W50",
                                    "Height:=", "h", "WhichAxis:=", "X"],
                                   attributes("PortSheet"))
            port_faces = [int(v) for v in editor.GetFaceIDs("PortSheet")]
            if len(port_faces) != 1:
                raise RuntimeError("P1 PortSheet must have exactly one face.")
            px_mm = "%.10gmm" % px.v
            py_mm = "%.10gmm" % py.v
            h_mm = "%.10gmm" % model.h.v
            boundaries.AssignLumpedPort([
                "NAME:P1", "Faces:=", port_faces, "DoDeembed:=", False,
                "RenormalizeAllTerminals:=", True,
                ["NAME:Modes", ["NAME:Mode1", "ModeNum:=", 1, "UseIntLine:=", True,
                                ["NAME:IntLine", "Coordinate System:=", "Global",
                                 "Start:=", [px_mm, py_mm, "0mm"],
                                 "End:=", [px_mm, py_mm, h_mm]],
                                "AlignmentGroup:=", 0, "CharImp:=", "Zpi",
                                "RenormImp:=", "50ohm"]],
                "ShowReporterFilter:=", False, "ReporterFilter:=", [True],
                "Impedance:=", "50ohm"])

            create_setup(design)
            create_farfield(design)
            create_reports(design, log)
            log.write("%s: geometry, material, P1, radiation, setup, three "
                  "sweeps, far field and reports created.\n" % model.name)
            log.flush()
            if VALIDATE_AFTER_BUILD:
                validation_file = os.path.join(out, model.name + "_validation.log")
                result = design.ValidateDesign(validation_file)
                log.write("%s: ValidateDesign returned %s.\n"
                          % (model.name, str(result)))
                log.flush()
                if not result:
                    raise RuntimeError("HFSS ValidateDesign failed; see " +
                                       validation_file)
            project.SaveAs(project_path, True)
            if WRITE_GERBERS_WITH_HFSS_BUILD and "Array4x4" in model.name:
                try:
                    zip_path = write_gerbers(model, out)
                    log.write("%s: JLC Gerbers written: %s\n"
                              % (model.name, zip_path))
                except Exception:
                    log.write("%s: Gerber export failed inside AEDT (the HFSS "
                              "model is fine).  Run  python %s --gerber  "
                              "instead.\n%s\n"
                              % (model.name, os.path.basename(__file__),
                                 traceback.format_exc()))
        project.SetActiveDesign(models[-1].name)
        project.Save()
        log.write("SAVED: " + project_path + "\n")
    return project_path


# ----------------------------------------------------------------------------
# 13b. JLCPCB GERBER + DRILL EXPORT (pure Python, no extra packages)
# ----------------------------------------------------------------------------
# Gerber origin = lower-left board corner, so every coordinate is positive.
# All copper is written from the SAME nominal geometry that HFSS draws:
#   boxes and sheets -> filled regions
#   traces           -> one rectangle per straight segment plus a round
#                       disc at every bend (AEDT "Curved" bend)
#   discs            -> round flashes
# Overlapping dark shapes simply add up, exactly like Unite in HFSS.

class _Gerber(object):
    def __init__(self, function, polarity="Positive", comment=""):
        self.function = function
        self.polarity = polarity
        self.comment = comment
        self.apertures = {}          # diameter string -> D code
        self.body = []
        self.pol = "D"

    def _ap(self, diameter):
        key = "%.6f" % diameter
        if key not in self.apertures:
            self.apertures[key] = 10 + len(self.apertures)
        return self.apertures[key]

    @staticmethod
    def _c(v):
        return "%d" % int(round(v * 1e6))

    def polarity_dark(self, dark=True):
        want = "D" if dark else "C"
        if want != self.pol:
            self.body.append("%%LP%s*%%" % want)
            self.pol = want

    def region(self, pts):
        self.body.append("G36*")
        x0, y0 = pts[0]
        self.body.append("X%sY%sD02*" % (self._c(x0), self._c(y0)))
        for x, y in list(pts[1:]) + [pts[0]]:
            self.body.append("X%sY%sD01*" % (self._c(x), self._c(y)))
        self.body.append("G37*")

    def rect(self, x, y, w, h):
        self.region([(x, y), (x + w, y), (x + w, y + h), (x, y + h)])

    def flash(self, x, y, diameter):
        self.body.append("D%d*" % self._ap(diameter))
        self.body.append("X%sY%sD03*" % (self._c(x), self._c(y)))

    def line(self, pts, width):
        self.body.append("D%d*" % self._ap(width))
        x0, y0 = pts[0]
        self.body.append("X%sY%sD02*" % (self._c(x0), self._c(y0)))
        for x, y in pts[1:]:
            self.body.append("X%sY%sD01*" % (self._c(x), self._c(y)))

    def text(self):
        out = ["G04 %s*" % self.comment if self.comment else "G04 layer*",
               "%TF.GenerationSoftware,build_4x4_FR4_JLC,script,1*%",
               "%%TF.FileFunction,%s*%%" % self.function,
               "%%TF.FilePolarity,%s*%%" % self.polarity,
               "%FSLAX46Y46*%", "%MOMM*%"]
        for key, code in sorted(self.apertures.items(), key=lambda kv: kv[1]):
            out.append("%%ADD%dC,%s*%%" % (code, key))
        out += ["%LPD*%", "G01*", "G75*"]
        out += self.body
        out.append("M02*")
        return "\n".join(out) + "\n"


def _segment_quad(p, q, width):
    dx, dy = q[0] - p[0], q[1] - p[1]
    length = math.hypot(dx, dy)
    nx, ny = -dy / length * width / 2.0, dx / length * width / 2.0
    return [(p[0] + nx, p[1] + ny), (q[0] + nx, q[1] + ny),
            (q[0] - nx, q[1] - ny), (p[0] - nx, p[1] - ny)]


# Tiny stroke font, only the letters JLC needs (unit cell 0.6 x 1.0).
_FONT = {"J": [[(0.6, 1.0), (0.6, 0.2), (0.4, 0.0), (0.2, 0.0), (0.0, 0.2)]],
         "L": [[(0.0, 1.0), (0.0, 0.0), (0.6, 0.0)]],
         "C": [[(0.6, 0.85), (0.45, 1.0), (0.15, 1.0), (0.0, 0.85),
                (0.0, 0.15), (0.15, 0.0), (0.45, 0.0), (0.6, 0.15)]]}


def write_gerbers(model, out):
    """Write the JLC Gerber set + drill files and zip them.  Returns zip path."""
    import zipfile
    bx, by = model.bx.v, model.by.v
    ox, oy = bx / 2.0, by / 2.0             # shift: board lower-left -> (0,0)

    def T(pt):
        return (pt[0] + ox, pt[1] + oy)

    folder = os.path.join(out, GERBER_FOLDER_NAME)
    if not os.path.isdir(folder):
        os.makedirs(folder)
    base = GERBER_BASENAME
    files = {}

    # ---- top copper -------------------------------------------------------
    top = _Gerber("Copper,L1,Top", comment="Top copper: antenna and feed")
    for piece in model.pieces:
        if piece["cls"] == "box":
            x, y, w, h = piece["xywh"]
            top.rect(x + ox, y + oy, w, h)
        elif piece["cls"] == "sheet":
            top.region([T(p) for p in piece["polys"][0]])
        elif piece["cls"] == "trace":
            pts = [T(p) for p in piece["pts"]]
            w = piece["width_mm"]
            for i in range(len(pts) - 1):
                top.region(_segment_quad(pts[i], pts[i + 1], w))
            for pt in pts[1:-1]:
                top.flash(pt[0], pt[1], w)
        elif piece["cls"] == "disc":
            x, y = T(piece["xy"])
            top.flash(x, y, 2.0 * piece["radius_mm"])
    for pad in model.sma_pads:
        x, y, w, h = pad["xywh"]
        top.rect(x + ox, y + oy, w, h)
    files[base + ".GTL"] = top.text()

    # ---- bottom copper: solid ground, pulled back, holes cleared ------------
    pb = EDGE_PULLBACK_MM
    bot = _Gerber("Copper,L2,Bot", comment="Bottom copper: ground plane")
    bot.rect(pb, pb, bx - 2 * pb, by - 2 * pb)
    if model.mount_holes:
        bot.polarity_dark(False)
        for hole in model.mount_holes:
            x, y = T(hole["xy"])
            bot.flash(x, y, hole["drill_mm"] + 2 * hole["clear_mm"])
        bot.polarity_dark(True)
    files[base + ".GBL"] = bot.text()

    # ---- solder mask (a drawn shape = an OPENING, bare metal) --------------
    px, py = T((model.port[0].v, model.port[1].v))
    leg_len = SMA_PAD_LENGTH_MM + pb
    leg_half = model.w50.v / 2.0 + SMA_PAD_GAP_MM + SMA_PAD_WIDTH_MM
    right_side = model.port[0].v > 0
    leg_x0 = bx - leg_len if right_side else 0.0

    tmask = _Gerber("Soldermask,Top", "Negative", "Top solder mask openings")
    if TOP_SOLDER_MASK_OFF:
        tmask.rect(0.0, 0.0, bx, by)        # whole top side open = no mask
    else:
        for pad in model.sma_pads:
            x, y, w, h = pad["xywh"]
            tmask.rect(x + ox - 0.05, y + oy - 0.05, w + 0.1, h + 0.1)
        tmask.rect(leg_x0, py - model.w50.v / 2.0 - 0.05, leg_len,
                   model.w50.v + 0.1)
    files[base + ".GTS"] = tmask.text()

    bmask = _Gerber("Soldermask,Bot", "Negative", "Bottom solder mask openings")
    bmask.rect(leg_x0, py - leg_half, leg_len, 2 * leg_half)  # SMA bottom legs
    for hole in model.mount_holes:
        x, y = T(hole["xy"])
        bmask.flash(x, y, hole["drill_mm"] + 2 * hole["clear_mm"])
    files[base + ".GBS"] = bmask.text()

    # ---- bottom silkscreen: JLC order-number marker (mirrored: bottom) -------
    if JLC_ORDER_NUMBER_ON_BOTTOM:
        silk = _Gerber("Legend,Bot", comment="Bottom silkscreen")
        word, hgt, pitch = "JLCJLCJLCJLC", 1.5, 1.5 * 0.9
        total = pitch * len(word) - 0.3 * hgt
        cx, cy = bx / 2.0, 4.0
        for i, ch in enumerate(word):
            for stroke in _FONT[ch]:
                pts = []
                for u, v in stroke:
                    x = cx - total / 2.0 + i * pitch + u * hgt
                    x = 2 * cx - x                      # mirror for bottom
                    pts.append((x, cy - hgt / 2.0 + v * hgt))
                silk.line(pts, 0.2)
        files[base + ".GBO"] = silk.text()

    # ---- board outline -----------------------------------------------------
    edge = _Gerber("Profile,NP", comment="Board outline")
    edge.line([(0, 0), (bx, 0), (bx, by), (0, by), (0, 0)], 0.1)
    files[base + ".GKO"] = edge.text()

    # ---- drill files (Excellon, metric, decimal) ----------------------------
    def excellon(holes, plated):
        tools = sorted(set("%.3f" % d for d, _x, _y in holes))
        lines = ["M48",
                 "; DRILL file build_4x4_FR4_JLC",
                 "; FORMAT={-:-/ absolute / metric / decimal}",
                 "; #@! TF.FileFunction,%s,1,2,%s"
                 % (("Plated", "PTH") if plated else ("NonPlated", "NPTH")),
                 "FMAT,2", "METRIC"]
        for i, t in enumerate(tools):
            lines.append("T%dC%s" % (i + 1, t))
        lines += ["%", "G90", "G05"]
        for i, t in enumerate(tools):
            lines.append("T%d" % (i + 1))
            for d, x, y in holes:
                if "%.3f" % d == t:
                    lines.append("X%.4fY%.4f" % (x, y))
        lines += ["T0", "M30"]
        return "\n".join(lines) + "\n"

    vias = [(v["drill_mm"],) + T(v["xy"]) for v in model.vias]
    mh = [(h["drill_mm"],) + T(h["xy"]) for h in model.mount_holes]
    if vias:
        files[base + "-PTH.drl"] = excellon(vias, True)
    if mh:
        files[base + "-NPTH.drl"] = excellon(mh, False)

    for name, text in files.items():
        with open(os.path.join(folder, name), "w") as stream:
            stream.write(text)
    zip_path = os.path.join(out, base + "_JLC_Gerbers.zip")
    archive = zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED)
    try:
        for name in sorted(files):
            archive.write(os.path.join(folder, name), name)
    finally:
        archive.close()
    return zip_path


# ----------------------------------------------------------------------------
# 14. ENTRY POINT
# ----------------------------------------------------------------------------
def main():
    out = OUTPUT_DIR or os.path.dirname(os.path.abspath(__file__))
    if not os.path.isdir(out):
        os.makedirs(out)

    models = []
    all_ok = True
    for name in BUILD_DESIGNS:
        model = make_layout(name)
        ok, report = run_checks(model)
        text = "\n".join(report)
        print(text)
        with open(os.path.join(out, model.name + "_report.txt"), "w") as stream:
            stream.write(text + "\n")
        models.append(model)
        all_ok = all_ok and ok

    if "--preview" in sys.argv or "--gerber" in sys.argv:
        for model in models:
            write_preview(model, out)
        print("\nWrote geometry JSON, SVG preview and report. HFSS was not used.")
        if "--gerber" in sys.argv:
            if not all_ok:
                raise RuntimeError("Geometry validation failed; Gerbers not "
                                   "written.  See the printed report.")
            for model in models:
                if "Array4x4" in model.name:
                    print("JLC Gerbers written: " + write_gerbers(model, out))
        return

    if not all_ok and STRICT:
        raise RuntimeError("Geometry validation failed; see the printed report. "
                           "Set STRICT = False to build anyway.")

    import ScriptEnv
    ScriptEnv.Initialize("Ansoft.ElectronicsDesktop")
    import __main__
    desktop = getattr(__main__, "oDesktop", globals().get("oDesktop"))
    if desktop is None:
        raise RuntimeError("Run this script from AEDT: Tools > Run Script.")
    try:
        path = build_native(desktop, models, out)
        desktop.AddMessage("", "", 0, "Created sweep-ready 5.85 GHz 4x4 project: " + path +
                           ". Geometry checks passed. No solve was run.")
    except Exception:
        with open(os.path.join(out, "hfss_build_error.txt"), "w") as stream:
            stream.write(traceback.format_exc())
        raise


if __name__ == "__main__":
    main()
