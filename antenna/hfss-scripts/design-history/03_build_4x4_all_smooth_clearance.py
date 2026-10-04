# -*- coding: utf-8 -*-
"""4x4 corporate array with smooth transitions and cleared internal buses.

Run inside AEDT Student 2025 R2: Tools > Run Script.
Uses AEDT's native IronPython API; no PyAEDT installation is required.
Creates a NEW, uniquely named project containing one 16-patch design.
Does not start a solve. The geometry and setup are preliminary and must be
reported as such until a mesh-converged full-array solution is obtained.

The geometry planner also runs under ordinary Python without HFSS:
    python build_4x4_all_smooth_clearance.py --preview
This writes nominal geometry JSON and SVG beside this script, not an AEDT file.
All lengths below are millimetres. No dimensions are inferred from image pixels.
"""
from __future__ import division, print_function
import json
import math
import os
import sys
import time
import traceback

# USER SETTINGS. Confirm er/tand with the board manufacturer at about 6 GHz.
F0_GHZ = 5.85
ER = 4.4
TAND = 0.02
H_MM = 1.6
COPPER_MM = 0.035
EDGE_R_OHM = 141.45  # Implied real load from the verified 1.1 mm transformer.
# This pitch makes the last branch contain, in sequence: 7.193 mm of
# 70.71-ohm divider line, 7.75 mm of 50-ohm line, and the 4.25 mm horizontal
# part of the verified 7.25 mm patch transformer. Its last 3.0 mm runs upward.
DX_MM = 38.386399606
DY_MM = 34.0
FEED_RISE_MM = 3.0
# Keep the long common 50-ohm input trace in the empty channel between the
# middle patch rows.  Its final approach uses a 45-degree diagonal followed by
# a horizontal landing into the root tee.
INPUT_Y_MM = 0.0
# A quarter free-space wavelength is 12.81 mm at 5.85 GHz. 13 mm keeps the
# first-order radiation-boundary spacing while reducing Student-edition mesh.
AIR_PAD_MM = 13.0
BUILD_DESIGNS = ("Array4x4_AllSmooth_Clearance",)
UNITE_TOP_COPPER = True
ROUND_JUNCTIONS = True
JUNCTION_RADIUS_MM = 1.5
# Linear taper placed after every 70.71-ohm quarter-wave section. The taper
# occupies the first part of the following 50-ohm section, so Lq1...Lq4,
# every branch endpoint, and all port-to-patch centerline lengths stay fixed.
Q50_TAPER_MM = 3.0
# Final transition from the 3 mm 50-ohm line to the 1.1 mm patch transformer.
# It occupies the end of the existing 50-ohm section; LpT stays unchanged.
FIFTY_TO_PATCH_TAPER_MM = 2.0
# The stage-2 routes rise into the clear channel immediately after their Q50
# tapers, then return through a second 45-degree diagonal.  A short horizontal
# landing keeps each final connection perpendicular to its quarter-wave tee.
TEE_APPROACH_MM = 4.0
# Native script output defaults to its own folder. An explicit path can be used.
OUTPUT_DIR = None
# For the assistant's verification only: command-line ScriptArgument may set this.
VALIDATE_AFTER_BUILD = True


def synthesize(er=ER, h=H_MM, f=F0_GHZ, edge_r=EDGE_R_OHM):
    """Quasi-static, zero-metal-thickness synthesis, for initialization only."""
    c_mm_ghz = 299.792458

    def effective(w):
        u = w / h
        correction = 0.04 * (1-u)**2 if u < 1 else 0
        return (er+1)/2 + (er-1)/2 * (1/math.sqrt(1+12/u)+correction)

    def impedance(w):
        u, ee = w/h, effective(w)
        if u <= 1:
            return 60/math.sqrt(ee)*math.log(8/u+u/4)
        return 120*math.pi/(math.sqrt(ee)*(u+1.393+0.667*math.log(u+1.444)))

    def line(target):
        low, high = 0.001*h, 100*h
        for unused in range(100):
            mid = (low+high)/2
            if impedance(mid) > target:
                low = mid
            else:
                high = mid
        width = (low+high)/2
        return {"Z_ohm": target, "W_mm": width,
                "eeff": effective(width),
                "Lquarter_mm": c_mm_ghz/(4*f*math.sqrt(effective(width)))}

    wp = c_mm_ghz/(2*f)*math.sqrt(2/(er+1))
    ep = effective(wp)
    dl = 0.412*h*(ep+0.3)*(wp/h+0.264)/((ep-0.258)*(wp/h+0.8))
    lp = c_mm_ghz/(2*f*math.sqrt(ep))-2*dl
    return {"Wp_mm": wp, "Lp_mm": lp, "eeff_patch": ep, "deltaL_mm": dl,
            "line50": line(50), "divider": line(math.sqrt(50*100)),
            "patch_transformer": line(math.sqrt(50*edge_r)),
            "edge_R_assumed_ohm": edge_r, "lambda0_mm": c_mm_ghz/f}


def verified_element():
    """Return line seeds with the user's simulated single-patch dimensions."""
    s = synthesize(edge_r=EDGE_R_OHM)
    s["Wp_mm"] = 15.594
    # Latest full-array tuning value supplied by the user.  The clean-feed
    # design must be solved afresh because removing nearby-line coupling can
    # shift the match.
    s["Lp_mm"] = 10.82
    s["line50"]["W_mm"] = 3.0
    s["patch_transformer"]["W_mm"] = 1.1
    s["patch_transformer"]["Lquarter_mm"] = 7.25
    s["patch_transformer"]["Z_ohm"] = 84.10
    s["verified_patch_input_50_length_mm"] = 7.75
    s["element_source"] = "User-verified single-patch HFSS model"
    return s


class E(object):
    """A nominal value paired with its HFSS parameter expression."""
    def __init__(self, value, expr=None):
        self.v = float(value)
        self.s = expr if expr is not None else "%.10gmm" % value

    def __add__(self, other):
        other = as_e(other)
        return E(self.v+other.v, "(%s+%s)" % (self.s, other.s))

    __radd__ = __add__

    def __sub__(self, other):
        other = as_e(other)
        return E(self.v-other.v, "(%s-%s)" % (self.s, other.s))

    def __rsub__(self, other):
        return as_e(other).__sub__(self)

    def __mul__(self, scalar):
        return E(self.v*scalar, "(%s*%.10g)" % (self.s, scalar))

    __rmul__ = __mul__

    def __neg__(self):
        return self * -1


def as_e(value):
    return value if isinstance(value, E) else E(value)


class Layout(object):
    def __init__(self, name, board_x, board_y, synthesis):
        self.name = name
        self.vars = []
        self.rects = []
        self.tapers = []
        self.curved_traces = []
        self.junctions = []
        self.routes = []
        self.leaf_paths = []
        self.synthesis = synthesis
        self.Wp = self.var("Wp", synthesis["Wp_mm"])
        self.Lp = self.var("Lp", synthesis["Lp_mm"])
        self.h = self.var("h", H_MM)
        self.t = self.var("tCu", COPPER_MM)
        self.dx = self.var("dx", DX_MM)
        self.dy = self.var("dy", DY_MM)
        self.rise = self.var("feedRise", FEED_RISE_MM)
        self.bx = self.var("boardX", board_x)
        self.by = self.var("boardY", board_y)
        self.pad = self.var("airPad", AIR_PAD_MM)
        self.junction_r = self.var("junctionR", JUNCTION_RADIUS_MM)
        self.q50_taper = self.var("Lq50Taper", Q50_TAPER_MM)
        self.fifty_patch_taper = self.var("L50PatchTaper", FIFTY_TO_PATCH_TAPER_MM)
        self.tee_approach = self.var("teeApproach", TEE_APPROACH_MM)
        self.input_y = self.var("inputY", INPUT_Y_MM)
        self.w50 = self.var("W50", synthesis["line50"]["W_mm"])
        self.wpT = self.var("WpT", synthesis["patch_transformer"]["W_mm"])
        self.lpT = self.var("LpT", synthesis["patch_transformer"]["Lquarter_mm"])
        self.wq, self.lq = {}, {}
        for stage in (1, 2, 3, 4):
            self.wq[stage] = self.var("Wq%d" % stage, synthesis["divider"]["W_mm"])
            stage_length = 7.10 if stage == 4 else synthesis["divider"]["Lquarter_mm"]
            self.lq[stage] = self.var("Lq%d" % stage, stage_length)
        self.bus_y = -(self.Lp * 0.5) - self.rise
        self.port = None

    def var(self, name, value):
        self.vars.append((name, "%.10gmm" % value))
        return E(value, name)

    def rect(self, name, x, y, w, h, kind, owner):
        vals = [as_e(v) for v in (x, y, w, h)]
        if vals[2].v <= 0 or vals[3].v <= 0:
            raise ValueError("Non-positive geometry: " + name)
        self.rects.append({"name": name, "expr": [v.s for v in vals],
                           "xywh": [v.v for v in vals], "kind": kind, "owner": owner})

    def segment(self, name, p, q, width, kind, owner):
        x1, y1 = [as_e(v) for v in p]
        x2, y2 = [as_e(v) for v in q]
        if abs(y1.v-y2.v) < 1e-8 and abs(x1.v-x2.v) > 1e-8:
            start, end = (x1, x2) if x1.v < x2.v else (x2, x1)
            self.rect(name, start, y1-width*0.5, end-start, width, kind, owner)
        elif abs(x1.v-x2.v) < 1e-8 and abs(y1.v-y2.v) > 1e-8:
            start, end = (y1, y2) if y1.v < y2.v else (y2, y1)
            self.rect(name, x1-width*0.5, start, width, end-start, kind, owner)
        else:
            raise ValueError("Segment must be nonzero and axis-aligned: " + name)

    def patch(self, name, x, y):
        self.rect(name, x-self.Wp*0.5, y-self.Lp*0.5,
                  self.Wp, self.Lp, "patch", name)

    def taper(self, name, p, q, start_width, end_width, kind, owner):
        """Add an axis-aligned linear-width taper with a CCW top-view polygon."""
        x1, y1 = [as_e(v) for v in p]
        x2, y2 = [as_e(v) for v in q]
        w1, w2 = as_e(start_width), as_e(end_width)
        if abs(y1.v-y2.v) < 1e-8 and abs(x1.v-x2.v) > 1e-8:
            if x1.v < x2.v:
                polygon = [(x1, y1-w1*0.5), (x2, y2-w2*0.5),
                           (x2, y2+w2*0.5), (x1, y1+w1*0.5)]
            else:
                polygon = [(x2, y2-w2*0.5), (x1, y1-w1*0.5),
                           (x1, y1+w1*0.5), (x2, y2+w2*0.5)]
        elif abs(x1.v-x2.v) < 1e-8 and abs(y1.v-y2.v) > 1e-8:
            if y1.v < y2.v:
                polygon = [(x1-w1*0.5, y1), (x1+w1*0.5, y1),
                           (x2+w2*0.5, y2), (x2-w2*0.5, y2)]
            else:
                polygon = [(x2-w2*0.5, y2), (x2+w2*0.5, y2),
                           (x1+w1*0.5, y1), (x1-w1*0.5, y1)]
        else:
            raise ValueError("Taper must be nonzero and axis-aligned: " + name)
        self.tapers.append({"name": name,
            "points": [{"expr": [x.s, y.s], "xy": [x.v, y.v]}
                       for x, y in polygon],
            "start_width_expr": w1.s, "start_width_mm": w1.v,
            "end_width_expr": w2.s, "end_width_mm": w2.v,
            "kind": kind, "owner": owner})

    def curved_trace(self, name, points, width, kind, owner):
        pts = []
        for x, y in points:
            x, y = as_e(x), as_e(y)
            pts.append({"expr": [x.s, y.s], "xy": [x.v, y.v]})
        self.curved_traces.append({"name": name, "points": pts,
            "width_expr": as_e(width).s, "width_mm": as_e(width).v,
            "kind": kind, "owner": owner})

    def rounded_junction(self, name, point):
        x, y = [as_e(v) for v in point]
        self.junctions.append({"name": name, "expr": [x.s, y.s],
                               "xy": [x.v, y.v]})

    def transformer_to_patch(self, prefix, start, patch_x, patch_y, sign, owner):
        """Last LpT of a route: horizontal arm then feedRise upward."""
        bus = patch_y-self.Lp*0.5-self.rise
        edge = patch_y-self.Lp*0.5
        # AEDT's Curved rectangular-polyline bend rounds the 90-degree corner.
        # Centerline length remains (LpT-feedRise)+feedRise = LpT.
        self.curved_trace(prefix+"_curved", [start, (patch_x, bus),
                          (patch_x, edge)], self.wpT, "patchT", owner)

    def branch(self, name, p, q, stage, leaf=None):
        p = tuple(as_e(v) for v in p)
        q = tuple(as_e(v) for v in q)
        axis = 0 if abs(p[0].v-q[0].v) > 1e-8 else 1
        sign = 1 if q[axis].v > p[axis].v else -1
        length = abs(q[axis].v-p[axis].v)
        end_q = list(p)
        end_q[axis] = p[axis]+sign*self.lq[stage]
        self.segment(name+"_Q", p, end_q, self.wq[stage], "divider", name)
        fifty_end = list(q)
        if leaf is not None:
            if axis != 0:
                raise ValueError("Final branch must run horizontally")
            fifty_end[0] = q[0]-sign*(self.lpT-self.rise)
            required_tapers = self.q50_taper.v+self.fifty_patch_taper.v
            if (fifty_end[0].v-end_q[0].v)*sign <= required_tapers:
                raise ValueError("Final branch too short for both transformers")
            length += self.rise.v
        else:
            if length <= self.lq[stage].v+self.q50_taper.v:
                raise ValueError("Branch too short for its transformer")
        end_taper = list(end_q)
        end_taper[axis] = end_q[axis]+sign*self.q50_taper
        self.taper(name+"_Q50_Taper", end_q, end_taper,
                   self.wq[stage], self.w50, "transition", name)

        clearance_rerouted = stage == 2 and axis == 0 and leaf is None
        if clearance_rerouted:
            # Move only the 50-ohm part of the stage-2 branch to the geometric
            # channel halfway between its two neighboring patch rows.  The
            # original child tee remains fixed.  Both channel transitions use
            # true 45-degree diagonals (equal horizontal and vertical offsets).
            # The first starts directly at the Q50 taper, removing the short
            # close-clearance 50-ohm run and its vertical leg over the adjacent
            # patch.  A short final horizontal landing keeps the connection to
            # the next quarter-wave branches perpendicular and symmetric.
            clear_y = p[1]-self.bus_y
            clear_offset = clear_y-p[1]
            inner_diagonal_end_x = end_taper[0]+sign*clear_offset
            diagonal_end_x = fifty_end[0]-sign*self.tee_approach
            diagonal_start_x = diagonal_end_x-sign*clear_offset
            if (diagonal_start_x.v-inner_diagonal_end_x.v)*sign <= 0:
                raise ValueError("Stage-2 branch is too short for its two 45-degree clearance approaches")
            points = [tuple(end_taper),
                      (inner_diagonal_end_x, clear_y),
                      (diagonal_start_x, clear_y),
                      (diagonal_end_x, p[1]), tuple(fifty_end)]
            self.curved_trace(name+"_50_Clearance", points,
                              self.w50, "line50", name)
            length += 2.0*(math.sqrt(2.0)-1.0)*abs(clear_y.v-p[1].v)
        elif leaf is not None:
            patch_taper_start = list(fifty_end)
            patch_taper_start[axis] = fifty_end[axis]-sign*self.fifty_patch_taper
            self.segment(name+"_50", end_taper, patch_taper_start,
                         self.w50, "line50", name)
            self.taper(name+"_50PT_Taper", patch_taper_start, fifty_end,
                       self.w50, self.wpT, "transition", name)
        else:
            self.segment(name+"_50", end_taper, fifty_end,
                         self.w50, "line50", name)
        if leaf is not None:
            self.transformer_to_patch(name+"_PT", fifty_end, q[0], leaf[1], sign, name)
        self.routes.append({"name": name, "stage": stage,
                            "start": [v.v for v in p], "end": [v.v for v in q],
                            "length_mm": length, "leaf": leaf[0] if leaf else None,
                            "clearance_rerouted": clearance_rerouted})
        return length

    def data(self):
        return {"name": self.name, "parameters": dict(self.vars),
                "board_mm": [self.bx.v, self.by.v], "copper_rectangles": self.rects,
                "linear_tapers": self.tapers,
                "curved_traces": self.curved_traces, "rounded_junctions": self.junctions,
                "branches": self.routes, "leaf_paths": self.leaf_paths,
                "port": [v.v for v in self.port], "synthesis": self.synthesis}


def make_layout(name, synthesis=None):
    s = synthesis or verified_element()
    size = {"SinglePatch": (50, 40), "Subarray1x2": (80, 55),
            "Subarray2x2": (95, 90), "Array4x4": (155, 145),
            "Array4x4_Curved": (155, 145),
            "Array4x4_Curved_FeedClearance": (155, 145),
            "Array4x4_Smooth_Q50_Transitions": (155, 145),
            "Array4x4_AllSmooth_Clearance": (155, 145)}[name]
    m = Layout(name, size[0], size[1], s)
    if name == "SinglePatch":
        x, y = E(0), E(0)
        m.patch("Patch_1", x, y)
        start_t = (x-(m.lpT-m.rise), m.bus_y)
        m.port = (-m.bx*0.5, m.bus_y)
        m.segment("Input50", m.port, start_t, m.w50, "line50", "input")
        m.transformer_to_patch("PatchMatch", start_t, x, y, 1, "PatchMatch")
        m.leaf_paths.append({"patch": "Patch_1", "path_mm": m.bx.v*0.5+m.rise.v,
                             "stages": [], "R_edge_assumed_ohm": EDGE_R_OHM})
        return m

    if name == "Subarray1x2":
        # A single, uncluttered divider for tuning. The port approaches the tee
        # from below; each final branch exactly preserves the verified element's
        # 7.75 mm 50-ohm input line and 7.25 mm patch transformer.
        parent = (E(0), m.bus_y)
        m.port = (E(0), -m.by*0.5)
        m.segment("Input50", m.port, parent, m.w50, "line50", "input")
        for index, sign in enumerate((-1, 1), 1):
            patch_x, patch_y = m.dx*0.5*sign, E(0)
            patch_name = "Patch_1x2_%d" % index
            m.patch(patch_name, patch_x, patch_y)
            m.branch("Branch_1x2_%d" % index, parent,
                     (patch_x, m.bus_y), 4, (patch_name, patch_y))
            m.leaf_paths.append({"patch": patch_name,
                "path_mm": m.by.v*0.5-m.bus_y.v+m.dx.v*0.5+m.rise.v,
                "stages": [4], "R_edge_assumed_ohm": EDGE_R_OHM})
        return m

    n = 4 if name in ("Array4x4", "Array4x4_Curved",
                      "Array4x4_Curved_FeedClearance",
                      "Array4x4_Smooth_Q50_Transitions",
                      "Array4x4_AllSmooth_Clearance") else 2
    stages = [1, 2, 3, 4] if n == 4 else [3, 4]
    xs = [(i-(n-1)/2)*m.dx for i in range(n)]
    ys = [(j-(n-1)/2)*m.dy for j in range(n)]
    # Common input route.  The long horizontal section is centered at y=0,
    # giving about 10.09 mm edge clearance to both middle patch rows for the
    # nominal Lp=10.82 mm geometry.  A 45-degree diagonal replaces the old
    # right-angle dogleg, followed by a horizontal landing into the root tee.
    m.port = (m.bx*0.5, m.input_y)
    input_drop = m.input_y-m.bus_y
    input_diagonal_end_x = m.tee_approach
    input_diagonal_start_x = input_diagonal_end_x+input_drop
    m.curved_trace("Input50_Rerouted", [m.port,
        (input_diagonal_start_x, m.input_y),
        (input_diagonal_end_x, m.bus_y),
        (E(0), m.bus_y)], m.w50, "line50", "input")

    def mean(values):
        return sum(values, E(0))*(1/len(values))

    def divide(ixs, iys, depth, path_length, prefix):
        parent = (mean([xs[i] for i in ixs]), mean([ys[j] for j in iys])+m.bus_y)
        if ROUND_JUNCTIONS:
            m.rounded_junction("RoundTee_"+(prefix or "root"), parent)
        axis = "y" if depth % 2 == 0 else "x"
        indices = iys if axis == "y" else ixs
        half = len(indices)//2
        for side, child_indices in enumerate((indices[:half], indices[half:])):
            cx = ixs if axis == "y" else child_indices
            cy = child_indices if axis == "y" else iys
            child = (mean([xs[i] for i in cx]), mean([ys[j] for j in cy])+m.bus_y)
            label = prefix+str(side)
            last = depth == len(stages)-1
            leaf = None
            if last:
                patch_name = "Patch_r%d_c%d" % (cy[0]+1, cx[0]+1)
                leaf = (patch_name, ys[cy[0]])
                m.patch(patch_name, xs[cx[0]], ys[cy[0]])
            step = m.branch("Branch_"+label, parent, child, stages[depth], leaf)
            if last:
                m.leaf_paths.append({"patch": leaf[0],
                    "path_mm": path_length+step, "stages": stages,
                    "R_edge_assumed_ohm": EDGE_R_OHM})
            else:
                divide(cx, cy, depth+1, path_length+step, label)

    input_path_length = (abs(m.bx.v*0.5-input_diagonal_start_x.v) +
                         math.sqrt(2.0)*abs(input_drop.v) +
                         abs(input_diagonal_end_x.v))
    divide(list(range(n)), list(range(n)), 0, input_path_length, "")
    return m


def write_preview(models, out):
    """Dimensioned, nominal top view; SVG is NOT a Gerber or simulated result."""
    for model in models:
        data = model.data()
        with open(os.path.join(out, model.name+"_geometry.json"), "w") as stream:
            json.dump(data, stream, indent=2)
        bx, by = data["board_mm"]
        scale = 5
        margin = 55
        width, height = bx*scale+2*margin, by*scale+2*margin+60
        parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="%g" height="%g" viewBox="0 0 %g %g">' % (width,height,width,height),
                 '<rect width="100%" height="100%" fill="#ffffff"/>',
                 '<g font-family="Arial, sans-serif" fill="#152838">',
                 '<text x="%g" y="24" font-size="19">%s | 5.85 GHz FR-4 starting geometry</text>' % (margin,model.name),
                 '<text x="%g" y="44" font-size="12">Nominal dimensions in mm. Unoptimized; no simulated performance is shown.</text>' % margin,
                 '<rect x="%g" y="%g" width="%g" height="%g" fill="#eef1f3" stroke="#87949d"/>' % (margin,margin,bx*scale,by*scale)]
        colors = {"patch":"#ddb640", "line50":"#aa7519", "divider":"#197896",
                  "transition":"#428f77", "patchT":"#985cb4"}
        for rect in model.rects:
            x,y,w,h = rect["xywh"]
            parts.append('<rect x="%.4f" y="%.4f" width="%.4f" height="%.4f" fill="%s"><title>%s</title></rect>' %
                (margin+(x+bx/2)*scale,margin+(by/2-y-h)*scale,w*scale,h*scale,colors[rect["kind"]],rect["name"]))
            if rect["kind"] == "patch":
                parts.append('<text x="%.3f" y="%.3f" text-anchor="middle" font-size="11">%s</text>' %
                    (margin+(x+w/2+bx/2)*scale,margin+(by/2-y-h/2)*scale+4,rect["name"].replace("Patch_", "")))
        for taper in model.tapers:
            coords = []
            for point in taper["points"]:
                x, y = point["xy"]
                coords.append("%.4f,%.4f" %
                    (margin+(x+bx/2)*scale, margin+(by/2-y)*scale))
            parts.append('<polygon points="%s" fill="%s"><title>%s</title></polygon>' %
                (" ".join(coords), colors[taper["kind"]], taper["name"]))
        for trace in model.curved_traces:
            coords = []
            for point in trace["points"]:
                x, y = point["xy"]
                coords.append("%.4f,%.4f" %
                    (margin+(x+bx/2)*scale, margin+(by/2-y)*scale))
            parts.append('<polyline points="%s" fill="none" stroke="%s" stroke-width="%.4f" stroke-linejoin="round"/>' %
                (" ".join(coords), colors[trace["kind"]], trace["width_mm"]*scale))
        for junction in model.junctions:
            x, y = junction["xy"]
            parts.append('<circle cx="%.4f" cy="%.4f" r="%.4f" fill="#197896"/>' %
                (margin+(x+bx/2)*scale, margin+(by/2-y)*scale, model.junction_r.v*scale))
        yy = margin+by*scale+22
        parts.append('<text x="%g" y="%g" font-size="13">Board %g x %g | Patch %.2f x %.2f | Pitch %.1f x %.1f</text>' %
            (margin,yy,bx,by,model.Wp.v,model.Lp.v,model.dx.v,model.dy.v))
        parts.append('<text x="%g" y="%g" font-size="12">Gold: patches / 50 ohm | Blue: 70.71 ohm | Green: smooth tapers | Purple: patch transformers</text>' % (margin,yy+21))
        parts.append('</g></svg>')
        with open(os.path.join(out, model.name+"_layout.svg"), "w") as stream:
            stream.write("\n".join(parts))


def attributes(name, material="vacuum", solve_inside=True, color="(160 160 160)", transparency=0):
    return ["NAME:Attributes", "Name:=", name, "Flags:=", "",
            "Color:=", color, "Transparency:=", transparency,
            "PartCoordinateSystem:=", "Global", "MaterialValue:=", '"'+material+'"',
            "SolveInside:=", solve_inside]


def box(editor, name, xyz, sizes, material, solve_inside, color, transparency=0):
    editor.CreateBox(["NAME:BoxParameters",
        "XPosition:=", xyz[0], "YPosition:=", xyz[1], "ZPosition:=", xyz[2],
        "XSize:=", sizes[0], "YSize:=", sizes[1], "ZSize:=", sizes[2]],
        attributes(name, material, solve_inside, color, transparency))


def curved_polyline(editor, name, points, width, color):
    """Create a flat rectangular copper trace with AEDT-rounded 90-degree bends."""
    pl_points = ["NAME:PolylinePoints"]
    for x, y in points:
        pl_points.append(["NAME:PLPoint", "X:=", x, "Y:=", y,
                          "Z:=", "h+tCu/2"])
    pl_segments = ["NAME:PolylineSegments"]
    for index in range(len(points)-1):
        pl_segments.append(["NAME:PLSegment", "SegmentType:=", "Line",
                            "StartIndex:=", index, "NoOfPoints:=", 2])
    editor.CreatePolyline(["NAME:PolylineParameters", "IsPolylineCovered:=", True,
        "IsPolylineClosed:=", False, pl_points, pl_segments,
        ["NAME:PolylineXSection", "XSectionType:=", "Rectangle",
         "XSectionOrient:=", "Auto", "XSectionWidth:=", width,
         "XSectionTopWidth:=", width, "XSectionHeight:=", "tCu",
         "XSectionNumSegments:=", "0", "XSectionBendType:=", "Curved"]],
        attributes(name, "copper", False, color))


def tapered_trace(editor, name, points, color):
    """Create a covered taper polygon at z=h and thicken it by tCu."""
    pl_points = ["NAME:PolylinePoints"]
    for x, y in points:
        pl_points.append(["NAME:PLPoint", "X:=", x, "Y:=", y, "Z:=", "h"])
    pl_segments = ["NAME:PolylineSegments"]
    for index in range(len(points)):
        pl_segments.append(["NAME:PLSegment", "SegmentType:=", "Line",
                            "StartIndex:=", index, "NoOfPoints:=", 2])
    editor.CreatePolyline(["NAME:PolylineParameters", "IsPolylineCovered:=", True,
        "IsPolylineClosed:=", True, pl_points, pl_segments,
        ["NAME:PolylineXSection", "XSectionType:=", "None",
         "XSectionOrient:=", "Auto", "XSectionWidth:=", "0mm",
         "XSectionTopWidth:=", "0mm", "XSectionHeight:=", "0mm",
         "XSectionNumSegments:=", "0", "XSectionBendType:=", "Corner"]],
        attributes(name, "copper", False, color))
    editor.ThickenSheet(["NAME:Selections", "Selections:=", name,
        "NewPartsModelFlag:=", "Model"],
        ["NAME:SheetThickenParameters", "Thickness:=", "tCu",
         "BothSides:=", False])


def round_junction(editor, name, xy, color):
    editor.CreateCylinder(["NAME:CylinderParameters", "XCenter:=", xy[0],
        "YCenter:=", xy[1], "ZCenter:=", "h", "Radius:=", "junctionR",
        "Height:=", "tCu", "WhichAxis:=", "Z", "NumSides:=", "0"],
        attributes(name, "copper", False, color))


def set_variables(design, variables):
    props = ["NAME:NewProps"]
    for name, value in variables:
        props.append(["NAME:"+name, "PropType:=", "VariableProp", "UserDef:=", True, "Value:=", value])
    design.ChangeProperty(["NAME:AllTabs", ["NAME:LocalVariableTab",
                          ["NAME:PropServers", "LocalVariables"], props]])


def build_native(desktop, models, out):
    """Create a new project using native AEDT commands; existing projects untouched."""
    project = desktop.NewProject()
    project_path = os.path.join(out, "FR4_5850_4x4_AllSmooth_Clearance_"+
                                time.strftime("%Y%m%d_%H%M%S")+".aedt")
    definition = project.GetDefinitionManager()
    material = "FR4_Assumed_6GHz"
    definition.AddMaterial(["NAME:"+material, "CoordinateSystemType:=", "Cartesian",
        "BulkOrSurfaceType:=", 1, ["NAME:PhysicsTypes", "set:=", ["Electromagnetic"]],
        "permittivity:=", str(ER), "dielectric_loss_tangent:=", str(TAND),
        "permeability:=", "1", "conductivity:=", "0"])
    log_path = os.path.join(out, "hfss_4x4_all_smooth_clearance_build_log.txt")
    with open(log_path, "w") as log:
        log.write("Preliminary all-smooth 4x4 with cleared stage-2 buses; no simulation started.\n")
        for model in models:
            project.InsertDesign("HFSS", model.name, "DrivenModal", "")
            design = project.SetActiveDesign(model.name)
            set_variables(design, model.vars+[("f0", str(F0_GHZ)+"GHz")])
            editor = design.SetActiveEditor("3D Modeler")
            editor.SetModelUnits(["NAME:Units Parameter", "Units:=", "mm", "Rescale:=", False])
            box(editor, "Substrate", ["-boardX/2","-boardY/2","0mm"],
                ["boardX","boardY","h"], material, True, "(140 166 145)", 0.6)
            box(editor, "Ground", ["-boardX/2","-boardY/2","-tCu"],
                ["boardX","boardY","tCu"], "copper", False, "(205 170 55)")
            copper_names = []
            copper_colors = {
                "patch": "(225 185 65)",       # gold
                "line50": "(181 116 14)",     # orange/brown
                "divider": "(31 145 170)",    # blue
                "transition": "(66 143 119)", # green
                "patchT": "(145 84 180)"      # purple
            }
            for rect in model.rects:
                x,y,w,h = rect["expr"]
                box(editor, rect["name"], [x,y,"h"], [w,h,"tCu"],
                    "copper", False, copper_colors.get(rect["kind"], "(225 185 65)"))
                copper_names.append(rect["name"])
            for taper in model.tapers:
                pts = [(p["expr"][0], p["expr"][1]) for p in taper["points"]]
                tapered_trace(editor, taper["name"], pts,
                    copper_colors.get(taper["kind"], "(66 143 119)"))
                copper_names.append(taper["name"])
            for trace in model.curved_traces:
                pts = [(p["expr"][0], p["expr"][1]) for p in trace["points"]]
                curved_polyline(editor, trace["name"], pts, trace["width_expr"],
                    copper_colors.get(trace["kind"], "(145 84 180)"))
                copper_names.append(trace["name"])
            for junction in model.junctions:
                round_junction(editor, junction["name"], junction["expr"], "(31 145 170)")
                copper_names.append(junction["name"])
            if UNITE_TOP_COPPER:
                # Union the touching/overlapping copper solids for the solve.
                # Parametric history is retained by AEDT.
                editor.Unite(["NAME:Selections", "Selections:=", ",".join(copper_names)],
                             ["NAME:UniteParameters", "KeepOriginals:=", False])
                survivor = copper_names[0]
                editor.ChangeProperty(["NAME:AllTabs", ["NAME:Geometry3DAttributeTab",
                    ["NAME:PropServers", survivor],
                    ["NAME:ChangedProps", ["NAME:Name", "Value:=", "TopCopper"]]]])
            # Full containment is intentional: HFSS resolves substrate/conductors
            # inside the enclosing vacuum solution region.
            box(editor, "Air", ["-boardX/2-airPad","-boardY/2-airPad","-tCu-airPad"],
                ["boardX+2*airPad","boardY+2*airPad","h+2*tCu+2*airPad"],
                "vacuum", True, "(180 210 230)", 0.92)
            boundaries = design.GetModule("BoundarySetup")
            boundaries.AssignRadiation(["NAME:Radiation", "Objects:=", ["Air"],
                "IsFssReference:=", False, "IsForPML:=", False])
            px, py = model.port
            # X-normal rectangle: Width is along Y, Height is along Z.
            editor.CreateRectangle(["NAME:RectangleParameters", "IsCovered:=", True,
                "XStart:=", px.s, "YStart:=", (py-model.w50*0.5).s, "ZStart:=", "0mm",
                "Width:=", "W50", "Height:=", "h", "WhichAxis:=", "X"],
                attributes("PortSheet"))
            port_faces = [int(v) for v in editor.GetFaceIDs("PortSheet")]
            # AEDT 2025 R2's native IntLine parser requires evaluated coordinates.
            # Keep this line on the port if changing board size/h substantially.
            px_mm = "%.10gmm" % px.v
            py_mm = "%.10gmm" % py.v
            h_mm = "%.10gmm" % model.h.v
            boundaries.AssignLumpedPort(["NAME:P1", "Faces:=", port_faces,
                "DoDeembed:=", False, "RenormalizeAllTerminals:=", True,
                ["NAME:Modes", ["NAME:Mode1", "ModeNum:=", 1, "UseIntLine:=", True,
                    ["NAME:IntLine", "Coordinate System:=", "Global",
                     "Start:=", [px_mm,py_mm,"0mm"], "End:=", [px_mm,py_mm,h_mm]],
                    "AlignmentGroup:=", 0, "CharImp:=", "Zpi", "RenormImp:=", "50ohm"]],
                "ShowReporterFilter:=", False, "ReporterFilter:=", [True], "Impedance:=", "50ohm"])
            analysis = design.GetModule("AnalysisSetup")
            analysis.InsertSetup("HfssDriven", ["NAME:Setup1", "SolveType:=", "Single",
                "Frequency:=", str(F0_GHZ)+"GHz", "MaxDeltaS:=", 0.04, "PortsOnly:=", False,
                "UseMatrixConv:=", False, "MaximumPasses:=", 14, "MinimumPasses:=", 3,
                "MinimumConvergedPasses:=", 2, "PercentRefinement:=", 8,
                "IsEnabled:=", True, "BasisOrder:=", 1, "DoLambdaRefine:=", True,
                "DoMaterialLambda:=", True, "SetLambdaTarget:=", False,
                "PortAccuracy:=", 2, "SaveAnyFields:=", True, "SaveRadFieldsOnly:=", False])
            analysis.InsertFrequencySweep("Setup1", ["NAME:WideSweep", "IsEnabled:=", True,
                "RangeType:=", "LinearStep", "RangeStart:=", "5.5GHz", "RangeEnd:=", "6.2GHz",
                "RangeStep:=", "0.01GHz", "Type:=", "Interpolating", "SaveFields:=", False,
                "SaveRadFields:=", False, "InterpTolerance:=", 0.5,
                "InterpMaxSolns:=", 250, "InterpMinSolns:=", 0, "InterpMinSubranges:=", 1])
            analysis.InsertFrequencySweep("Setup1", ["NAME:BandFields", "IsEnabled:=", True,
                "RangeType:=", "LinearStep", "RangeStart:=", "5.7GHz", "RangeEnd:=", "6GHz",
                "RangeStep:=", "0.15GHz", "Type:=", "Discrete", "SaveFields:=", True,
                "SaveRadFields:=", True])
            radiation = design.GetModule("RadField")
            radiation.InsertInfiniteSphereSetup(["NAME:Sphere", "UseCustomRadiationSurface:=", False,
                "CSDefinition:=", "Theta-Phi", "Polarization:=", "Linear",
                "ThetaStart:=", "0deg", "ThetaStop:=", "180deg", "ThetaStep:=", "2deg",
                "PhiStart:=", "-180deg", "PhiStop:=", "180deg", "PhiStep:=", "2deg",
                "UseLocalCS:=", False])
            log.write(model.name+": geometry, material, P1, radiation, setup, two sweeps created.\n")
            log.flush()
            if VALIDATE_AFTER_BUILD:
                validation_file = os.path.join(out, model.name+"_validation.log")
                result = design.ValidateDesign(validation_file)
                log.write(model.name+": ValidateDesign returned "+str(result)+".\n")
                log.flush()
            project.SaveAs(project_path, True)
        project.SetActiveDesign(models[-1].name)
        project.Save()
        log.write("SAVED: "+project_path+"\n")
    return project_path


def main():
    out = OUTPUT_DIR or os.path.dirname(os.path.abspath(__file__))
    if not os.path.isdir(out):
        os.makedirs(out)
    models = [make_layout(name) for name in BUILD_DESIGNS]
    if "--preview" in sys.argv:
        write_preview(models, out)
        print("Wrote analytical previews. HFSS was not used.")
        return
    import ScriptEnv
    ScriptEnv.Initialize("Ansoft.ElectronicsDesktop")
    # ScriptEnv installs oDesktop into __main__; in AEDT it may also be injected.
    import __main__
    desktop = getattr(__main__, "oDesktop", globals().get("oDesktop"))
    if desktop is None:
        raise RuntimeError("Run this script using Tools > Run Script in AEDT.")
    try:
        path = build_native(desktop, models, out)
        desktop.AddMessage("", "", 0, "Created all-smooth cleared-bus 4x4 project: "+path+". No solve was run.")
    except Exception:
        with open(os.path.join(out, "hfss_build_error.txt"), "w") as stream:
            stream.write(traceback.format_exc())
        raise


if __name__ == "__main__":
    main()
