"""Write PCBWay 2-layer design rules and net classes into drone-esc.kicad_pro.

PCBWay standard 2-layer, 2 oz finished copper (their published capabilities,
checked 2026-10):
  * track / space: 8 / 8 mil (0.2 mm) at 2 oz (6/6 mil is 1 oz only)
  * min drill 0.3 mm, min annular ring 0.15 mm -> via 0.6 / 0.3 mm
  * hole to hole 0.5 mm, copper to board edge 0.3 mm
  * silkscreen line 0.15 mm, text height 0.8 mm
Custom rules on top of these are in drone-esc.kicad_dru.
"""
import json
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "drone-esc.kicad_pro"
d = json.load(open(path))
d["meta"]["filename"] = "drone-esc.kicad_pro"
d["text_variables"] = {
    "DOC_NAME": "drone-show ESC",
    "DOC_INPUT": "+BATT 4S-6S",
    "DOC_MCU": "AT32F421G8U7 (QFN-28) x4",
    "DOC_FIRMWARE": "AM32",
    "DOC_PROTOCOL": "DShot",
    "DOC_RATING": "4S-6S, 35 A per channel continuous",
    "DOC_FAB": "PCBWay 2-layer 1.6 mm, 2 oz copper",
}
rules = d["board"]["design_settings"]["rules"]
rules.update({
    "min_clearance": 0.2,
    "min_connection": 0.2,
    "min_track_width": 0.2,
    "min_copper_edge_clearance": 0.3,
    "min_hole_clearance": 0.25,
    "min_hole_to_hole": 0.5,
    "min_through_hole_diameter": 0.3,
    "min_via_diameter": 0.6,
    "min_via_annular_width": 0.15,
    "min_microvia_diameter": 0.6,
    "min_microvia_drill": 0.3,
    "min_text_height": 0.8,
    "min_text_thickness": 0.15,
    "min_silk_clearance": 0.0,
    "solder_mask_to_copper_clearance": 0.0,
})
ds = d["board"]["design_settings"]
ds.setdefault("defaults", {})
base = dict(d["net_settings"]["classes"][0])


def cls(name, clearance, track, via, drill, prio):
    c = dict(base)
    c.update({"name": name, "clearance": clearance, "track_width": track,
              "via_diameter": via, "via_drill": drill, "priority": prio,
              "diff_pair_width": 0.2, "diff_pair_gap": 0.25, "microvia_diameter": 0.6,
              "microvia_drill": 0.3})
    return c


d["net_settings"]["classes"] = [
    cls("Default", 0.2, 0.2, 0.6, 0.3, 2147483647),
    cls("Power", 0.2, 1.0, 0.8, 0.4, 0),
    cls("Gate", 0.2, 0.3, 0.6, 0.3, 1),
    cls("Supply", 0.2, 0.3, 0.6, 0.3, 2),
    cls("Phase", 0.2, 0.3, 0.8, 0.4, 3),
]
d["net_settings"]["netclass_patterns"] = [
    {"netclass": "Power", "pattern": "+BATT"},
    {"netclass": "Power", "pattern": "GND"},
    {"netclass": "Phase", "pattern": "/CSA+"},
    {"netclass": "Phase", "pattern": "*/Motor*"},
    {"netclass": "Gate", "pattern": "*/GH*"},
    {"netclass": "Gate", "pattern": "*/GL*"},
    {"netclass": "Gate", "pattern": "Net-(Q*-G)"},
    {"netclass": "Supply", "pattern": "+10V"},
]
# track and via presets for interactive routing
ds["track_widths"] = [0.0, 0.2, 0.25, 0.3, 0.4, 0.6, 1.0, 2.0]
ds["via_dimensions"] = [{"diameter": 0.0, "drill": 0.0}, {"diameter": 0.6, "drill": 0.3},
                        {"diameter": 0.8, "drill": 0.4}]
json.dump(d, open(path, "w"), indent=2)
print("updated", path)
