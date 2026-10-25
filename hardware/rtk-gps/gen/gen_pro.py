"""Write rtk_gps.kicad_pro (PCBWay 2-layer standard-process rules + net classes) and rtk_gps.kicad_dru."""
import json, sys, os
PRJ = sys.argv[1]
def nc(name, clr, trk, via=0.65, drill=0.3, prio=None):
    d = {"name": name, "clearance": clr, "track_width": trk, "via_diameter": via, "via_drill": drill,
         "microvia_diameter": 0.3, "microvia_drill": 0.1, "diff_pair_width": 0.2, "diff_pair_gap": 0.25,
         "diff_pair_via_gap": 0.25, "wire_width": 6, "bus_width": 12, "line_style": 0,
         "pcb_color": "rgba(0, 0, 0, 0.000)", "schematic_color": "rgba(0, 0, 0, 0.000)"}
    if prio is not None: d["priority"] = prio
    return d
pro = {
 "board": {
  "3dviewports": [], "layer_presets": [], "viewports": [],
  "design_settings": {
   "defaults": {"board_outline_line_width": 0.1, "copper_line_width": 0.2, "copper_text_size_h": 1.0,
                "copper_text_size_v": 1.0, "copper_text_thickness": 0.15, "silk_line_width": 0.15,
                "silk_text_size_h": 0.8, "silk_text_size_v": 0.8, "silk_text_thickness": 0.15,
                "pads": {"drill": 0.8, "height": 1.5, "width": 1.5}, "zones": {"min_clearance": 0.25}},
   "rules": {
     "min_clearance": 0.15,            # PCBWay std: 6 mil
     "min_track_width": 0.15,          # 6 mil
     "min_connection": 0.15,
     "min_via_diameter": 0.6,          # 0.3 drill + 2 x 0.15 ring
     "min_via_annular_width": 0.15,
     "min_through_hole_diameter": 0.3, # PCBWay min mechanical drill
     "min_hole_to_hole": 0.5,
     "min_hole_clearance": 0.25,
     "min_copper_edge_clearance": 0.3,
     "min_silk_clearance": 0.0,
     "min_text_height": 0.8,
     "min_text_thickness": 0.15,
     "solder_mask_to_copper_clearance": 0.0,
     "min_microvia_diameter": 0.3, "min_microvia_drill": 0.1,
     "allow_blind_buried_vias": False, "allow_microvias": False,
     "max_error": 0.005, "min_groove_width": 0.0, "min_resolved_spokes": 2,
     "use_height_for_length_calcs": True},
   "rule_severities": {"silk_overlap": "ignore", "silk_over_copper": "warning", "lib_footprint_mismatch": "ignore",
                       "lib_footprint_issues": "ignore", "text_height": "warning", "text_thickness": "warning"},
   "track_widths": [0.0, 0.2, 0.25, 0.4, 0.6],
   "via_dimensions": [{"diameter": 0.0, "drill": 0.0}, {"diameter": 0.65, "drill": 0.3}],
   "diff_pair_dimensions": [],
   "solder_mask_min_width": 0.1, "solder_mask_expansion": 0.05,
   "apply_defaults_to_fp_fields": False, "apply_defaults_to_fp_shapes": False, "apply_defaults_to_fp_text": False,
  },
  "ipc2581": {}, "layer_pairs": [], "teardrop_options": [], "teardrop_parameters": [], "tuning_pattern_settings": {}
 },
 "boards": [], "cvpcb": {"equivalence_files": []},
 "libraries": {"pinned_footprint_libs": [], "pinned_symbol_libs": []},
 "meta": {"filename": "rtk_gps.kicad_pro", "version": 3},
 "net_settings": {
  "classes": [nc("Default", 0.15, 0.2, prio=2147483647), nc("Power", 0.15, 0.2, prio=0), nc("RF", 0.25, 0.6, prio=1)],
  "meta": {"version": 4}, "net_colors": None, "netclass_assignments": None,
  "netclass_patterns": [{"netclass": "Power", "pattern": "/" + p} for p in
                        ("+5V", "VIN", "VBUS", "+3V3", "+1V1", "GND", "VREG_LX", "VCC_RF", "ANT_BIAS")] +
                       [{"netclass": "RF", "pattern": "/RF_IN"}]},
 "pcbnew": {"last_paths": {}, "page_layout_descr_file": ""},
 "schematic": {"legacy_lib_dir": "", "legacy_lib_list": []},
 "sheets": [], "text_variables": {}
}
json.dump(pro, open(os.path.join(PRJ, "rtk_gps.kicad_pro"), "w"), indent=2)

dru = '''(version 1)
# PCBWay standard 2-layer process (1.6 mm FR-4, 1 oz, HASL/ENIG). Values match the board setup
# in rtk_gps.kicad_pro; the rules below add the checks the global settings cannot express.

(rule "PCBWay min annular ring"
  (constraint annular_width (min 0.15mm)))

(rule "PCBWay min drill"
  (constraint hole_size (min 0.3mm)))

(rule "PCBWay hole to hole"
  (constraint hole_to_hole (min 0.5mm)))

(rule "PCBWay copper to board edge"
  (constraint edge_clearance (min 0.3mm)))

(rule "PCBWay silkscreen line and text"
  (layer "?.Silkscreen")
  (constraint text_thickness (min 0.15mm))
  (constraint text_height (min 0.8mm)))

(rule "Silk off exposed copper"
  (layer "?.Silkscreen")
  (constraint silk_clearance (min 0.1mm)))

# Fine-pitch parts (RP2350 QFN-60 0.4 mm, MMC5983MA LGA-16 0.5 mm, USB-C 0.5 mm): pad gaps are 0.2 mm,
# so 0.15 mm is the floor there. Everywhere else keep the more comfortable 0.2 mm.
(rule "General clearance 0.2 mm"
  (condition "A.Type != 'Pad' || B.Type != 'Pad'")
  (constraint clearance (min 0.15mm) (opt 0.2mm)))

(rule "RF feed keep-clear"
  (condition "A.NetName == '/RF_IN' && B.Type != 'Zone'")
  (constraint clearance (min 0.25mm)))

(rule "Plane clearance"
  (condition "A.Type == 'Zone' || B.Type == 'Zone'")
  (constraint clearance (min 0.25mm)))

(rule "Thermal spokes"
  (condition "A.Type == 'Zone'")
  (constraint thermal_spoke_width (min 0.3mm))
  (constraint thermal_relief_gap (min 0.25mm)))
'''
open(os.path.join(PRJ, "rtk_gps.kicad_dru"), "w").write(dru)
