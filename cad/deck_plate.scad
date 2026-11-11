// =====================================================================
//  deck_plate.scad  -  extra top deck for the MR2006B car
//
//  A flat plate with a 10 mm grid of M3 holes (screws, standoffs or zip
//  ties anywhere), 4 mounting holes for brass standoffs, and two cable
//  slots. Mount it on M3 brass standoffs screwed into the existing upper
//  plate.
//
//  HOW TO FIT IT TO YOUR CAR (2 minutes with a ruler):
//   1. Pick 4 holes in the existing upper plate for the new standoffs.
//   2. Measure their centre-to-centre spacing across (x) and along (y).
//   3. Put the numbers in mount_x / mount_y below (or edit mount_holes).
//   4. OpenSCAD (free): F6 to render, File > Export > STL. Print flat.
//  Print: PLA/PETG, 0.2 mm layers, 3 walls, 20 % infill, no supports.
// =====================================================================

/* [Plate] */
plate_w      = 110;   // across the car (x), mm
plate_l      = 180;   // along the car (y), mm
thickness    = 3;     // mm (3 is stiff enough; 4 for a heavy battery)
corner_r     = 8;

/* [Mounting to the chassis] */
mount_x      = 70;    // centre-to-centre spacing across, mm   <- MEASURE
mount_y      = 120;   // centre-to-centre spacing along, mm    <- MEASURE
mount_offset_y = 0;   // shift the hole pattern forward (+) / back (-)
mount_d      = 3.4;   // M3 clearance
mount_boss_d = 9;     // solid ring around each mounting hole

/* [Grid] */
grid_pitch   = 10;
grid_d       = 3.2;   // fits M3 screws and 2.5 mm zip ties
edge_margin  = 6;

/* [Cable slots] */
slot_w       = 26;
slot_h       = 9;
slot_y       = [55, -55];   // positions along the car (0 = centre)

$fn = 32;

mount_holes = [for (sx = [-1, 1], sy = [-1, 1]) [sx * mount_x / 2, sy * mount_y / 2 + mount_offset_y]];

module rounded_rect(w, l, r, h) {
  linear_extrude(h)
    offset(r) offset(-r) square([w, l], center = true);
}

function near_mount(p) = min([for (m = mount_holes) norm(p - m)]) < mount_boss_d / 2 + grid_d / 2 + 1;
function in_slot(p) = len([for (sy = slot_y) if (abs(p[0]) < slot_w / 2 + grid_d && abs(p[1] - sy) < slot_h / 2 + grid_d) 1]) > 0;

difference() {
  rounded_rect(plate_w, plate_l, corner_r, thickness);

  // mounting holes
  for (m = mount_holes) translate([m[0], m[1], -1]) cylinder(d = mount_d, h = thickness + 2);

  // grid
  nx = floor((plate_w - 2 * edge_margin) / grid_pitch / 2);
  ny = floor((plate_l - 2 * edge_margin) / grid_pitch / 2);
  for (ix = [-nx : nx], iy = [-ny : ny]) {
    p = [ix * grid_pitch, iy * grid_pitch];
    if (!near_mount(p) && !in_slot(p))
      translate([p[0], p[1], -1]) cylinder(d = grid_d, h = thickness + 2);
  }

  // cable slots
  for (sy = slot_y)
    translate([0, sy, -1]) linear_extrude(thickness + 2)
      offset(slot_h / 2 - 0.01) square([slot_w - slot_h, 0.02], center = true);
}
