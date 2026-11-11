// =====================================================================
//  sonar_bracket.scad  -  front bracket for the HC-SR04
//
//  The two "eyes" push through the round holes from behind; a drop of
//  hot glue (or two small zip ties) holds the board. The foot has two
//  M3 slots to screw it onto the front bumper plate or the deck.
//
//  Check with calipers before printing: eye diameter (usually 16 mm)
//  and centre-to-centre spacing of the eyes (usually ~26 mm).
//  Print standing on the foot, no supports.
// =====================================================================

eye_d        = 16.6;  // hole for the transducers (16 mm + clearance)
eye_spacing  = 26;    // centre-to-centre                       <- CHECK
face_w       = 52;
face_t       = 2.5;
sensor_h     = 45;    // eye centre height above the foot, mm
                      // (higher = fewer false alarms from the ground)
tilt_up      = 3;     // degrees, aims slightly up to ignore the ground
foot_d       = 22;    // foot depth
foot_t       = 4;
slot_len     = 8;     // M3 slots, lets you adjust the position
slot_spacing = 34;

$fn = 48;
face_h = sensor_h + eye_d / 2 + 6;

difference() {
  union() {
    // foot
    translate([-face_w / 2, 0, 0]) cube([face_w, foot_d, foot_t]);
    // face (tilted back by tilt_up)
    rotate([-tilt_up, 0, 0]) translate([-face_w / 2, 0, 0]) cube([face_w, face_t, face_h]);
    // gussets
    for (sx = [-1, 1])
      translate([sx * (face_w / 2 - 2) - 1.5, 0, 0])
        rotate([90, 0, 90]) linear_extrude(3)
          polygon([[0, 0], [foot_d - 2, 0], [0, face_h * 0.45]]);
  }
  // eyes
  for (sx = [-1, 1])
    rotate([-tilt_up, 0, 0]) translate([sx * eye_spacing / 2, -1, sensor_h])
      rotate([-90, 0, 0]) cylinder(d = eye_d, h = face_t + 2);
  // M3 slots in the foot
  for (sx = [-1, 1])
    translate([sx * slot_spacing / 2, foot_d / 2 + 3, -1]) hull() {
      translate([0, -slot_len / 2, 0]) cylinder(d = 3.4, h = foot_t + 2);
      translate([0,  slot_len / 2, 0]) cylinder(d = 3.4, h = foot_t + 2);
    }
}
