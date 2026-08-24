// Contre-piece (striker) du microswitch V-156 pour le bumper VECTOR
// - se visse dans la rainure centrale d'un profile alu 20 mm (rainure a 10 mm du bord)
// - le BOUT depasse de 12 mm du bord du profile => bout a 22 mm du CENTRE des trous
// - largeur 30 mm, deux trous sur l'axe de la rainure, t-nuts M4 (passage 4.4)
// Impression : TPU (metrox). Editable : toutes les cotes sont des parametres.

width        = 30;    // largeur, le long de la rainure [mm]
tip_from_hole = 22;   // centre des trous -> bout [mm] (10 sortie profile + 12 depassement)
tail         = 8;     // matiere derriere les trous [mm]
thickness    = 10;   // epaisseur [mm] (metrox 24/08)
hole_d       = 4.4;   // passage vis M4
hole_spacing = 16;    // entraxe des deux trous, le long de la rainure [mm]
head_d       = 8.4;   // fraisage tete conique M4 (0 = pas de fraisage)
head_depth   = 2.2;
nose_chamfer = 2;     // petit chanfrein du bout

module striker() {
    difference() {
        // plaque : y=0 est l'axe des trous ; le bout est a y=+tip_from_hole
        translate([-width/2, -tail, 0])
            cube([width, tail + tip_from_hole, thickness]);
        // chanfrein du nez
        translate([-width/2 - 1, tip_from_hole - nose_chamfer, thickness])
            rotate([45, 0, 0])
            cube([width + 2, nose_chamfer * 2, nose_chamfer * 2]);
        // deux trous + fraisage
        for (x = [-hole_spacing/2, hole_spacing/2]) {
            translate([x, 0, -1]) cylinder(d = hole_d, h = thickness + 2, $fn = 48);
            if (head_d > 0)
                translate([x, 0, thickness - head_depth])
                    cylinder(d1 = hole_d, d2 = head_d, h = head_depth + 0.01, $fn = 48);
        }
    }
}
striker();
