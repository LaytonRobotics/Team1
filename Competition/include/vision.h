/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature SIG_1 = vex::vision::signature (1, 6817, 8095, 7456, -2267, -1821, -2044, 3, 0); // ORANGE
vex::vision::signature SIG_2 = vex::vision::signature (2, -9069, -1473, -5271, -4509, -745, -2627, 1.2, 0); // GREEN 
vex::vision::signature SIG_3 = vex::vision::signature (3, 971, 2287, 1629, 6397, 10159, 8278, 2.5, 0); // PURPLE
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision Vision1 = vex::vision (vex::PORT1, 50, SIG_1, SIG_2, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/