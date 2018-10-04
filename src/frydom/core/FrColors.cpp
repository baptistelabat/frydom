//
// Created by frongere on 04/10/18.
//

#include "FrColors.h"
#include "FrException.h"



frydom::FrColor::FrColor(frydom::NAMED_COLOR colorName) {

    switch (colorName) {
        case (AliceBlue):
            *this = FrColor(240, 248, 255); break;
        case (AntiqueWhite):
            *this = FrColor(250, 235, 215); break;
        case (Aqua):
            *this = FrColor(0, 255, 255); break;
        case (Aquamarine):
            *this = FrColor(127, 255, 212); break;
        case (Azure):
            *this = FrColor(240, 255, 255); break;
        case (Beige):
            *this = FrColor(245, 245, 220); break;
        case (Bisque):
            *this = FrColor(255, 228, 196); break;
        case (Black):
            *this = FrColor(0, 0, 0); break;
        case (BlanchedAlmond):
            *this = FrColor(255, 255, 205); break;
        case (Blue):
            *this = FrColor(0, 0, 255); break;
        case (BlueViolet):
            *this = FrColor(138, 43, 226); break;
        case (Brown):
            *this = FrColor(165, 42, 42); break;
        case (Burlywood):
            *this = FrColor(222, 184, 135); break;
        case (CadetBlue):
            *this = FrColor(95, 158, 160); break;
        case (Chartreuse):
            *this = FrColor(127, 255, 0); break;
        case (Chocolate):
            *this = FrColor(210, 105, 30); break;
        case (Coral):
            *this = FrColor(255, 127, 80); break;
        case (CornflowerBlue):
            *this = FrColor(100, 149, 237); break;
        case (Cornsilk):
            *this = FrColor(255, 248, 220); break;
        case (Crimson):
            *this = FrColor(220, 20, 60); break;
        case (Cyan):
            *this = FrColor(0, 255, 255); break;
        case (DarkBlue):
            *this = FrColor(0, 0, 139); break;
        case (DarkCyan):
            *this = FrColor(0, 139, 139); break;
        case (DarkGoldenRod):
            *this = FrColor(184, 134, 11); break;
        case (DarkGray):
            *this = FrColor(169, 169, 169); break;
        case (DarkGreen):
            *this = FrColor(0, 100, 0); break;
        case (DarkKhaki):
            *this = FrColor(189, 183, 107); break;
        case (DarkMagenta):
            *this = FrColor(139, 0, 139); break;
        case (DarkOliveGreen):
            *this = FrColor(85, 107, 47); break;
        case (DarkOrange):
            *this = FrColor(255, 140, 0); break;
        case (DarkOrchid):
            *this = FrColor(153, 50, 204); break;
        case (DarkRed):
            *this = FrColor(139, 0, 0); break;
        case (DarkSalmon):
            *this = FrColor(233, 150, 122); break;
        case (DarkSeaGreen):
            *this = FrColor(143, 188, 143); break;
        case (DarkSlateBlue):
            *this = FrColor(72, 61, 139); break;
        case (DarkSlateGray):
            *this = FrColor(47, 79, 79); break;
        case (DarkTurquoise):
            *this = FrColor(0, 206, 209); break;
        case (DarkViolet):
            *this = FrColor(148, 0, 211); break;
        case (DeepPink):
            *this = FrColor(255, 20, 147); break;
        case (DeepSkyBlue):
            *this = FrColor(0, 191, 255); break;
        case (DimGray):
            *this = FrColor(105, 105, 105); break;
        case (DodgerBlue):
            *this = FrColor(30, 144, 255); break;
        case (FireBrick):
            *this = FrColor(178, 34, 34); break;
        case (FloralWhite):
            *this = FrColor(255, 250, 240); break;
        case (ForestGreen):
            *this = FrColor(34, 139, 34); break;
        case (Fuchsia):
            *this = FrColor(255, 0, 255); break;
        case (Gainsboro):
            *this = FrColor(220, 220, 220); break;
        case (GhostWhite):
            *this = FrColor(248, 248, 255); break;
        case (Gold):
            *this = FrColor(255, 215, 0); break;
        case (GoldenRod):
            *this = FrColor(218, 165, 32); break;
        case (Gray):
            *this = FrColor(127, 127, 127); break;
        case (Green):
            *this = FrColor(0, 128, 0); break;
        case (GreenYellow):
            *this = FrColor(173, 255, 47); break;
        case (HoneyDew):
            *this = FrColor(240, 255, 240); break;
        case (HotPink):
            *this = FrColor(255, 105, 180); break;
        case (IndianRed):
            *this = FrColor(205, 92, 92); break;
        case (Indigo):
            *this = FrColor(75, 0, 130); break;
        case (Ivory):
            *this = FrColor(255, 255, 240); break;
        case (Khaki):
            *this = FrColor(240, 230, 140); break;
        case (Lavender):
            *this = FrColor(230, 230, 250); break;
        case (LavenderBlush):
            *this = FrColor(255, 240, 245); break;
        case (Lawngreen):
            *this = FrColor(124, 252, 0); break;
        case (LemonChiffon):
            *this = FrColor(255, 250, 205); break;
        case (LightBlue):
            *this = FrColor(173, 216, 230); break;
        case (LightCoral):
            *this = FrColor(240, 128, 128); break;
        case (LightCyan):
            *this = FrColor(224, 255, 255); break;
        case (LightGoldenRodYellow):
            *this = FrColor(250, 250, 210); break;
        case (LightGreen):
            *this = FrColor(144, 238, 144); break;
        case (LightGrey):
            *this = FrColor(211, 211, 211); break;
        case (LightPink):
            *this = FrColor(255, 182, 193); break;
        case (LightSalmon):
            *this = FrColor(255, 160, 122); break;
        case (LightSeaGreen):
            *this = FrColor(32, 178, 170); break;
        case (LightSkyBlue):
            *this = FrColor(135, 206, 250); break;
        case (LightSlateGray):
            *this = FrColor(119, 136, 153); break;
        case (LightSteelBlue):
            *this = FrColor(176, 196, 222); break;
        case (LightYellow):
            *this = FrColor(255, 255, 224); break;
        case (Lime):
            *this = FrColor(0, 255, 0); break;
        case (LimeGreen):
            *this = FrColor(50, 205, 50); break;
        case (Linen):
            *this = FrColor(250, 240, 230); break;
        case (Magenta):
            *this = FrColor(255, 0, 255); break;
        case (Maroon):
            *this = FrColor(128, 0, 0); break;
        case (MediumAquamarine):
            *this = FrColor(102, 205, 170); break;
        case (MediumBlue):
            *this = FrColor(0, 0, 205); break;
        case (MediumOrchid):
            *this = FrColor(186, 85, 211); break;
        case (MediumPurple):
            *this = FrColor(147, 112, 219); break;
        case (MediumSeaGreen):
            *this = FrColor(60, 179, 113); break;
        case (MediumSlateBlue):
            *this = FrColor(123, 104, 238); break;
        case (MediumSpringGreen):
            *this = FrColor(0, 250, 154); break;
        case (MediumTurquoise):
            *this = FrColor(72, 209, 204); break;
        case (MediumVioletRed):
            *this = FrColor(199, 21, 133); break;
        case (MidnightBlue):
            *this = FrColor(25, 25, 112); break;
        case (MintCream):
            *this = FrColor(245, 255, 250); break;
        case (MistyRose):
            *this = FrColor(255, 228, 225); break;
        case (Moccasin):
            *this = FrColor(255, 228, 181); break;
        case (NavajoWhite):
            *this = FrColor(255, 222, 173); break;
        case (Navy):
            *this = FrColor(0, 0, 128); break;
        case (Navyblue):
            *this = FrColor(159, 175, 223); break;
        case (OldLace):
            *this = FrColor(253, 245, 230); break;
        case (Olive):
            *this = FrColor(128, 128, 0); break;
        case (OliveDrab):
            *this = FrColor(107, 142, 35); break;
        case (Orange):
            *this = FrColor(255, 165, 0); break;
        case (OrangeRed):
            *this = FrColor(255, 69, 0); break;
        case (Orchid):
            *this = FrColor(218, 112, 214); break;
        case (PaleGoldenRod):
            *this = FrColor(238, 232, 170); break;
        case (PaleGreen):
            *this = FrColor(152, 251, 152); break;
        case (PaleTurquoise):
            *this = FrColor(175, 238, 238); break;
        case (PaleVioletRed):
            *this = FrColor(219, 112, 147); break;
        case (PapayaWhip):
            *this = FrColor(255, 239, 213); break;
        case (PeachPuff):
            *this = FrColor(255, 218, 185); break;
        case (Peru):
            *this = FrColor(205, 133, 63); break;
        case (Pink):
            *this = FrColor(255, 192, 203); break;
        case (Plum):
            *this = FrColor(221, 160, 221); break;
        case (PowderBlue):
            *this = FrColor(176, 224, 230); break;
        case (Purple):
            *this = FrColor(128, 0, 128); break;
        case (Red):
            *this = FrColor(255, 0, 0); break;
        case (RosyBrown):
            *this = FrColor(188, 143, 143); break;
        case (RoyalBlue):
            *this = FrColor(65, 105, 225); break;
        case (SaddleBrown):
            *this = FrColor(139, 69, 19); break;
        case (Salmon):
            *this = FrColor(250, 128, 114); break;
        case (SandyBrown):
            *this = FrColor(244, 164, 96); break;
        case (SeaGreen):
            *this = FrColor(46, 139, 87); break;
        case (SeaShell):
            *this = FrColor(255, 245, 238); break;
        case (Sienna):
            *this = FrColor(160, 82, 45); break;
        case (Silver):
            *this = FrColor(192, 192, 192); break;
        case (SkyBlue):
            *this = FrColor(135, 206, 235); break;
        case (SlateBlue):
            *this = FrColor(106, 90, 205); break;
        case (SlateGray):
            *this = FrColor(112, 128, 144); break;
        case (Snow):
            *this = FrColor(255, 250, 250); break;
        case (SpringGreen):
            *this = FrColor(0, 255, 127); break;
        case (SteelBlue):
            *this = FrColor(70, 130, 180); break;
        case (Tan):
            *this = FrColor(210, 180, 140); break;
        case (Teal):
            *this = FrColor(0, 128, 128); break;
        case (Thistle):
            *this = FrColor(216, 191, 216); break;
        case (Tomato):
            *this = FrColor(255, 99, 71); break;
        case (Turquoise):
            *this = FrColor(64, 224, 208); break;
        case (Violet):
            *this = FrColor(238, 130, 238); break;
        case (Wheat):
            *this = FrColor(245, 222, 179); break;
        case (White):
            *this = FrColor(255, 255, 255); break;
        case (WhiteSmoke):
            *this = FrColor(245, 245, 245); break;
        case (Yellow):
            *this = FrColor(255, 255, 0); break;
        case (YellowGreen):
            *this = FrColor(139, 205, 50); break;
        default:
            throw FrException("Unknown color name");
    }
}
