package org.firstinspires.ftc.teamcode.Util;

public class Toggle {
    public static boolean toggleButton(boolean input) {
        boolean wasPressed;
        int counter = 1;

        if(input) {
            counter++;
        }

        if(counter % 2 == 0) {
            wasPressed = true;
        } else {
            wasPressed = false;
        }

        return wasPressed;
    }
}
