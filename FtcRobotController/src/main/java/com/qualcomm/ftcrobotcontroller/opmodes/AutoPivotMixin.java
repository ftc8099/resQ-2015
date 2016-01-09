package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by students on 12/12/15.
 */
public class AutoPivotMixin {
    public double m1speed;
    public double m2speed;

    // Motor 1 right, motor2 left
    public AutoPivotMixin(double m1, double m2) {
        m1speed = m1;
        m2speed = m2;
    }

    public static AutoPivotMixin RED = new AutoPivotMixin(0.75, 0.5);
    public static AutoPivotMixin BLUE = new AutoPivotMixin(0.5, 0.75);
}
