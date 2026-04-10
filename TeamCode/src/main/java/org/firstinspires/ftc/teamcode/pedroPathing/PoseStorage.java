package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

public class PoseStorage {
    public static double x = 105.3;
    public static double y = 33.3;
    public static double heading = Math.PI;

    public static void save(Pose pose) {
        x = pose.getX();
        y = pose.getY();
        heading = pose.getHeading();
    }

    public static Pose get() {
        return new Pose(x, y, heading);
    }
}