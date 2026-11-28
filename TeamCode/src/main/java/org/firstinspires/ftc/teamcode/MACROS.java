package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MACROS {

    private HardwareMap hwMap;
    private Follower follower;

    public void initMacros(HardwareMap hardwareMap, Follower follower) {
        this.hwMap = hardwareMap;
        this.follower = follower;
    }

    public enum TLockState {
        TLock_ON(1.0),
        TLock_OFF(0.0);

        public final double state;

        TLockState(double state) {
            this.state = state;
        }
    }

    public class TargetLock {

        public void toggleTL() {

        }

        // Gets lock angle automatically using Pedro's follower odometry
        public double runGetLockAngle(double targetX, double targetY, double width) {
            if (TLockState. = TLockState.TLock_ON)
            if (follower == null) return 0.0;
            Pose pose = follower.pose();
            return getLockAngle(pose.x(), pose.y(), targetX, targetY, width);
        }

        public double getLockAngle(double x, double y, double targetX, double targetY, double width) {
            double a = targetX - x;
            double b = targetY - y;
            double theta = Math.atan2(b, a);

            if (y > width / 2) {
                theta *= -1;
            }

            return theta;
        }
    }
}
