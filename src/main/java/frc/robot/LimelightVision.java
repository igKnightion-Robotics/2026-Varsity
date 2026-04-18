// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class LimelightVision {
    private static final String LIMELIGHT = "limelight-shooter";

    private static final double MAX_AMIGUITY = 0.7;
    private static final double MAX_Z_ERROR = 0.5;  // meters

    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public Optional<LimelightHelpers.PoseEstimate> getAcceptedMt2Pose(double yawDeg, double yawRateDegPerSec) {
        // Must occur every loop BEFORE requesting MT2 pose.
        LimelightHelpers.SetRobotOrientation(LIMELIGHT, yawDeg, yawRateDegPerSec, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT);
        double[] botpose = LimelightHelpers.getBotPose_wpiBlue(LIMELIGHT);
        double z = (botpose.length > 2) ? botpose[2] : 0.0;

        // Verification checks to reject bad pose estimates from the LL.
        // First, if we have one tag, reject it if its ambiguity (how much the LL distrusts the tag)
        // is too high.
        boolean singleTagHighAmbiguity =
            est.tagCount == 1
            && est.rawFiducials != null
            && est.rawFiducials.length > 0
            && est.rawFiducials[0].ambiguity > MAX_AMIGUITY;

        // Then, on all tags regardless of count, reject if............................
        boolean reject =
            est.tagCount == 0                                                     // ...there are no tags
            || singleTagHighAmbiguity                                             // ...or there is one tag, but ambiguity is too high
            || Math.abs(z) > MAX_Z_ERROR                                          // ...or there is a change of > 0.5 meters from the current location
            || est.pose.getX() < 0.0                                              // ...or the result is beyond the blue alliance wall
            || est.pose.getX() > fieldLayout.getFieldLength()                     // ...or beyond the red alliance wall
            || est.pose.getY() < 0.0                                              // ...or beyond the bottom of the field
            || est.pose.getY() > fieldLayout.getFieldWidth()                      // ...or beyond the top of the field
            || !MathUtil.isNear(yawDeg, est.pose.getRotation().getDegrees(),
                20.0, 0, 360)                                   // ...or more than 20 degress from the gyro
            || Math.abs(yawRateDegPerSec) > 360;                                  // ...or the robot is spinning more than one rotation per second

        // Return nothing if the new pose violates the above rules, or the pose if it is OK.
        return reject ? Optional.empty() : Optional.of(est);
        }
}
