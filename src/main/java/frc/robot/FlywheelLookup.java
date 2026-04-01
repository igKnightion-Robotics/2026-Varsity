// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class FlywheelLookup {
    private static final InterpolatingDoubleTreeMap kDistanceToRpm = new InterpolatingDoubleTreeMap();
    static {
        // Add distance to RPM mappings here
        kDistanceToRpm.put(1.7, 0.0);
        kDistanceToRpm.put(1.967, 3575.0);
        kDistanceToRpm.put(2.367, 3575.0);
        kDistanceToRpm.put(2.887, 3700.0);
        kDistanceToRpm.put(3.337, 3850.0);
        kDistanceToRpm.put(3.747, 4150.0);
        kDistanceToRpm.put(4.187, 4400.0);
        kDistanceToRpm.put(5.427, 5000.0);
    }

    public static double getRpmForDistance(double distanceMeters) {
        return kDistanceToRpm.get(distanceMeters);
    }

}
