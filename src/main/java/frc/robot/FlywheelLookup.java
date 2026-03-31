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
        kDistanceToRpm.put(2.097, 3500.0);
        kDistanceToRpm.put(2.677, 3500.0);
        kDistanceToRpm.put(3.127, 3700.0);
        kDistanceToRpm.put(3.627, 3850.0);
        kDistanceToRpm.put(4.127, 2500.0);
        kDistanceToRpm.put(5.0, 3000.0);
    }

    public static double getRpmForDistance(double distanceMeters) {
        return kDistanceToRpm.get(distanceMeters);
    }

    
}
