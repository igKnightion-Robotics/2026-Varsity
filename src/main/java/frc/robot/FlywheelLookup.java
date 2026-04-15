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
        kDistanceToRpm.put(1.9, 3500.0);
        kDistanceToRpm.put(2.5, 3650.0);
        kDistanceToRpm.put(3.0, 3825.0);
        kDistanceToRpm.put(3.5, 4000.0);
        kDistanceToRpm.put(4.0, 4200.0);
        kDistanceToRpm.put(4.5, 4500.0);
        kDistanceToRpm.put(5.0, 4750.0);
        kDistanceToRpm.put(5.5, 5050.0);
    }

    public static double getRpmForDistance(double distanceMeters) {
        return kDistanceToRpm.get(distanceMeters);
    }

}
