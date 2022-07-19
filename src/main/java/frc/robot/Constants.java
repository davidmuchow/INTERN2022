// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ID {
        public final static int MOTORLEFT_ONE = 1;
        public final static int MOTORLEFT_TWO = 2;

        public final static int MOTORRIGHT_ONE = 3;
        public final static int MOTORRIGHT_TWO = 4;
    }
    public static final class ENCODER_CONSTANTS {
        public static final double GEAR_RATIO = 10.71;
        public static final double TICKS_PER_ROTATION = 42;
        public static final int WHEEL_DIAMETER = 6; //inches
        public static final double WHEEL_CIRCUMFRENCE = Math.PI * 2 * WHEEL_DIAMETER / 2;
        public static final double WHEEM_CIRCUMFRENCE_METERS = Units.inchesToMeters(WHEEL_CIRCUMFRENCE);
    }
    
    public static final class AUTO_CONSTANTS {
        public static final double ksVolts = 0.11532;
        public static final double kvVoltSecondsPerMeter = 2.9005;
        public static final double kaVoltSecondsSquaredPerMeter =0.48898;
        public static final double kTrackwidthMeters = 0.64;
        public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

        DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    ksVolts,
                    kvVoltSecondsPerMeter,
                    kaVoltSecondsSquaredPerMeter),
                kDriveKinematics,
            10);
    
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 5.3853E-06;
        // max speeds n shit 
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
        TrajectoryConfig config = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(kDriveKinematics)
            .addConstraint(autoVoltageConstraint);
        
        // ramsete coefficents
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final String[] pathNames = {"New Path"};
    }


}
