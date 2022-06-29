// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final class ROBOT_MODES {
        public final static int MANUAL = 0;
        public final static int AUTO = 1;
    }

    public static final class AUTO_MODES {
        public final static int STATICV_ENCODER_DRIVE = 0;
        public final static int PATHPLANNER_MODE = 1;
    }
}
