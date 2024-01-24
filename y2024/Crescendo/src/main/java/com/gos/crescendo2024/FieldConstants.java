package com.gos.crescendo2024;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final AprilTagFieldLayout TAG_LAYOUT =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final Translation2d AMP_CENTER =
        new Translation2d(Units.inchesToMeters(72.455), Units.inchesToMeters(322.996));

    public static final double FIELD_LENGTH = Units.inchesToMeters(651.223);
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.223);
    public static final double FIELD_WIDTH = Units.inchesToMeters(323.277);

    public static final class Speaker {

        /** Center of the speaker opening (blue alliance) */
        public static final Pose2d CENTER_SPEAKER_OPENING =
            new Pose2d(0.0, FIELD_WIDTH - Units.inchesToMeters(104.0), new Rotation2d());
    }


    /** Staging locations for each note */
    public static final class StagingLocations {
        public static double CENTER_LINE_X = FIELD_LENGTH / 2;

        // need to update
        public static final double CENTER_LINE_FIRST_Y = Units.inchesToMeters(29.638);
        public static final double CENTER_LINE_SEPARATION_Y = Units.inchesToMeters(66);
        public static final double SPIKE_X = Units.inchesToMeters(114);
        // need
        public static final double SPIKE_FIRST_Y = Units.inchesToMeters(161.638);
        public static final double SPIKE_SEPARATION_Y = Units.inchesToMeters(57);

        public static final Translation2d[] CENTER_LINE_TRANSLATIONS = new Translation2d[5];
        public static final Translation2d[] SPIKE_TRANSLATIONS = new Translation2d[3];

        static {
            for (int i = 0; i < CENTER_LINE_TRANSLATIONS.length; i++) {
                CENTER_LINE_TRANSLATIONS[i] =
                    new Translation2d(CENTER_LINE_X, CENTER_LINE_FIRST_Y + (i * CENTER_LINE_SEPARATION_Y));
            }
        }

        static {
            for (int i = 0; i < SPIKE_TRANSLATIONS.length; i++) {
                SPIKE_TRANSLATIONS[i] = new Translation2d(SPIKE_X, SPIKE_FIRST_Y + (i * SPIKE_SEPARATION_Y));
            }
        }
    }
}
