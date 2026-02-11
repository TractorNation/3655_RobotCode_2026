// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";

  // Robot to camera transforms
  // used for sim
  public static Transform3d robotToCamera0 = new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));

  public static Transform3d robotToCamera1 = new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.25;
  public static double maxZError = 0.5;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 1; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors = new double[] {
      1, // Camera 0
      1 // Camera 1
  };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.25; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

  public static double linearStdDevMegatag1Factor = 0.8;
  public static double angularStdDevMegatag1Factor = Double.POSITIVE_INFINITY;

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {
  }

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {
  }

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }
}