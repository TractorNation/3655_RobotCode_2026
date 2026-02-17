// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionConstants.PoseObservation;
import frc.robot.subsystems.vision.VisionConstants.PoseObservationType;
import frc.robot.subsystems.vision.VisionConstants.TargetObservation;
import frc.robot.util.LimelightHelpers;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  private Set<Integer> tagIds;
  private List<PoseObservation> poseObservations;

  private TimestampedDoubleArray llSampleMT1;
  private TimestampedDoubleArray llSampleMT2;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name             The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for
   *                         MegaTag 2.
   */
  public VisionIOLimelight(String name) {
    var table = NetworkTableInstance.getDefault().getTable(name);

    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);

    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);

    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});

    tagIds = new HashSet<>();
    poseObservations = new LinkedList<>();

    LimelightHelpers.SetIMUMode(name, 0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    tagIds.clear();
    poseObservations.clear();

    // Update connection status based on whether an update has been seen in the last
    // 250ms
    inputs.connected = ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation = new TargetObservation(
        Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] { RobotState.getInstance().getPose().getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0 });

    // NetworkTableInstance.getDefault()
    // .flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    llSampleMT1 = megatag1Subscriber.getAtomic();

    for (int i = 11; i < llSampleMT1.value.length; i += 7) {
      tagIds.add((int) llSampleMT1.value[i]);
    }
    poseObservations.add(
        new PoseObservation(
            // Timestamp, based on server timestamp of publish and latency
            llSampleMT1.timestamp * 1.0e-6 - llSampleMT1.value[6] * 1.0e-3,

            // 3D pose estimate
            parsePose(llSampleMT1.value),

            // Ambiguity, using only the first tag because ambiguity isn't applicable for
            // multitag
            llSampleMT1.value.length >= 18 ? llSampleMT1.value[17] : 0.0,

            // Tag count
            (int) llSampleMT1.value[7],

            // Average tag distance
            llSampleMT1.value[9],

            // Observation type
            PoseObservationType.MEGATAG_1));

    llSampleMT2 = megatag2Subscriber.getAtomic();

    for (int i = 11; i < llSampleMT2.value.length; i += 7) {
      tagIds.add((int) llSampleMT2.value[i]);
    }
    
    poseObservations.add(
        new PoseObservation(
            // Timestamp, based on server timestamp of publish and latency
            llSampleMT2.timestamp * 1.0e-6 - llSampleMT2.value[6] * 1.0e-3,

            // 3D pose estimate
            parsePose(llSampleMT2.value),

            // Ambiguity, zeroed because the pose is already disambiguated
            0.0,

            // Tag count
            (int) llSampleMT2.value[7],

            // Average tag distance
            llSampleMT2.value[9],

            // Observation type
            PoseObservationType.MEGATAG_2));

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] llLLArray) {
    return new Pose3d(
        llLLArray[0],
        llLLArray[1],
        llLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(llLLArray[3]),
            Units.degreesToRadians(llLLArray[4]),
            Units.degreesToRadians(llLLArray[5])));
  }
}