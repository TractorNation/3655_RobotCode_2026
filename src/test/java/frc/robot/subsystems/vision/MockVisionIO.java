package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.VisionConstants.PoseObservation;
import frc.robot.subsystems.vision.VisionConstants.TargetObservation;

/**
 * Mock implementation of VisionIO for unit testing.
 * 
 * <p>This allows testing VisionSubsystem without requiring actual cameras.
 * You can set vision observations and verify filtering behavior programmatically.
 */
public class MockVisionIO implements VisionIO {
  private VisionIOInputs inputs = new VisionIOInputs();

  public MockVisionIO() {
    inputs.connected = true;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = this.inputs.connected;
    inputs.latestTargetObservation = this.inputs.latestTargetObservation;
    inputs.poseObservations = this.inputs.poseObservations.clone();
    inputs.tagIds = this.inputs.tagIds.clone();
  }

  // Test helper methods
  public void setConnected(boolean connected) {
    inputs.connected = connected;
  }

  public void setLatestObservation(TargetObservation observation) {
    inputs.latestTargetObservation = observation;
  }

  public void setPoseObservations(PoseObservation[] observations) {
    inputs.poseObservations = observations.clone();
  }

  public void setTagIds(int[] tagIds) {
    inputs.tagIds = tagIds.clone();
  }
}

