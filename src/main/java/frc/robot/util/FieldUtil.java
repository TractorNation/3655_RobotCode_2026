package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class FieldUtil {
  public static double getDistanceToWall(Pose2d pose) {
    Translation2d translation = pose.getTranslation();
    double rotation = MathUtil.inputModulus(pose.getRotation().getDegrees(), -180, 180);
    double blueWallDistance = translation.getX();
    double redWallDistance = Constants.Field.FIELD_BOUNDS.getXWidth() - translation.getX();
    double scoringTableWallDistance = translation.getY();
    double otherWallDistance = Constants.Field.FIELD_BOUNDS.getYWidth() - translation.getY();

    if (rotation <= 30 && rotation >= -30) {
      return redWallDistance;
    } else if (rotation <= -30 && rotation >= -150) {
      return otherWallDistance;
    } else if ((rotation <= -150 && rotation >= -180) || (rotation <= 180 && rotation >= 150)) {
      return blueWallDistance;
    } else if (rotation <= 150 && rotation >= 30) {
      return scoringTableWallDistance;
    }

    return 0.0;
  }
}
