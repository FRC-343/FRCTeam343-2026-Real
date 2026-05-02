package frc.robot.subsystems.vision2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.List;
import java.util.Optional;
import org.photonvision.simulation.VisionSystemSim;

public class VisionConstants {
  public static final record AprilTagCameraConfig(VisionSource source, SimCameraConfig simConfig) {}

  public static enum PoseEstimationMethod {
    MULTI_TAG,
    SINGLE_TAG
  }

  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final Optional<VisionSystemSim> aprilTagSim =
      Constants.currentMode == Mode.SIM
          ? Optional.of(new VisionSystemSim("AprilTagSim"))
          : Optional.empty();

  private static final List<AprilTagCameraConfig> warrigConfigs =
      List.of(
          new AprilTagCameraConfig(
              new VisionSource(
                  "BRight",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(12.313521), // forward+
                          Units.inchesToMeters(8.042081), // left+
                          Units.inchesToMeters(8.014049)), // up+
                      new Rotation3d(
                          0, Units.degreesToRadians(-15), Units.degreesToRadians(-15.504090)))),
              SimCameraConfig.ARDUCAM_OV9281_70),
          new AprilTagCameraConfig(
              new VisionSource(
                  "SRight",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(.344), // forward+
                          Units.inchesToMeters(13.618), // left+
                          Units.inchesToMeters(17.342)), // up+
                      new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(90)))),
              SimCameraConfig.ARDUCAM_OV9281_70),
          new AprilTagCameraConfig(
              new VisionSource(
                  "SLeft",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(.344), // forward+
                          Units.inchesToMeters(-13.618), // left+
                          Units.inchesToMeters(17.342)), // up+
                      new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(-90)))),
              SimCameraConfig.ARDUCAM_OV9281_70),
          new AprilTagCameraConfig(
              new VisionSource(
                  "BLeft",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(12.106229), // forward+
                          Units.inchesToMeters(-8.203269), // left+
                          Units.inchesToMeters(8.014049)), // up+
                      new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(15)))),
              SimCameraConfig.ARDUCAM_OV9281_70));
  public static final List<AprilTagCameraConfig> aprilTagCamerasConfigs = warrigConfigs;

  public static final double ambiguityCutoff = .1;
  public static final double singleTagPoseCutoffMeters = 4;

  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(2, 2, 4);
  public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(4, 4, 8);
}
