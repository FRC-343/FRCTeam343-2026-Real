package frc.robot.field;

public enum HubFaces {
  A(FieldConstants.blueHubA, FieldConstants.redHubA),
  B(FieldConstants.blueHubB, FieldConstants.redHubB),
  C(FieldConstants.blueHubC, FieldConstants.redHubC),
  D(FieldConstants.blueHubD, FieldConstants.redHubD),
  E(FieldConstants.blueHubE, FieldConstants.redHubE),
  F(FieldConstants.blueHubF, FieldConstants.redHubF),
  G(FieldConstants.blueHubG, FieldConstants.redHubG),
  H(FieldConstants.blueHubH, FieldConstants.redHubH);

  public final HubFace blue;
  public final HubFace red;

  private HubFaces(HubFace blue, HubFace red) {
    this.blue = blue;
    this.red = red;
  }

  public HubFace get() {
    return FieldUtils.isBlueAlliance() ? blue : red;
  }
}
