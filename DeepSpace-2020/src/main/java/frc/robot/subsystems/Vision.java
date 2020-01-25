/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {

  private static NetworkTable table;

  public static class Values {
    public double tX = Double.NaN;
    public double tY = Double.NaN;
    public double tA = Double.NaN;
    public double matrixDistance = Double.NaN;
    public double matrixRotationAngle = Double.NaN;
    public double matrixApproachAngle = Double.NaN;
    public double tDistance = Double.NaN;
    public double leftRightRatio = Double.NaN;
    public double tSkew = Double.NaN;
    public double tShort = Double.NaN;
    public double tLong = Double.NaN;
    public double tHor = Double.NaN;
    public double tVert = Double.NaN;
    public double aspectApproachAngle = Double.NaN; // angle determined by hor/vert
    public double latency = Double.NaN;
    public boolean tValid = false;

    public Values(double initial) {
      tX = tY = tA = matrixDistance = matrixRotationAngle = matrixApproachAngle = tDistance = leftRightRatio = tSkew = tShort = tLong = tHor = tVert = aspectApproachAngle = latency = initial;
      tValid = false;
    }

    public Values(Values other) {
      tX = other.tX;
      tY = other.tY;
      // tA = other.tA;
      // matrixDistance = other.matrixDistance;
      // matrixRotationAngle = other.matrixRotationAngle;
      // matrixApproachAngle = other.matrixApproachAngle;
      tDistance = other.tDistance;
      // leftRightRatio = other.leftRightRatio;
      // tSkew = other.tSkew;
      // tShort = other.tShort;
      // tLong = other.tLong;
      // tHor = other.tHor;
      // tVert = other.tVert;
      // aspectApproachAngle = other.aspectApproachAngle;
      // latency = other.latency;
      tValid = other.tValid;
    }

    public void sum(Values other) {
      tX += other.tX;
      tY += other.tY;
      // tA += other.tA;
      // matrixDistance += other.matrixDistance;
      // matrixRotationAngle += other.matrixRotationAngle;
      // matrixApproachAngle += other.matrixApproachAngle;
      // tDistance += other.tDistance;
      leftRightRatio += other.leftRightRatio;
      // tSkew += other.tSkew;
      // tShort += other.tShort;
      // tLong += other.tLong;
      // tHor += other.tHor;
      // tVert += other.tVert;
      // aspectApproachAngle += other.aspectApproachAngle;
    }

    public void averageOf(double denonminator) {
      tX /= denonminator;
      tY /= denonminator;
      // tA /= denonminator;
      // matrixDistance /= denonminator;
      // matrixRotationAngle /= denonminator;
      // matrixApproachAngle /= denonminator;
      // tDistance /= denonminator;
      // leftRightRatio /= denonminator;
      // tSkew /= denonminator;
      // tShort /= denonminator;
      // tLong /= denonminator;
      // tHor /= denonminator;
      // tVert /= denonminator;
      // aspectApproachAngle /= denonminator;
    }

    /** Project what the next value might be */
    public static double linearFwdInterpolation(double x1, double y1, double x2, double y2, double x3) {
      return y2 + ((x3 - x2) * ((y2 - y1) / (x2 - x1)));
    }

    /** Determine an estimated value in between */
    public static double linearMidInterpolation(double x1, double y1, double x2, double y2, double x3) {
      return y1 + ((x3 - x1) * ((y2 - y1) / (x2 - x1)));
    }

    public Values(Values one, Values two, double ts1, double ts2, double ts3) {
      tX = linearFwdInterpolation(ts1, one.tX, ts2, two.tX, ts3);
      tY = linearFwdInterpolation(ts1, one.tY, ts2, two.tY, ts3);
      // tA = linearFwdInterpolation(ts1, one.tA, ts2, two.tA, ts3);
      // matrixDistance = linearFwdInterpolation(ts1, one.matrixDistance, ts2, two.matrixDistance, ts3);
      // matrixRotationAngle = linearFwdInterpolation(ts1, one.matrixRotationAngle, ts2, two.matrixRotationAngle, ts3);
      // matrixApproachAngle = linearFwdInterpolation(ts1, one.matrixApproachAngle, ts2, two.matrixApproachAngle, ts3);
      tDistance = calculateDistance(tY, targetHeight);
      // leftRightRatio = linearFwdInterpolation(ts1, one.leftRightRatio, ts2, two.leftRightRatio, ts3);
      // tSkew = linearFwdInterpolation(ts1, one.tSkew, ts2, two.tSkew, ts3);
      // tShort = linearFwdInterpolation(ts1, one.tShort, ts2, two.tShort, ts3);
      // tLong = linearFwdInterpolation(ts1, one.tLong, ts2, two.tLong, ts3);
      // tHor = linearFwdInterpolation(ts1, one.tHor, ts2, two.tHor, ts3);
      // tVert = linearFwdInterpolation(ts1, one.tVert, ts2, two.tVert, ts3);
      // aspectApproachAngle = calculateAspectRatio(tHor, tVert);
      latency = 0.0;
      tValid = true;
    }
  }

  private static double[] cornXc;
  private static double[] cornYc;

  public static Values current = new Values(0.0);
  private static Values smoothed = new Values(0.0);

  private static long lastValid = 0;

  public static double LIME_LIGHT_HEIGHT = 36.75; // Measured on Comp Bot
  public static double VISION_PITCH = -3; // Measured on Comp Bot
  public static final double targetHeight = 29;

  // The largest possible ratio from the front
  private static double originalRatio = 2.9; // TODO: figure out the correct ratio when facing directly in front

  // Matrix stuff
  private MatOfPoint3f mObjectPoints;
  private Mat mCameraMatrix;
  private MatOfDouble mDistortionCoefficients;

  // general properties
  public static boolean lightOn = true;
  public static int pipelineNumber = 1;

  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    mObjectPoints = new MatOfPoint3f(
      // first target
      new Point3(-6, 2.91, 0),
      new Point3(-7.38, 2.41, 0),
      new Point3(-5.44, 2.91, 0),
      new Point3(-4.06, -2.41, 0),
      // second target
      new Point3(4.06, -2.41, 0),
      new Point3(5.44, 2.91, 0),
      new Point3(7.38, 2.41, 0),
      new Point3(6, -2.91, 0)
    );

    mCameraMatrix = Mat.eye(3, 3, CvType.CV_64F);
    mCameraMatrix.put(0, 0, 2.5751292067328632e+02);
    mCameraMatrix.put(0, 2, 1.5971077914723165e+02);
    mCameraMatrix.put(1, 1, 2.5635071715912881e+02);
    mCameraMatrix.put(1, 2, 1.1971433393615548e+02);

    mDistortionCoefficients = new MatOfDouble(2.9684613693070039e-01, -1.4380252254747885e+00, -2.2098421479494509e-03,
         -3.3894563533907176e-03, 2.5344430354806740e+00);
//    mDistortionCoefficients = new MatOfDouble(0);
    SmartDashboard.putNumber("Vision Pitch", VISION_PITCH);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void update(long timestamp) {
//    SmartDashboard.putNumber("Timestamp", timestamp);
    current.tValid = table.getEntry("tv").getDouble(0.0) == 0.0 ? false : true;

  if (current.tValid) {
      // get the raw values from the camera
      current.tX = table.getEntry("tx").getDouble(0.0);
      current.tY = table.getEntry("ty").getDouble(0.0);
      // current.tA = table.getEntry("ta").getDouble(0.0);
      current.tDistance = calculateDistance(current.tY, targetHeight);
      // current.tSkew = table.getEntry("ts").getDouble(0.0);
      // current.tShort = table.getEntry("tshort").getDouble(0.0);
      // current.tLong = table.getEntry("tlong").getDouble(0.0);
      // current.tHor = table.getEntry("thor").getDouble(0.0);
      // current.tVert = table.getEntry("tvert").getDouble(0.0);
      // current.latency = table.getEntry("tl").getDouble(0.0);
      // cornXc = table.getEntry("tcornx").getDoubleArray(new double[0]);
      // cornYc = table.getEntry("tcorny").getDoubleArray(new double[0]);

      // current.aspectApproachAngle = calculateAspectRatio(current.tHor, current.tVert);
      // int left = (isLeftOfTarget()? -1 : 0);
      // int right = (isRightOfTarget()? 1 : 0);
      // current.leftRightRatio = left + right;
      //matrixMathOnCorners();

      if (!smoothed.tValid) {
        // If tValids was false, our previous saved position data is also bad (we set to
        // NaN), reset to our first raw values.
        smoothed = new Values(current);
        smoothed.tValid = true;
      } else {
        smoothMultipleValues(5);
      }

      lastValid = timestamp;
    } else {     
      // If target is invalid for more that 0.5 seconds, it is likely off screen
      if (smoothed.tValid && (lastValid + 500) < timestamp) {
        // Don't need to run this code if tValids is already false
        smoothed = new Values(Double.NaN);
      } else if (smoothed.tValid) {
        // We don't currently have valid data, but
        // we can use a projection algorithm
        // Currently, that means not changing the saved values
        // Eventually, we should try to use past pose
        // data to predict current updated values.

        // Linear interpolating the past poses to predict the current value (avoid jitterness)
        List<Pose> lastTwo = Pose.getPreviousPoses(2);
        if (lastTwo.size() == 2) {
          Pose p2 = lastTwo.get(1);
          Pose p1 = lastTwo.get(0);
          if (p2.limeLight.tValid && p1.limeLight.tValid) {
            smoothed = new Values(p1.limeLight, p2.limeLight, p1.timestamp, p2.timestamp, timestamp);
          }
        }
      }
    }

    printValues();
  }

  void printValues() {
    SmartDashboard.putBoolean("Valid Target c", current.tValid);
    SmartDashboard.putBoolean("Valid Target s", smoothed.tValid);
    //    SmartDashboard.putNumber("lastValid TS", lastValid);

    // outputting all current/raw values
    SmartDashboard.putNumber("Target X raw", Double.isNaN(current.tX) ? 0.0 : current.tX);
    SmartDashboard.putNumber("Target Y raw", Double.isNaN(current.tY) ? 0.0 : current.tY);
    // SmartDashboard.putNumber("Target Area raw", Double.isNaN(current.tA) ? 0.0 : current.tA);
    SmartDashboard.putNumber("Target Distance raw", Double.isNaN(current.tDistance) ? 0.0 : current.tDistance);
    // SmartDashboard.putNumber("Target Skew raw", Double.isNaN(current.tSkew) ? 0.0 : current.tSkew);
    // SmartDashboard.putNumber("Target Short raw", Double.isNaN(current.tShort) ? 0.0 : current.tShort);
    // SmartDashboard.putNumber("Target Long raw", Double.isNaN(current.tLong) ? 0.0 : current.tLong);
    // SmartDashboard.putNumber("Target Horizontal raw", Double.isNaN(current.tHor) ? 0.0 : current.tHor);
    // SmartDashboard.putNumber("Target Vertical raw", Double.isNaN(current.tVert) ? 0.0 : current.tVert);
    // SmartDashboard.putNumber("Target Matrix Rotation Angle raw", Double.isNaN(current.matrixRotationAngle) ? 0.0 : current.matrixRotationAngle);
    // SmartDashboard.putNumber("Target Matrix Approach Angle raw ", Double.isNaN(current.matrixApproachAngle) ? 0.0 : current.matrixApproachAngle);
    // SmartDashboard.putNumber("Target Matrix Distance raw", Double.isNaN(current.matrixDistance) ? 0.0 : current.matrixDistance);
    // SmartDashboard.putNumber("Target Left/Right Ratio raw", Double.isNaN(current.leftRightRatio) ? 0.0 : current.leftRightRatio);
    // SmartDashboard.putNumber("Target Aspect Approach Angle raw", Double.isNaN(current.aspectApproachAngle) ? 0.0 : current.aspectApproachAngle);
    // SmartDashboard.putNumber("Limelight Latency raw", Double.isNaN(current.latency) ? 0.0 : current.latency);
    // SmartDashboard.putNumber("Limelight CrossHair-A X", table.getEntry("cx0").getDouble(0));
    // SmartDashboard.putNumber("Limelight CrossHair-A Y", table.getEntry("cy0").getDouble(0));
    // SmartDashboard.putNumber("Limelight CrossHair-B X", table.getEntry("cx1").getDouble(0));
    // SmartDashboard.putNumber("Limelight CrossHair-B Y", table.getEntry("cy1").getDouble(0));
    // outputing all smoothed values
    SmartDashboard.putNumber("Target X smoothed", Double.isNaN(smoothed.tX) ? 0.0 : smoothed.tX);
    SmartDashboard.putNumber("Target Y smoothed", Double.isNaN(smoothed.tY) ? 0.0 : smoothed.tY);
    // SmartDashboard.putNumber("Target Area smoothed", Double.isNaN(smoothed.tA) ? 0.0 : smoothed.tA);
    SmartDashboard.putNumber("Target Distance smoothed", Double.isNaN(smoothed.tDistance) ? 0.0 : smoothed.tDistance);
    // SmartDashboard.putNumber("Target Skew smoothed", Double.isNaN(smoothed.tSkew) ? 0.0 : smoothed.tSkew);
    // SmartDashboard.putBoolean("RightOfTarget", isRightOfTarget());
    // SmartDashboard.putBoolean("LeftOfTarget", isLeftOfTarget());
    // SmartDashboard.putNumber("Target Short smoothed", Double.isNaN(smoothed.tShort) ? 0.0 : smoothed.tShort);
    // SmartDashboard.putNumber("Target Long smoothed", Double.isNaN(smoothed.tLong) ? 0.0 : smoothed.tLong);
    // SmartDashboard.putNumber("Target Horizontal smoothed", Double.isNaN(smoothed.tHor) ? 0.0 : smoothed.tHor);
    // SmartDashboard.putNumber("Target Vertical smoothed", Double.isNaN(smoothed.tVert) ? 0.0 : smoothed.tVert);
    // SmartDashboard.putNumber("Target Matrix Rotation Angle smoothed", Double.isNaN(smoothed.matrixRotationAngle) ? 0.0 : smoothed.matrixRotationAngle);
    // SmartDashboard.putNumber("Target Matrix Approach Angle smoothed ", Double.isNaN(smoothed.matrixApproachAngle) ? 0.0 : smoothed.matrixApproachAngle);
    // SmartDashboard.putNumber("Target Matrix Distance smoothed", Double.isNaN(smoothed.matrixDistance) ? 0.0 : smoothed.matrixDistance);
    // SmartDashboard.putNumber("Target Left/Right Ratio smoothed", Double.isNaN(smoothed.leftRightRatio) ? 0.0 : smoothed.leftRightRatio);
    // SmartDashboard.putNumber("Target Aspect Approach Angle smoothed", Double.isNaN(smoothed.aspectApproachAngle) ? 0.0 : smoothed.aspectApproachAngle);
    // SmartDashboard.putNumber("Limelight Latency smoothed", Double.isNaN(smoothed.latency) ? 0.0 : smoothed.latency);
    // corner stuff
    if (cornXc != null ) {
      for (int i = 0; i < cornXc.length; ++i) {
        SmartDashboard.putNumber("Corner X ["+i+"]", cornXc[i]);
      }
    }
    if (cornYc != null ) {
        for (int i = 0; i < cornYc.length; ++i) {
        SmartDashboard.putNumber("Corner Y ["+i+"]", cornYc[i]);
      }
    }
    double[] camtranValues = table.getEntry("camtran").getDoubleArray(new double[0]);
    if (camtranValues != null && camtranValues.length == 6) {
      SmartDashboard.putNumber("Camtran x", camtranValues[0]);
      SmartDashboard.putNumber("Camtran y", camtranValues[1]);
      SmartDashboard.putNumber("Camtran z", camtranValues[2]);
      SmartDashboard.putNumber("Camtran pitch", camtranValues[3]);
      SmartDashboard.putNumber("Camtran yaw", camtranValues[4]);
      SmartDashboard.putNumber("Camtran roll", camtranValues[5]);
    }
  }

  static double LIMELIGHT_SKEW_CLOCKWISE_MAX = -60;
  static double LIMELIGHT_SKEW_CLOCKWISE_MIN = -90;
  static double LIMELIGHT_SKEW_COUNTERCLOCKWISE_MAX = 0;
  static double LIMELIGHT_SKEW_COUNTERCLOCKWISE_MIN = -35;
  public static boolean isRightOfTarget() {
    double ts = Pose.getCurrentPose().limeLight.tSkew;
    return (ts <= LIMELIGHT_SKEW_CLOCKWISE_MAX &&
      ts >= LIMELIGHT_SKEW_CLOCKWISE_MIN) ;
  }
  
  public static boolean isLeftOfTarget() {
    double ts = Pose.getCurrentPose().limeLight.tSkew;
    return ts <= LIMELIGHT_SKEW_COUNTERCLOCKWISE_MAX &&
      ts >= LIMELIGHT_SKEW_COUNTERCLOCKWISE_MIN;
  }

  private void matrixMathOnCorners() {
    if (!current.tValid || cornXc == null || cornYc == null || cornXc.length != 8 || cornYc.length != 8) {
      return; // Bail out if we can't calculate
    }

    MatOfPoint2f imagePoints = new MatOfPoint2f(
      new Point(cornXc[0], cornYc[0]),
      new Point(cornXc[1], cornYc[1]),
      new Point(cornXc[2], cornYc[2]),
      new Point(cornXc[3], cornYc[3]),
      new Point(cornXc[4], cornYc[4]),
      new Point(cornXc[5], cornYc[5]),
      new Point(cornXc[6], cornYc[6]),
      new Point(cornXc[7], cornYc[7])
    );

    Mat rotationVector = new Mat();
    Mat translationVector = new Mat();
    Calib3d.solvePnP(mObjectPoints, imagePoints, mCameraMatrix, mDistortionCoefficients, rotationVector, translationVector);
    SmartDashboard.putNumber("rotVec0", rotationVector.get(0, 0)[0]);
    SmartDashboard.putNumber("rotVec1", rotationVector.get(1, 0)[0]);
    SmartDashboard.putNumber("rotVec2", rotationVector.get(2, 0)[0]);
    SmartDashboard.putNumber("translationVec0", translationVector.get(0, 0)[0]);
    SmartDashboard.putNumber("translationVec1", translationVector.get(1, 0)[0]);
    SmartDashboard.putNumber("translationVec2", translationVector.get(2, 0)[0]);
    double tvXc = translationVector.get(0, 0)[0];
    double tvYc = translationVector.get(1, 0)[0];
    double tvZc = translationVector.get(2, 0)[0];

    // converting from rotational matrix to vector
    Mat rotationMatrix = new Mat();
    Calib3d.Rodrigues(rotationVector, rotationMatrix);
    Mat projectionMatrix = new Mat(3, 4, CvType.CV_64F);
    projectionMatrix.put(0, 0, rotationMatrix.get(0, 0)[0], rotationMatrix.get(0, 1)[0], rotationMatrix.get(0, 2)[0],
        translationVector.get(0, 0)[0], rotationMatrix.get(1, 0)[0], rotationMatrix.get(1, 1)[0],
        rotationMatrix.get(1, 2)[0], translationVector.get(1, 0)[0], rotationMatrix.get(2, 0)[0],
        rotationMatrix.get(2, 1)[0], rotationMatrix.get(2, 2)[0], translationVector.get(2, 0)[0]);

    Mat cameraMatrix = new Mat();
    Mat rotMatrix = new Mat();
    Mat transVect = new Mat();
    Mat rotMatrixX = new Mat();
    Mat rotMatrixY = new Mat();
    Mat rotMatrixZ = new Mat();
    Mat eulerAngles = new Mat();
    Calib3d.decomposeProjectionMatrix(projectionMatrix, cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY,
        rotMatrixZ, eulerAngles);

    double rollInDegrees = eulerAngles.get(2, 0)[0];
    double pitchInDegrees = eulerAngles.get(0, 0)[0];
    double yawInDegrees = eulerAngles.get(1, 0)[0];
    current.matrixRotationAngle = Math.atan2(tvXc, tvZc);
    current.matrixDistance = Math.sqrt(Math.pow(tvXc, 2) + Math.pow(tvZc, 2));
    current.matrixApproachAngle = yawInDegrees;
  }

  private static double calculateAspectRatio(double tHor, double tVert) {
    // Attempt to find angle between target perpendicular and camera
    // using a horizontal vs vertical ratio
    double currentRatio = tHor / tVert;
    double ratio = Math.min(1, currentRatio / originalRatio); // finding acos of a value > 1 will give NaN
    return Math.toDegrees(Math.acos(ratio));
    //return currentRatio;
  }

  private boolean smoothValues() {
    Pose previousPose = Pose.getCurrentPose();
    if (previousPose.limeLight.tValid && current.tValid) {
      Values acc = new Values(current);
      acc.sum(current);
      acc.averageOf(2.0);
      smoothed = acc;
      return true;
    }
    return false;
  }

  private boolean smoothMultipleValues(int size) {
    if (current.tValid) {
      List<Pose> poseList = Pose.getPreviousPoses(size);
      Values acc = new Values(current);
      double denonminator = 1;
      for (Pose i : poseList) {
        if (i.limeLight.tValid) {
          acc.sum(i.limeLight);
          denonminator++;
        }
      }
      acc.averageOf(denonminator);
      acc.tDistance = calculateDistance(acc.tY, targetHeight);
      smoothed = acc;
      return true;
    }
    return false;
  }

  public double getX() {
    return smoothed.tX;
  }

  public double getY() {
    return smoothed.tY;
  }

  public double getA() {
    return smoothed.tA;
  }

  public double getLatency() {
    return smoothed.latency;
  }

  public boolean isTargetValid() {
    return smoothed.tValid;
  }

  public static double calculateDistance(double tY, double targetHeight) {
    double vPitch = SmartDashboard.getNumber("Vision Pitch", 0.0);
    double yInRadians = Math.toRadians(tY + vPitch);// - RobotMap.direction.getPitch());
    double distance = Math.abs((LIME_LIGHT_HEIGHT - targetHeight) / Math.tan(yInRadians));
    return distance;
  }

  public double getDistance() {
    return smoothed.tDistance;
  }

  public double getSkew() {
    return smoothed.tSkew;
  }

  public double getShort() {
    return smoothed.tShort;
  }

  public double getLong() {
    return smoothed.tLong;
  }

  public double getHor() {
    return smoothed.tHor;
  }

  public double getVert() {
    return smoothed.tVert;
  }

  public double[] getCornX() {
    return cornXc;
  }

  public double[] getCornY() {
    return cornYc;
  }

  public double getMatrixRotationAngle() {
    return smoothed.matrixRotationAngle;
  }

  public double getMatrixApproachAngle() {
    return smoothed.matrixApproachAngle;
  }

  public double getMatrixDistance() {
    return smoothed.matrixDistance;
  }

  public double getLeftRightRatio() {
    return smoothed.leftRightRatio;
  }

  public double getAspectApproachAngle() {
    return smoothed.aspectApproachAngle;
  }

  public static enum LEDMode { PIPELINE, OFF, BLINK, ON }
  public void setLimeLightLEDMode(LEDMode state) {
    table.getEntry("ledMode").setNumber(state.ordinal());
  }

  public void toggleLimelight() {
    lightOn = !lightOn;
    table.getEntry("ledMode").setNumber(lightOn ? 3 : 1);
  }

  public void toggleLimelight(boolean forceOn) {
    lightOn = forceOn;
    table.getEntry("ledMode").setNumber(forceOn ? 3 : 1);
  }

  public static enum CamMode { VISION, DRIVER }

  public void setCamMode(CamMode mode) {
    NetworkTableEntry camMode = table.getEntry("camMode");
    camMode.setNumber(mode.ordinal());
  }

  public void toggleCamMode() {
    NetworkTableEntry camMode = table.getEntry("camMode");
    double current = camMode.getDouble(0);
    camMode.setDouble((0.0 == current) ? 1 : 0);
  }

  public static enum Stream { SIDE_BY_SIDE, PIP_MAIN, PIP_SECONDARY }
  public void setStreaming(Stream stream) {
    NetworkTableEntry streamMode = table.getEntry("stream");
    streamMode.setNumber(stream.ordinal());
  }

  public double getPipeline() {
    return table.getEntry("getpipe").getDouble(0.0);
  }

  // if pipelineNumber = -1, switch to driver's camera
  public void changePipeline(int ppipelineNumber) {
    pipelineNumber = ppipelineNumber;
    if (ppipelineNumber != -1) {
      table.getEntry("camMode").setNumber(0);
      //table.getEntry("stream").setNumber(0);
      table.getEntry("pipeline").setNumber(ppipelineNumber);
    } else {
      table.getEntry("camMode").setNumber(0);
      table.getEntry("stream").setNumber(2);
      return;
    }
  }
}
