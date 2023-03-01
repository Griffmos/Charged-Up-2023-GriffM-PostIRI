package frc.robot.interfaces;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BetterLimelightInterface {
  // declare for easy calls
  private static BetterLimelightInterface limelightInterface;

  // network table declarations
  private static NetworkTable leftLimelight = NetworkTableInstance.getDefault().getTable("limelight-left");
  private static NetworkTable rightLimelight = NetworkTableInstance.getDefault().getTable("limelight-right");


  private double aprilTagID = 0;
  private boolean hasValidTarget = false;


  ArrayList<AprilTag> aprilTagList = new ArrayList<AprilTag>();
  public AprilTagFieldLayout aprilTagFieldLayout; 

  public BetterLimelightInterface() {
    fillAprilTagList();
    aprilTagFieldLayout = new AprilTagFieldLayout(aprilTagList, 16.54175, 8.0137);
  }


  /*
   * A singleton that creates the sole instance of BetterLimelightInterface,
   * this is also used in other classes as well, so we don't have to pass in
   * new instances all the time
   */
  public static synchronized BetterLimelightInterface getInstance() {
    if (limelightInterface == null) {
      limelightInterface = new BetterLimelightInterface();
    }
    return limelightInterface;
  }

  // an emun to keep track
  public enum SpecificLimelight {
    LEFT_LIMELIGHT,
    RIGHT_LIMELIGHT
  }

  public double getDoubleEntry(String entry, SpecificLimelight limelight) {
    if (limelight == SpecificLimelight.LEFT_LIMELIGHT) {
      return leftLimelight.getEntry(entry).getDouble(0);
    } else {
      return rightLimelight.getEntry(entry).getDouble(0);
    }
  }

  public double[] getArrayEntry(String entry, SpecificLimelight limelight) {
    if (limelight == SpecificLimelight.LEFT_LIMELIGHT) {
      return leftLimelight.getEntry(entry).getDoubleArray(new double[6]);
    } else {
      return rightLimelight.getEntry(entry).getDoubleArray(new double[6]);
    }
  }

  /*
   * the only setting we really need to do on the limelight
   * is the pipeline (if we decide to vision track)
   */
  public void setPipeline(int pipeline, SpecificLimelight limelight) {
    if (limelight == SpecificLimelight.LEFT_LIMELIGHT) {
      leftLimelight.getEntry("pipeline").setNumber(pipeline);
    } else {
      rightLimelight.getEntry("pipeline").setNumber(pipeline);
    }
  }

  public boolean hasValidTargets(SpecificLimelight limelight) {
    hasValidTarget = getDoubleEntry("tv", limelight) == 1.0;
    return hasValidTarget;
  }

  public double getTargetArea(SpecificLimelight limelight) {
    return getDoubleEntry("ta", limelight);
  }

  /*
   * Specific AprilTag methods we will need
   */

  /*
   * This method returns the pose of the robot, a combination of translational
   * and rotational offset relative to the april tag
   */
  public double[] getBotPose(SpecificLimelight limelight) {
    return getArrayEntry("botpose_targetspace", limelight);
  }

  public double getID(SpecificLimelight limelight) {
    aprilTagID = getDoubleEntry("tid", limelight);
    return aprilTagID;
  }

  public boolean hasScoringTarget() {
    return (((aprilTagID == 1) || (aprilTagID == 2) || (aprilTagID == 3)
        || (aprilTagID == 6) || (aprilTagID == 7) || (aprilTagID == 8)) && hasValidTarget);
  }


  /*
   * create a Pose3D object for trajectory generation
   */
  public Pose3d getRobotPose3d(SpecificLimelight limelight) {
    double[] result = getBotPose(limelight);
    Translation3d tran3d = new Translation3d(result[0], result[1], result[2]);
    Rotation3d r3d = new Rotation3d(result[3], result[4], result[5]);
    Pose3d p3d = new Pose3d(tran3d, r3d);

    return p3d;
  }

  public Pose3d getAprilTagPose(SpecificLimelight limelight){
    if(hasValidTargets(limelight)) {
      Optional<Pose3d> aprilTagPose = aprilTagFieldLayout.getTagPose((int)getID(limelight));
      if(aprilTagPose.isEmpty()) {
        return new Pose3d();
      }
      return aprilTagPose.get();
    }
    return new Pose3d();
  }

  public void fillAprilTagList(){
    aprilTagList.add(new AprilTag(1, new Pose3d(15.513558, 1.071626, 0.462788, new Rotation3d(new Quaternion(0, 0, 0, 1)))));
    aprilTagList.add(new AprilTag(2, new Pose3d(15.513558, 2.748026, 0.462788, new Rotation3d(new Quaternion(0, 0, 0, 1)))));
    aprilTagList.add(new AprilTag(3, new Pose3d(15.513558, 4.424426, 0.462788, new Rotation3d(new Quaternion(0, 0, 0, 1)))));
    aprilTagList.add(new AprilTag(4, new Pose3d(16.178784, 6.749796, 0.695452, new Rotation3d(new Quaternion(0, 0, 0, 1)))));
    aprilTagList.add(new AprilTag(5, new Pose3d(0.36195, 6.749796, 0.695452, new Rotation3d(new Quaternion(1, 0, 0, 0)))));
    aprilTagList.add(new AprilTag(6, new Pose3d(1.02743, 4.424426, 0.462788, new Rotation3d(new Quaternion(1, 0, 0, 0)))));
    aprilTagList.add(new AprilTag(7, new Pose3d(1.02743, 2.748026, 0.462788, new Rotation3d(new Quaternion(1, 0, 0, 0)))));
    aprilTagList.add(new AprilTag(8, new Pose3d(1.02743, 1.071626, 0.462788, new Rotation3d(new Quaternion(1, 0, 0, 0)))));
}

  

}
