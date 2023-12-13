// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

public class LimelightSubsystem extends SubsystemBase {

  private NetworkTable m_limelightTable;

  private final Field2d m_field = new Field2d();

  /** Creates a new ExampleSubsystem. */
  public LimelightSubsystem() {

    SmartDashboard.putData("Field", m_field);

    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  }

  public void getNetworkTableValues(){
    

  }

  public Pose2d getBotPose(){

    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    double[] botPoseArray = m_limelightTable.getEntry("botpose_wpired").getDoubleArray(new double[10]); 

    Pose2d botPose = new Pose2d(botPoseArray[0], botPoseArray[1], Rotation2d.fromDegrees(botPoseArray[5]));

    m_field.setRobotPose(botPose);
    SmartDashboard.putString("Bot pose string thing ", botPose.toString());
   
    return botPose;
  }

  public double getCurrentAprilTagId(){

    double id = m_limelightTable.getEntry("tid").getDouble(0);

    return id;

  }

  public Pose2d getAprilTagDistanceRobotSpace(){

    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    double[] botPoseArray = m_limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[10]); 

    Pose2d botPose = new Pose2d(botPoseArray[0], botPoseArray[1], Rotation2d.fromDegrees(botPoseArray[5]));

    m_field.setRobotPose(botPose);
   
    return botPose;
  }

  public double getTargetRotation(){

    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight-reflect");
    // double[] botPoseArray = m_limelightTable.getEntry("tx").getDoubleArray(new double[10]); 

    // double targetRotation = botPoseArray[5];
   
    return m_limelightTable.getEntry("tx").getDouble(0);

  }

  public double getPoseDiff(){

    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight-reflect");
    double Yaw = m_limelightTable.getEntry("botpose_wpired").getDoubleArray(new double[10])[5]; 
    return Yaw;

  } 


  public double getTargetDegree(){


    double xDistance = TurretConstants.targetPose.getX() - getBotPose().getX();
    double yDistance = TurretConstants.targetPose.getY() - getBotPose().getY();
    
    double optimalDegree = Math.atan(yDistance/xDistance); 
    return optimalDegree;
  

  }

  public boolean getTagWithinSight(){
    if (getCurrentAprilTagId() == -1){
      return false;
    }
    else if (getPoseDiff() <= (getTargetDegree() + 10) && getPoseDiff() >= (getTargetDegree() - 10)){
      return true;
    }
    return false;
  }
  

  @Override
  public void periodic() {

    getBotPose();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
