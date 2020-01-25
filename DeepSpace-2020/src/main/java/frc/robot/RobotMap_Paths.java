/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;

//import com.ctre.phoenix.motion.TrajectoryPoint;

//import java.nio.file.DirectoryStream;

import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

/**
 * Add your docs here.
 */
public class RobotMap_Paths {
     public static final double max_velocity = 17.89;
     public static final double wheel_diameter = 0.5;
     public static Map<MotionProfiles, Trajectory> leftTrajectories;
     public static Map<MotionProfiles, Trajectory> rightTrajectories;
     public static String deployPath; 

     public static enum MotionProfiles {
          // Middle Start
          MStoShip1L, MStoShip1S, MStoShip1R, HAB1,
          // Left Rocket
          LStoLRkt1, //backupLRkt1, LRkt1toLP, LPtoLRkt2, finalLRkt2,
          // Left Start Side X2 
          LStoShip2L, //backupShip2L, Ship2LtoLP, LPtoShip3L, finalShip3L,
          // Left Start Front & Side
          LStoShip1L, backupShip1L, // Ship1LtoLP, //LPtoShip2L, finalShip2L,
          // Right Rocket
          RStoRRkt1, //backupRRkt1, RRkt1toRP, RPtoRRkt2, finalRRkt2,
          // Right Start Front & Side
          RStoShip1R, backupShip1R, // Ship1RtoRP, //RPtoShip2R, finalShip2R,
          // Right Start Side X2
          RStoShip2R, backupShip2R, //Ship2RtoRP, RPtoShip3R, finalShip3R,
          // Test routines
          testPath,
     }

     private static boolean errorLoadingPaths = false;

     public static void init() {
          leftTrajectories = new HashMap<>();
          rightTrajectories = new HashMap<>();

          for(MotionProfiles mp : MotionProfiles.values()) {
               loadTrajectories(mp);
          }
     }

     private static void loadTrajectories(MotionProfiles basePathName) {
          String leftPath = deployPath + basePathName + ".left.pf1.csv";
          String rightPath = deployPath + basePathName + ".right.pf1.csv";
          File leftFile = new File(leftPath);
          File rightFile = new File(rightPath);
          if (!errorLoadingPaths) {
               SmartDashboard.putString("Motion Profile Path Status", "YEET!");
          }
          try {
               leftTrajectories.put(basePathName, Pathfinder.readFromCSV(leftFile));
               System.out.println(leftPath + ": " + leftTrajectories.get(basePathName).length());
               rightTrajectories.put(basePathName, Pathfinder.readFromCSV(rightFile));
               System.out.println(rightPath + ": " + rightTrajectories.get(basePathName).length());
          } catch (IOException e) {
               errorLoadingPaths = true;
               System.err.println("******************* AAAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHH!!!!!!!!!!!!! *****************");
               System.err.println("***   " + e.getMessage());
               System.err.println("******************* AAAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHH!!!!!!!!!!!!! *****************");
               SmartDashboard.putString("Motion Profile Path Status", e.getMessage());
          }
     }
}
