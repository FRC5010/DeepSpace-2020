/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * This class contains sets of values that coorespond to various locations on
 * the field
 */
public class FieldMap {

    // These are for the openCV solvePnP function
    private static double[] camera_matrix = { 2.5751292067328632e+02, 0., 1.5971077914723165e+02, 0.,
            2.5635071715912881e+02, 1.1971433393615548e+02, 0., 0., 1. };
    private static double[] distortion_coefficients = { 2.9684613693070039e-01, -1.4380252254747885e+00,
            -2.2098421479494509e-03, -3.3894563533907176e-03, 2.5344430354806740e+00 };
    private static Calib3d openCV = new Calib3d();
    private static Mat rVec;
    private static Mat tVec;

    // TODO: Add an enum for each field target
    public static enum Target {
        // These need to be the same as the labels in the CSV file
        OUR_LEFT_ROCKET_NEAR_HATCH, OUR_LEFT_ROCKET_CARGO, OUR_LEFT_ROCKET_FAR_HATCH,

        OUR_LEFT_SHIP_BAY_1, OUR_LEFT_SHIP_BAY_2,

        OUR_LEFT_LOADING_STATION,

        THEIR_LEFT_ROCKET_NEAR_HATCH, THEIR_LEFT_ROCKET_CARGO, THEIR_LEFT_ROCKET_FAR_HATCH,

        THEIR_RIGHT_ROCKET_NEAR_HATCH,

        THEIR_LEFT_SHIP_BAY_1, THEIR_LEFT_SHIP_BAY_2
    }

    public static class Position {
        public final MatOfPoint3f allPoints;

        public Position(MatOfPoint3f allPoints) {
            this.allPoints = allPoints;
        }

        public Position setPoints(List<Point3> points) {
            allPoints.fromList(points);
            return this;
        }

        public double distanceFromPosition(Position pos2) {
            // TODO: Calculate the distance between this position and pos2
            return 0.0;
        }

        public Point3 findCenterPoint() {
            // TODO: Calculate center point
            return new Point3();
        }
    }

    private static Map<Target, Position> fieldMap;

    static {
        File fieldMapFile = new File(Filesystem.getDeployDirectory().toPath() + "/fieldMap.csv");
        String data;
        String[] targetData;
        Target target;
        double lowX, lowY, lowZ, highX, highY, highZ;
        List<Point3> points;
        try {
            BufferedReader reader = new BufferedReader(new FileReader(fieldMapFile));
            while (reader.ready()) {
                data = reader.readLine();
                targetData = data.split(",");
                target = Target.valueOf(targetData[0]);
                lowX = Double.valueOf(targetData[1]);
                highX = Double.valueOf(targetData[2]);
                lowY = Double.valueOf(targetData[3]);
                highY = Double.valueOf(targetData[4]);
                lowZ = Double.valueOf(targetData[5]);
                highZ = Double.valueOf(targetData[6]);
                fieldMap = new HashMap<>();
                points = new ArrayList<Point3>();
                points.add(new Point3(lowX, lowY, highZ)); // Top left
                points.add(new Point3(highX, highY, highZ)); // Top right
                points.add(new Point3(highX, highY, lowZ)); // Bottom right
                points.add(new Point3(lowX, lowY, lowZ)); // Bottom left
                fieldMap.put(target, new Position(new MatOfPoint3f()).setPoints(points));
                points.clear();
            }
        } catch (FileNotFoundException e) {
        } catch (IOException e) {
        }
    }

    public static Position getPosition(Target trg) {
        return fieldMap.get(trg);
    }

    public static Target getNearestTarget(Position pos) {
        // TODO: Use the distanceFromPosition code to find the target
        // nearest the passed in position
        Collection<Position> fieldMapPosistions = fieldMap.values();
        Target nearestTarget = Target.OUR_LEFT_ROCKET_CARGO;
        double lastNearestDistance = Double.NaN;
        for (Position nextPos : fieldMapPosistions) {

        }
        return nearestTarget;
    }
}
