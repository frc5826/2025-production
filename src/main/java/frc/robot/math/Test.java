package frc.robot.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class Test {

    public static void main(String[] args) {
        Pose2d pose = Constants.BluePositions.coralStationRB;
        System.out.println("starting pose: " + pose);
        System.out.println("calculated pose: " + MathHelper.offsetPose(pose, 0.75, new Rotation2d(-Math.PI / 2)));
    }

}
