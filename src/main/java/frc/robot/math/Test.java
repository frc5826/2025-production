package frc.robot.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

import static frc.robot.Constants.BluePositions.cPipeApart;
import static frc.robot.Constants.BluePositions.cRobotLength;

public class Test {

    public static void main(String[] args) {
//        Pose2d goalPose = MathHelper.offsetPoseReverse(Constants.BluePositions.reefSideAB, cRobotLength / 2);
//
//        double diff = MathHelper.getAlignYDiff(goalPose, Constants.BluePositions.reefA);
//        System.out.println(diff);
//        System.out.println(cPipeApart / 2);

        Pose2d offsetReefPose = MathHelper.offsetPoseReverse(Constants.BluePositions.reefSideAB, 0.6 + (cRobotLength / 2));
        Pose2d againstReefPose = MathHelper.offsetPoseReverse(Constants.BluePositions.reefSideAB, (cRobotLength / 2));

        System.out.println("offset: " + offsetReefPose);
        System.out.println("against: " + againstReefPose);
    }

}
