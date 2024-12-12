package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-12.42, -61.11, 90))
                        .splineTo(new Vector2d(-42.72, -15.47),Math.toRadians(-90))
                        .splineTo(new Vector2d(-59.2, -54.1), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-30.32, -17.37),Math.toRadians(-90))
                        .splineTo(new Vector2d(-52.9, -10.3), Math.toRadians(-90))
                        .splineTo(new Vector2d(-57.01, -58.47), Math.toRadians(-90))
//                        .splineToConstantHeading(new Vector2d(-52.9, -14.3), Math.toRadians(-90))
//                        .splineTo(new Vector2d(-53, -16.81), Math.toRadians(180))
//                     .splineTo(new Vector2d(-60.41, -58.06), Math.toRadians(-90))
//                        .back(20)
//                      .lineTo(new Vector2d(58, -59.25))

//
////
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}