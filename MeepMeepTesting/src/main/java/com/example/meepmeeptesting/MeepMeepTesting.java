package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity leftRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        RoadRunnerBotEntity rightRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        RoadRunnerBotEntity rightRedSad = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        leftRed.runAction(leftRed.getDrive().actionBuilder(new Pose2d(-36, -58, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .waitSeconds(2) //drop off sample

                .strafeToLinearHeading(new Vector2d(-34, -26), Math.toRadians(180))
                .waitSeconds(2) //pick up sample

                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .waitSeconds(2) //drop off sample

                .strafeToLinearHeading(new Vector2d(-43, -26), Math.toRadians(180))
                .waitSeconds(2) //pick up sample

                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .waitSeconds(2) //drop off sample

                .strafeToLinearHeading(new Vector2d(-52, -26), Math.toRadians(180))
                .waitSeconds(2) //pick up sample

                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .waitSeconds(2) //drop off sample

                .splineToLinearHeading(new Pose2d(-27, 12, Math.toRadians(180)),Math.toRadians(0))
                .build());


        rightRed.runAction(rightRed.getDrive().actionBuilder(new Pose2d(12, -58, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(0, -34, Math.toRadians(90)),Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(225))
                .waitSeconds(2) //drop off sample

                .strafeToLinearHeading(new Vector2d(-34, -26), Math.toRadians(180))
                .waitSeconds(2) //pick up sample

                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .waitSeconds(2) //drop off sample

                .strafeToLinearHeading(new Vector2d(-43, -26), Math.toRadians(180))
                .waitSeconds(2) //pick up sample

                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .waitSeconds(2) //drop off sample

                .strafeToLinearHeading(new Vector2d(-52, -26), Math.toRadians(180))
                .waitSeconds(2) //pick up sample

                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .waitSeconds(2) //drop off sample

                .splineToLinearHeading(new Pose2d(-27, 12, Math.toRadians(180)),Math.toRadians(0))
                .build());

        rightRedSad.runAction(rightRedSad.getDrive().actionBuilder(new Pose2d(12, -58, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(48,-12))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(rightRedSad)
        .start();
                //.start();

    }
}