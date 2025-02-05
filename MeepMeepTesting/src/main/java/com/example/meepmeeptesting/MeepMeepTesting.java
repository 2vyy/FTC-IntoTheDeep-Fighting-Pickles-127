package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {



    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        Pose2d initialPose = new Pose2d(5, -61, Math.toRadians(90));
        Pose2d centerBarPose = new Pose2d(5, -33, Math.toRadians(90));
        Pose2d centerBarSecondPose = new Pose2d(30, -34, Math.toRadians(90));
        Vector2d innerSplinePose = new Vector2d(35, -20);
        Vector2d innerSplineSecondPose = new Vector2d(45.5, -15);
        Vector2d centerSplinePose = new Vector2d(56, -20);
        Vector2d outerSplinePose = new Vector2d(62, -20);


        Pose2d innerSamplePose = new Pose2d(45.5, -12, Math.toRadians(90));
        Pose2d centerSamplePose = new Pose2d(56, -12, Math.toRadians(90));

        Pose2d innerSamplePushPose = new Pose2d(45.5, -55, Math.toRadians(90));
        Pose2d centerSamplePushPose = new Pose2d(56, -55, Math.toRadians(90));
        Pose2d outerSamplePushPose = new Pose2d(62, -55, Math.toRadians(90));

        Pose2d waitPose = new Pose2d(55, -40, Math.toRadians(90));
        Pose2d parkPose = new Pose2d(55, -61, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

/*
        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .strafeToLinearHeading(centerBarPose.position, centerBarPose.heading)
                .strafeToLinearHeading(centerBarSecondPose.position, centerBarSecondPose.heading)

                .splineToConstantHeading(innerSplinePose, Math.toRadians(90))
                .splineToConstantHeading(innerSplineSecondPose, Math.toRadians(0))

                .strafeToLinearHeading(innerSamplePushPose.position, innerSamplePushPose.heading)
                .strafeToLinearHeading(innerSamplePose.position, innerSamplePose.heading)
                .splineToConstantHeading(centerSplinePose, Math.toRadians(270))

                .strafeToLinearHeading(centerSamplePushPose.position, centerSamplePushPose.heading)
                .strafeToLinearHeading(centerSamplePose.position, centerSamplePose.heading)
                .splineToConstantHeading(outerSplinePose, Math.toRadians(270))

                .strafeToLinearHeading(outerSamplePushPose.position, outerSamplePushPose.heading)

                .strafeToLinearHeading(waitPose.position, waitPose.heading)
                .strafeToLinearHeading(parkPose.position, parkPose.heading)

                .build());
*/
        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                //Start with Specimen In Arm Hand
                .strafeToLinearHeading(centerBarPose.position, centerBarPose.heading)
                //Place Specimen
                .strafeTo(new Vector2d(5,-42))
                .strafeTo(new Vector2d(35,-42))

                //.strafeToLinearHeading(centerBarSecondPose.position, centerBarSecondPose.heading)

                .splineToConstantHeading(innerSplinePose, Math.toRadians(90))
                .splineToConstantHeading(innerSplineSecondPose, Math.toRadians(0))

                .strafeToLinearHeading(innerSamplePushPose.position, innerSamplePushPose.heading)
                .strafeToLinearHeading(innerSamplePose.position, innerSamplePose.heading)
                .splineToConstantHeading(centerSplinePose, Math.toRadians(270))

                .strafeToLinearHeading(centerSamplePushPose.position, centerSamplePushPose.heading)
                .strafeToLinearHeading(centerSamplePose.position, centerSamplePose.heading)
                .splineToConstantHeading(outerSplinePose, Math.toRadians(270))

                .strafeToLinearHeading(outerSamplePushPose.position, outerSamplePushPose.heading)

                .strafeToLinearHeading(waitPose.position, waitPose.heading)
                .strafeToLinearHeading(parkPose.position, parkPose.heading)

                .strafeTo(new Vector2d(-4,-34))
                //place Specimen
                .strafeToLinearHeading(parkPose.position, parkPose.heading)
                //pickup Specimen

                .strafeTo(new Vector2d(4,-34))
                //place Specimen
                .strafeToLinearHeading(parkPose.position, parkPose.heading)
                //pickup Specimen

                .strafeTo(new Vector2d(8,-34))
                //place Specimen
                .strafeToLinearHeading(parkPose.position, parkPose.heading)
                //pickup Specimen

                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}