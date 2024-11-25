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

        RoadRunnerBotEntity leftRedSimple = new DefaultBotBuilder(meepMeep)
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
                .splineToConstantHeading(new Vector2d(36,-24),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46,-12),Math.toRadians(270))
                .strafeTo(new Vector2d(46,-50))
                .waitSeconds(0.1)

                .splineToConstantHeading(new Vector2d(46,-20),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(56,-12),Math.toRadians(270))
                .strafeTo(new Vector2d(56,-50))
                .waitSeconds(0.1)

                .splineToConstantHeading(new Vector2d(56,-20),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(61,-12),Math.toRadians(270))
                .strafeTo(new Vector2d(61,-50))
                .waitSeconds(0.1)

                .strafeTo(new Vector2d(55,-40))
                .waitSeconds(19)
                .strafeTo(new Vector2d(55,-60))



//                .strafeToConstantHeading(new Vector2d(48,-12))
                .build());

        leftRedSimple.runAction(leftRedSimple.getDrive().actionBuilder(new Pose2d(-36, -60, Math.toRadians(0)))
                .strafeTo(new Vector2d(-36, -55))
                .strafeTo(new Vector2d(-54, -55)) //move to basket

                .turn(Math.toRadians(45)) //drop off into basket
                .waitSeconds(2)
                .turn(Math.toRadians(-45))

                .strafeTo(new Vector2d(-36, -55)) // go to sample
                .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(-36, -26))
                .waitSeconds(2)
//
                .strafeTo(new Vector2d(-36, -55)) // move to basket
                .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(-54, -55))

                .turn(Math.toRadians(45)) //drop off into basket
                .waitSeconds(2)
                .turn(Math.toRadians(-45))

                // third sample

                .strafeTo(new Vector2d(-48, -55)) // go to sample
                .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(-48, -26))
                .waitSeconds(2)
//
                .strafeTo(new Vector2d(-48, -48))

                .turn(Math.toRadians(-135)) //drop off into basket
                .strafeTo(new Vector2d(-55, -55))
                .waitSeconds(2)

                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(leftRedSimple)
        .start();
                //.start();

    }
}