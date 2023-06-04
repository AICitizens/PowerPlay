package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(15.75, 14)
                .setConstraints(52, 52, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34.58, -64.28, Math.toRadians(89.57)))
                                .splineTo(new Vector2d(-28.17, -8.66), Math.toRadians(62.18))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-61.23, -12, Math.toRadians(179.14)), Math.toRadians(179.14))
                                .waitSeconds(0)
                                .splineToSplineHeading(new Pose2d(-27.88, -9.25, Math.toRadians(59.66)), Math.toRadians(59.66))
                                .waitSeconds(0)
                                .splineToSplineHeading(new Pose2d(-61.23, -12, Math.toRadians(179.14)), Math.toRadians(179.14))
                                .waitSeconds(0)
                                .splineToSplineHeading(new Pose2d(-27.88, -9.25, Math.toRadians(59.66)), Math.toRadians(59.66))
                                .waitSeconds(0)
                                .splineToSplineHeading(new Pose2d(-61.23, -12, Math.toRadians(179.14)), Math.toRadians(179.14))
                                .waitSeconds(0)
                                .splineToSplineHeading(new Pose2d(-27.88, -9.25, Math.toRadians(59.66)), Math.toRadians(59.66))
                                .waitSeconds(0)
                                .splineToSplineHeading(new Pose2d(-61.23, -12, Math.toRadians(179.14)), Math.toRadians(179.14))
                                .waitSeconds(0)
                                .splineToSplineHeading(new Pose2d(-27.88, -9.25, Math.toRadians(59.66)), Math.toRadians(59.66))
                                .waitSeconds(0)
                                .splineToSplineHeading(new Pose2d(-61.23, -12, Math.toRadians(179.14)), Math.toRadians(179.14))
                                .waitSeconds(0)
                                .splineToSplineHeading(new Pose2d(-27.88, -9.25, Math.toRadians(59.66)), Math.toRadians(59.66))
                                .waitSeconds(0)
                                .setReversed(false)
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}