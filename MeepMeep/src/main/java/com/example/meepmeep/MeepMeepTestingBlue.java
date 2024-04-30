package com.example.meepmeep;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTestingBlue {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800,60);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 1408.3621921992083)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(11.8, 61.7, Math.toRadians(90)))



                .strafeToConstantHeading(new Vector2d(23.37, -38.95))
                .strafeToConstantHeading(new Vector2d(49, -42.01))


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}