package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.goalbot.OdometryBot;
import org.firstinspires.ftc.teamcode.mecbot.MecBotAutonomous;
import org.firstinspires.ftc.teamcode.util.CubicSpline2D;
import org.firstinspires.ftc.teamcode.util.MotionProfile;

@Autonomous (name = "Drive Spline", group = "test")

public class DriveSpline extends MecBotAutonomous {

    OdometryBot bot = new OdometryBot();
    CubicSpline2D spline = new CubicSpline2D(new float[]{24,24,60,72,120,24}, 0, -90);

    @Override
    public void runLoggingOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        super.setBot(bot);

        waitForStart();

        bot.setPose(24,24, 0);

        driveSpline(new MotionProfile(8,20,6), 0.8f, -90, 12, 2, spline);
        bot.setDrivePower(0,0,0);

    }
}
