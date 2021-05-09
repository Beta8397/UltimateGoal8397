package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.goalbot.GoalBot;
import org.firstinspires.ftc.teamcode.goalbot.OdometryBot;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;
import org.firstinspires.ftc.teamcode.mecbot.MecBotAutonomous;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.Pose;

@Autonomous (name = "TestDiagonal", group = "Test")
public class TestDiagonal extends MecBotAutonomous {

    GoalBot bot = new GoalBot();

    @Override
    public void runLoggingOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        super.setBot(bot);
        bot.setPose(0, 0, 90);
        waitForStart();
        driveToPosition(new MotionProfile(6, 12, 3), 0f, 47.5f, 90, 1);
        Pose pose1 = bot.getPose();
        driveToPosition(new MotionProfile(6, 12, 3), 47.5f, 47.5f, 90, 1);
        Pose pose2 = bot.getPose();

        driveToPosition(new MotionProfile(6, 12, 3), 0f, 0f, 90, 1);
        Pose pose3 = bot.getPose();
        while (opModeIsActive()) {
            telemetry.addData("pose1", "x = %.1f     y = %.1f    th = %.1f", pose1.x, pose1.y, Math.toDegrees(pose1.theta));
            telemetry.addData("pose2", "x = %.1f     y = %.1f    th = %.1f", pose2.x, pose2.y, Math.toDegrees(pose2.theta));
            telemetry.addData("pose3", "x = %.1f     y = %.1f    th = %.1f", pose3.x, pose3.y, Math.toDegrees(pose3.theta));
            telemetry.update();
        }
    }
}
