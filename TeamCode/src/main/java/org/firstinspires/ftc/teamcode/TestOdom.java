package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.goalbot.OdometryBot;

@TeleOp (name ="Test Odom", group = "test")

public class TestOdom extends LinearOpMode {

    OdometryBot bot = new OdometryBot();

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        gamepad1.setJoystickDeadzone(0.05f);
        waitForStart();

        bot.setPose(0,0,90);

        while (opModeIsActive()){

            bot.updateOdometry();
            telemetry.addData("ticks", "L %d R %d H %d", bot.leftTicks, bot.rightTicks, bot.horizTicks);
            telemetry.addData("pose", "x %.1f y %.1f t %.1f", bot.getPose().x, bot.getPose().y, Math.toDegrees(bot.getPose().theta));

            float px = gamepad1.left_stick_x * 0.5f;
            float py = -gamepad1.left_stick_y * 0.5f;
            float pa = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.5f;
            telemetry.update();

            bot.setDrivePower(px, py, pa);

        }
    }
}
