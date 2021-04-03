package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
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

            Orientation armOrientation = bot.getArmOrientation();
            telemetry.addData("arm", "1: %.1f  2: %.1f  3: %.1f", armOrientation.firstAngle,
                    armOrientation.secondAngle, armOrientation.thirdAngle);

            Quaternion q = bot.getArmQuaternion();
            telemetry.addData("quat", "w: %.2f x: %.2f y: %.2f z: %.2f", q.w, q.x, q.y, q.z);
            float mag = q.magnitude();
            telemetry.addData("qautMAG", mag);

            float zDOT = 1 - 2 * mag * (q.y * q.y + q.x * q.x);
            float xDOT = 2 * mag * (q.x * q.z - q.y * q.w);
            telemetry.addData("armAxes", "zDOT %.3f  xDOT %.3f", zDOT, xDOT);

            float armPose = bot.getArmPosition();
            telemetry.addData("armPose", armPose);

            float px = gamepad1.left_stick_x * 0.5f;
            float py = -gamepad1.left_stick_y * 0.5f;
            float pa = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.5f;
            telemetry.update();

            bot.setDrivePower(px, py, pa);

        }
    }
}
