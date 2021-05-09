package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.goalbot.OdometryBot;
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.logging.LoggingLinearOpMode;

@TeleOp (name ="Test Odom", group = "test")

public class TestOdom extends LoggingLinearOpMode {

    OdometryBot bot = new OdometryBot();

    @Override
    public void runLoggingOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        gamepad1.setJoystickDeadzone(0.05f);
        waitForStart();

        bot.setPose(0,0,90);

        float armTarget = 0;

        while (opModeIsActive()){

            bot.updateOdometry();
            telemetry.addData("ticks", "L %d R %d H %d", bot.leftTicks, bot.rightTicks, bot.horizTicks);
            telemetry.addData("pose", "x %.1f y %.1f t %.1f", bot.getPose().x, bot.getPose().y, Math.toDegrees(bot.getPose().theta));

            Orientation armOrientation = bot.getArmOrientation();
            telemetry.addData("arm", "1: %.1f  2: %.1f  3: %.1f", armOrientation.firstAngle,
                    armOrientation.secondAngle, armOrientation.thirdAngle);



//            float armPose = bot.getArmPosition();
//            telemetry.addData("armPose", armPose);
//
//            if(gamepad1.a){
//                bot.armControl.update();
//                telemetry.addData("arm updated", "");
//            } else if(gamepad1.dpad_up){
//                bot.armMotor.setPower(0.4);
//                BetaLog.dd("dpad_up", "arm pos = %.1f", armPose);
//            } else if(gamepad1.dpad_down) {
//                bot.armMotor.setPower(-0.4);
//                BetaLog.dd("dpad_down", "arm pos = %.1f", armPose);
//
//            }
//            else {
//                bot.armMotor.setPower(0);
//                telemetry.addData("arm zero", "");
//            }
//            if(gamepad1.x){
//                armTarget = (float)Math.max(armTarget-0.25, 0);
//            } else if(gamepad1.b){
//                armTarget = (float)Math.min(armTarget+0.25,200);
//            }
//            bot.setArmPosition(armTarget);

//            telemetry.addData("arm target",armTarget);
//            telemetry.addData("arm power",bot.armMotor.getPower());

            float px = gamepad1.left_stick_x * 0.5f;
            float py = -gamepad1.left_stick_y * 0.5f;
            float pa = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.5f;
            telemetry.update();


            bot.setDrivePower(px, py, pa);

        }
    }
}
