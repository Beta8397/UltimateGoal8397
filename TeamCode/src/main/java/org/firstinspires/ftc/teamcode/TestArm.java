package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.goalbot.GoalBot;

@TeleOp (name = "test arm", group = "test")
public class TestArm extends LinearOpMode {

    GoalBot bot = new GoalBot();

    public void runOpMode() {
        bot.init(hardwareMap);
//        bot.setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
        float target = 0;
//        bot.setArmPosition(0);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.y) {
                target++;
            } else if(gamepad1.a) {
                target--;
            }
            if (target > 280) {
                target = 280;
            } else if (target < 0) {
                target = 0;
            }
//            bot.setArmPosition((int) target);
            telemetry.addData("target =", target);
            telemetry.addData("ticks =", bot.armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
