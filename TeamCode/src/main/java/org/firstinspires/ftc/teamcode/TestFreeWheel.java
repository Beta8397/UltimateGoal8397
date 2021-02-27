package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;

@TeleOp (name = "Test Free Wheel", group = "Test")
public class TestFreeWheel extends LinearOpMode {

    MecBot bot = new MecBot(MecBot.MotorType.NeverestOrbital20, 13.25f, 13, 4.0f, 40.9f, 1, BNO055Enhanced.AxesMap.XZY,
            BNO055Enhanced.AxesSign.NPP);

    DcMotorEx encoder1 = null;
    DcMotorEx encoder2 = null;
    DcMotorEx encoder3 = null;

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        encoder1 = hardwareMap.get(DcMotorEx.class, "intake front");
        encoder2 = hardwareMap.get(DcMotorEx.class, "intake back");
        encoder3 = hardwareMap.get(DcMotorEx.class, "arm_motor");
        encoder1.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder2.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder3.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            float leftY = gamepad1.left_stick_y;
            float rightX = gamepad1.right_stick_x;
            float pForward = 0;
            float pTurn = 0;
            if (Math.abs(leftY) > 0.05) {
                pForward = -leftY / 4;
            } else if (Math.abs(rightX) > 0.05) {
                pTurn = -rightX / 4;
            }
            bot.setDrivePower(0, pForward, pTurn);
            telemetry.addData("encoder1", encoder1.getCurrentPosition());
            telemetry.addData("encoder2", encoder2.getCurrentPosition());
            telemetry.addData("encoder3", encoder3.getCurrentPosition());
            telemetry.update();
        }
    }
}
