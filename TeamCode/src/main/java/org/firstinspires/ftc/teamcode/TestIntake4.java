package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test Intake 4", group = "Test")
public class TestIntake4 extends LinearOpMode {

    public DcMotor motor;

    public void runOpMode() {
        gamepad1.setJoystickDeadzone(0.5f);
        motor = hardwareMap.get(DcMotor.class, "intake_motor");
        motor.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(gamepad1.left_stick_y);
            int motorPosition = motor.getCurrentPosition();
            telemetry.addData("Position", motorPosition);
            telemetry.update();

        }
    }
}
