package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by FTC Team 8397 on 11/21/2019.
 *
 * Made to test problems because one set of phones inconsistently recognized the Gamepads,
 * and I didn't want to bother with having to connect to the robot every time.
 */
@Disabled
@TeleOp(name="Test Gamepad", group="Test")
public class TestGamepad extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Gamepad 1", "");
            telemetry.addData("Stick", "L: %3f, %3f  R: %3f, %3f", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("Trigger/Bumper", "LT: %3f  RT: %3f  LB: %1b  RB: %1b", gamepad1.left_trigger, gamepad1.right_trigger, gamepad1.left_bumper, gamepad1.right_bumper);
            telemetry.addData("D-pad", "U: %1b  D: %1b  L: %1b  R: %1b", gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right);
            telemetry.addData("Button", "A: %1b  B: %1b  X: %1b  Y: %1b", gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y);
            // Beeg == Guide because I don't know if people know what the guide button is
            telemetry.addData("Other", "Start: %1b  Back: %1b  Beeg: %1b", gamepad1.start, gamepad1.back, gamepad1.guide);
            
            telemetry.addData("Gamepad 2", "");
            telemetry.addData("Stick", "L: %3f, %3f  R: %3f, %3f", gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_x, gamepad2.right_stick_y);
            telemetry.addData("Trigger/Bumper", "LT: %3f  RT: %3f  LB: %1b  RB: %1b", gamepad2.left_trigger, gamepad2.right_trigger, gamepad2.left_bumper, gamepad2.right_bumper);
            telemetry.addData("D-pad", "U: %1b  D: %1b  L: %1b  R: %1b", gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.dpad_left, gamepad2.dpad_right);
            telemetry.addData("Button", "A: %1b  B: %1b  X: %1b  Y: %1b", gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y);
            // Beeg == Guide because I don't know if people know what the guide button is
            telemetry.addData("Other", "Start: %1b  Back: %1b  Beeg: %1b", gamepad2.start, gamepad2.back, gamepad2.guide);

            telemetry.update();
        }
    }
}
