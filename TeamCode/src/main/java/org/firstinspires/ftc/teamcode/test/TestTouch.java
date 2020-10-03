package org.firstinspires.ftc.teamcode.test;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@Autonomous(name = "TestTouch", group = "Test")
public class TestTouch extends LinearOpMode {

    ButtonToggle touchToggle = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return touchState;
        }
    };

    DigitalChannel touch = null;
    boolean touchState = false;
    int pressCount = 0;

    public void runOpMode(){

        touch = hardwareMap.digitalChannel.get("touch");

        waitForStart();

        while (opModeIsActive()){

            touchState = !touch.getState();
            if (touchToggle.update()) pressCount++;
            telemetry.addData("PRESSED", touchState);
            telemetry.addData("COUNT", pressCount);
            telemetry.update();

        }
    }
}
