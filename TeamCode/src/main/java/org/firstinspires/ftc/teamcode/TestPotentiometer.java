package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
@TeleOp (name = "TestPotentiometer", group = "Test")
public class TestPotentiometer extends LinearOpMode {
    AnalogInput analog;
    @Override
    public void runOpMode() throws InterruptedException {
        analog = hardwareMap.get(AnalogInput.class, "analog");
        double maxVolt = analog.getMaxVoltage();
        telemetry.addData("MaxVolt", maxVolt);
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            double volts = analog.getVoltage();
            telemetry.addData("volts", volts);
            telemetry.update();
        }
    }
}
