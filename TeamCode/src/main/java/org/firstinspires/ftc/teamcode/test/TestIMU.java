package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.i2c.BNO055EnhancedImpl;
import org.firstinspires.ftc.teamcode.logging.LoggingLinearOpMode;

/**
 * Created by FTC Team 8397 on 9/13/2019.
 */
@Disabled
@Autonomous(name="TestIMU", group="Test")
public class TestIMU extends LoggingLinearOpMode {

    public BNO055Enhanced imu;
    public BNO055Enhanced.AxesMap axesMap = BNO055Enhanced.AxesMap.ZYX;
    public BNO055Enhanced.AxesSign axesSign = BNO055Enhanced.AxesSign.PPP;


    @Override
    public void runLoggingOpmode() throws InterruptedException {
        imu = hardwareMap.get(BNO055EnhancedImpl.class, "imu");
        BNO055Enhanced.Parameters parameters = new BNO055Enhanced.Parameters();
        parameters.angleUnit = BNO055Enhanced.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055Enhanced.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BN055Cali.json"; // see the calibration sample opmode
        parameters.loggingTag = "IMU";
        parameters.axesMap = axesMap;
        parameters.axesSign = axesSign;
        imu.initialize(parameters);

        waitForStart();

        ElapsedTime et = new ElapsedTime();
        while(opModeIsActive()) {
            if(et.milliseconds() < 200)
                continue;
            et.reset();

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Angles", " %.1f  %.1f  %.1f", angles.firstAngle, angles.secondAngle, angles.thirdAngle);
            telemetry.update();
        }
    }
}
