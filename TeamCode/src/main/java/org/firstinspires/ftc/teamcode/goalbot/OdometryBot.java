package org.firstinspires.ftc.teamcode.goalbot;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Updatable;
import org.firstinspires.ftc.teamcode.util.odometry.Encoder;

import java.util.List;

public class OdometryBot extends GoalBot{
    Encoder rightEncoder = null;
    Encoder leftEncoder = null;
    Encoder horizEncoder = null;
    public static final float TICKS_PER_INCH = 1865f; //was 1875.4
    public static final float HORIZ_TICKS_PER_RAD = 4056f; //was 4008
    public static final float ROTATION_COEFF = 26767f; // was 26695
    public static final float FRAC_LEFT = 0.503f; // was 0.507
    public static final float FRAC_RIGHT = 0.497f; // was 0.493
    public int rightTicks, leftTicks, horizTicks;

    BNO055Enhanced armIMU = null;

    public OdometryBot() {
        super();
    }

    @Override
    public boolean init(HardwareMap hwMap) {

        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        boolean result = super.init(hwMap);
        rightEncoder = new Encoder(intakeBack, true);
        leftEncoder = new Encoder(armMotor, true);
        horizEncoder = new Encoder(intakeFront, true);

        armIMU = hwMap.get(BNO055Enhanced.class, "arm_imu");
        imu = hwMap.get(BNO055Enhanced.class, "imu");

        BNO055Enhanced.Parameters parameters = new BNO055Enhanced.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BN055Cali.json";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.axesMap = BNO055Enhanced.AxesMap.XYZ;
        parameters.axesSign = BNO055Enhanced.AxesSign.PPN;

        boolean armSuccess = armIMU.initialize(parameters);

        return armSuccess;
    }

    @Override
    public Pose updateOdometry(){
        int rightCurrentTicks = rightEncoder.getCurrentPosition();
        int leftCurrentTicks = leftEncoder.getCurrentPosition();
        int horizCurrentTicks = horizEncoder.getCurrentPosition();
        int rightNewTicks = rightCurrentTicks - rightTicks;
        int leftNewTicks = leftCurrentTicks - leftTicks;
        int horizNewTicks = horizCurrentTicks - horizTicks;
        rightTicks = rightCurrentTicks;
        leftTicks = leftCurrentTicks;
        horizTicks = horizCurrentTicks;

        float dyR = (FRAC_LEFT * rightNewTicks + FRAC_RIGHT * leftNewTicks) / TICKS_PER_INCH;
        float dT = (rightNewTicks - leftNewTicks) / (ROTATION_COEFF);
        float horizEncoderAngleTicks = HORIZ_TICKS_PER_RAD * dT;
        float dxR = (horizNewTicks - horizEncoderAngleTicks) / TICKS_PER_INCH;
        float avgHeading = (float) AngleUtils.normalizeRadians(pose.theta + 0.5 * dT);

        float dX = dxR * (float)Math.sin(avgHeading) + dyR * (float)Math.cos(avgHeading);
        float dY = -dxR * (float)Math.cos(avgHeading) + dyR * (float)Math.sin(avgHeading);

        /*
         * Update the Pose object with the new values for X, Y, and Heading
         */
        float heading = (float) AngleUtils.normalizeRadians(pose.theta + dT);
        pose = new Pose(pose.x + dX, pose.y + dY, heading);

        /*
         * Return the updated Pose object
         */
        return pose;

    }
    @Override
    public void updateTicks(){
        leftTicks = leftEncoder.getCurrentPosition();
        rightTicks = rightEncoder.getCurrentPosition();
        horizTicks = horizEncoder.getCurrentPosition();

    }

    public Orientation getArmOrientation(){
        return armIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public Quaternion getArmQuaternion(){
        return armIMU.getQuaternionOrientation();
    }

    public float getArmPosition(){
        Quaternion q = armIMU.getQuaternionOrientation();
        float mag = q.magnitude();

        float zDOT = 1 - 2 * mag * (q.y * q.y + q.x * q.x);
        float xDOT = 2 * mag * (q.x * q.z - q.y * q.w);

        float result = (float)Math.acos(zDOT);
        result = (float)Math.toDegrees(result);
        if(xDOT > 0) result = - result;
        return result;
    }

    public class ArmControl implements Updatable{
        private float target;

        public ArmControl(float target){
            this.target = target;
        }

        public void setTarget(float target){
            this.target = target;
        }

        public void update(){

        }

    }

}
