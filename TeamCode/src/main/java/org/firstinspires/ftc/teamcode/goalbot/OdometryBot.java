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
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Updatable;
import org.firstinspires.ftc.teamcode.util.odometry.Encoder;

import java.util.List;

public class OdometryBot extends GoalBot{
    Encoder rightEncoder = null;
    Encoder leftEncoder = null;
    Encoder horizEncoder = null;
    public static final float TICKS_PER_INCH = 1882.3f; //was 1875.4
    public static final float HORIZ_TICKS_PER_RAD = 4530.2f; //was 4008
    public static final float ROTATION_COEFF = 26919.5f; // was 26695
    public static final float FRAC_LEFT = 0.5012f; // was 0.507
    public static final float FRAC_RIGHT = 0.4988f; // was 0.493
    public int rightTicks, leftTicks, horizTicks;

    BNO055Enhanced armIMU = null;
    public ArmControl armControl = null;

    public OdometryBot() {
        super();
    }

    @Override
    public boolean init(HardwareMap hwMap) {
        super.init(hwMap);
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

        armControl = new ArmControl(0);

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

        BetaLog.dd("ODOM", "New ticks   Left = %d   Right = %d   Horiz = %d",
                leftNewTicks, rightNewTicks, horizNewTicks);

        float dyR = (FRAC_LEFT * rightNewTicks + FRAC_RIGHT * leftNewTicks) / TICKS_PER_INCH;
        float dT = (rightNewTicks - leftNewTicks) / (ROTATION_COEFF);
        float horizEncoderAngleTicks = HORIZ_TICKS_PER_RAD * dT;
        float dxR = (horizNewTicks - horizEncoderAngleTicks) / TICKS_PER_INCH;
        BetaLog.dd("ODOM", "dT = %.1f   horizEncoderAngleTicks = %.1f", dT, horizEncoderAngleTicks);
        float avgHeading = (float) AngleUtils.normalizeRadians(pose.theta + 0.5 * dT);
        BetaLog.dd("ODOM", "dxR = %.3f    dyR = %.3f", dxR, dyR);
        float dX = dxR * (float)Math.sin(avgHeading) + dyR * (float)Math.cos(avgHeading);
        float dY = -dxR * (float)Math.cos(avgHeading) + dyR * (float)Math.sin(avgHeading);
        BetaLog.dd("ODOM", "dX = %.3f   dY = %.3f", dX, dY);
        /*
         * Update the Pose object with the new values for X, Y, and Heading
         */
        float heading = (float) AngleUtils.normalizeRadians(pose.theta + dT);
        pose = new Pose(pose.x + dX, pose.y + dY, heading);
        BetaLog.dd("ODOM", "NewPose: x = %.3f   y = %.3f    theta = %.1f", pose.x, pose.y, Math.toDegrees(pose.theta));
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
        float magSquare = mag * mag;

        float zDOT = 1 - 2  * (q.y * q.y + q.x * q.x)/magSquare;
        float xDOT = 2  * (q.x * q.z - q.y * q.w)/magSquare;

        float result = (float)Math.atan2(-xDOT, zDOT);
        result = (float)Math.toDegrees(result);
        if(result < -60) result += 360;
        return result;
    }

    @Override
    public void setArmPosition (float target){
        armControl.setTarget(target);
    }

    public class ArmControl implements Updatable{
        private float target;
        private float coef = 0.02f;

        public ArmControl(float target){
            this.target = target;
        }

        public ArmControl(float target, float coef){
            this(target);
            this.coef = coef;
        }

        public void setTarget(float target){
            this.target = target;
        }

        public void update(){
            float armPosition = getArmPosition();

            float error = target - armPosition;
            float power = error * coef;
            if(Math.abs(power)>0.4) power = 0.4f * (float) Math.signum(power);
            if(target<1 && target>-60 && armPosition<10 && armPosition>-60) power = 0;
            BetaLog.dd("armcontrol", "pos = %.1f   targpos = %.1f   error = %.1f   power = %.3f",
                    armPosition, target, error, power);
            armMotor.setPower(power);


        }

    }

}
