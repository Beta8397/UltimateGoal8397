package org.firstinspires.ftc.teamcode.goalbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;
import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.Pose;

public class GoalBot extends MecBot {

    public static final float GRABBER_OPEN_POSITION = 0.2f;
    public static final float GRABBER_CLOSED_POSITION = 0.7f;
    public static final float SHOOTER_POWER_NORMAL = 0.74f;
    public static final float SHOOTER_POWER_HIGH = 0.9f;
    public static final float KICKER_ENGAGED = 0.14f;
    public static final float KICKER_HALF_ENGAGED = 0.18f;
    public static final float KICKER_UNENGAGED = 0.35f;
    public static final float RING_KICKER_ENGAGED = 0.4f;
    public static final float RING_KICKER_UNENGAGED = 0;
    public static final int ENCODER_TICKS_PER_ROTATION = 280;
    public static final float ENCODER_WHEEL_CIRCUMFERENCE = (float)Math.PI * 4;
    public static final float ENCODER_DISTANCE = 10;
    public static final float SIDE_ENCODER_OFFSET = 0;

    DcMotorEx intakeFront;
    DcMotorEx intakeBack;
    public DcMotorEx armMotor;
    DcMotorEx shooter;
    Servo kicker;
    Servo grabber;
    Servo ringKicker;
    DcMotorEx rightEncoder, leftEncoder, sideEncoder;
    public int rightTicks, leftTicks, sideTicks;
    public enum IntakeState {
        OFF, FWD, REV
    }

    public GoalBot(){
        super(MotorType.NeverestOrbital20, 13.25f, 13, 4.0f, 40.9f, 1, BNO055Enhanced.AxesMap.XZY,
                BNO055Enhanced.AxesSign.NPP);
    }


    public boolean init(HardwareMap hwMap){
        boolean result = super.init(hwMap);
        intakeFront = hwMap.get(DcMotorEx.class, "intake front");
        intakeBack = hwMap.get(DcMotorEx.class, "intake back");
        intakeFront.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor = hwMap.get(DcMotorEx.class, "arm_motor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        grabber = hwMap.get(Servo.class, "grabber");
        shooter = hwMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE );
        kicker = hwMap.get(Servo.class, "kicker");
        ringKicker = hwMap.get(Servo.class, "ring kicker");
//        leftEncoder = hwMap.get(DcMotorEx.class, "leftEncoder");
//        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightEncoder = hwMap.get(DcMotorEx.class, "rightEncoder");
//        sideEncoder = hwMap.get(DcMotorEx.class, "sideEncoder");
        return result;
    }

    public Pose updateFreeWheel(){
        int rightCurrentTicks = rightEncoder.getCurrentPosition();
        int leftCurrentTicks = leftEncoder.getCurrentPosition();
        int sideCurrentTicks = sideEncoder.getCurrentPosition();
        int rightNewTicks = rightCurrentTicks - rightTicks;
        int leftNewTicks = leftCurrentTicks - leftTicks;
        int sideNewTicks = sideCurrentTicks - sideTicks;
        rightTicks = rightCurrentTicks;
        leftTicks = leftCurrentTicks;
        sideTicks = sideCurrentTicks;
        float rightDist = (rightNewTicks / ENCODER_TICKS_PER_ROTATION) * ENCODER_WHEEL_CIRCUMFERENCE;
        float leftDist = (leftNewTicks / ENCODER_TICKS_PER_ROTATION) * ENCODER_WHEEL_CIRCUMFERENCE;
        float sideDist = (sideNewTicks / ENCODER_TICKS_PER_ROTATION) * ENCODER_WHEEL_CIRCUMFERENCE;
        float dyR = (rightDist + leftDist) / 2;
        float dT = (rightDist - leftDist) / ENCODER_DISTANCE;
        float sideEncoderAngleDist = -SIDE_ENCODER_OFFSET * dT;
        float dxR = sideDist - sideEncoderAngleDist;
        float avgHeading = (float)AngleUtils.normalizeRadians(pose.theta + 0.5 * dT);

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
    /*public void updateTicks(){
        leftTicks = leftEncoder.getCurrentPosition();
        rightTicks = rightEncoder.getCurrentPosition();
        sideTicks = sideEncoder.getCurrentPosition();

    }*/

    public void setArmMode(DcMotor.RunMode Mode){
        if (Mode == DcMotor.RunMode.RUN_TO_POSITION) {
            armMotor.setTargetPosition(armMotor.getCurrentPosition());
        }
        armMotor.setMode(Mode);
    }

    public void setArmPosition(int ticks){
        armMotor.setTargetPosition(ticks);
        if (armMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        armMotor.setPower(.3);
    }
    public int getArmActualPos(){
        return armMotor.getCurrentPosition();
    }

    public int getArmTargetPosition() {
        return armMotor.getTargetPosition();
    }

    public void setIntake(IntakeState intakeState) {
        if (intakeState == IntakeState.OFF) {
            setIntakePower(0);
        } else if (intakeState == IntakeState.FWD) {
            setIntakePower(1);
        } else {
            setIntakePower(-1);
        }
    }

    public void setShooterPower(float pwr) {
        shooter.setPower(pwr);
    }

    public void setShooterPowerNormal() {
        setShooterPower(SHOOTER_POWER_NORMAL);
    }

    public void setShooterPowerHigh() {
        setShooterPower(SHOOTER_POWER_HIGH);
    }

    public void setKickerPosition(float pos) {
        kicker.setPosition(pos);
    }

    public void setKickerEngaged() {
        setKickerPosition(KICKER_ENGAGED);
    }

    public void setKickerHalfEngaged() { setKickerPosition(KICKER_HALF_ENGAGED); }

    public void setKickerUnengaged() {
        setKickerPosition(KICKER_UNENGAGED);
    }

    public void setRingKickerPosition(float pos) {
        ringKicker.setPosition(pos);
    }

    public void setRingKickerEngaged() {
        setRingKickerPosition(RING_KICKER_ENGAGED);
    }

    public void setRingKickerUnengaged() {
        setRingKickerPosition(RING_KICKER_UNENGAGED);
    }

    public void setArmPower(float pwr){
        armMotor.setPower(pwr);
    }

    public void setGrabberPosition(float pos){
        grabber.setPosition(pos);
    }

    public void setGrabberOpen(){
        grabber.setPosition(GRABBER_OPEN_POSITION);
    }

    public void setGrabberClosed(){
        grabber.setPosition(GRABBER_CLOSED_POSITION);
    }

    public void setIntakePower(float p){
        intakeFront.setPower(p);
        intakeBack.setPower(p);
    }
}
