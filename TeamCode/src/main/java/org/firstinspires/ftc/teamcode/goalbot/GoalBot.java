package org.firstinspires.ftc.teamcode.goalbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;
import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.Pose;

public class GoalBot extends MecBot {

    public static final float GRABBER_OPEN_POSITION = 0;
    public static final float GRABBER_CLOSED_POSITION = 0.8f;
    public static final float SHOOTER_POWER_NORMAL = 0.78f;
    public static final float SHOOTER_POWER_HIGH = 0.9f;
    public static final float KICKER_ENGAGED = 0.32f;
    public static final float KICKER_HALF_ENGAGED = 0.20f;
    public static final float KICKER_UNENGAGED = 0.14f;
    public static final float RING_KICKER_ENGAGED = 0.3f;
    public static final float RING_KICKER_UNENGAGED = 0;
    public static final float GATE_UP = 0.9f;
    public static final float GATE_DOWN = 0f;

    DcMotorEx intakeFront;
    DcMotorEx intakeBack;
    public DcMotorEx armMotor;
    DcMotorEx shooter;
    Servo kicker;
    Servo grabber;
    Servo ringKicker;
    Servo gate;
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
        intakeFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeBack = hwMap.get(DcMotorEx.class, "intake back");
        intakeBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeFront.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor = hwMap.get(DcMotorEx.class, "arm_motor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        grabber = hwMap.get(Servo.class, "grabber");
        shooter = hwMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE );
        PIDFCoefficients pidfCoefficients= new PIDFCoefficients(100, 0.13, 0, 13);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//        shooter.setVelocityPIDFCoefficients(100, 0.13, 0, 13);
        kicker = hwMap.get(Servo.class, "kicker");
        ringKicker = hwMap.get(Servo.class, "ring kicker");
        gate = hwMap.get(Servo.class, "front_gate");
        return result;
    }



    public void setArmMode(DcMotor.RunMode Mode){
        if(Mode == armMotor.getMode()) return;
        if (Mode == DcMotor.RunMode.RUN_TO_POSITION) {
            armMotor.setTargetPosition(armMotor.getCurrentPosition());
        }
        armMotor.setMode(Mode);
    }

    public void setArmPosition(float ticks){
        armMotor.setTargetPosition((int)ticks);
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

    public void setGateUp() { gate.setPosition(GATE_UP); }

    public void setGateDown() { gate.setPosition(GATE_DOWN); }


    public void setIntakePower(float p){
        intakeFront.setPower(p);
        intakeBack.setPower(p);
    }
}
