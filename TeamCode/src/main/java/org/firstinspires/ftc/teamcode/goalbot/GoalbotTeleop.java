package org.firstinspires.ftc.teamcode.goalbot;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mecbot.MecBotTeleOp;
import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.Updatable;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@TeleOp(name = "GoalBotTeleOp", group = "GoalBot")
public class GoalbotTeleop extends MecBotTeleOp {

    public static final float SHOOTER_DELAYED = 1.0f;

    GoalBot bot= new GoalBot();

    private GoalBot.IntakeState intakeState = GoalBot.IntakeState.OFF;

    private Updatable autoDrive = null;

    ButtonToggle toggleRightBumper1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.right_bumper;
        }
    };

    ButtonToggle toggleA2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.a;
        }
    };

    ButtonToggle toggleRightBumper2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.right_bumper;
        }
    };

    ButtonToggle toggleLeftBumper2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.left_bumper;
        }
    };

    ButtonToggle toggleB2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.b;
        }
    };
    ButtonToggle toggleX2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() { return gamepad2.x; }
    };
    ButtonToggle toggleDpadDown2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.dpad_down;
        }
    };
    ButtonToggle toggleDpadUp2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.dpad_up;
        }
    };
    ButtonToggle toggleA1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.a;
        }
    };
    ButtonToggle toggleB1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.b;
        }
    };
    ButtonToggle toggleX1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.x;
        }
    };
    ButtonToggle toggleY1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.y;
        }
    };
    ButtonToggle toggleDpadRight1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.dpad_right;
        }
    };


    private boolean grabberClosed = true;
    private boolean shooterOn = false;
    private boolean shooterHigh = false;
    private boolean shooting = false;
    private boolean ringStuck = true;
    private boolean adjustmode = false;
    private ElapsedTime shootingTimer= new ElapsedTime();
    private enum ArmPos{
        IN, UP, OUT
    }

    private ArmPos armPos = ArmPos.IN;

    public void runOpMode() {
        bot.init(hardwareMap);
        super.setup(bot);
        bot.setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
        float armPosition = bot.getArmTargetPosition();

        waitForStart();

        while (opModeIsActive()) {
            handleIntake();
            if (toggleA2.update()){
                grabberClosed = !grabberClosed;
                if (grabberClosed) {
                    bot.setGrabberClosed();
                } else {
                    bot.setGrabberOpen();
                }
//                bot.grabber.setPosition    TODO: Handle grabber state
            }

            boolean dUp = toggleDpadUp2.update();
            boolean dDown = toggleDpadDown2.update();
            boolean armStateChange = false;

            if (armPos == ArmPos.IN){
                if (dDown){
                    armPos = ArmPos.OUT;
                    armStateChange = true;
                }

            } else {
                if (dDown){
                    armPos = ArmPos.IN;
                    armStateChange = true;
                } else {
                    if(dUp){
                        armPos = armPos == ArmPos.OUT? ArmPos.UP:ArmPos.OUT;
                        armStateChange = true;
                    }
                }
            }
            if(armStateChange) {

                int armTarget = armPos == ArmPos.OUT ? 590 :
                        armPos == ArmPos.UP ? 300 : 80;
                bot.setArmPosition(armTarget);
            }
            if(armPos == ArmPos.IN && bot.getArmActualPos() < 100){
                bot.setArmPower(0);
            }


            if (toggleRightBumper2.update()) {
                shooterOn = !shooterOn;
            }

            if (toggleLeftBumper2.update()) {
                shooterHigh = !shooterHigh;
            }
            if (shooterOn) {
                if (shooterHigh) {
                    bot.setShooterPowerHigh();
                } else {
                    bot.setShooterPowerNormal();
                }
            } else {
                bot.setShooterPower(0);
            }

            if (toggleX2.update()) {
                ringStuck = !ringStuck;
            }
            if (ringStuck) {
                bot.setRingKickerEngaged();
            } else {
                bot.setRingKickerUnengaged();
            }


            float shooterRPM = (float)bot.shooter.getVelocity(AngleUnit.DEGREES) / 6;
            telemetry.addData("shooter_RPM", shooterRPM);

            if (toggleB2.update()) {
                shooting = true;
                shootingTimer.reset();
            } else if(!gamepad2.b) {
                shooting = false;
            }

            if (shooting) {
                if (shootingTimer.seconds() < .5 * SHOOTER_DELAYED) {
                    bot.setKickerEngaged();
                } else if (shootingTimer.seconds() < SHOOTER_DELAYED) {
                    bot.setKickerUnengaged();
                } else {
                    shootingTimer.reset();
                }
            } else {
                bot.setKickerUnengaged();
            }

            if (toggleA1.update()) {
                bot.setPose(9, 9, 180);
            }
            bot.updateOdometry();

            if (toggleB1.update() && autoDrive == null) {
                autoDrive = new AutoDrive(36, 34, -162, 18, 4, 2, 6, 1, 1);
            } else if (toggleY1.update() && autoDrive == null) {
                autoDrive = new AutoDrive(67.5f, 59f, -156, 18, 4, 2, 6, 1, 1);
            } else if (toggleDpadRight1.update() && autoDrive == null ) {
                autoDrive = new AutoTurn((float)AngleUtils.normalizeRadians(bot.getPose().theta - Math.toRadians(5)));
            }

            if (toggleX1.update()) {
                adjustmode = !adjustmode;
            }
            telemetry.addData("adjust mode =", adjustmode);
            telemetry.addData("auto drive mode", autoDrive != null);
            telemetry.addData("Heading", Math.toDegrees(bot.getPose().theta));

            if (autoDrive != null) {
                if (gamepad1.b || gamepad1.y || gamepad1.dpad_right) {
                    autoDrive.update();
                } else {
                    autoDrive = null;
                    bot.setDrivePower(0, 0, 0);
                }
            } else if (adjustmode) {
                oneAdjustmentCycle();
            } else {
                doDriveControl();
             }
            telemetry.update();

        }
    }

    private void oneAdjustmentCycle() {
        float px = gamepad1.left_stick_x / 4;
        float py = -gamepad1.left_stick_y / 4;
        float heading = bot.getPose().theta;

        float sin = (float) Math.sin(heading);
        float cos = (float) Math.cos(heading);
        float pxr = px * sin - py * cos;
        float pyr = px * cos + py * sin;
        float angleOffset = (float) AngleUtils.normalizeRadians(Math.toRadians(-164) - heading);
        float pa = angleOffset / (float) Math.PI;
        bot.setDrivePower(pxr, pyr, pa);

    }

    private class AutoTurn implements Updatable {
        private float autoTurnTarget;
        public AutoTurn(float target) {
            autoTurnTarget = target;
        }
        public void update() {
            float offSet = (float) AngleUtils.normalizeRadians(autoTurnTarget - bot.getPose().theta);
            bot.setDriveSpeed(0, 0, 6 * offSet);
        }
    }

    private class AutoDrive implements Updatable {
        float targetX, targetY, targetHeadingRadians, vMax, vMin, propCoeffXY, propCoeffHeading, toleranceXY, toleranceRadians;
        final float VA_MAX = (float) Math.toRadians(60);
        final float VA_MIN = (float) Math.toRadians(5);
        public AutoDrive(float tX, float tY, float tHD, float vMax, float vMin, float pCXY, float pCH, float tolXY, float tolD){
            targetX = tX;
            targetY = tY;
            targetHeadingRadians =(float) Math.toRadians(tHD);
            this.vMax = vMax;
            this.vMin = vMin;
            propCoeffXY = pCXY;
            propCoeffHeading = pCH;
            toleranceXY = tolXY;
            toleranceRadians = (float) Math.toRadians(tolD);
        }

        public void update(){
            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float thetaError = (float) AngleUtils.normalizeRadians(targetHeadingRadians - bot.getPose().theta);

            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);

            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            float vx = xErrorRobot * propCoeffXY;
            float vy = yErrorRobot * propCoeffXY;
            float v = (float)Math.hypot(vx, vy);
            if (Math.hypot(xError, yError) < toleranceXY) {
                vx = 0;
                vy = 0;
            } else if(v > vMax) {
                vx *= vMax / v;
                vy *= vMax / v;
            } else if(v < vMin) {
                vx *= vMin / v;
                vy *= vMin / v;
            }

            float va = propCoeffHeading*thetaError;
            if (bot.getPose().x < 15 || bot.getPose().y < 15 || Math.abs(thetaError) < toleranceRadians) {
                va = 0;
            } else if (Math.abs(va) > VA_MAX) {
                va = Math.signum(va) * VA_MAX;
            }else if (Math.abs(va) < VA_MIN) {
                va = Math.signum(va) * VA_MIN;
            }

            bot.setDriveSpeed(vx, vy, va);
        }
    }

    private void handleIntake() {

        boolean RightBumperToggled = toggleRightBumper1.update();
        switch (intakeState) {
            case OFF:
                if (gamepad1.left_bumper) {
                    intakeState = GoalBot.IntakeState.REV;
                } else if (RightBumperToggled){
                    intakeState = GoalBot.IntakeState.FWD;
                }
                break;
            case FWD:
                if (gamepad1.left_bumper) {
                    intakeState = GoalBot.IntakeState.REV;
                } else if (RightBumperToggled) {
                    intakeState = GoalBot.IntakeState.OFF;
                }
                break;
            case REV:
                if (!gamepad1.left_bumper) {
                    intakeState = GoalBot.IntakeState.OFF;
                }
                break;

        }
        bot.setIntake(intakeState);

    }



}
