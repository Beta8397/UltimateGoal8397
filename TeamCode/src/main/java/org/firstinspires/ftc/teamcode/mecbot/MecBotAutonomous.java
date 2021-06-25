package org.firstinspires.ftc.teamcode.mecbot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.logging.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.CubicSpline2D;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.ParametricFunction2D;
import org.firstinspires.ftc.teamcode.util.Updatable;

/**
 * An abstract class that extends LinearOpMode and provides navigation methods that can be called by autonomous op modes
 * that utilize a MechBot object.
 */
public abstract class MecBotAutonomous extends LoggingLinearOpMode {

    public static final float STD_TURN_COEFF = 2.0f;        //Proportionate coefficient for turning
    private final float HEADING_CORRECTION_FACTOR = 2.0f;
    private final float DISTANCE_CORRECTION_FACTOR = 2.0f;
    public static final float STD_MAX_TURN_SPEED = 1.0f;    //Radians per sec

    /*
     * The MechBot object that will be used by the navigation methods in this class. This must be assigned a value
     * by calling the setBot method.
     */
    MecBot bot;

    public interface Predicate {
        boolean isTrue();
    }

    /**
     * Assign a MechBot object to bot. This object should already be initialized before being provided.
     *
     * @param b
     */
    public void setBot(MecBot b) {
        bot = b;
    }

    /**
     * Drive the robot straight in the specified direction, at the specified speed, while maintaining a specified
     * orientation, until it has travelled the specified number of inches.
     *
     * @param speed                Speed, in inches per second.
     * @param directionDegrees     Direction of travel (world coordinates), in degrees
     * @param targetHeadingDegrees Target orientation (world coordinates), in degrees
     * @param finished             Predicate which tells the bot when to stop
     */
    public void driveStraight(float speed, float directionDegrees,
                              float targetHeadingDegrees, Predicate finished) {

        float directionRadians = (float) Math.toRadians(directionDegrees);
        float targetHeadingRadians = (float) Math.toRadians(targetHeadingDegrees);

        /*
         * Control loop for this operation. Break from the loop after the specified distance has been travelled.
         */
        while (opModeIsActive()) {
            bot.updateOdometry();                                               //Determine current bot position             //Distance travelled (inches)
            if (finished.isTrue()) {
                break;                                                //Break from loop if we've travelled far enouch
            }

            float vx = -speed * (float) Math.sin(directionRadians - bot.getPose().theta);
            float vy = speed * (float) Math.cos(directionRadians - bot.getPose().theta);

            float angleOffset = (float) AngleUtils.normalizeRadians(targetHeadingRadians - bot.getPose().theta);
            float va = STD_TURN_COEFF * angleOffset;

            bot.setDriveSpeed(vx, vy, va);
        }
        // We have travelled far enough and broken from loop, so stop the robot.
        bot.setDrivePower(0, 0, 0);
    }

    /**
     * Turn to the specified heading using proportionate control
     *
     * @param targetHeadingDegrees
     * @param toleranceDegrees
     * @param propCoeff
     */
    public void turnToHeading(float targetHeadingDegrees, float toleranceDegrees,
                              float propCoeff, float maxDegreesPerSec) {
        float targetHeadingRadians = targetHeadingDegrees * (float) Math.PI / 180;
        float toleranceRadians = toleranceDegrees * (float) Math.PI / 180;
        float maxRadiansPerSec = maxDegreesPerSec * (float)Math.PI/180;
        float priorHeading = bot.getHeadingRadians();
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive()) {
            bot.updateOdometry();
            float currentHeading = bot.getPose().theta;
            /*
             * Normalized difference between target heading and current heading
             */
            float angleDiff = (float) AngleUtils.normalizeRadians(targetHeadingRadians - currentHeading);
            /*
             * Only check for completion if we believe we have a new reading from gyro. Assume a new reading
             * if: current reading is different from old one OR more than 50 ms has elapsed since the last
             * (assumed) new reading. After the test for completion, reset the timer and update priorHeading.
             */
            if (currentHeading != priorHeading || et.milliseconds() > 50) {
                float headingChange = (float) AngleUtils.normalizeRadians(currentHeading - priorHeading);
                if (Math.abs(angleDiff) < toleranceRadians && Math.abs(headingChange) < toleranceRadians / 5) {
                    break;
                } else {
                    et.reset();
                    priorHeading = currentHeading;
                }
            }
            float va = propCoeff * angleDiff;
            if (Math.abs(va) > maxRadiansPerSec){
                va = (float)Math.signum(va) * maxRadiansPerSec;
            }
            bot.setDriveSpeed(0, 0, va);
        }
        bot.setDrivePower(0, 0, 0);
    }

    /**
     * Turn to the specified heading using proportionate control
     *
     * @param targetHeadingDegrees
     * @param toleranceDegrees
     * @param propCoeff
     */
    public void turnToHeadingPD(float targetHeadingDegrees, float toleranceDegrees,
                              float propCoeff, float derivCoeff, float maxDegreesPerSec) {
        float targetHeadingRadians = targetHeadingDegrees * (float) Math.PI / 180;
        float toleranceRadians = toleranceDegrees * (float) Math.PI / 180;
        float maxRadiansPerSec = maxDegreesPerSec * (float)Math.PI/180;
        float priorHeading = bot.getHeadingRadians();
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive()) {
            bot.updateOdometry();
            float currentHeading = bot.getPose().theta;
            /*
             * Normalized difference between target heading and current heading
             */
            float angleDiff = (float) AngleUtils.normalizeRadians(targetHeadingRadians - currentHeading);
            float turnSpeed = 0;
            /*
             * Only check for completion if we believe we have a new reading from gyro. Assume a new reading
             * if: current reading is different from old one OR more than 50 ms has elapsed since the last
             * (assumed) new reading. After the test for completion, reset the timer and update priorHeading.
             */
            if (currentHeading != priorHeading || et.milliseconds() > 50) {
                float headingChange = (float) AngleUtils.normalizeRadians(currentHeading - priorHeading);
                if (Math.abs(angleDiff) < toleranceRadians && Math.abs(headingChange) < toleranceRadians / 5) {
                    break;
                } else {
                    turnSpeed = headingChange/ (float) et.milliseconds();
                    et.reset();
                    priorHeading = currentHeading;
                }
            }
            float va = propCoeff * angleDiff - derivCoeff * turnSpeed;
            if (Math.abs(va) > maxRadiansPerSec){
                va = (float)Math.signum(va) * maxRadiansPerSec;
            }
            bot.setDriveSpeed(0, 0, va);
        }
        bot.setDrivePower(0, 0, 0);
    }

    public void turnToHeading(float targetHeadingDegrees, float toleranceDegrees,
                              float propCoeff, float maxDegreesPerSec, Updatable action) {
        float targetHeadingRadians = targetHeadingDegrees * (float) Math.PI / 180;
        float toleranceRadians = toleranceDegrees * (float) Math.PI / 180;
        float maxRadiansPerSec = maxDegreesPerSec * (float)Math.PI/180;
        float priorHeading = bot.getHeadingRadians();
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive()) {
            bot.updateOdometry();
            action.update();
            float currentHeading = bot.getPose().theta;
            /*
             * Normalized difference between target heading and current heading
             */
            float angleDiff = (float) AngleUtils.normalizeRadians(targetHeadingRadians - currentHeading);
            /*
             * Only check for completion if we believe we have a new reading from gyro. Assume a new reading
             * if: current reading is different from old one OR more than 50 ms has elapsed since the last
             * (assumed) new reading. After the test for completion, reset the timer and update priorHeading.
             */
            if (currentHeading != priorHeading || et.milliseconds() > 50) {
                float headingChange = (float) AngleUtils.normalizeRadians(currentHeading - priorHeading);
                if (Math.abs(angleDiff) < toleranceRadians && Math.abs(headingChange) < toleranceRadians / 5) {
                    break;
                } else {
                    et.reset();
                    priorHeading = currentHeading;
                }
            }
            float va = propCoeff * angleDiff;
            if (Math.abs(va) > maxRadiansPerSec){
                va = (float)Math.signum(va) * maxRadiansPerSec;
            }
            bot.setDriveSpeed(0, 0, va);
        }
        bot.setDrivePower(0, 0, 0);
    }


    protected void driveSpline(float speed, boolean reverse, CubicSpline2D spline) {
        spline.setIndex(0);
        float s0 = 0;
        speed = (float) Math.abs(speed);
        while (opModeIsActive()) {
            bot.updateOdometry();

            s0 = spline.nextClosestPt(bot.getPose().x, bot.getPose().y, s0, this);

            if (s0 >= 1 && spline.getIndex() == (spline.getNumSegments() - 1)) break;
            VectorF targetPos = spline.p(s0);
            VectorF posOffset = targetPos.subtracted(new VectorF(bot.getPose().x, bot.getPose().y));
            VectorF d1 = spline.d1(s0);
            VectorF d2 = spline.d2(s0);

            VectorF fwdVel = d1.multiplied(speed / d1.magnitude());
            VectorF corrVel = posOffset.multiplied(2 * DISTANCE_CORRECTION_FACTOR);
            VectorF totalV = fwdVel.added(corrVel);
            VectorF totalVBot = fieldToBot(totalV, bot.getPose().theta);

            float targetHeading = reverse ? (float) Math.atan2(-d1.get(1), -d1.get(0)) : (float) Math.atan2(d1.get(1), d1.get(0));
            float targetHeadingChangeRate = (d2.get(1) * d1.get(0) - d2.get(0) * d1.get(1)) / (float) Math.pow(d1.dotProduct(d1), 1.5f) * speed;
            float headingOffset = (float) AngleUtils.normalizeRadians(targetHeading - bot.getPose().theta);
            float va = targetHeadingChangeRate + headingOffset * HEADING_CORRECTION_FACTOR;

            bot.setDriveSpeed(totalVBot.get(0), totalVBot.get(1), va);
        }

        bot.setDriveSpeed(0, 0, 0);
    }

    /**
     * Drive spline while turning from initial heading to requested final heading.
     * The turn happens during the middle "turnFraction" of the total path length of the spline.
     * At the end there is a brief adjustment of position to the final point in the spline.
     *
     * @param motionProfile     vMin, vMax, and accel for the operation
     * @param turnFraction      The fraction of the total path length over which the turn occurs
     * @param finalHeadingDegrees   Requested final heading
     * @param tol                   Tolerance for end point
     * @param spline                The 2D cubic spline
     */
    protected void driveSpline(MotionProfile motionProfile, float turnFraction, float finalHeadingDegrees, float adjDist, float tol, CubicSpline2D spline) {
        spline.setIndex(0);
        float s0 = 0;

        float totalPathLength = spline.getTotalPathLength();
        float turnPathLength = turnFraction * totalPathLength;
        float preTurnPathLength = (totalPathLength - turnPathLength) / 2.0f;
        float finalHeadingRadians = (float)Math.toRadians(finalHeadingDegrees);
        float initHeadingRadians = bot.updateOdometry().theta;
        float turnRadians = (float)AngleUtils.normalizeRadians(finalHeadingRadians - initHeadingRadians);
        float radiansPerInch = turnRadians / turnPathLength;
        float vMinSquared = motionProfile.vMin * motionProfile.vMin;
        float finalX = spline.getPoints()[spline.getNumSegments()].get(0);
        float finalY = spline.getPoints()[spline.getNumSegments()].get(1);

        // Drive the spline
        while (opModeIsActive()) {
            bot.updateOdometry();

            s0 = spline.nextClosestPt(bot.getPose().x, bot.getPose().y, s0, this);
            float currentPathLength = spline.getPathLength(s0);
            float remainingPathLength = totalPathLength - currentPathLength;

            if (remainingPathLength < adjDist ||  (s0 >= 1 && spline.getIndex() == (spline.getNumSegments() - 1))) break;

            VectorF targetPos = spline.p(s0);
            VectorF posOffset = targetPos.subtracted(new VectorF(bot.getPose().x, bot.getPose().y));
            VectorF d1 = spline.d1(s0);
            VectorF d2 = spline.d2(s0);

            float speed = (float)Math.min(Math.sqrt(vMinSquared + 2.0*motionProfile.accel*currentPathLength),
                    Math.sqrt(vMinSquared + 2.0*motionProfile.accel*(remainingPathLength)));
            speed = Math.min(speed, motionProfile.vMax);
            VectorF fwdVel = d1.multiplied(speed / d1.magnitude());
            VectorF corrVel = posOffset.multiplied(2 * DISTANCE_CORRECTION_FACTOR);
            VectorF totalV = fwdVel.added(corrVel);
            VectorF totalVBot = fieldToBot(totalV, bot.getPose().theta);

            float nominalAngularSpeed;
            float targetHeadingRadians;
            if (currentPathLength < preTurnPathLength) {
                nominalAngularSpeed = 0;
                targetHeadingRadians = initHeadingRadians;
            } else if (currentPathLength < (preTurnPathLength + turnPathLength)){
                nominalAngularSpeed = radiansPerInch * speed;
                targetHeadingRadians = (float)AngleUtils.normalizeRadians(((currentPathLength - preTurnPathLength) / turnPathLength) * turnRadians + initHeadingRadians);
            } else {
                nominalAngularSpeed = 0;
                targetHeadingRadians = finalHeadingRadians;
            }
            float headingOffset = (float)AngleUtils.normalizeRadians(targetHeadingRadians - bot.getPose().theta);
            float va = nominalAngularSpeed + headingOffset * HEADING_CORRECTION_FACTOR;

            bot.setDriveSpeed(totalVBot.get(0), totalVBot.get(1), va);
        }

        //Adjust position at the end
        while (opModeIsActive()){
            bot.updateOdometry();
            float xOffset = finalX - bot.getPose().x;
            float yOffset = finalY - bot.getPose().y;
            float dOffset = (float)Math.hypot(xOffset, yOffset);

            if (dOffset < tol) break;

            float speed = (float)Math.min(vMinSquared + 2.0*motionProfile.accel*dOffset, motionProfile.vMax);
            VectorF velocity = new VectorF(speed * xOffset/dOffset, speed * yOffset/dOffset);
            VectorF totalVBot = fieldToBot(velocity, bot.getPose().theta);
            float headingOffset = (float)AngleUtils.normalizeRadians(finalHeadingRadians - bot.getPose().theta);
            float va = headingOffset * HEADING_CORRECTION_FACTOR;
            bot.setDriveSpeed(totalVBot.get(0), totalVBot.get(1), va);
        }

        bot.setDriveSpeed(0, 0, 0);
    }

    protected void driveFunction(float speed, float s0, ParametricFunction2D pf, Predicate finish) {
        while (opModeIsActive()) {
            bot.updateOdometry();
            if (finish.isTrue()) break;
            s0 = findClosestPt(bot.getPose().x, bot.getPose().y, s0, pf);
            VectorF targetPos = pf.p(s0);
            VectorF posOffset = targetPos.subtracted(new VectorF(bot.getPose().x, bot.getPose().y));
            VectorF d1 = pf.d1(s0);
            VectorF d2 = pf.d2(s0);

            VectorF totalV = posOffset.multiplied(2 * DISTANCE_CORRECTION_FACTOR).added(d1.multiplied(speed / d1.magnitude()));
            VectorF totalVBot = fieldToBot(totalV, bot.getPose().theta);

            float targetHeading = (float) Math.atan2(d1.get(1), d1.get(0));
            float targetHeadingChangeRate = (d2.get(1) * d1.get(0) - d2.get(0) * d1.get(1)) / (float) Math.pow(d1.dotProduct(d1), 1.5f) * speed;
            float headingOffset = (float) AngleUtils.normalizeRadians(targetHeading - bot.getPose().theta);
            float va = targetHeadingChangeRate + headingOffset * HEADING_CORRECTION_FACTOR;

            bot.setDriveSpeed(totalVBot.get(0), totalVBot.get(1), va);
        }
    }

    protected VectorF fieldToBot(VectorF vField, float heading) {
        float sinTheta = (float) Math.sin(heading);
        float cosTheta = (float) Math.cos(heading);
        return new VectorF(vField.get(0) * sinTheta - vField.get(1) * cosTheta, vField.get(0) * cosTheta + vField.get(1) * sinTheta);
    }

    protected float findClosestPt(float x0, float y0, float s0, ParametricFunction2D pf) {
        float epsilon = .0001f;
        float delta = 100;
        while(delta > epsilon && opModeIsActive()) {
            VectorF p = pf.p(s0);
            VectorF d1 = pf.d1(s0);
            VectorF d2 = pf.d2(s0);
            float f = (p.get(0) - x0) * d1.get(0) + (p.get(1) - y0) * d1.get(1);
            float fDeriv = (p.get(0) - x0) * d2.get(0) + d1.get(0) * d1.get(0) + (p.get(1) - y0) * d2.get(1) + d1.get(1) * d1.get(1);
            float s = s0 - f / fDeriv;
            delta = Math.abs(s0 - s);
            s0 = s;
        }
        return s0;
    }

    protected void driveToPosition(float vMax, float vMin, float targetX, float targetY, float targetThetaDegrees, float cp, float tolerance) {
        float headingTargetRadians = targetThetaDegrees * (float)Math.PI / 180;

        while (opModeIsActive()) {
            bot.updateOdometry();

            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float thetaError = (float)AngleUtils.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            if(Math.hypot(xError, yError) < tolerance) break;

            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);

            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            float vx = xErrorRobot * cp;
            float vy = yErrorRobot * cp;
            float v = (float)Math.hypot(vx, vy);
            if(v > vMax) {
                vx *= vMax / v;
                vy *= vMax / v;
            } else if(v < vMin) {
                vx *= vMin / v;
                vy *= vMin / v;
            }
            float va = HEADING_CORRECTION_FACTOR * thetaError;
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDriveSpeed(0, 0,0);
        bot.updateOdometry();
    }

    protected void driveToPosition(float vMax, float vMin, float targetX, float targetY, float targetThetaDegrees, float cp, float tolerance, Updatable action) {
        float headingTargetRadians = targetThetaDegrees * (float)Math.PI / 180;

        while (opModeIsActive()) {
            bot.updateOdometry();
            action.update();

            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float thetaError = (float)AngleUtils.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            if(Math.hypot(xError, yError) < tolerance) break;

            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);

            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            float vx = xErrorRobot * cp;
            float vy = yErrorRobot * cp;
            float v = (float)Math.hypot(vx, vy);
            if(v > vMax) {
                vx *= vMax / v;
                vy *= vMax / v;
            } else if(v < vMin) {
                vx *= vMin / v;
                vy *= vMin / v;
            }
            float va = HEADING_CORRECTION_FACTOR * thetaError;
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDriveSpeed(0, 0,0);
        bot.updateOdometry();
    }

    protected void driveToPosition(MotionProfile mProf, float targetX, float targetY, float targetThetaDegrees, float tolerance) {
        float headingTargetRadians = targetThetaDegrees * (float)Math.PI / 180;

        bot.updateOdometry();
        float x0 = bot.getPose().x;
        float y0 = bot.getPose().y;

        while (opModeIsActive()) {
            bot.updateOdometry();
            float d0 = (float)Math.hypot(bot.getPose().y - y0, bot.getPose().x - x0);

            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float d1 = (float)Math.hypot(xError, yError);
            float thetaError = (float)AngleUtils.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            if(d1 < tolerance) break;

            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);

            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            float v0 = (float)Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d0);
            float v1 = (float)Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d1);
            float v = Math.min(v0, v1);
            v = Math.min(v, mProf.vMax);

            float vx = xErrorRobot * v / d1;
            float vy = yErrorRobot * v / d1;

            float va = HEADING_CORRECTION_FACTOR * thetaError;
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDriveSpeed(0, 0,0);
        bot.updateOdometry();
    }

    protected void driveToPosition(MotionProfile mProf, float targetX, float targetY, float targetThetaDegrees, float tolerance, Updatable action) {
        float headingTargetRadians = targetThetaDegrees * (float)Math.PI / 180;

        bot.updateOdometry();
        float x0 = bot.getPose().x;
        float y0 = bot.getPose().y;
        action.update();

        while (opModeIsActive()) {
            bot.updateOdometry();
            float d0 = (float)Math.hypot(bot.getPose().y - y0, bot.getPose().x - x0);

            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float d1 = (float)Math.hypot(xError, yError);
            float thetaError = (float)AngleUtils.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            if(d1 < tolerance) break;
            action.update();

            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);

            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            float v0 = (float)Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d0);
            float v1 = (float)Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d1);
            float v = Math.min(v0, v1);
            v = Math.min(v, mProf.vMax);

            float vx = xErrorRobot * v / d1;
            float vy = yErrorRobot * v / d1;

            float va = HEADING_CORRECTION_FACTOR * thetaError;
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDriveSpeed(0, 0,0);
        bot.updateOdometry();
    }

    protected void driveToPositionGyro(float vMax, float vMin, float targetX, float targetY, float targetThetaDegrees, float cp, float tolerance) {
        float headingTargetRadians = targetThetaDegrees * (float)Math.PI / 180;

        while (opModeIsActive()) {
            bot.updateOdometry();

            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float thetaError = (float)AngleUtils.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            if(Math.hypot(xError, yError) < tolerance) {
                break;
            }

            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);

            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            float vx = xErrorRobot * cp;
            float vy = yErrorRobot * cp;
            float v = (float)Math.hypot(vx, vy);
            if(v > vMax) {
                vx *= vMax / v;
                vy *= vMax / v;
            } else if(v < vMin) {
                vx *= vMin / v;
                vy *= vMin / v;
            }
            float va = HEADING_CORRECTION_FACTOR * thetaError;

            bot.setDriveSpeed(vx, vy, va);

        }
        bot.setDriveSpeed(0, 0,0);
    }


}