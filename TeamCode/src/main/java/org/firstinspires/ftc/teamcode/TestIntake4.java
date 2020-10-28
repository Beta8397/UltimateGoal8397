package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@TeleOp(name = "Test Intake 4", group = "Test")
public class TestIntake4 extends LinearOpMode {

    public DcMotor intakeMotor;

    enum IntakeState {
        OFF, FWD, REV
    }

    private IntakeState intakeState = IntakeState.OFF;

    ButtonToggle toggleA = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.a;
        }
    };

    public void runOpMode() {
        gamepad1.setJoystickDeadzone(0.5f);
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            boolean aToggled = toggleA.update();
          switch (intakeState) {
              case OFF:
                  if (gamepad1.b) {
                      intakeState = IntakeState.REV;
                  } else if (aToggled){
                      intakeState = IntakeState.FWD;
                  }
                  break;
              case FWD:
                  if (gamepad1.b) {
                      intakeState = IntakeState.REV;
                  } else if (aToggled) {
                      intakeState = IntakeState.OFF;
                  }
                  break;
              case REV:
                  if (!gamepad1.b) {
                      intakeState = IntakeState.OFF;
                  }
                  break;
          }
          setIntake(intakeState);
        }

    }

    public void setIntake(IntakeState intakeState) {
        if (intakeState == IntakeState.OFF) {
            intakeMotor.setPower(0);
        } else if (intakeState == IntakeState.FWD) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(-1);
        }
    }
}
