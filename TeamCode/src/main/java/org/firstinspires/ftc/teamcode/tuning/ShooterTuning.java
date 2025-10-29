package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "ShooterTuningFormula")
@Config
public class ShooterTuning extends LinearOpMode {

    DcMotorEx leftMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");

        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        GamepadEx gp = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            gp.readButtons();
            leftMotor.setPower(gp.getLeftY());
            telemetry.addData("current", leftMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();
        }
    }
}