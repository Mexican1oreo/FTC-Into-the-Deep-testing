package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.Arm;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Util.RobotStates;

@TeleOp
public class Robot extends OpMode {
    Drivetrain drivetrain = new Drivetrain();
    Arm arm = new Arm();
    LinearSlide linearSlide = new LinearSlide();

    public void init() {
        this.drivetrain.init(hardwareMap);
        this.arm.init(hardwareMap);
        this.linearSlide.init(hardwareMap);
    }

    @Override
    public void loop() {
        this.linearSlide.slideData(telemetry);

        this.drivetrain.mechanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        if(gamepad2.a) {
            this.linearSlide.goToState(RobotStates.LinearSlide.HIGH_SCORE, -10_000);
        } else if(gamepad2.b) {
            this.linearSlide.goToState(RobotStates.LinearSlide.START_POS, 10_000);
        }
    }
}