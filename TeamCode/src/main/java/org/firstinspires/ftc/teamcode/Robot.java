package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.Arm;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Util.RobotStates;

import static org.firstinspires.ftc.teamcode.Util.Toggle.toggleButton;

@TeleOp
public class Robot extends OpMode {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Arm arm = new Arm();
    private final LinearSlide linearSlide = new LinearSlide();

    public void init() {
        this.drivetrain.init(hardwareMap);
        this.arm.init(hardwareMap);
        this.linearSlide.init(hardwareMap);
    }

    @Override
    public void loop() {
        this.linearSlide.slideData(telemetry);

        if(gamepad2.a) {
            if(toggleButton(gamepad2.a)) {
                this.linearSlide.setState(RobotStates.LinearSlide.HIGH_SCORE);
            } else if(toggleButton(gamepad2.a)) {
                this.linearSlide.setState(RobotStates.LinearSlide.START_POS);
            }
        }

        this.drivetrain.mechanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        this.linearSlide.goToState(telemetry);
    }
}