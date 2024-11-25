package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Util.Constants.*;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;

@Autonomous (name = "Test For acceleration")
public class Auto extends LinearOpMode {
    private final Drivetrain drivetrain = new Drivetrain();
    private final ElapsedTime timer = new ElapsedTime();

    private final long timeInMilliseconds = 1000000;

    @Override
    public void runOpMode() throws InterruptedException {
        this.waitForStart();
        this.timer.reset();
        this.drivetrain.init(hardwareMap);

        while(opModeIsActive()) {
            this.timer.startTime();
            this.telemetry.update();
            this.drivetrain.drivetrainData(telemetry);

            // ToDo: this the MAX_VEL value might need to be lowered by a bit
            if(this.drivetrain.getCurrentVelocity() == MAX_MOTOR_WHEEL_VELOCITY) {
                telemetry.addLine("Time: " + this.timer.seconds());
                this.drivetrain.autoDriveStraight(10000000, 0);
                this.drivetrain.autoInit(hardwareMap);
            } else {
                this.drivetrain.autoDriveStraight(10000000, MAX_VELOCITY);
                this.drivetrain.autoInit(hardwareMap);
            }
        }
    }
}
