package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.RobotStates;
import org.firstinspires.ftc.teamcode.Util.PIDController;

import static org.firstinspires.ftc.teamcode.Util.IDs.*;
import static org.firstinspires.ftc.teamcode.Util.Tuning.*;

public class LinearSlide {
    private DcMotorEx leftSlideMotor;
    private DcMotorEx rightSlideMotor;

    private PIDController leftPIDController;
    private PIDController rightPIDController;

    private RobotStates.LinearSlide linearSlideStates = RobotStates.LinearSlide.START_POS;
    private int slideEncoderValue;
    private VoltageSensor controlHubVoltageSensor;

    public void init(HardwareMap hardwareMap) {
        this.leftSlideMotor = hardwareMap.get(DcMotorEx.class, LEFT_SLIDE_MOTOR_ID);
        this.rightSlideMotor = hardwareMap.get(DcMotorEx.class, RIGHT_SLIDE_MOTOR_ID);

        this.leftSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.rightSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        this.leftSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.rightSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        this.leftSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        this.rightSlideMotor.setDirection(DcMotorEx.Direction.FORWARD);

        this.leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.controlHubVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        this.leftPIDController = new PIDController(LINEAR_SLIDE_P, LINEAR_SLIDE_I, LINEAR_SLIDE_D);
        this.rightPIDController = new PIDController(LINEAR_SLIDE_P, LINEAR_SLIDE_I, LINEAR_SLIDE_D);
    }

    public RobotStates.LinearSlide getCurrentState() {
        return linearSlideStates;
    }

    public int getStateEncoderVal(RobotStates.LinearSlide linearSlideState) {
        switch (linearSlideState) {
            case START_POS:
                slideEncoderValue = 0;
                break;

            case LOW_SCORE:
                slideEncoderValue = 100;
                break;

            case HIGH_SCORE:
                slideEncoderValue = 4_000;
                break;
        }
        return slideEncoderValue;
    }

//    public void goToState(RobotStates.LinearSlide desiredState, double velocity) {
//        int currentLeftPos = this.leftSlideMotor.getCurrentPosition() * -1;
//        int currentRightPos = this.rightSlideMotor.getCurrentPosition() * -1;
//
//        this.linearSlideStates = desiredState;
//
//        int desiredEncoderVal = this.getStateEncoderVal(desiredState);
//
//        int error = desiredEncoderVal - ((currentLeftPos + currentRightPos) / 2);
//
//        this.leftSlideMotor.setTargetPosition(desiredEncoderVal);
//        this.rightSlideMotor.setTargetPosition(desiredEncoderVal * -1);
//
//        if(Math.abs(error) > 50) {
//            this.leftSlideMotor.setVelocity(velocity);
//            this.rightSlideMotor.setVelocity(velocity);
//        } else {
//            this.leftSlideMotor.setVelocity(0);
//            this.rightSlideMotor.setVelocity(0);
//        }
//    }

    public void goToState(RobotStates.LinearSlide desiredState) {
        this.linearSlideStates = desiredState;

        int desiredStateEncoderVal = this.getStateEncoderVal(desiredState);

        while((this.leftSlideMotor.getCurrentPosition() * -1) != desiredStateEncoderVal){
            int currentLeftPos = this.leftSlideMotor.getCurrentPosition() * -1;
            double leftOutput = leftPIDController.calculate(desiredStateEncoderVal, currentLeftPos);
            this.leftSlideMotor.setPower(leftOutput);
        }

        while((this.rightSlideMotor.getCurrentPosition() * -1) != desiredStateEncoderVal){
            int currentRightPos = this.rightSlideMotor.getCurrentPosition() * -1;
            double rightOutput = rightPIDController.calculate(desiredStateEncoderVal, currentRightPos);
            this.rightSlideMotor.setPower(rightOutput);
        }
    }

    public void setSlidePower(double leftTrigger, double rightTrigger) {
        double voltageCorrection = 12 / controlHubVoltageSensor.getVoltage();

        this.leftSlideMotor.setPower((rightTrigger - leftTrigger) * voltageCorrection);
        this.rightSlideMotor.setPower((rightTrigger - leftTrigger) * voltageCorrection);
    }

    public void slideData(Telemetry telemetry) {
        telemetry.addData("Left Slide Encoder: ", this.leftSlideMotor.getCurrentPosition());
        telemetry.addData("Right Slide Encoder: ", this.rightSlideMotor.getCurrentPosition());
    }
}
