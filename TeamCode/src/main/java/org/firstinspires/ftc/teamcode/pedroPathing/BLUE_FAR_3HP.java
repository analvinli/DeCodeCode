package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDFController;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.List;

import com.pedropathing.follower.Follower;


@Autonomous(name = "🔵 BLUE far 3hp")
public class BLUE_FAR_3HP extends LinearOpMode {
    //Drivetrain
    DcMotorEx RightFront;
    DcMotorEx RightRear;
    DcMotorEx LeftRear;
    DcMotorEx LeftFront;
    //Spindexer
    Servo KickerServo;
    NormalizedColorSensor IntakeSensor;
    NormalizedColorSensor LeftSensor;
    NormalizedColorSensor BackSensor;
    NormalizedColorSensor RightSensor;
    DcMotorEx SpindexerMotor;
    //Intake or Outtake
    DcMotorEx IntakeMotor;
    DcMotorEx RightFlywheelMotor;
    DcMotorEx LeftFlywheelMotor;
    Servo HoodServo;

    int[] SpindexPos = {
            0,//shooting0
            1365,//intaking1
            2730,//shooting2
            4095,//intaking3
            5460,//shooting4
            6825,//intaking5
    };
    ElapsedTime KickerTimer = new ElapsedTime();
    int KickerState = 0;
    int CurrentSpindexerPos;
    static int ticksPerRevolution = 8192;
    int IntakeState = 0;

    boolean IntakeActive = false;
    int velocity = 0;
    PIDFController SpindexController = new PIDFController(0.00031,0,0.00001,0);
    //PIDFController SpindexController = new PIDFController(0.00024,0,0.00001,0);


    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;


    private Follower follower;
    private ElapsedTime pathTimer = new ElapsedTime();
    private ElapsedTime actionTimer;
    private ElapsedTime opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(47.62, 8.88, Math.toRadians(90));

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;

    public void runOpMode() {
        //HARDWARE MAPPING
        RightFront = hardwareMap.get(DcMotorEx.class, "fr");
        RightRear = hardwareMap.get(DcMotorEx.class, "br");
        LeftRear = hardwareMap.get(DcMotorEx.class, "bl");
        LeftFront = hardwareMap.get(DcMotorEx.class, "fl");
        SpindexerMotor = hardwareMap.get(DcMotorEx.class, "spindex");
        IntakeSensor = hardwareMap.get(NormalizedColorSensor.class, "csi");
        LeftSensor = hardwareMap.get(NormalizedColorSensor.class, "csl");
        BackSensor = hardwareMap.get(NormalizedColorSensor.class, "csb");
        RightSensor = hardwareMap.get(NormalizedColorSensor.class, "csr");
        KickerServo = hardwareMap.get(Servo.class, "kick");
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        RightFlywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelr");
        LeftFlywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheell");
        HoodServo = hardwareMap.get(Servo.class,"hood");

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        RightFlywheelMotor.setVelocityPIDFCoefficients(300,0,0,15.5);


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        motorConfigs();

        waitForStart();
        KickerState = 0;
        SpindexIncrementOuttake(2);
        int x = 0;
        while (opModeIsActive()) {
            if(pathState == 0){//start
                velocity = 1570;//start up flywheel
                HoodServo.setPosition(1);
                follower.followPath(Path1);//move to read pos
                SpindexController.setSetPoint(0);
                pathState++;
                pathTimer.reset();
            }else if(pathState == 1){
                if(!follower.isBusy() && FlywheelGood()){//at scoring position
                    if(ShootUnsorted()){//shoot
                        pathState++;
                        velocity = 0;
                        Intake();
                        follower.followPath(Path2);//move to hp intake pos
                        pathTimer.reset();
                    }
                }
            }else if(pathState == 2){
                Intake();
                if(!follower.isBusy() || pathtime(1500)){
                    pathState++;
                    follower.followPath(Path3);
                    pathTimer.reset();
                }
            }else if(pathState == 3){
                Intake();
                if(!follower.isBusy() || pathtime(1500)){
                    follower.followPath(Path4);
                    pathTimer.reset();
                    pathState++;
                }
            }else if(pathState == 4){
                if(!follower.isBusy()){
                    pathState++;
                    break;
                }
            }


            follower.update();
            kickSM();
            SpindexSM();
            FlywheelSM(velocity);

            telemetry.addData("detect", detectTag());
            telemetry.addData("motif0", motif[0]);
            telemetry.addData("motif1", motif[1]);
            telemetry.addData("motif2", motif[2]);
            telemetry.addData("path state", pathState);
//            telemetry.addData("x", follower.getPose().getX());
//            telemetry.addData("y", follower.getPose().getY());
//            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("overshoot", SpindexerMotor.getCurrentPosition()-SpindexController.getSetPoint());
//            telemetry.addData("Intake ColorSensor: ", findColor(IntakeSensor));
//            telemetry.addData("Left ColorSensor: ", findColor(LeftSensor));
//            telemetry.addData("Right ColorSensor: ", findColor(RightSensor));
//            telemetry.addData("Back ColorSensor: ", findColor(BackSensor));
//
//            telemetry.addData("Intake ColorSensor hue: ", JavaUtil.colorToHue(IntakeSensor.getNormalizedColors().toColor()));
//            telemetry.addData("Left ColorSensor hue: ", JavaUtil.colorToHue(LeftSensor.getNormalizedColors().toColor()));
//            telemetry.addData("Right ColorSensor hue: ", JavaUtil.colorToHue(RightSensor.getNormalizedColors().toColor()));
//            telemetry.addData("Back ColorSensor hue: ", JavaUtil.colorToHue(BackSensor.getNormalizedColors().toColor()));
            telemetry.update();
        }
    }
    public boolean pathtime(int t){
        if(pathTimer.milliseconds()>t){
            return true;
        }
        return false;
    }

    public void buildPaths(){
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(47.620, 8.880),

                                new Pose(56.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 12.000),

                                new Pose(12.099, 16.088)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(200))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(12.099, 16.088),
                                new Pose(22.807, 8.892),
                                new Pose(11.307, 9.456)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(11.307, 9.456),
                                new Pose(29.335, 17.642),
                                new Pose(56.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))

                .build();
    }

    int ScoreStateSorted = 0;
    int numOuttakedSorted = 0;
    ElapsedTime ShootTimerSorted = new ElapsedTime();
    int[] motif = new int[] {1,1,2};////////////////////////////////
    public boolean ShootSorted(){
        if(ScoreStateSorted == 0){//set up to outtake position
            SpindexSpinToClosestOuttake();
            numOuttakedSorted = 0;
            ScoreStateSorted = 1;
        }else if(ScoreStateSorted == 1){//wait until at tolerance
            if(SpindexWithinTolerance()){
                ShootTimerSorted.reset();
                ScoreStateSorted = 2;
            }
        }else if(ScoreStateSorted == 2){//have settle time
            if(ShootTimerSorted.milliseconds() > 50){
                ScoreStateSorted = 3;
            }
        }

        else if(ScoreStateSorted == 3){//spin to color
            SpindexSpinToColor(motif[0]);
            ScoreStateSorted = 4;
        }else if(ScoreStateSorted == 4){//wait until at tolerance
            if(SpindexWithinTolerance()){
                ShootTimerSorted.reset();
                ScoreStateSorted = 5;
            }
        }else if(ScoreStateSorted == 5){//have settle time
            if(ShootTimerSorted.milliseconds() > 50){
                ScoreStateSorted = 6;
            }
        }

        else if(ScoreStateSorted == 6){//start kick
            KickerState = 1;
            ScoreStateSorted = 7;
        }else if(ScoreStateSorted == 7){//wait until finished kicking
            if(KickerState == 0){
                numOuttakedSorted++;
                ScoreStateSorted = 8;
            }
        }else if(ScoreStateSorted == 8){//spin to next chamber
            if(numOuttakedSorted == 3){//exit after 3 balls shot
                ScoreStateSorted = 99;
                return true;
            }else{
                SpindexSpinToColor(motif[numOuttakedSorted]);
                ScoreStateSorted = 9;
            }
        }else if(ScoreStateSorted == 9){//wait until at tolerance
            if(SpindexWithinTolerance()){
                ShootTimerSorted.reset();
                ScoreStateSorted = 10;
            }
        }else if(ScoreStateSorted == 10){
            if(ShootTimerSorted.milliseconds() > 50){//allow to settle
                ScoreStateSorted = 6;//loop back to kick
            }
        }
        return false;
    }

    int ScoreState = 0;
    int numOuttaked = 0;
    ElapsedTime ShootTimer = new ElapsedTime();
    public boolean ShootUnsorted(){
        if(ScoreState == 0){//set up to outtake position
            SpindexSpinToClosestOuttake();
            numOuttaked = 0;
            ScoreState = 1;
        }else if(ScoreState == 1){//wait until at tolerance
            if(SpindexWithinTolerance(800)){
                ShootTimer.reset();
                ScoreState = 2;
            }
        }else if(ScoreState == 2){//have settle time
            if(ShootTimer.milliseconds() > 50){
                ScoreState = 3;
            }
        }

        else if(ScoreState == 3){//start kick
            KickerState = 1;
            ScoreState = 4;
        }else if(ScoreState == 4){//wait until finished kicking
            if(KickerState == 0){
                numOuttaked++;
                ScoreState = 5;
            }
        }else if(ScoreState == 5){//spin to next chamber
            if(numOuttaked == 3){//exit after 3 balls shot
                ScoreState = 99;
                return true;
            }else{
                SpindexIncrementOuttake(2);
                ScoreState = 6;
            }
        }else if(ScoreState == 6){//wait until at tolerance
            if(SpindexWithinTolerance(1200)){
                ShootTimer.reset();
                ScoreState = 7;
            }
        }else if(ScoreState == 7){
            if(ShootTimer.milliseconds() > 50){//allow to settle
                ScoreState = 3;//loop back to kick
            }
        }
        return false;
    }

    int numIntaked = 0;
    ElapsedTime IntakeTimer = new ElapsedTime();
    public boolean Intake(){
        if(IntakeState == 0){
            SpindexSpinToClosestIntake();//set up for intaking
            if(SpindexWithinTolerance()){
                IntakeState = 1;
            }
        }else if(IntakeState == 1) {
            if (detectColor(IntakeSensor) && SpindexWithinTolerance(500)) {
                SpindexIncrementIntake(2);
            }
            IntakeMotor.setPower(0.9);
            IntakeActive = true;
        }
        return false;
    }
    int prevInc = 0;
    public boolean SpindexSpinToColor(int color){
        //1 = purple
        //2 = green
        if(findColor(BackSensor) == color){
            prevInc = 2;
            return true;
        }else if(findColor(RightSensor) == color){
            prevInc = 2;
            SpindexIncrementOuttake(2);
            return true;
        }else if(findColor(LeftSensor) == color){
            prevInc = -2;
            SpindexIncrementOuttake(-2);
            return true;
        }else{//spin to one with empty
            SpindexIncrementOuttake(prevInc);
        }
        return false;
    }
    public void SpindexIncrementOuttake(int i){
        //clockwise is positive
        int indexOfClosest = findIndexSpindex(FindClosestOuttake());
        indexOfClosest = incrementIndex(indexOfClosest,i,SpindexPos.length);
        SpindexController.setSetPoint(SpindexGetAbsTarget(SpindexPos[indexOfClosest]));
    }
    public void SpindexIncrementIntake(int i){
        //clockwise is positive
        int indexOfClosest = findIndexSpindex(FindClosestIntake());
        indexOfClosest = incrementIndex(indexOfClosest,i,SpindexPos.length);
        SpindexController.setSetPoint(SpindexGetAbsTarget(SpindexPos[indexOfClosest]));
    }

    public boolean SpindexWithinTolerance(){
        if(Math.abs(SpindexController.getSetPoint()-SpindexerMotor.getCurrentPosition())<600){
            return true;
        }
        return false;
    }
    public boolean SpindexWithinTolerance(int tol){
        if(Math.abs(SpindexController.getSetPoint()-SpindexerMotor.getCurrentPosition())<tol){
            return true;
        }
        return false;
    }

    public void SpindexSpinToClosestOuttake(){
        int indexOfClosest = findIndexSpindex(FindClosestOuttake());
        SpindexController.setSetPoint(SpindexGetAbsTarget(SpindexPos[indexOfClosest]));
    }
    public void SpindexSpinToClosestIntake(){
        int indexOfClosest = findIndexSpindex(FindClosestIntake());
        SpindexController.setSetPoint(SpindexGetAbsTarget(SpindexPos[indexOfClosest]));
    }
    public void SpindexSM(){
        double error = SpindexerMotor.getCurrentPosition()-SpindexController.getSetPoint();
        if(SpindexWithinTolerance()){
            SpindexerMotor.setPower(0);
            if(!IntakeActive){
                IntakeMotor.setPower(0);
            }
        }else{
            IntakeMotor.setPower(0.7);
            double raw = SpindexController.calculate(SpindexerMotor.getCurrentPosition());
            SpindexerMotor.setPower(raw+(Math.signum(raw)*0.06));
        }
    }
    public int FlywheelSM(int v){
        if(v == 0){
            RightFlywheelMotor.setPower(0);
            LeftFlywheelMotor.setPower(0);
        }
        if(v - RightFlywheelMotor.getVelocity() > 50){
            RightFlywheelMotor.setPower(1);
            LeftFlywheelMotor.setPower(1);
            return 0;
        }else{
            RightFlywheelMotor.setVelocity(v);
            LeftFlywheelMotor.setPower(RightFlywheelMotor.getPower());
            return 1;
        }
    }
    public boolean FlywheelGood(){
        if(Math.abs(velocity-RightFlywheelMotor.getVelocity())<100){
            return true;
        }
        return false;
    }

    public void kickSM(){
        if(KickerState == 0){
            KickerServo.setPosition(0.5);//low position
        }else if(KickerState == 1){
            KickerServo.setPosition(0.1);//high position
            KickerTimer.reset();
            KickerState = 2;//ready to wait
        }else if(KickerState == 2){
            if(KickerTimer.milliseconds()>150){//time needed to move up has passed
                KickerServo.setPosition(0.5);//low position
                KickerTimer.reset();
                KickerState = 3;
            }
        }else if(KickerState == 3) {
            if (KickerTimer.milliseconds() > 20) {//cooldown period has passed
                KickerState = 0;
            }
        }
    }
    public boolean detection(){
        int lockedtag = -1;
        int tag = detectTag();

        if(lockedtag != -1){
            return true;
        }
        if(solveMotif(tag).length == 3){
            motif = solveMotif(tag);
            return true;
        }
        return false;
    }

    public int detectTag() {
        int selectedTagId = -1;
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for(AprilTagDetection detection : detections) {
            if(detection.id == 21 || detection.id == 22 || detection.id == 23) {
                selectedTagId = detection.id;
                telemetry.addData("Selected ID", selectedTagId);

            }
        }
        return selectedTagId;
    }

    public int[] solveMotif(int selectedTagId){
        //ID# 21 = gpp
        //22 = pgp
        //23 = ppg
        int[] thismotif = new int[] {0};
        if(selectedTagId == 21){
            thismotif = new int[] {2,1,1};//gpp
        }else if(selectedTagId == 22){
            thismotif = new int[] {1,2,1};//pgp
        }else if(selectedTagId == 23){
            thismotif = new int[] {1,1,2};//ppg
        }

        return thismotif;
    }

    public void motorConfigs(){
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        RightFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftFlywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        SpindexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        RightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        RightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        LeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        LeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        SpindexerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        SpindexerMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        SpindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //SpindexerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        RightFlywheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        LeftFlywheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        RightFlywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        RightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public int incrementIndex(int index, int delta, int length){
        index += delta;

        if(index >= length){
            index -= length;
        }else if(index < 0){
            index += length;
        }
        return index;
    }

    public int findIndexSpindex(int target){
        for(int i = 0;i<6;i++){
            if(SpindexPos[i] == target){
                return i;
            }
        }
        return 0;
    }

    public int FindClosestOuttake() {
        CurrentSpindexerPos = SpindexerMotor.getCurrentPosition()%ticksPerRevolution;
        if(CurrentSpindexerPos<0) CurrentSpindexerPos+=ticksPerRevolution;

        if(CurrentSpindexerPos >= 0 && CurrentSpindexerPos < 1365){             //after 0, first outtake
            return 0;
        }else if(CurrentSpindexerPos >= 1365 && CurrentSpindexerPos < 2730){    //before 2730, second outtake
            return 2730;
        }else if(CurrentSpindexerPos >= 2730 && CurrentSpindexerPos < 4095){    //after 2730, second outtake
            return 2730;
        }else if(CurrentSpindexerPos >= 4095 && CurrentSpindexerPos < 5460){    //before 5460, third outtake
            return 5460;
        }else if(CurrentSpindexerPos >= 5460 && CurrentSpindexerPos < 6825){    //after 5460, third outtake
            return 5460;
        }else if(CurrentSpindexerPos >= 6825){                                  //before 0, first outtake
            return 0;
        }
        return 0;
    }

    public int FindClosestIntake() {
        CurrentSpindexerPos = SpindexerMotor.getCurrentPosition()%ticksPerRevolution;
        if(CurrentSpindexerPos<0) CurrentSpindexerPos+=ticksPerRevolution;

        if(CurrentSpindexerPos >= 0 && CurrentSpindexerPos < 1365) {           //after 0, first intake
            return 1365;
        }else if(CurrentSpindexerPos >= 1365 && CurrentSpindexerPos < 2730){   //before 2730, first intake
            return 1365;
        }else if(CurrentSpindexerPos >= 2730 && CurrentSpindexerPos < 4095){   //after 2730, second intake
            return 4095;
        }else if(CurrentSpindexerPos >= 4095 && CurrentSpindexerPos < 5460){   //before 5460, second intake
            return 4095;
        }else if(CurrentSpindexerPos >= 5460 && CurrentSpindexerPos < 6825){   //after 5460, third intake
            return 6825;
        }else if(CurrentSpindexerPos >= 6825){                                 //before 0, third intake
            return 6825;
        }
        return 1365;
    }

    public int unwrapTarget(int current, int wrappedTarget) {
        int rev = Math.floorDiv(current, ticksPerRevolution);//# of revolutions made by spindexer
        int best = rev * ticksPerRevolution + wrappedTarget;//find the closest unwrapped position

        int up = best + ticksPerRevolution;//check adjacent revolutions, one rotation up and one rotation down
        int down = best - ticksPerRevolution;

        if (Math.abs(up - current) < Math.abs(best - current)) best = up;
        if (Math.abs(down - current) < Math.abs(best - current)) best = down;

        return best;//returns the closest unwrapped target
    }

    public int SpindexGetAbsTarget(int wrappedTarget) {
        CurrentSpindexerPos = SpindexerMotor.getCurrentPosition();
        int absoluteTarget = unwrapTarget(CurrentSpindexerPos, wrappedTarget);
        return absoluteTarget;
    }

    public boolean detectColor(NormalizedColorSensor colorsensor){
        NormalizedRGBA colors = colorsensor.getNormalizedColors();
        double hue = JavaUtil.colorToHue(colors.toColor());
        if(hue > 90 && hue < 170){//green
            return true;
        }else if(hue > 200 && hue < 350){//purple
            return true;
        }
        return false;
    }

    public int findColor(NormalizedColorSensor colorsensor){
        NormalizedRGBA colors = colorsensor.getNormalizedColors();
        double hue = JavaUtil.colorToHue(colors.toColor());
        if(hue > 90 && hue < 170){//green
            return 2;
        }else if(hue > 200 && hue < 350){//purple
            return 1;
        }
        return 0;
    }
}
