package frc.robot.subsystems.vision;
import java.math.MathContext;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Dictionary;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.DriveSubsystem;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.databind.type.ArrayType;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
public class CameraSystem{

    public enum PoleSide{
        LEFT,
        RIGHT;
    }
    public PoleSide poleSide = PoleSide.LEFT;
    private final Map<Integer, Pose3d> fiducialMap = new HashMap<>();
    private ArrayList<PhotonCamera> cameras;
    private ArrayList<Transform3d> offsets;
    private ArrayList<PhotonPoseEstimator> estimators;
    private ArrayList<Boolean> hasAprilTagDetection;
    private String test = "test";
    public int lastTag = 0;
    private ArrayList<PhotonPipelineResult> lastestResults;
    //public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.valueOf("BucketFieldLayout.json").loadAprilTagLayoutField();
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    public SwerveDrivePoseEstimator swerveEst;
    public boolean isBlueSide;
    public boolean isBlue;
    public int focusCamIndex;
    //AprilTagFields.valueOf("BucketFieldLayout.json").loadAprilTagLayoutField();
    //new AprilTagFieldLayout("c:\\Documents/GitHub/2024-OffSeason-Juno/src/main/java/frc/robot/subsystems/vision/BucketFieldLayout.json");
    // AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public HashMap<Integer, ArrayList<Pose2d>> dictionary = new HashMap<>();
    private static CameraSystem instance;
    public SendableChooser<Boolean> side = new SendableChooser<>();

    private CameraSystem() {
        focusCamIndex = 0;
        double inchesToMeters = 0.0254;
        cameras = new ArrayList<PhotonCamera>();
        offsets  = new ArrayList<Transform3d>();
        estimators = new ArrayList<PhotonPoseEstimator>();
        hasAprilTagDetection = new ArrayList<Boolean>();
        lastestResults = new ArrayList<PhotonPipelineResult>();
        side.addOption("Blue", true);
        side.addOption("Red", false);
        SmartDashboard.putData(side);
        // Initialize fiducial map with Pose3d
        initializeFiducialMap(inchesToMeters);
        fillDictionary();
    }
    // creates a table for for each tag ID (key) and the poses required to line up with the left and right pole
    private void fillDictionary(){
        // Positiions for the red side of the field and their associated april tag
        ArrayList<Pose2d> poses6 = new ArrayList<Pose2d>();
        poses6.add(new Pose2d(new Translation2d(13.620, 2.810), new Rotation2d()));
        poses6.add(new Pose2d(new Translation2d(13.938, 3.008), new Rotation2d()));
        dictionary.put(6, poses6);
        ArrayList<Pose2d> poses7 = new ArrayList<Pose2d>();
        poses7.add(new Pose2d(new Translation2d(14.319, 3.915), new Rotation2d()));
        poses7.add(new Pose2d(new Translation2d(14.319, 4.169), new Rotation2d()));
        dictionary.put(7, poses7);
        ArrayList<Pose2d> poses8 = new ArrayList<Pose2d>();
        poses8.add(new Pose2d(new Translation2d(13.772, 5.002), new Rotation2d()));
        poses8.add(new Pose2d(new Translation2d(13.444, 5.314), new Rotation2d()));
        dictionary.put(8, poses8);
        ArrayList<Pose2d> poses9 = new ArrayList<Pose2d>();
        poses9.add(new Pose2d(new Translation2d(12.563, 5.170), new Rotation2d()));
        poses9.add(new Pose2d(new Translation2d(12.358, 5.072), new Rotation2d()));
        dictionary.put(9, poses9);
        ArrayList<Pose2d> poses10 = new ArrayList<Pose2d>();
        poses10.add(new Pose2d(new Translation2d(11.802, 4.135), new Rotation2d()));
        poses10.add(new Pose2d(new Translation2d(11.802, 3.823), new Rotation2d()));
        dictionary.put(10, poses10);
        ArrayList<Pose2d> poses11 = new ArrayList<Pose2d>();
        poses11.add(new Pose2d(new Translation2d(12.313, 2.904), new Rotation2d()));
        poses11.add(new Pose2d(new Translation2d(12.539, 2.752), new Rotation2d()));
        dictionary.put(11, poses11);
        // Positions for the blue side of the field and their associated april tag
        ArrayList<Pose2d> poses17 = new ArrayList<Pose2d>();
        poses17.add(new Pose2d(new Translation2d(3.760, 2.846), new Rotation2d()));
        poses17.add(new Pose2d(new Translation2d(4.016, 2.728), new Rotation2d()));
        dictionary.put(17, poses17);
        ArrayList<Pose2d> poses18 = new ArrayList<Pose2d>();
        poses18.add(new Pose2d(new Translation2d(3.131, 4.135), new Rotation2d()));
        poses18.add(new Pose2d(new Translation2d(3.131, 3.823), new Rotation2d()));
        dictionary.put(18, poses18);
        ArrayList<Pose2d> poses19 = new ArrayList<Pose2d>();
        poses19.add(new Pose2d(new Translation2d(3.872, 5.232), new Rotation2d()));
        poses19.add(new Pose2d(new Translation2d(3.564, 5.074), new Rotation2d()));
        dictionary.put(19, poses19);
        ArrayList<Pose2d> poses20 = new ArrayList<Pose2d>();
        poses20.add(new Pose2d(new Translation2d(5.249, 5.104), new Rotation2d()));
        poses20.add(new Pose2d(new Translation2d(4.989, 5.300), new Rotation2d()));
        dictionary.put(20, poses20);
        ArrayList<Pose2d> poses21 = new ArrayList<Pose2d>();
        poses21.add(new Pose2d(new Translation2d(5.813, 3.901), new Rotation2d()));
        poses21.add(new Pose2d(new Translation2d(5.813, 4.249), new Rotation2d()));
        dictionary.put(21, poses21);
        ArrayList<Pose2d> poses22 = new ArrayList<Pose2d>();
        poses22.add(new Pose2d(new Translation2d(4.957, 2.750), new Rotation2d()));
        poses22.add(new Pose2d(new Translation2d(5.406, 2.988), new Rotation2d()));
        dictionary.put(22, poses22);
    }
    // checks to see if the camera at the given position sees a tag
    public List<PhotonPipelineResult> getResult(int position){
        return cameras.get(position).getAllUnreadResults();
    }
    // Updates the results list to include only the latest piprline result and sets the last tag seen as the closest target instead of
    // the "best" target, made for Reefscape 2025 Season
    public void updateLatestResult(boolean buttonPressed)
    {
        int cameraCount = 0;
        for(PhotonCamera cam : cameras){
            List<PhotonPipelineResult> results = cam.getAllUnreadResults();
            if(!results.isEmpty()){
                lastestResults.set(cameraCount, results.get(results.size() - 1));
            }
            cameraCount++;
        }
        cameraCount = 0;
        if(lastestResults.get(0).hasTargets() && lastestResults.get(1).hasTargets())
        {
            //used to calculate if both cameras see the same tag on a side of the reef
            int highestSeenCount = 0;
            int seenCount = 0;
            // used for a back up case if the cameras don't see a common tag
            //Set to 100 to garuntee if there are targets that one will be closer
            double closestTarget = 100;
            if(!side.getSelected()){
                for(int i = 6; i <=11; i++){
                    for (PhotonPipelineResult result : lastestResults) {
                        for(PhotonTrackedTarget target : result.targets){
                            if(target.fiducialId == i){
                                seenCount++;
                            }
                        }
                        if(seenCount > highestSeenCount){
                            highestSeenCount = seenCount;
                            lastTag = i;
                        }
                        seenCount = 0;
                    }
                }
                if(highestSeenCount < 2){
                    for(PhotonPipelineResult result : lastestResults){
                        for (PhotonTrackedTarget target : result.getTargets()) {
                            if(target.fiducialId >= 6 && target.fiducialId <= 11 
                            && getTargetRange(cameraCount, target.fiducialId) != null 
                            && getTargetRange(cameraCount, target.fiducialId) < closestTarget){
                                closestTarget = getTargetRange(cameraCount, target.fiducialId);
                                lastTag = target.fiducialId;
                            }
                        }
                    }
                }
            }
            else{
                for(int i = 17; i <=22; i++){
                    for (PhotonPipelineResult result : lastestResults) {
                        for(PhotonTrackedTarget target : result.targets){
                            if(target.fiducialId == i){
                                seenCount++;
                            }
                        }
                        if(seenCount > highestSeenCount){
                            highestSeenCount = seenCount;
                            lastTag = i;
                        }
                        seenCount = 0;
                    }
                }
                if(highestSeenCount < 2){
                    for(PhotonPipelineResult result : lastestResults){
                        for (PhotonTrackedTarget target : result.getTargets()) {
                            if(target.fiducialId >= 17 && target.fiducialId <= 22 
                            && getTargetRange(cameraCount, target.fiducialId) != null 
                            && getTargetRange(cameraCount, target.fiducialId) < closestTarget){
                                closestTarget = getTargetRange(cameraCount, target.fiducialId);
                                lastTag = target.fiducialId;
                            }
                        }
                    }
                }
            }
            
        }
        
        
        // if(lastestResults.get(focusCamIndex).hasTargets() 
        // && ((lastestResults.get(focusCamIndex).getBestTarget().fiducialId >= 17 
        // && lastestResults.get(focusCamIndex).getBestTarget().fiducialId <= 22)
        // || (lastestResults.get(focusCamIndex).getBestTarget().fiducialId >= 6 
        // && lastestResults.get(focusCamIndex).getBestTarget().fiducialId <= 11))
        // && !buttonPressed)
        // {
        //     lastTag = lastestResults.get(0).getBestTarget().fiducialId;
        //     PhotonTrackedTarget closestTarget = null;
        //     for(PhotonTrackedTarget target : lastestResults.get(0).targets){
        //         if(closestTarget == null 
        //         && ((target.fiducialId >= 17 && target.fiducialId <= 22) || (target.fiducialId >= 6 && target.fiducialId <= 11))){
        //             for(PhotonTrackedTarget otherTarget : lastestResults.get(1).targets){
        //                 if(target.fiducialId == otherTarget.fiducialId){
        //                     closestTarget = target;
        //                 }
        //             }
                    
        //         }
        //         else{
        //             // Transform3d currTar = closestTarget.getBestCameraToTarget();
        //             // Transform3d newTar = target.getBestCameraToTarget();
        //             // if(Math.abs(currTar.getX()) > Math.abs(newTar.getX()) && Math.abs(currTar.getY()) > Math.abs(newTar.getY()) 
        //             // && ((target.fiducialId >= 17 && target.fiducialId <= 22) || (target.fiducialId >= 6 && target.fiducialId <= 11))){
        //             //     closestTarget = target;
        //             // } 
        //             if(closestTarget != null){
        //                 Double currAng = getYawForTag(focusCamIndex, closestTarget.fiducialId);
        //                 Double tarAng = getYawForTag(focusCamIndex, target.fiducialId);
        //                 if(currAng != null && tarAng != null && Math.abs(currAng) > Math.abs(tarAng) 
        //                 && ((target.fiducialId >= 17 && target.fiducialId <= 22) || (target.fiducialId >= 6 && target.fiducialId <= 11)))
        //                 {
        //                     for(PhotonTrackedTarget otherTarget : lastestResults.get(1).targets){
        //                         if(target.fiducialId == otherTarget.fiducialId){
        //                             closestTarget = target;
        //                         }
        //                     }
        //                 } 
        //             }
        //         }
        //     }
        //     if(closestTarget != null){
        //         lastTag = closestTarget.fiducialId;
        //     }
        //}
    }
    // updating to get all the latest results
    public PhotonCamera getCamera(int position)
    {
        return cameras.get(position);
    }
    public boolean CameraHasAprilTagDetection(int position){
        return hasAprilTagDetection.get(position);
    }
    // adds the camera, offset, and estimator to their arraylists; each camera, offset, and estimator have the same position in each arraylist
    public void AddCamera(PhotonCamera camera, Transform3d offset, boolean hasAprilTagDetection){
        cameras.add(camera);
        offsets.add(offset);
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        lastestResults.add(results.get(results.size() -1));
        estimators.add(new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, offset));
        estimators.get(estimators.size() - 1).setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        this.hasAprilTagDetection.add(hasAprilTagDetection);
    }
    public void AddVisionMeasurements(SwerveDrivePoseEstimator swerveEstimator, Field2d visionField)
    {
        swerveEst = swerveEstimator;
        int cameraCount = 0;
        for (var estimator : estimators) 
        {
            if(CameraHasAprilTagDetection(cameraCount))
            {
                Optional<EstimatedRobotPose> estimatedPose = estimator.update(lastestResults.get(cameraCount));
                Matrix<N3,N1> camStdDev = getEstimationStdDevs(estimatedPose, lastestResults.get(cameraCount).getTargets());
                if(!estimatedPose.isEmpty() && camStdDev != null)
                {
                    visionField.setRobotPose(estimatedPose.get().estimatedPose.toPose2d());
                    Pose2d camRobotPose = estimatedPose.get().estimatedPose.toPose2d();
                    double timestamp = estimatedPose.get().timestampSeconds;
                    swerveEstimator.addVisionMeasurement(camRobotPose, timestamp, camStdDev);
                }
            }
            cameraCount++;
        }
    } 
    // private Matrix<N3, N1> confidenceCalculator(int position) 
    // {
    //     Optional<EstimatedRobotPose> estimationPose = usePoseEstimator(position, null);
    //     if(!estimationPose.isEmpty())
    //     {
    //         EstimatedRobotPose estimation = estimationPose.get();
    //         double smallestDistance = Double.POSITIVE_INFINITY;
    //         for (var target : estimation.targetsUsed)  
    //         {
    //             var t3d = target.getBestCameraToTarget();
    //             double distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
    //             if (distance < smallestDistance)
    //                 smallestDistance = distance;
    //         }
    //         double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
    //         ? 1
    //         : Math.max(
    //             1,
    //             (estimation.targetsUsed.get(0).getPoseAmbiguity()
    //                 + Constants.VisionConstants.POSE_AMBIGUITY_SHIFTER)
    //                 * Constants.VisionConstants.POSE_AMBIGUITY_MULTIPLIER);
    //         double confidenceMultiplier = Math.max(
    //         1,
    //         (Math.max(
    //             1,
    //             Math.max(0, smallestDistance - Constants.VisionConstants.NOISY_DISTANCE_METERS)
    //             * Constants.VisionConstants.DISTANCE_WEIGHT) * poseAmbiguityFactor)
    //             / (1 + ((estimation.targetsUsed.size() - 1) * Constants.VisionConstants.TAG_PRESENCE_WEIGHT)));

    //         return Constants.VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    //     }
    //     return null;
    // }
    private Matrix<N3, N1> getEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) 
    {
        Matrix<N3, N1> curStdDevs;
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = Constants.VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = Constants.VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
        return curStdDevs;
    }
    // calculates robot position
     public Pose2d calculateRobotPosition() {
        int cameraCount = 0;
        int cameraTagCount = 0;
        double sumX = 0;
        double sumY = 0;
        
        double rotationSumx = 0;
        double rotationSumY = 0;
        double rotationSumZ = 0;
        
        // Goes through every camera in the camera array list and checks if it sees a tag
        for(PhotonCamera cam : cameras)
        {
            if(cam.getLatestResult().hasTargets() && CameraHasAprilTagDetection(cameraCount))
            {
                // if the camera picks up a tag, it calculates the position from the tag and runs it through a pose estimator
                Pose3d orig = calculatePoseFromCameraResult(cam.getLatestResult(), offsets.get(cameraCount));
                if(orig != null){
                    Optional<EstimatedRobotPose> estimatedPose = usePoseEstimator(cameraCount, orig.toPose2d());
                    if(estimatedPose != null && !estimatedPose.isEmpty()){
                        Pose3d temp = estimatedPose.get().estimatedPose;
                        // add the components of the pose 3d to the sums
                        sumX += temp.getX();
                        sumY += temp.getY();
                        rotationSumx += temp.getRotation().getX();
                        rotationSumY += temp.getRotation().getY();
                        rotationSumZ += temp.getRotation().getZ();              
                        cameraTagCount++;     
                    }
                }
            }
            cameraCount++;
        
        }
        if(cameraTagCount > 0)
        {
        // Average X, Y, and Z from camera results
        double avgX = sumX / cameraTagCount;
        double avgY = sumY / cameraTagCount;
        // We can ignore Z for Pose2d
        Rotation3d avgRotation = new Rotation3d(
            (double)rotationSumx / cameraTagCount,
            (double)rotationSumY / cameraTagCount,
            (double)rotationSumZ/ cameraTagCount
        );

        // Convert the 3D Pose to 2D Pose
        return new Pose2d(avgX, avgY, new Rotation2d(avgRotation.getZ()));
        }
        // if no cameras detect tags, return blank Pose2d
        return new Pose2d();
        
    }
    // runs the orginal pose and puts it through it corresponding estimator
    private Optional<EstimatedRobotPose> usePoseEstimator(int position, Pose2d prevPose){
        if(prevPose != null){
            estimators.get(position).setReferencePose(prevPose);
        }
        return estimators.get(position).update(lastestResults.get(position));
    }
    // calculates the postition of tag to robot from one camera's results 
    private Pose3d calculatePoseFromCameraResult(PhotonPipelineResult result, Transform3d cameraOffset) {
        if (result != null && result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            Optional<MultiTargetPNPResult> pnpResult = result.getMultiTagResult();
            
                
            // gets the position of the april tag scanned
            //Pose3d fiducialPose = fiducialMap.get(target.getFiducialId());

            if(!pnpResult.isEmpty())
            {
                Transform3d fieldToCamera = pnpResult.get().estimatedPose.best;
                Pose3d cameraToTargetPose = new Pose3d().transformBy(fieldToCamera);
                Pose3d robotPose3d = cameraToTargetPose.transformBy(cameraOffset);
                return new Pose3d(
                    robotPose3d.getX(),
                    robotPose3d.getY(),
                    robotPose3d.getZ(),
                    robotPose3d.getRotation()
                );

            }

            // if (fiducialPose != null) {
            //     // calcuates the april tag position to the camera
            //     Transform3d transform = target.getBestCameraToTarget().inverse();
            //     Pose3d cameraToTargetPose = fiducialPose.transformBy(transform);
            //     // finds the pose of the robot from the camera position
            //     Pose3d robotPose3d = cameraToTargetPose.transformBy(cameraOffset);
            //     return new Pose3d(
            //         robotPose3d.getX(),
            //         robotPose3d.getY(),
            //         robotPose3d.getZ(),
            //         robotPose3d.getRotation()
            //     );
            // }
        }
        return null;
    }
    // returns the yaw required to be perpendicular to a tag to make it easy to score
    public double getPerpendicularYaw(){
        double desiredDegrees = 
        aprilTagFieldLayout.getTagPose(lastTag).get().getRotation().toRotation2d().getDegrees();
        double currentDegrees = swerveEst.getEstimatedPosition().getRotation().getDegrees();
        // some tags have a angle smaller than -90, so in those cases you need to add 90 to be perpendicular instead of subtract, and
        // this is because the odemetry is between (180, -180) degrees, so subtracting 90 is an impossible angle to reach
        if(desiredDegrees < -90){
            desiredDegrees += 270;
        }
        else{
            desiredDegrees -= 90;
        }
        
      
        // else{
        //     desiredDegrees += 90;
        // }
        return desiredDegrees - currentDegrees;
    }
    // calculates and returns the difference between the x and y values of the current and desired position to travel to
    public ArrayList<Double> getPoseToTravel(int leftOrRight){
        Pose2d desPose = null;
        Pose2d curPose = swerveEst.getEstimatedPosition();
        if(lastTag != 0){
            desPose = dictionary.get(lastTag).get(leftOrRight);
        }
        else{
            desPose = curPose;
        }
        ArrayList<Double> powers = new ArrayList<>();
        powers.add((isBlueSide ? 1 : 1) *(desPose.getX() - curPose.getX()));
        powers.add((isBlueSide ? 1 : 1) *(desPose.getY() - curPose.getY()));
        return powers;
    }
    public static CameraSystem getInstance() {
        if (instance == null) {
            instance = new CameraSystem();
        }
        return instance;
    }
    
    // checks to see if the result sees any april tags
    public boolean hasTargets(){
        
       for(PhotonCamera cam : cameras)
        {
            if(cam.getLatestResult().hasTargets()){
                return true;
            } 
        }
        return false;
    }
    
    public boolean hasDesiredTarget(int position, int ID){
        for(PhotonTrackedTarget target : lastestResults.get(position).targets){
            if(target.fiducialId == ID){
                return true;
            }
        }
        return false;
    }
    public double getTimeStamp() 
    {
        int cameraCount = 0;
        for (PhotonCamera cam : cameras){
            if(cam.getLatestResult().hasTargets()){
                Pose3d orig = calculatePoseFromCameraResult(cam.getLatestResult(), offsets.get(cameraCount));
                if(orig != null){
                    Optional<EstimatedRobotPose> estimatedPose = usePoseEstimator(cameraCount, orig.toPose2d());
                    if(estimatedPose != null && !estimatedPose.isEmpty())
                        return estimatedPose.get().timestampSeconds;
                }
                
            }
            cameraCount++;
        }
        return -1;
    }
    // returns a Double Object, so need check if it is null
    public Double getYawForTag(int position, int ID){
            if(lastestResults.get(position).hasTargets())
            {
                List<PhotonTrackedTarget> targets = lastestResults.get(position).getTargets();
                for(var target : targets)
                {
                    if(target != null && target.getFiducialId() == ID)
                    {
                        return target.getYaw();
                    }
                }
            } 
            // else if(getResult(position).hasTargets() && getResult(position).getBestTarget().getFiducialId() == 3){
            //     List<PhotonTrackedTarget> targets = getResult(position).getTargets();
            //     for(PhotonTrackedTarget target : targets){
            //         if(target.getFiducialId() == 4){
            //             return target.getYaw();
            //         }
            //     }
            // }
            return null;
    } 
    public Double getTargetRange(int position, int ID){
        Double targetRange = null;
        // if(getResult(position).hasTargets() && getResult(position).getBestTarget().getFiducialId() == 4){
        //     targetRange = PhotonUtils.calculateDistanceToTargetMeters(-offsets.get(position).getZ(), 57.13 * 0.0254, -offsets.get(position).getRotation().getY(), Units.degreesToRadians(getResult(position).getBestTarget().getPitch()));
        // }
        // else if(getResult(position).hasTargets() && getResult(position).getBestTarget().getFiducialId() == 3){
        //     List<PhotonTrackedTarget> targets = getResult(position).getTargets();
        //         for(PhotonTrackedTarget target : targets){
        //             if(target.getFiducialId() == 4){
        //                 targetRange = PhotonUtils.calculateDistanceToTargetMeters(-offsets.get(position).getZ(), 57.13 * 0.0254, -offsets.get(position).getRotation().getY(), Units.degreesToRadians(getResult(position).getBestTarget().getPitch()));
        //             }
        //         }
        // }
        
            List<PhotonTrackedTarget> targets = lastestResults.get(position).getTargets();
            for(PhotonTrackedTarget target : targets)
            {
               if(target.getFiducialId() == ID)
               {
                    targetRange = PhotonUtils.calculateDistanceToTargetMeters(offsets.get(position).getZ(), 
                    aprilTagFieldLayout.getTagPose(ID).get().getZ(), 
                    offsets.get(position).getRotation().getY(), 
                    Units.degreesToRadians(target.getPitch()));
                } 
            }

        return targetRange;
    }
    public double getPitch(int position, int ID){
        List<PhotonTrackedTarget> targets = lastestResults.get(position).getTargets();
            for(PhotonTrackedTarget target : targets)
            {
               if(target.getFiducialId() == ID)
               {
                    return Units.degreesToRadians(target.getPitch());
                } 
            }
        return -1;
    }
    public void ChangeCamOffset(double encoderVal)
    {
        double camHeight = (encoderVal * -0.570855) + 19.67;
        double camLength = (encoderVal * -0.000839631) + 0.11;
        offsets.set(0, new Transform3d(new Translation3d(camLength,0.0,camHeight), 
        new Rotation3d(0,Math.toRadians(15),0)));
        estimators.get(0).setRobotToCameraTransform(
            new Transform3d(new Translation3d(camLength,0.0,camHeight), new Rotation3d(0,Math.toRadians(15),0)));
    }
    // Field coordinates for the april tags (converting inches to meters)
    private void initializeFiducialMap(double inchesToMeters) {
        fiducialMap.put(1, new Pose3d(593.68 * inchesToMeters, 9.68 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(120))));
        fiducialMap.put(2, new Pose3d(637.21 * inchesToMeters, 34.79 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(120))));
        fiducialMap.put(3, new Pose3d(652.73 * inchesToMeters, 196.17 * inchesToMeters, 57.13 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(180))));
        fiducialMap.put(4, new Pose3d(652.73 * inchesToMeters, 218.42 * inchesToMeters, 57.13 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(180))));
        fiducialMap.put(5, new Pose3d(578.77 * inchesToMeters, 323.00 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(270))));
        fiducialMap.put(6, new Pose3d(72.50 * inchesToMeters, 323.00 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(270))));
        fiducialMap.put(7, new Pose3d(-1.50 * inchesToMeters, 218.42 * inchesToMeters, 57.13 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(0))));
        fiducialMap.put(8, new Pose3d(-1.50 * inchesToMeters, 196.17 * inchesToMeters, 57.13 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(0))));
        fiducialMap.put(9, new Pose3d(14.02 * inchesToMeters, 34.79 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(60))));
        fiducialMap.put(10, new Pose3d(57.54 * inchesToMeters, 9.68 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(60))));
        fiducialMap.put(11, new Pose3d(468.69 * inchesToMeters, 146.19 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(300))));
        fiducialMap.put(12, new Pose3d(468.69 * inchesToMeters, 177.10 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(60))));
        fiducialMap.put(13, new Pose3d(441.74 * inchesToMeters, 161.62 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(180))));
        fiducialMap.put(14, new Pose3d(209.48 * inchesToMeters, 161.62 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(0))));
        fiducialMap.put(15, new Pose3d(182.73 * inchesToMeters, 177.10 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(120))));
        fiducialMap.put(16, new Pose3d(182.73 * inchesToMeters, 146.19 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(240))));
    }
}