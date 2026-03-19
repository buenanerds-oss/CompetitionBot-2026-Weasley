package frc.robot.SubSystem.Vision;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {

    /**
     * 
     * @return - the latest results from all cameras
     */
    public default PhotonPipelineResult[] getLatestResults() {
        return new PhotonPipelineResult[0];
    }

    /**
     * if the returned pose is  = new Pose3d(), don't use it that is a uncomfirmed pose
     * @return
     */
    public default Optional<Pose3d>[] getposeEstimations() {
        return new Optional[0];
    }

    public default Optional<List<PhotonTrackedTarget>>[] getTargets() {
        return new Optional[0];
    }

    public default void periodic() {

    }
    
}