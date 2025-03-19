// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

// align
public class CameraDeltaFeedback extends Command {
    
    // List of cameras to check
    private final List<PhotonCamera> m_cameras = new ArrayList<>();
    
    // Reef tag IDs by alliance
    private final int[] m_blueReefTags = {17, 18, 19, 20, 21, 22};
    private final int[] m_redReefTags = {6, 7, 8, 9, 10, 11};
    
    // Desired offsets for scoring
    private final double m_targetYaw = 0.0; // 0 = centered
    private final double m_targetDistance = 0.8; // meters
    private final String m_targetSide; // "left", "right", or "center"
    
    // Target side offset in degrees (how far to the side we want to be)
    private double m_sideOffset = 0.0;
    
    /**
     * Creates a new CameraDeltaFeedback command.
     * 
     * @param side Which side of the tag to align to ("left", "right", or "center")
     */
    public CameraDeltaFeedback(String side) {
        m_targetSide = side;
        
        // Set the side offset based on the desired side
        if (side.equalsIgnoreCase("left")) {
            m_sideOffset = 15.0; // 15 degrees to the left
        } else if (side.equalsIgnoreCase("right")) {
            m_sideOffset = -15.0; // 15 degrees to the right
        }
        
        // Initialize cameras
        m_cameras.add(new PhotonCamera(VisionConstants.camera0Name));
        m_cameras.add(new PhotonCamera(VisionConstants.camera1Name));
        m_cameras.add(new PhotonCamera(VisionConstants.camera2Name));
        m_cameras.add(new PhotonCamera(VisionConstants.camera3Name));
    }
    
    @Override
    public void initialize() {
        SmartDashboard.putString("Align/Status", "Looking for reef tags...");
        SmartDashboard.putString("Align/Side", m_targetSide);
    }
    
    @Override
    public void execute() {
        // Get alliance color
        boolean isBlueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) 
            == DriverStation.Alliance.Blue;
        
        // Get the appropriate reef tags for our alliance
        int[] reefTags = isBlueAlliance ? m_blueReefTags : m_redReefTags;
        
        // Variables to track best reef tag detection
        PhotonTrackedTarget bestReefTarget = null;
        PhotonCamera bestCamera = null;
        double bestScore = Double.MAX_VALUE; // Lower is better
        
        // Check all cameras for reef tags
        for (PhotonCamera camera : m_cameras) {
            try {
                var result = camera.getLatestResult();
                
                if (!result.hasTargets()) {
                    continue;
                }
                
                for (PhotonTrackedTarget target : result.getTargets()) {
                    int tagId = target.getFiducialId();
                    
                    // Check if this is a reef tag for our alliance
                    boolean isReefTag = false;
                    for (int reefTagId : reefTags) {
                        if (tagId == reefTagId) {
                            isReefTag = true;
                            break;
                        }
                    }
                    
                    if (!isReefTag) {
                        continue;
                    }
                    
                    // Calculate score based on yaw (prefer more centered targets)
                    double score = Math.abs(target.getYaw());
                    
                    // Update best tag if this one is better
                    if (bestReefTarget == null || score < bestScore) {
                        bestReefTarget = target;
                        bestCamera = camera;
                        bestScore = score;
                    }
                }
            } catch (Exception e) {
                // Skip camera if theres an error
                continue;
            }
        }
        
        // Process the best reef tag if found
        if (bestReefTarget != null) {
            int tagId = bestReefTarget.getFiducialId();
            double yaw = bestReefTarget.getYaw();
            double distance = 0.0;
            
            // Calculate the desired yaw (center + side offset)
            double desiredYaw = m_targetYaw + m_sideOffset;
            
            // Calculate the difference between current and desired yaw
            double yawDelta = desiredYaw - yaw;
            
            try {
                // Try to get 3D pose data for distance
                var transform = bestReefTarget.getBestCameraToTarget();
                distance = transform.getX(); // X is forward distance in camera space
                
                // Calculate distance error (positive means too far, negative means too close)
                double distanceDelta = distance - m_targetDistance;
                
                // Display all the raw values
                SmartDashboard.putNumber("Align/Yaw", yaw);
                SmartDashboard.putNumber("Align/DesiredYaw", desiredYaw);
                SmartDashboard.putNumber("Align/YawDelta", yawDelta);
                SmartDashboard.putNumber("Align/Distance", distance);
                SmartDashboard.putNumber("Align/DistanceDelta", distanceDelta);
                
                // Display tag info
                SmartDashboard.putNumber("Align/TagID", tagId);
                SmartDashboard.putString("Align/Camera", bestCamera.getName());
                
                // Simple alignment status
                boolean yawAligned = Math.abs(yawDelta) < 3.0; // Within 3 degrees
                boolean distanceAligned = Math.abs(distanceDelta) < 0.1; // Within 10cm
                
                // Create simple driver instructions
                String yawDirection;
                if (yawDelta > 3.0) {
                    yawDirection = "TURN LEFT";
                } else if (yawDelta < -3.0) {
                    yawDirection = "TURN RIGHT";
                } else {
                    yawDirection = "YAW GOOD";
                }
                
                String distanceDirection;
                if (distanceDelta > 0.1) {
                    distanceDirection = "MOVE BACK";
                } else if (distanceDelta < -0.1) {
                    distanceDirection = "MOVE FORWARD";
                } else {
                    distanceDirection = "DISTANCE GOOD";
                }
                
                // Update dashboard with driver instructions
                SmartDashboard.putString("Align/YawDirection", yawDirection);
                SmartDashboard.putString("Align/DistanceDirection", distanceDirection);
                
                // Overall status
                if (yawAligned && distanceAligned) {
                    SmartDashboard.putString("Align/Status", "ALIGNED TO TAG " + tagId);
                    SmartDashboard.putBoolean("Align/Ready", true);
                } else {
                    SmartDashboard.putString("Align/Status", "ALIGNING TO TAG " + tagId);
                    SmartDashboard.putBoolean("Align/Ready", false);
                }
                
            } catch (Exception e) {
                // If we can't get distance, just show yaw information
                SmartDashboard.putNumber("Align/Yaw", yaw);
                SmartDashboard.putNumber("Align/DesiredYaw", desiredYaw);
                SmartDashboard.putNumber("Align/YawDelta", yawDelta);
                SmartDashboard.putString("Align/Status", "TAG " + tagId + " DETECTED (NO DISTANCE)");
                
                // Simple yaw instruction
                String yawDirection;
                if (yawDelta > 3.0) {
                    yawDirection = "TURN LEFT";
                } else if (yawDelta < -3.0) {
                    yawDirection = "TURN RIGHT";
                } else {
                    yawDirection = "YAW GOOD";
                }
                
                SmartDashboard.putString("Align/YawDirection", yawDirection);
                SmartDashboard.putString("Align/DistanceDirection", "NO DISTANCE DATA");
                SmartDashboard.putBoolean("Align/Ready", false);
            }
            
        } else {
            // No reef tag detected
            SmartDashboard.putString("Align/Status", "NO REEF TAG DETECTED");
            SmartDashboard.putString("Align/YawDirection", "");
            SmartDashboard.putString("Align/DistanceDirection", "");
            SmartDashboard.putBoolean("Align/Ready", false);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Clear the status when we stop
        SmartDashboard.putString("Align/Status", "ALIGNMENT TOOL STOPPED");
        SmartDashboard.putBoolean("Align/Ready", false);
    }
    
    @Override
    public boolean isFinished() {
        // Run until interrupted
        return false;
    }
}