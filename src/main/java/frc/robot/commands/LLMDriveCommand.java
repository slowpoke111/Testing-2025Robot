// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Base64;
import java.awt.Desktop.Action;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import com.ctre.phoenix6.swerve.SwerveRequest;

/** Command that captures an image and sends it to an LLM service for processing */
public class LLMDriveCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
    private final CommandSwerveDrivetrain m_Swerve;
    private final VisionSubsystem m_Vision;
    private String llmResponse = "";
    private String sessionId = null; // Session ID for memory persistence

    // State tracking
    private boolean requestInProgress = false;
    private boolean requestSuccessful = false;
    private boolean requestFailed = false;
    private boolean actionExecuted = false; // Flag to track if we've executed the action
    private double requestStartTime = 0;
    private double actionStartTime = 0;  // Track when the action started
    private final double REQUEST_TIMEOUT = 25.0; // 25 seconds timeout
    private final double ACTION_DURATION = 2.0;  // Run actions for 2 seconds

    // For async processing
    private ExecutorService executor;
    private Future<?> currentTask;

    // Add flag to track if this is the first execution
    private boolean firstExecution = true;

    public LLMDriveCommand(CommandSwerveDrivetrain swerve, VisionSubsystem vision) {
        m_Swerve = swerve;
        m_Vision = vision;
        addRequirements(m_Swerve, m_Vision);
        // Create thread pool for async operations
        executor = Executors.newSingleThreadExecutor();
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Host_IP", SmartDashboard.getString("Host_IP", "127.0.0.1:5000"));
        SmartDashboard.putString("LLM_Status", "Initializing");
        SmartDashboard.putString("LLM_Response", "");
        
        // Display session info
        if (sessionId != null) {
            SmartDashboard.putString("Session_ID", sessionId);
        }

        // Reset state
        requestInProgress = false;
        requestSuccessful = false;
        requestFailed = false;
        actionExecuted = false;
        llmResponse = "";

        // Start the API request immediately
        requestStartTime = Timer.getFPGATimestamp();
        requestInProgress = true;
        SmartDashboard.putString("LLM_Status", "Processing request...");

        // Submit the task using the extracted method
        currentTask = executor.submit(this::sendImageProcessingRequest);
    }

    @Override
    public void execute() {
        // Show status in SmartDashboard
        SmartDashboard.putBoolean("LLM_Active", true);

        // If we're processing a request, check its status
        if (requestInProgress) {
            // Check for timeout
            if (Timer.getFPGATimestamp() - requestStartTime > REQUEST_TIMEOUT) {
                SmartDashboard.putString("LLM_Status", "Timeout");
                cancelCurrentRequest();
                requestInProgress = false;
                requestFailed = true;
                return;
            }

            // If task is done, update status
            if (currentTask != null && currentTask.isDone()) {
                if (requestSuccessful) {
                    SmartDashboard.putString("LLM_Status", "Success");
                    SmartDashboard.putString("LLM_Response", llmResponse);
                } else if (requestFailed) {
                    SmartDashboard.putString("LLM_Status", "Failed");
                }
                requestInProgress = false;
            }
        }

        // Only execute the action once when we have a successful response and haven't executed it yet
        if (requestSuccessful && !actionExecuted && !llmResponse.isEmpty()) {
            executeAction();
            actionExecuted = true; // Mark that we've executed the action
            actionStartTime = Timer.getFPGATimestamp(); // Record when action started
        }

        // Stop movement after the specified duration
        if (actionExecuted && Timer.getFPGATimestamp() - actionStartTime > ACTION_DURATION) {
            m_Swerve.setControl(idle); // Stop the movement
        }
    }

    private void executeAction() {
        // Extract json property from llm response
        JSONParser parser = new JSONParser();
        String action = "none";
        try {
            JSONObject jsonResponse = (JSONObject) parser.parse(llmResponse);
            action = (String) jsonResponse.get("action");
            
            // Capture session_id for future requests (memory persistence)
            if (jsonResponse.get("session_id") != null) {
                sessionId = (String) jsonResponse.get("session_id");
                SmartDashboard.putString("Session_ID", sessionId);
            }
            
            // Display environment description if available
            if (jsonResponse.get("environment_description") != null) {
                String envDesc = (String) jsonResponse.get("environment_description");
                SmartDashboard.putString("Environment", envDesc);
            }
            
            SmartDashboard.putString("Action", action);
        } catch (Exception e) {
            System.err.println("Error parsing LLM response: " + e.getMessage());
            action = "error"; 
            e.printStackTrace();
        }

        // Follow action from llm response
        switch (action) {
            case "forward":
                m_Swerve.setControl(drive.withVelocityX(3));
                break;
            case "back":
                m_Swerve.setControl(drive.withVelocityX(-3));
                break;
            case "rotate_left":
                m_Swerve.setControl(drive.withRotationalRate(-0.5));
                break;
            case "rotate_right":
                m_Swerve.setControl(drive.withRotationalRate(0.5));
                break;
            default:
                System.out.println("Unknown action: " + action);
                break;
        }
    }

    private void sendImageProcessingRequest() {
        try {
            // Get host IP from SmartDashboard
            String hostIp = SmartDashboard.getString("Host_IP", "127.0.0.1:5000");
            String endpoint = "http://" + hostIp + "/process_image";

            // Get image and check if we got valid data
            byte[] imageData = m_Vision.getImageBytes();
            if (imageData == null || imageData.length == 0) {
                System.err.println("No image data available");
                requestFailed = true;
                return;
            }

            String base64Image = Base64.getEncoder().encodeToString(imageData);

            // Configure connection with proper timeouts
            URL url = new URL(endpoint);
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("POST");
            connection.setRequestProperty("Content-Type", "application/json");
            connection.setDoOutput(true);
            connection.setConnectTimeout(10000); // 10 second connect timeout
            connection.setReadTimeout(15000);   // 15 second read timeout

            // Create JSON payload with memory support
            JSONObject payload = new JSONObject();
            payload.put("image", base64Image);
            
            // Include session_id for memory persistence if we have one
            if (sessionId != null && !sessionId.isEmpty()) {
                payload.put("session_id", sessionId);
            }
            
            String jsonInputString = payload.toJSONString();

            // Send the request
            try (OutputStream os = connection.getOutputStream()) {
                byte[] input = jsonInputString.getBytes("utf-8");
                os.write(input, 0, input.length);
            }

            // Check response code before reading response
            int responseCode = connection.getResponseCode();
            if (responseCode != HttpURLConnection.HTTP_OK) {
                System.err.println("HTTP Error: " + responseCode);
                System.err.println("Error message: " + connection.getResponseMessage());
                requestFailed = true;
                return;
            }

            // Read the response
            StringBuilder response = new StringBuilder();
            try (BufferedReader br = new BufferedReader(new InputStreamReader(connection.getInputStream(), "utf-8"))) {
                String responseLine = null;
                while ((responseLine = br.readLine()) != null) {
                    response.append(responseLine.trim());
                }
            }

            // Store the result
            llmResponse = response.toString();
            System.out.println("API Response: " + llmResponse);
            requestSuccessful = true;

        } catch (Exception e) {
            System.err.println("Error in request: " + e.getMessage());
            e.printStackTrace();
            requestFailed = true;
        }
    }

    private void cancelCurrentRequest() {
        if (currentTask != null && !currentTask.isDone()) {
            currentTask.cancel(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Clean up executor service
        cancelCurrentRequest();
        // Don't shut down the executor as it might be reused
        if (interrupted) {
            executor.shutdown();
            executor = Executors.newSingleThreadExecutor();
        }
        SmartDashboard.putBoolean("LLM_Active", false);
    }

    @Override
    public boolean isFinished() {
        if (requestFailed) {
            return true;
        }
        if (actionExecuted && Timer.getFPGATimestamp() - actionStartTime > ACTION_DURATION) {
            return true; 
        }
        return false; 
    }
}
