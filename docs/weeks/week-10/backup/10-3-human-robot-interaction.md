---
sidebar_label: Human-Robot Interaction
title: Human-Robot Interaction - Collaborative Systems and Communication
description: Understanding human-robot interaction including collaborative systems, communication protocols, and social robotics
keywords: [HRI, human-robot interaction, collaborative robotics, social robotics, communication, multimodal interaction, safety]
---

# 10.3 Human-Robot Interaction

## Introduction

Human-Robot Interaction (HRI) is a critical component of Physical AI systems that operate in human-centered environments. As robots become increasingly integrated into our daily lives, workplaces, and social spaces, the ability to interact naturally and safely with humans becomes paramount. HRI encompasses the design, development, and evaluation of robots that can perceive, interpret, and respond appropriately to human behavior, intentions, and communication.

Effective HRI systems must understand human social cues, communicate their own intentions clearly, and adapt their behavior to different users and contexts. This requires sophisticated perception systems to interpret human actions, advanced AI to understand human intentions, and careful design to ensure safety and trust. The field draws from multiple disciplines including robotics, cognitive science, psychology, and human-computer interaction.

This chapter explores the theoretical foundations of HRI, practical implementation techniques, safety considerations, and the design principles that enable effective human-robot collaboration. We'll examine both technical aspects of interaction and the social dynamics that govern successful human-robot partnerships.

## Theoretical Foundations of HRI

### Social Robotics Principles

#### Proxemics and Spatial Behavior
Proxemics, the study of human spatial behavior, is fundamental to designing robots that respect human comfort zones and social conventions:

##### Edward T. Hall's Proxemic Zones
- **Intimate Distance** (0-45cm): Reserved for close relationships, rarely appropriate for robots
- **Personal Distance** (45-120cm): Normal distance for conversations with friends
- **Social Distance** (120-360cm): Appropriate for formal interactions and group settings
- **Public Distance** (360cm+): Used for public speaking and formal presentations

##### Robot Implementation
```cpp
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ProxemicManager {
private:
    double intimate_distance_;
    double personal_distance_;
    double social_distance_;
    double public_distance_;
    
    // Current human-robot distance
    double current_distance_;
    
    // Robot behavior state based on proxemic zones
    ProxemicState current_state_;
    
    // Safety margins
    double safety_buffer_;

public:
    ProxemicManager(double intimate = 0.4, double personal = 0.8, 
                   double social = 2.0, double public = 4.0, 
                   double buffer = 0.2)
        : intimate_distance_(intimate), personal_distance_(personal),
          social_distance_(social), public_distance_(public),
          safety_buffer_(buffer), current_distance_(0.0),
          current_state_(ProxemicState::PUBLIC) {}
    
    ProxemicState evaluateProxemicZone(const geometry_msgs::msg::Point& human_pos,
                                     const geometry_msgs::msg::Point& robot_pos) {
        
        // Calculate distance between human and robot
        double dx = human_pos.x - robot_pos.x;
        double dy = human_pos.y - robot_pos.y;
        double dz = human_pos.z - robot_pos.z;
        
        current_distance_ = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        // Determine which proxemic zone the robot is in relative to human
        if (current_distance_ <= intimate_distance_ - safety_buffer_) {
            return ProxemicState::INTIMATE;
        } else if (current_distance_ <= personal_distance_ - safety_buffer_) {
            return ProxemicState::PERSONAL;
        } else if (current_distance_ <= social_distance_ - safety_buffer_) {
            return ProxemicState::SOCIAL;
        } else {
            return ProxemicState::PUBLIC;
        }
    }
    
    double getRecommendedDistance(ProxemicState desired_state) const {
        switch (desired_state) {
            case ProxemicState::INTIMATE:
                return intimate_distance_ + safety_buffer_;
            case ProxemicState::PERSONAL:
                return personal_distance_ + safety_buffer_;
            case ProxemicState::SOCIAL:
                return social_distance_ + safety_buffer_;
            case ProxemicState::PUBLIC:
                return public_distance_ + safety_buffer_;
            default:
                return social_distance_ + safety_buffer_;  // Default to social distance
        }
    }
    
    bool shouldMaintainDistance(const geometry_msgs::msg::Point& human_pos,
                              const geometry_msgs::msg::Point& robot_pos,
                              ProxemicState target_state) {
        
        double target_dist = getRecommendedDistance(target_state);
        double current_dist = calculateDistance(human_pos, robot_pos);
        
        // Return true if current distance is too close to human
        return current_dist < target_dist - safety_buffer_;
    }
    
    geometry_msgs::msg::Point calculateSafePosition(
        const geometry_msgs::msg::Point& human_pos,
        const geometry_msgs::msg::Point& robot_pos,
        ProxemicState target_state) {
        
        double target_dist = getRecommendedDistance(target_state);
        double current_dist = calculateDistance(human_pos, robot_pos);
        
        if (current_dist >= target_dist) {
            // Robot is already at safe distance
            return robot_pos;
        }
        
        // Calculate direction vector from human to robot
        double dx = robot_pos.x - human_pos.x;
        double dy = robot_pos.y - human_pos.y;
        double dz = robot_pos.z - human_pos.z;
        
        // Normalize direction vector
        double norm = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (norm > 0.01) {  // Avoid division by zero
            dx /= norm;
            dy /= norm;
            dz /= norm;
        }
        
        // Calculate safe position at target distance
        geometry_msgs::msg::Point safe_pos;
        safe_pos.x = human_pos.x + dx * target_dist;
        safe_pos.y = human_pos.y + dy * target_dist;
        safe_pos.z = human_pos.z + dz * target_dist;
        
        return safe_pos;
    }

private:
    double calculateDistance(const geometry_msgs::msg::Point& p1,
                           const geometry_msgs::msg::Point& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    enum class ProxemicState {
        INTIMATE,
        PERSONAL,
        SOCIAL,
        PUBLIC
    };
};
```

#### Social Navigation
Social navigation involves moving through human spaces in ways that are perceived as natural and non-threatening:

```cpp
class SocialNavigationManager {
private:
    std::unique_ptr<ProxemicManager> proxemic_manager_;
    std::unique_ptr<HumanBehaviorPredictor> behavior_predictor_;
    
    // Social navigation parameters
    double social_force_strength_;
    double personal_space_radius_;
    double following_distance_;
    double passing_distance_;
    
    // Navigation states
    SocialNavigationState current_state_;
    
    // Human tracking
    std::vector<TrackedHuman> tracked_humans_;

public:
    SocialNavigationManager(double force_strength = 5.0, 
                          double personal_radius = 0.8,
                          double follow_dist = 1.0,
                          double pass_dist = 1.5)
        : social_force_strength_(force_strength),
          personal_space_radius_(personal_radius),
          following_distance_(follow_dist),
          passing_distance_(pass_dist),
          current_state_(SocialNavigationState::NEUTRAL) {
        
        proxemic_manager_ = std::make_unique<ProxemicManager>();
        behavior_predictor_ = std::make_unique<HumanBehaviorPredictor>();
    }
    
    geometry_msgs::msg::Twist calculateSocialMotion(
        const RobotState& robot_state,
        const std::vector<HumanDetection>& human_detections,
        const geometry_msgs::msg::PoseStamped& goal) {
        
        geometry_msgs::msg::Twist social_motion;
        
        // Update tracked humans
        updateTrackedHumans(human_detections);
        
        // Calculate social forces for each detected human
        Eigen::Vector3d social_force = Eigen::Vector3d::Zero();
        
        for (const auto& human : tracked_humans_) {
            Eigen::Vector3d force = calculateSocialForce(robot_state.pose.position, human.pose);
            social_force += force;
        }
        
        // Calculate desired motion toward goal
        Eigen::Vector3d goal_direction = Eigen::Vector3d(
            goal.pose.position.x - robot_state.pose.position.x,
            goal.pose.position.y - robot_state.pose.position.y,
            goal.pose.position.z - robot_state.pose.position.z);
        
        double goal_distance = goal_direction.norm();
        if (goal_distance > 0.01) {
            goal_direction.normalize();
        }
        
        // Combine social forces with goal-directed motion
        Eigen::Vector3d combined_motion = goal_direction - 0.3 * social_force;  // Weight social forces
        combined_motion.normalize();
        
        // Convert to twist command
        social_motion.linear.x = combined_motion.x() * robot_state.max_linear_velocity;
        social_motion.linear.y = combined_motion.y() * robot_state.max_linear_velocity;
        social_motion.linear.z = combined_motion.z() * robot_state.max_linear_velocity;
        
        // Calculate angular velocity for orientation
        double desired_yaw = std::atan2(combined_motion.y(), combined_motion.x());
        double current_yaw = getYawFromQuaternion(robot_state.pose.orientation);
        double yaw_error = normalizeAngle(desired_yaw - current_yaw);
        
        social_motion.angular.z = yaw_error * robot_state.max_angular_velocity;
        
        return social_motion;
    }

private:
    struct TrackedHuman {
        geometry_msgs::msg::Point pose;
        geometry_msgs::msg::Vector3 velocity;
        rclcpp::Time last_seen;
        int id;
    };
    
    Eigen::Vector3d calculateSocialForce(const geometry_msgs::msg::Point& robot_pos,
                                       const geometry_msgs::msg::Point& human_pos) {
        
        Eigen::Vector3d force = Eigen::Vector3d::Zero();
        
        // Calculate vector from human to robot
        Eigen::Vector3d to_robot(
            robot_pos.x - human_pos.x,
            robot_pos.y - human_pos.y,
            robot_pos.z - human_pos.z);
        
        double distance = to_robot.norm();
        
        if (distance < personal_space_radius_ + 0.1) {
            // Calculate repulsive force
            double force_magnitude = social_force_strength_ / (distance * distance + 0.01);
            
            if (distance > 0.01) {
                to_robot.normalize();
                force = -to_robot * force_magnitude;  // Repulsive (away from human)
            }
        }
        
        return force;
    }
    
    void updateTrackedHumans(const std::vector<HumanDetection>& detections) {
        // Update tracked humans based on new detections
        // This would implement data association and tracking algorithms
        for (const auto& detection : detections) {
            bool matched = false;
            
            for (auto& tracked : tracked_humans_) {
                double dist = calculateDistance(detection.pose.position, tracked.pose);
                if (dist < 0.5) {  // Threshold for matching
                    // Update tracked human position and velocity
                    tracked.velocity.x = (detection.pose.position.x - tracked.pose.x) / 
                                        (this->now() - tracked.last_seen).seconds();
                    tracked.velocity.y = (detection.pose.position.y - tracked.pose.y) / 
                                        (this->now() - tracked.last_seen).seconds();
                    tracked.pose = detection.pose.position;
                    tracked.last_seen = this->now();
                    matched = true;
                    break;
                }
            }
            
            if (!matched) {
                // Add new human to tracking
                TrackedHuman new_human;
                new_human.pose = detection.pose.position;
                new_human.last_seen = this->now();
                new_human.id = generateHumanId();
                tracked_humans_.push_back(new_human);
            }
        }
        
        // Remove humans not seen for a while
        tracked_humans_.erase(
            std::remove_if(tracked_humans_.begin(), tracked_humans_.end(),
                          [this](const TrackedHuman& h) {
                              return (this->now() - h.last_seen).seconds() > 5.0;
                          }),
            tracked_humans_.end());
    }
    
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
    
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
        return std::atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z));
    }
    
    int generateHumanId() {
        static int id_counter = 0;
        return ++id_counter;
    }
    
    enum class SocialNavigationState {
        NEUTRAL,
        APPROACHING,
        MAINTAINING_DISTANCE,
        AVOIDING,
        FOLLOWING,
        PASSING
    };
};
```

### Communication Models

#### Multimodal Communication
Effective HRI requires integrating multiple communication channels to enable natural interaction:

##### Verbal Communication
- **Speech Recognition**: Understanding spoken commands and questions
- **Natural Language Processing**: Interpreting meaning and intent
- **Text-to-Speech**: Providing verbal feedback and responses
- **Dialogue Management**: Maintaining coherent conversations

##### Non-verbal Communication
- **Gestures**: Hand and body movements for communication
- **Facial Expressions**: Conveying emotions and states
- **Gaze**: Directing attention and indicating focus
- **Posture**: Communicating attitude and approachability

#### Implementation Example: Multimodal Communication System
```cpp
#include <speech_recognition_msgs/msg/recognizer_transcription.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>

class MultimodalCommunicationSystem {
private:
    // Communication modalities
    std::unique_ptr<SpeechRecognitionSystem> speech_recognition_;
    std::unique_ptr<NaturalLanguageProcessor> nlp_processor_;
    std::unique_ptr<GestureRecognitionSystem> gesture_recognition_;
    std::unique_ptr<FaceExpressionAnalyzer> face_analyzer_;
    std::unique_ptr<TextToSpeechSystem> tts_system_;
    std::unique_ptr<GestureExecutionSystem> gesture_executor_;
    
    // Communication state
    CommunicationState current_state_;
    std::vector<CommunicationEvent> communication_history_;
    
    // Dialogue management
    std::unique_ptr<DialogueManager> dialogue_manager_;
    
    // User state tracking
    std::map<int, UserState> user_states_;

public:
    MultimodalCommunicationSystem() {
        speech_recognition_ = std::make_unique<OnlineSpeechRecognizer>();
        nlp_processor_ = std::make_unique<SpaCyNLPProcessor>();
        gesture_recognition_ = std::make_unique<OpenPoseGestureRecognizer>();
        face_analyzer_ = std::make_unique<FacialExpressionAnalyzer>();
        tts_system_ = std::make_unique<GoogleTTS>();
        gesture_executor_ = std::make_unique<RobotGestureExecutor>();
        dialogue_manager_ = std::make_unique<RuleBasedDialogueManager>();
    }
    
    CommunicationResponse processCommunicationInput(
        const std::string& speech_input,
        const std::vector<Gesture>& gesture_input,
        const std::vector<FacialExpression>& face_input) {
        
        CommunicationResponse response;
        
        // Process speech input
        if (!speech_input.empty()) {
            auto speech_result = speech_recognition_->process(speech_input);
            response.speech_content = speech_result.text;
            response.speech_confidence = speech_result.confidence;
            
            // Process natural language
            auto nlp_result = nlp_processor_->process(speech_result.text);
            response.intent = nlp_result.intent;
            response.entities = nlp_result.entities;
        }
        
        // Process gesture input
        if (!gesture_input.empty()) {
            auto gesture_result = gesture_recognition_->process(gesture_input);
            response.gesture_meaning = gesture_result.meaning;
            response.gesture_confidence = gesture_result.confidence;
        }
        
        // Process facial expression input
        if (!face_input.empty()) {
            auto face_result = face_analyzer_->process(face_input);
            response.user_emotion = face_result.emotion;
            response.emotion_confidence = face_result.confidence;
        }
        
        // Integrate multimodal information
        auto integrated_result = integrateModalities(response);
        
        // Generate appropriate response
        response.response_content = dialogue_manager_->generateResponse(
            integrated_result, getCurrentUserState());
        
        // Execute response (speech + gesture)
        executeResponse(response);
        
        // Update communication history
        updateCommunicationHistory(response);
        
        return response;
    }
    
    void executeResponse(const CommunicationResponse& response) {
        // Execute verbal response
        if (!response.response_content.empty()) {
            tts_system_->speak(response.response_content);
        }
        
        // Execute gesture response
        if (response.requires_gesture) {
            auto gesture = selectAppropriateGesture(response.intent);
            gesture_executor_->execute(gesture);
        }
    }

private:
    struct CommunicationResponse {
        std::string speech_content;
        double speech_confidence;
        std::string gesture_meaning;
        double gesture_confidence;
        std::string user_emotion;
        double emotion_confidence;
        std::string intent;
        std::vector<Entity> entities;
        std::string response_content;
        bool requires_gesture;
        rclcpp::Time timestamp;
    };
    
    struct CommunicationEvent {
        CommunicationType type;
        std::string content;
        double confidence;
        rclcpp::Time timestamp;
        int user_id;
    };
    
    enum class CommunicationType {
        SPEECH,
        GESTURE,
        FACIAL_EXPRESSION,
        TEXT,
        TACTILE
    };
    
    struct UserState {
        int id;
        std::string name;
        std::string emotional_state;
        std::string interaction_history;
        rclcpp::Time last_interaction;
        double engagement_level;
        std::vector<Preference> preferences;
    };
    
    struct Preference {
        std::string category;
        std::string value;
        double certainty;
    };
    
    CommunicationResult integrateModalities(const CommunicationResponse& input) {
        CommunicationResult result;
        
        // Weight different modalities based on confidence and context
        double speech_weight = input.speech_confidence;
        double gesture_weight = input.gesture_confidence;
        double face_weight = input.emotion_confidence;
        
        // Combine modalities to determine overall meaning
        if (speech_weight > 0.7) {
            result.primary_meaning = input.speech_content;
            result.confidence = speech_weight;
        } else if (gesture_weight > 0.7) {
            result.primary_meaning = input.gesture_meaning;
            result.confidence = gesture_weight;
        } else {
            // Use combination of modalities
            result.primary_meaning = combineModalities(input);
            result.confidence = (speech_weight + gesture_weight + face_weight) / 3.0;
        }
        
        // Consider emotional context
        result.emotional_context = input.user_emotion;
        
        return result;
    }
    
    std::string combineModalities(const CommunicationResponse& input) {
        // Implement multimodal fusion logic
        // This would combine speech, gesture, and facial information
        // to determine the most likely intended meaning
        
        if (!input.speech_content.empty() && input.speech_confidence > 0.5) {
            return input.speech_content;
        } else if (!input.gesture_meaning.empty() && input.gesture_confidence > 0.5) {
            return input.gesture_meaning;
        } else {
            // Fallback to simple concatenation or more sophisticated fusion
            return input.speech_content + " " + input.gesture_meaning;
        }
    }
    
    Gesture selectAppropriateGesture(const std::string& intent) {
        // Select appropriate gesture based on intent and context
        if (intent == "greeting") {
            return Gesture::WAVE;
        } else if (intent == "acknowledgment") {
            return Gesture::NOD;
        } else if (intent == "direction") {
            return Gesture::POINT;
        } else if (intent == "attention") {
            return Gesture::RAISE_ARM;
        } else {
            return Gesture::NEUTRAL;  // Default gesture
        }
    }
    
    UserState getCurrentUserState() {
        // Return state of most recently interacting user
        // In practice, this would use face recognition or other identification
        if (!user_states_.empty()) {
            return user_states_.begin()->second;
        }
        
        // Return default user state
        UserState default_state;
        default_state.id = -1;
        default_state.emotional_state = "neutral";
        default_state.engagement_level = 0.5;
        return default_state;
    }
    
    void updateCommunicationHistory(const CommunicationResponse& response) {
        CommunicationEvent event;
        event.type = CommunicationType::SPEECH;  // Or appropriate type
        event.content = response.response_content;
        event.confidence = response.speech_confidence;
        event.timestamp = this->now();
        event.user_id = getCurrentUserState().id;
        
        communication_history_.push_back(event);
        
        // Maintain history size
        if (communication_history_.size() > MAX_HISTORY_SIZE) {
            communication_history_.erase(communication_history_.begin());
        }
    }
    
    struct CommunicationResult {
        std::string primary_meaning;
        std::string emotional_context;
        double confidence;
        std::vector<std::string> alternative_interpretations;
    };
    
    static constexpr size_t MAX_HISTORY_SIZE = 100;
};
```

## Technical Implementation

### Perception for HRI

#### Human Detection and Tracking
```cpp
class HumanDetectionTracker {
private:
    std::unique_ptr<PersonDetector> person_detector_;
    std::unique_ptr<MultiObjectTracker> multi_tracker_;
    std::unique_ptr<HumanPoseEstimator> pose_estimator_;
    std::unique_ptr<FaceDetector> face_detector_;
    
    // Tracking parameters
    double detection_threshold_;
    double tracking_threshold_;
    int max_disappeared_;
    int max_history_;
    
    // Tracked humans
    std::vector<TrackedPerson> tracked_persons_;

public:
    HumanDetectionTracker(double det_thresh = 0.5, double track_thresh = 0.3,
                        int max_dis = 30, int max_hist = 100)
        : detection_threshold_(det_thresh), tracking_threshold_(track_thresh),
          max_disappeared_(max_dis), max_history_(max_hist) {
        
        person_detector_ = std::make_unique<YoloPersonDetector>();
        multi_tracker_ = std::make_unique<SortTracker>();  // Simple Online and Realtime Tracking
        pose_estimator_ = std::make_unique<OpenPoseEstimator>();
        face_detector_ = std::make_unique<MtcnnFaceDetector>();
    }
    
    std::vector<HumanDetection> detectAndTrackHumans(
        const sensor_msgs::msg::Image::SharedPtr& image) {
        
        std::vector<HumanDetection> detections;
        
        // Detect persons in image
        auto person_boxes = person_detector_->detect(image, detection_threshold_);
        
        // Update tracker with new detections
        auto tracked_objects = multi_tracker_->update(person_boxes);
        
        // Update tracked persons list
        updateTrackedPersons(tracked_objects);
        
        // Estimate pose for each tracked person
        for (auto& person : tracked_persons_) {
            if (person.visible && person.confidence > tracking_threshold_) {
                // Estimate pose
                auto pose_result = pose_estimator_->estimate(
                    image, person.bounding_box);
                
                person.pose = pose_result;
                
                // Detect face
                auto face_result = face_detector_->detect(
                    image, person.bounding_box);
                
                person.face_detected = !face_result.empty();
                if (!face_result.empty()) {
                    person.face_roi = face_result[0];  // Use first face
                }
                
                // Convert to HumanDetection format
                HumanDetection detection;
                detection.id = person.id;
                detection.position = person.position;  // From tracking
                detection.bounding_box = person.bounding_box;
                detection.pose = person.pose;
                detection.face_detected = person.face_detected;
                detection.confidence = person.confidence;
                detection.timestamp = this->now();
                
                detections.push_back(detection);
            }
        }
        
        // Remove old tracked persons
        cleanupTrackedPersons();
        
        return detections;
    }

private:
    struct TrackedPerson {
        int id;
        cv::Rect bounding_box;
        geometry_msgs::msg::Point position;  // World coordinates
        geometry_msgs::msg::Vector3 velocity;
        int disappeared_count;
        int appeared_count;
        double confidence;
        HumanPose pose;
        bool face_detected;
        cv::Rect face_roi;
        bool visible;
        rclcpp::Time last_seen;
        std::vector<geometry_msgs::msg::Point> position_history;
    };
    
    void updateTrackedPersons(const std::vector<TrackedObject>& objects) {
        for (const auto& obj : objects) {
            bool found = false;
            
            for (auto& person : tracked_persons_) {
                if (person.id == obj.id) {
                    // Update existing person
                    person.bounding_box = obj.bbox;
                    person.confidence = obj.confidence;
                    person.disappeared_count = 0;
                    person.visible = true;
                    person.last_seen = this->now();
                    
                    // Update position (convert from image to world coordinates)
                    person.position = convertImageToWorld(obj.bbox, obj.center);
                    
                    // Update velocity
                    if (person.position_history.size() > 1) {
                        auto prev_pos = person.position_history.back();
                        double dt = (this->now() - person.last_seen).seconds();
                        if (dt > 0.01) {
                            person.velocity.x = (person.position.x - prev_pos.x) / dt;
                            person.velocity.y = (person.position.y - prev_pos.y) / dt;
                            person.velocity.z = (person.position.z - prev_pos.z) / dt;
                        }
                    }
                    
                    // Add to position history
                    person.position_history.push_back(person.position);
                    if (person.position_history.size() > max_history_) {
                        person.position_history.erase(person.position_history.begin());
                    }
                    
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                // Add new person
                TrackedPerson new_person;
                new_person.id = obj.id;
                new_person.bounding_box = obj.bbox;
                new_person.confidence = obj.confidence;
                new_person.disappeared_count = 0;
                new_person.appeared_count = 1;
                new_person.visible = true;
                new_person.last_seen = this->now();
                new_person.position = convertImageToWorld(obj.bbox, obj.center);
                new_person.face_detected = false;
                
                tracked_persons_.push_back(new_person);
            }
        }
        
        // Mark non-detected persons as disappeared
        for (auto& person : tracked_persons_) {
            bool detected = false;
            for (const auto& obj : objects) {
                if (person.id == obj.id) {
                    detected = true;
                    break;
                }
            }
            
            if (!detected) {
                person.visible = false;
                person.disappeared_count++;
            }
        }
    }
    
    void cleanupTrackedPersons() {
        tracked_persons_.erase(
            std::remove_if(tracked_persons_.begin(), tracked_persons_.end(),
                          [this](const TrackedPerson& p) {
                              return p.disappeared_count > max_disappeared_;
                          }),
            tracked_persons_.end());
    }
    
    geometry_msgs::msg::Point convertImageToWorld(const cv::Rect& bbox, 
                                               const cv::Point2f& center) {
        // Convert image coordinates to world coordinates
        // This would use camera calibration and robot localization
        geometry_msgs::msg::Point world_point;
        
        // In practice, this would use camera extrinsic/intrinsic parameters
        // and robot pose to triangulate world position
        world_point.x = center.x * 0.01;  // Simplified conversion
        world_point.y = center.y * 0.01;
        world_point.z = 0.0;  // Assume ground level
        
        return world_point;
    }
    
    struct HumanDetection {
        int id;
        geometry_msgs::msg::Point position;
        cv::Rect bounding_box;
        HumanPose pose;
        bool face_detected;
        double confidence;
        rclcpp::Time timestamp;
    };
    
    struct HumanPose {
        std::vector<cv::Point2f> keypoints;  // Joint positions
        std::vector<float> confidence;       // Keypoint confidence
        float pose_confidence;
    };
    
    struct TrackedObject {
        int id;
        cv::Rect bbox;
        cv::Point2f center;
        double confidence;
    };
};
```

#### Intention Recognition
```cpp
class IntentionRecognitionSystem {
private:
    std::unique_ptr<BehaviorAnalyzer> behavior_analyzer_;
    std::unique_ptr<ContextAnalyzer> context_analyzer_;
    std::unique_ptr<ActivityRecognizer> activity_recognizer_;
    std::unique_ptr<GoalPredictor> goal_predictor_;
    
    // Machine learning model for intention prediction
    std::unique_ptr<NeuralNetworkModel> intention_model_;
    
    // User behavior patterns
    std::map<int, UserBehaviorPattern> user_patterns_;
    
    // Interaction context
    InteractionContext current_context_;

public:
    IntentionRecognitionSystem() {
        behavior_analyzer_ = std::make_unique<BehaviorAnalyzer>();
        context_analyzer_ = std::make_unique<ContextAnalyzer>();
        activity_recognizer_ = std::make_unique<ActivityRecognizer>();
        goal_predictor_ = std::make_unique<GoalPredictor>();
        intention_model_ = std::make_unique<PyTorchModel>("intention_model.pt");
    }
    
    IntentionPrediction predictIntention(const HumanState& human_state,
                                      const EnvironmentalContext& env_context) {
        
        IntentionPrediction prediction;
        
        // Analyze human behavior patterns
        auto behavior_analysis = behavior_analyzer_->analyze(human_state);
        
        // Analyze current context
        auto context_analysis = context_analyzer_->analyze(env_context);
        
        // Recognize current activity
        auto activity = activity_recognizer_->recognize(human_state.pose, 
                                                      human_state.position_history);
        
        // Predict potential goals
        auto goal_prediction = goal_predictor_->predict(human_state.position,
                                                      human_state.velocity,
                                                      env_context);
        
        // Combine all information for intention prediction
        std::vector<float> feature_vector = createFeatureVector(
            behavior_analysis, context_analysis, activity, goal_prediction);
        
        // Run intention prediction model
        auto model_output = intention_model_->predict(feature_vector);
        
        // Decode prediction to intentions
        prediction = decodeIntention(model_output, human_state.user_id);
        
        // Validate prediction confidence
        if (prediction.confidence < MIN_INTENTION_CONFIDENCE) {
            prediction.type = IntentionType::UNKNOWN;
        }
        
        // Update user behavior patterns
        updateUserPattern(human_state.user_id, prediction);
        
        return prediction;
    }

private:
    struct IntentionPrediction {
        IntentionType type;
        std::string target_object;
        std::string target_location;
        double confidence;
        std::vector<IntentionType> alternative_intentions;
        rclcpp::Time timestamp;
    };
    
    struct HumanState {
        int user_id;
        geometry_msgs::msg::Point position;
        geometry_msgs::msg::Vector3 velocity;
        HumanPose pose;
        std::vector<geometry_msgs::msg::Point> position_history;
        std::vector<HumanPose> pose_history;
        rclcpp::Time timestamp;
    };
    
    struct EnvironmentalContext {
        std::vector<DetectedObject> objects;
        std::vector<geometry_msgs::msg::Point> locations;
        std::vector<HumanDetection> other_humans;
        geometry_msgs::msg::PoseStamped robot_pose;
        std::string current_room;
        std::vector<std::string> available_actions;
        rclcpp::Time timestamp;
    };
    
    struct UserBehaviorPattern {
        std::vector<IntentionType> common_intentions;
        std::vector<std::string> preferred_objects;
        std::vector<std::string> frequent_locations;
        double engagement_level;
        rclcpp::Time last_update;
    };
    
    enum class IntentionType {
        UNKNOWN,
        MOVE_TO_OBJECT,
        REQUEST_HELP,
        GREET_ROBOT,
        AVOID_ROBOT,
        FOLLOW_ROBOT,
        ASK_QUESTION,
        PROVIDE_INFORMATION,
        COLLABORATE,
        LEARN_DEMONSTRATION
    };
    
    std::vector<float> createFeatureVector(const BehaviorAnalysis& behavior,
                                         const ContextAnalysis& context,
                                         const Activity& activity,
                                         const GoalPrediction& goal) {
        
        std::vector<float> features;
        
        // Add behavior features
        features.insert(features.end(), behavior.features.begin(), behavior.features.end());
        
        // Add context features
        features.insert(features.end(), context.features.begin(), context.features.end());
        
        // Add activity features
        features.push_back(static_cast<float>(activity.type));
        features.push_back(activity.confidence);
        
        // Add goal prediction features
        features.push_back(goal.probability);
        features.push_back(goal.estimated_time);
        
        // Add user-specific features
        auto user_pattern = getUserPattern(behavior.user_id);
        features.insert(features.end(), user_pattern.features.begin(), user_pattern.features.end());
        
        return features;
    }
    
    IntentionPrediction decodeIntention(const std::vector<float>& model_output,
                                      int user_id) {
        
        IntentionPrediction prediction;
        
        // Find the intention type with highest probability
        auto max_elem = std::max_element(model_output.begin(), model_output.end());
        int max_idx = std::distance(model_output.begin(), max_elem);
        
        // Map index to intention type
        switch (max_idx) {
            case 0: prediction.type = IntentionType::MOVE_TO_OBJECT; break;
            case 1: prediction.type = IntentionType::REQUEST_HELP; break;
            case 2: prediction.type = IntentionType::GREET_ROBOT; break;
            case 3: prediction.type = IntentionType::AVOID_ROBOT; break;
            case 4: prediction.type = IntentionType::FOLLOW_ROBOT; break;
            case 5: prediction.type = IntentionType::ASK_QUESTION; break;
            case 6: prediction.type = IntentionType::PROVIDE_INFORMATION; break;
            case 7: prediction.type = IntentionType::COLLABORATE; break;
            case 8: prediction.type = IntentionType::LEARN_DEMONSTRATION; break;
            default: prediction.type = IntentionType::UNKNOWN; break;
        }
        
        prediction.confidence = *max_elem;
        
        // Get alternative intentions (top 3)
        std::vector<std::pair<float, int>> prob_idx_pairs;
        for (size_t i = 0; i < model_output.size(); i++) {
            prob_idx_pairs.push_back({model_output[i], static_cast<int>(i)});
        }
        
        std::sort(prob_idx_pairs.begin(), prob_idx_pairs.end(),
                 [](const auto& a, const auto& b) { return a.first > b.first; });
        
        for (int i = 1; i < std::min(3, static_cast<int>(prob_idx_pairs.size())); i++) {
            IntentionType alt_type;
            switch (prob_idx_pairs[i].second) {
                case 0: alt_type = IntentionType::MOVE_TO_OBJECT; break;
                case 1: alt_type = IntentionType::REQUEST_HELP; break;
                case 2: alt_type = IntentionType::GREET_ROBOT; break;
                case 3: alt_type = IntentionType::AVOID_ROBOT; break;
                case 4: alt_type = IntentionType::FOLLOW_ROBOT; break;
                case 5: alt_type = IntentionType::ASK_QUESTION; break;
                case 6: alt_type = IntentionType::PROVIDE_INFORMATION; break;
                case 7: alt_type = IntentionType::COLLABORATE; break;
                case 8: alt_type = IntentionType::LEARN_DEMONSTRATION; break;
                default: alt_type = IntentionType::UNKNOWN; break;
            }
            prediction.alternative_intentions.push_back(alt_type);
        }
        
        return prediction;
    }
    
    void updateUserPattern(int user_id, const IntentionPrediction& prediction) {
        if (user_patterns_.find(user_id) == user_patterns_.end()) {
            user_patterns_[user_id] = UserBehaviorPattern();
        }
        
        auto& pattern = user_patterns_[user_id];
        pattern.common_intentions.push_back(prediction.type);
        
        // Keep only recent patterns
        if (pattern.common_intentions.size() > MAX_PATTERN_HISTORY) {
            pattern.common_intentions.erase(pattern.common_intentions.begin());
        }
        
        pattern.last_update = this->now();
    }
    
    UserBehaviorPattern getUserPattern(int user_id) {
        if (user_patterns_.find(user_id) != user_patterns_.end()) {
            return user_patterns_[user_id];
        }
        
        // Return default pattern
        UserBehaviorPattern default_pattern;
        default_pattern.engagement_level = 0.5;
        return default_pattern;
    }
    
    static constexpr double MIN_INTENTION_CONFIDENCE = 0.6;
    static constexpr size_t MAX_PATTERN_HISTORY = 50;
};
```

### Natural Language Processing for HRI

#### Speech Recognition and Understanding
```cpp
class SpokenLanguageUnderstanding {
private:
    std::unique_ptr<ASRSystem> speech_recognizer_;
    std::unique_ptr<NLUProcessor> nlu_processor_;
    std::unique_ptr<DialogueStateTracker> dialogue_tracker_;
    std::unique_ptr<NamedEntityRecognizer> ner_system_;
    
    // Language models
    std::unique_ptr<LanguageModel> language_model_;
    std::unique_ptr<DomainOntology> domain_ontology_;
    
    // User context
    std::map<int, UserContext> user_contexts_;

public:
    SpokenLanguageUnderstanding() {
        speech_recognizer_ = std::make_unique<WhisperASR>();
        nlu_processor_ = std::make_unique<SpaCyNLU>();
        dialogue_tracker_ = std::make_unique<NeuralDialogueTracker>();
        ner_system_ = std::make_unique<SpaCyNER>();
        language_model_ = std::make_unique<GPTLanguageModel>();
        domain_ontology_ = std::make_unique<RobotDomainOntology>();
    }
    
    SpokenLanguageResponse processUtterance(const std::string& audio_input,
                                          int user_id) {
        
        SpokenLanguageResponse response;
        
        // Convert audio to text
        auto asr_result = speech_recognizer_->recognize(audio_input);
        response.recognized_text = asr_result.text;
        response.recognition_confidence = asr_result.confidence;
        
        if (asr_result.confidence < MIN_RECOGNITION_CONFIDENCE) {
            response.understanding_success = false;
            response.error_message = "Speech recognition confidence too low";
            return response;
        }
        
        // Perform natural language understanding
        auto nlu_result = nlu_processor_->process(asr_result.text);
        
        // Extract named entities
        auto entities = ner_system_->extract(asr_result.text);
        
        // Update dialogue state
        auto dialogue_state = dialogue_tracker_->update(
            asr_result.text, nlu_result, entities, getUserContext(user_id));
        
        // Determine intent and extract relevant information
        response.intent = nlu_result.intent;
        response.entities = entities;
        response.dialogue_state = dialogue_state;
        
        // Validate understanding
        response.understanding_success = validateUnderstanding(
            nlu_result, entities, dialogue_state);
        
        if (!response.understanding_success) {
            response.error_message = "Could not understand user intent";
        }
        
        // Update user context
        updateUserContext(user_id, asr_result.text, nlu_result.intent, entities);
        
        return response;
    }
    
    std::string generateResponse(const SpokenLanguageResponse& input,
                               const std::string& system_action) {
        
        // Generate appropriate response based on intent and context
        std::string response_template = getResponseTemplate(input.intent);
        
        // Fill in entities and context
        std::string filled_response = fillResponseTemplate(
            response_template, input.entities, input.dialogue_state);
        
        // Use language model for more natural responses
        std::string natural_response = language_model_->generate(
            filled_response, system_action);
        
        return natural_response;
    }

private:
    struct SpokenLanguageResponse {
        std::string recognized_text;
        double recognition_confidence;
        std::string intent;
        std::vector<Entity> entities;
        DialogueState dialogue_state;
        bool understanding_success;
        std::string error_message;
        rclcpp::Time timestamp;
    };
    
    struct Entity {
        std::string type;
        std::string value;
        double confidence;
        int start_pos;
        int end_pos;
    };
    
    struct DialogueState {
        std::string current_intent;
        std::map<std::string, std::string> slot_values;
        std::string previous_intent;
        int turn_number;
        rclcpp::Time last_update;
    };
    
    struct UserContext {
        std::string name;
        std::string preferences;
        std::vector<std::string> conversation_history;
        rclcpp::Time last_interaction;
        int total_interactions;
    };
    
    bool validateUnderstanding(const NLUResult& nlu_result,
                             const std::vector<Entity>& entities,
                             const DialogueState& state) {
        
        // Check if intent is recognized
        if (nlu_result.intent.empty() || nlu_result.intent == "unknown") {
            return false;
        }
        
        // Check if required entities are present for the intent
        auto required_entities = getRequiredEntitiesForIntent(nlu_result.intent);
        for (const auto& required : required_entities) {
            bool found = false;
            for (const auto& entity : entities) {
                if (entity.type == required) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                return false;  // Required entity missing
            }
        }
        
        return true;
    }
    
    std::vector<std::string> getRequiredEntitiesForIntent(const std::string& intent) {
        // Define required entities for each intent type
        if (intent == "navigation_request") {
            return {"destination"};
        } else if (intent == "object_request") {
            return {"object_name"};
        } else if (intent == "greeting") {
            return {};
        } else if (intent == "question") {
            return {"question_topic"};
        } else {
            return {};
        }
    }
    
    std::string getResponseTemplate(const std::string& intent) {
        // Define response templates for different intents
        if (intent == "navigation_request") {
            return "I can help you navigate to {destination}. Is that correct?";
        } else if (intent == "object_request") {
            return "I can bring you the {object_name}. Where would you like me to bring it?";
        } else if (intent == "greeting") {
            return "Hello! How can I assist you today?";
        } else if (intent == "question") {
            return "I understand you're asking about {question_topic}. Let me help.";
        } else {
            return "I understand your request. How else can I assist?";
        }
    }
    
    std::string fillResponseTemplate(const std::string& template_str,
                                   const std::vector<Entity>& entities,
                                   const DialogueState& state) {
        
        std::string filled_template = template_str;
        
        for (const auto& entity : entities) {
            std::string placeholder = "{" + entity.type + "}";
            size_t pos = filled_template.find(placeholder);
            if (pos != std::string::npos) {
                filled_template.replace(pos, placeholder.length(), entity.value);
            }
        }
        
        return filled_template;
    }
    
    UserContext getUserContext(int user_id) {
        if (user_contexts_.find(user_id) != user_contexts_.end()) {
            return user_contexts_[user_id];
        }
        
        // Return default context
        UserContext default_context;
        default_context.name = "Unknown User";
        default_context.total_interactions = 0;
        return default_context;
    }
    
    void updateUserContext(int user_id, const std::string& text,
                          const std::string& intent, const std::vector<Entity>& entities) {
        
        if (user_contexts_.find(user_id) == user_contexts_.end()) {
            user_contexts_[user_id] = UserContext();
        }
        
        auto& context = user_contexts_[user_id];
        context.conversation_history.push_back(text);
        context.last_interaction = this->now();
        context.total_interactions++;
        
        // Extract user name if mentioned
        for (const auto& entity : entities) {
            if (entity.type == "person_name") {
                context.name = entity.value;
            }
        }
        
        // Maintain history size
        if (context.conversation_history.size() > MAX_CONTEXT_HISTORY) {
            context.conversation_history.erase(context.conversation_history.begin());
        }
    }
    
    static constexpr double MIN_RECOGNITION_CONFIDENCE = 0.7;
    static constexpr size_t MAX_CONTEXT_HISTORY = 20;
};
```

## Safety and Ethics in HRI

### Safety Framework

#### Physical Safety
```cpp
class HRISafetyManager {
private:
    std::unique_ptr<SafeNavigationSystem> safe_navigation_;
    std::unique_ptr<CollisionAvoidanceSystem> collision_avoidance_;
    std::unique_ptr<EmergencyStopSystem> emergency_stop_;
    std::unique_ptr<HumanAwareControl> human_aware_control_;
    
    // Safety zones around humans
    std::vector<SafetyZone> human_safety_zones_;
    
    // Safety monitoring
    rclcpp::TimerBase::SharedPtr safety_monitor_timer_;
    double safety_check_frequency_;

public:
    HRISafetyManager(double check_freq = 100.0)  // 100 Hz safety checks
        : safety_check_frequency_(check_freq) {
        
        safe_navigation_ = std::make_unique<SafeNavigationSystem>();
        collision_avoidance_ = std::make_unique<CollisionAvoidanceSystem>();
        emergency_stop_ = std::make_unique<EmergencyStopSystem>();
        human_aware_control_ = std::make_unique<HumanAwareController>();
        
        // Start safety monitoring
        safety_monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / safety_check_frequency_)),
            std::bind(&HRISafetyManager::performSafetyChecks, this));
    }
    
    SafetyStatus checkHRIInteractions(const std::vector<HumanDetection>& humans,
                                    const RobotState& robot_state) {
        
        SafetyStatus status;
        status.safe = true;
        status.violations.clear();
        
        // Check proximity to humans
        for (const auto& human : humans) {
            double distance = calculateDistance(robot_state.pose.position, human.position);
            
            if (distance < MIN_SAFE_DISTANCE) {
                SafetyViolation violation;
                violation.type = SafetyViolationType::TOO_CLOSE_TO_HUMAN;
                violation.description = "Robot too close to human: " + 
                                      std::to_string(distance) + "m";
                violation.severity = SafetyViolationSeverity::HIGH;
                violation.timestamp = this->now();
                violation.human_id = human.id;
                
                status.safe = false;
                status.violations.push_back(violation);
            }
            
            // Check if robot's path intersects with human
            if (isPathIntersectingWithHuman(robot_state, human)) {
                SafetyViolation violation;
                violation.type = SafetyViolationType::PATH_INTERSECTS_HUMAN;
                violation.description = "Robot path intersects with human trajectory";
                violation.severity = SafetyViolationSeverity::MEDIUM;
                violation.timestamp = this->now();
                violation.human_id = human.id;
                
                status.safe = false;
                status.violations.push_back(violation);
            }
        }
        
        // Check velocity limits near humans
        if (robot_state.linear_velocity > MAX_VELOCITY_NEAR_HUMAN) {
            SafetyViolation violation;
            violation.type = SafetyViolationType::EXCESSIVE_VELOCITY_NEAR_HUMAN;
            violation.description = "Robot velocity too high near human: " + 
                                  std::to_string(robot_state.linear_velocity) + " m/s";
            violation.severity = SafetyViolationSeverity::MEDIUM;
            violation.timestamp = this->now();
            
            status.safe = false;
            status.violations.push_back(violation);
        }
        
        return status;
    }
    
    void performSafetyChecks() {
        // Get current robot state and human detections
        auto robot_state = getCurrentRobotState();
        auto humans = getLatestHumanDetections();
        
        // Check safety of current interaction
        auto safety_status = checkHRIInteractions(humans, robot_state);
        
        if (!safety_status.safe) {
            handleSafetyViolations(safety_status);
        }
        
        // Publish safety status
        publishSafetyStatus(safety_status);
    }

private:
    struct SafetyStatus {
        bool safe;
        std::vector<SafetyViolation> violations;
        rclcpp::Time timestamp;
    };
    
    struct SafetyViolation {
        SafetyViolationType type;
        std::string description;
        SafetyViolationSeverity severity;
        rclcpp::Time timestamp;
        int human_id;
    };
    
    enum class SafetyViolationType {
        TOO_CLOSE_TO_HUMAN,
        PATH_INTERSECTS_HUMAN,
        EXCESSIVE_VELOCITY_NEAR_HUMAN,
        UNEXPECTED_HUMAN_BEHAVIOR,
        IMPROPER_GESTURE,
        INAPPROPRIATE_COMMUNICATION
    };
    
    enum class SafetyViolationSeverity {
        LOW,
        MEDIUM,
        HIGH,
        CRITICAL
    };
    
    bool isPathIntersectingWithHuman(const RobotState& robot_state,
                                   const HumanDetection& human) {
        
        // Predict human trajectory
        auto human_trajectory = predictHumanTrajectory(human);
        
        // Predict robot trajectory
        auto robot_trajectory = predictRobotTrajectory(robot_state);
        
        // Check for intersections
        for (const auto& robot_point : robot_trajectory) {
            for (const auto& human_point : human_trajectory) {
                double distance = calculateDistance(robot_point, human_point);
                if (distance < SAFETY_INTERSECTION_THRESHOLD) {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    std::vector<geometry_msgs::msg::Point> predictHumanTrajectory(
        const HumanDetection& human) {
        
        // Simple constant velocity prediction
        std::vector<geometry_msgs::msg::Point> trajectory;
        
        // In practice, this would use more sophisticated human motion prediction
        // (social force models, machine learning, etc.)
        
        for (int i = 1; i <= PREDICTION_HORIZON_STEPS; i++) {
            geometry_msgs::msg::Point future_point = human.position;
            // Add predicted movement based on velocity
            trajectory.push_back(future_point);
        }
        
        return trajectory;
    }
    
    std::vector<geometry_msgs::msg::Point> predictRobotTrajectory(
        const RobotState& robot_state) {
        
        // Predict robot trajectory based on current control commands
        std::vector<geometry_msgs::msg::Point> trajectory;
        
        // In practice, this would use the robot's motion model and control inputs
        for (int i = 1; i <= PREDICTION_HORIZON_STEPS; i++) {
            geometry_msgs::msg::Point future_point = robot_state.pose.position;
            // Add predicted movement based on current velocity
            future_point.x += robot_state.linear_velocity * i * PREDICTION_DT;
            future_point.y += robot_state.linear_velocity * i * PREDICTION_DT;
            trajectory.push_back(future_point);
        }
        
        return trajectory;
    }
    
    void handleSafetyViolations(const SafetyStatus& status) {
        for (const auto& violation : status.violations) {
            switch (violation.severity) {
                case SafetyViolationSeverity::CRITICAL:
                case SafetyViolationSeverity::HIGH:
                    // Trigger emergency stop
                    emergency_stop_->activate();
                    RCLCPP_ERROR(this->get_logger(), 
                               "CRITICAL SAFETY VIOLATION: %s", 
                               violation.description.c_str());
                    break;
                    
                case SafetyViolationSeverity::MEDIUM:
                    // Reduce speed and maintain safe distance
                    human_aware_control_->reduceSpeed();
                    RCLCPP_WARN(this->get_logger(), 
                              "SAFETY WARNING: %s", 
                              violation.description.c_str());
                    break;
                    
                case SafetyViolationSeverity::LOW:
                    // Log for monitoring
                    RCLCPP_INFO(this->get_logger(), 
                              "SAFETY NOTICE: %s", 
                              violation.description.c_str());
                    break;
            }
        }
    }
    
    void publishSafetyStatus(const SafetyStatus& status) {
        auto msg = safety_msgs::msg::HRISafetyStatus();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        msg.safe = status.safe;
        
        for (const auto& violation : status.violations) {
            safety_msgs::msg::HRISafetyViolation ros_violation;
            ros_violation.type = static_cast<uint8_t>(violation.type);
            ros_violation.description = violation.description;
            ros_violation.severity = static_cast<uint8_t>(violation.severity);
            ros_violation.timestamp = violation.timestamp;
            ros_violation.human_id = violation.human_id;
            
            msg.violations.push_back(ros_violation);
        }
        
        safety_status_publisher_->publish(msg);
    }
    
    double calculateDistance(const geometry_msgs::msg::Point& p1,
                           const geometry_msgs::msg::Point& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    RobotState getCurrentRobotState() {
        // Implementation would get current robot state
        return RobotState{};  // Placeholder
    }
    
    std::vector<HumanDetection> getLatestHumanDetections() {
        // Implementation would get latest human detections
        return std::vector<HumanDetection>{};  // Placeholder
    }
    
    static constexpr double MIN_SAFE_DISTANCE = 0.8;  // meters
    static constexpr double MAX_VELOCITY_NEAR_HUMAN = 0.3;  // m/s
    static constexpr double SAFETY_INTERSECTION_THRESHOLD = 0.5;  // meters
    static constexpr int PREDICTION_HORIZON_STEPS = 10;
    static constexpr double PREDICTION_DT = 0.1;  // seconds
};
```

#### Ethical Considerations
```cpp
class HRIEthicsManager {
private:
    // Ethical guidelines and principles
    std::vector<EthicalPrinciple> ethical_principles_;
    
    // Privacy protection mechanisms
    std::unique_ptr<PrivacyManager> privacy_manager_;
    
    // Bias detection and mitigation
    std::unique_ptr<BiasDetector> bias_detector_;
    
    // Transparency mechanisms
    std::unique_ptr<ExplanationSystem> explanation_system_;
    
    // User consent management
    std::unique_ptr<ConsentManager> consent_manager_;
    
    // Ethical decision making
    std::unique_ptr<EthicalDecisionMaker> ethical_decision_maker_;

public:
    HRIEthicsManager() {
        privacy_manager_ = std::make_unique<PrivacyPreservingSystem>();
        bias_detector_ = std::make_unique<FaceRecognitionBiasDetector>();
        explanation_system_ = std::make_unique<ExplanationGenerator>();
        consent_manager_ = std::make_unique<UserConsentManager>();
        ethical_decision_maker_ = std::make_unique<RuleBasedEthicalDecisionMaker>();
        
        // Initialize ethical principles
        initializeEthicalPrinciples();
    }
    
    EthicalDecision evaluateAction(const RobotAction& action,
                                 const HumanState& human_state) {
        
        EthicalDecision decision;
        decision.approved = true;
        decision.reasoning = "";
        decision.alternatives = {};
        
        // Check privacy implications
        auto privacy_check = privacy_manager_->checkPrivacy(action, human_state);
        if (!privacy_check.approved) {
            decision.approved = false;
            decision.reasoning += "Privacy violation: " + privacy_check.reason + ". ";
        }
        
        // Check for bias
        auto bias_check = bias_detector_->checkBias(action, human_state);
        if (bias_check.bias_detected) {
            decision.approved = false;
            decision.reasoning += "Potential bias detected: " + bias_check.reason + ". ";
        }
        
        // Check consent
        auto consent_check = consent_manager_->checkConsent(action, human_state.user_id);
        if (!consent_check.has_consent) {
            decision.approved = false;
            decision.reasoning += "User consent not obtained: " + consent_check.reason + ". ";
        }
        
        // Apply ethical principles
        for (const auto& principle : ethical_principles_) {
            if (!principle.evaluate(action, human_state)) {
                decision.approved = false;
                decision.reasoning += "Violates principle '" + principle.name + "': " + 
                                    principle.getViolationReason() + ". ";
            }
        }
        
        // Generate explanation if action is not approved
        if (!decision.approved) {
            decision.explanation = explanation_system_->generateExplanation(
                action, human_state, decision.reasoning);
        }
        
        return decision;
    }
    
    void initializeEthicalPrinciples() {
        // Add fundamental ethical principles for HRI
        ethical_principles_.push_back(AutonomyPrinciple());
        ethical_principles_.push_back(BeneficencePrinciple());
        ethical_principles_.push_back(NonMaleficencePrinciple());
        ethical_principles_.push_back(JusticePrinciple());
        ethical_principles_.push_back(TransparencyPrinciple());
        ethical_principles_.push_back(PrivacyPrinciple());
    }

private:
    struct EthicalDecision {
        bool approved;
        std::string reasoning;
        std::string explanation;
        std::vector<RobotAction> alternatives;
        rclcpp::Time timestamp;
    };
    
    struct EthicalPrinciple {
        std::string name;
        std::string description;
        
        virtual bool evaluate(const RobotAction& action, const HumanState& human_state) = 0;
        virtual std::string getViolationReason() = 0;
    };
    
    struct AutonomyPrinciple : public EthicalPrinciple {
        AutonomyPrinciple() {
            name = "Respect for Autonomy";
            description = "Respect human autonomy and decision-making capacity";
        }
        
        bool evaluate(const RobotAction& action, const HumanState& human_state) override {
            // Check if action respects human autonomy
            return !action.interrupts_humans_without_reason;
        }
        
        std::string getViolationReason() override {
            return "Action does not respect human autonomy";
        }
    };
    
    struct BeneficencePrinciple : public EthicalPrinciple {
        BeneficencePrinciple() {
            name = "Beneficence";
            description = "Act in ways that benefit humans";
        }
        
        bool evaluate(const RobotAction& action, const HumanState& human_state) override {
            // Check if action provides benefit to human
            return action.has_positive_impact;
        }
        
        std::string getViolationReason() override {
            return "Action does not provide benefit to human";
        }
    };
    
    struct NonMaleficencePrinciple : public EthicalPrinciple {
        NonMaleficencePrinciple() {
            name = "Non-Maleficence";
            description = "Do no harm to humans";
        }
        
        bool evaluate(const RobotAction& action, const HumanState& human_state) override {
            // Check if action causes harm
            return !action.could_cause_harm;
        }
        
        std::string getViolationReason() override {
            return "Action could cause harm to human";
        }
    };
    
    struct JusticePrinciple : public EthicalPrinciple {
        JusticePrinciple() {
            name = "Justice";
            description = "Treat all humans fairly and equitably";
        }
        
        bool evaluate(const RobotAction& action, const HumanState& human_state) override {
            // Check if action treats human fairly
            return !action.discriminates_based_on_bias;
        }
        
        std::string getViolationReason() override {
            return "Action treats human unfairly";
        }
    };
    
    struct TransparencyPrinciple : public EthicalPrinciple {
        TransparencyPrinciple() {
            name = "Transparency";
            description = "Be transparent about robot capabilities and intentions";
        }
        
        bool evaluate(const RobotAction& action, const HumanState& human_state) override {
            // Check if action is transparent
            return action.is_explainable;
        }
        
        std::string getViolationReason() override {
            return "Action is not transparent or explainable";
        }
    };
    
    struct PrivacyPrinciple : public EthicalPrinciple {
        PrivacyPrinciple() {
            name = "Privacy";
            description = "Respect human privacy and data protection";
        }
        
        bool evaluate(const RobotAction& action, const HumanState& human_state) override {
            // Check if action respects privacy
            return action.respects_privacy;
        }
        
        std::string getViolationReason() override {
            return "Action violates human privacy";
        }
    };
    
    struct RobotAction {
        std::string type;
        std::string target;
        bool interrupts_humans_without_reason = false;
        bool has_positive_impact = true;
        bool could_cause_harm = false;
        bool discriminates_based_on_bias = false;
        bool is_explainable = true;
        bool respects_privacy = true;
    };
};
```

## Collaborative Interaction Models

### Shared Control Systems

#### Human-Robot Shared Autonomy
```cpp
class SharedAutonomySystem {
private:
    std::unique_ptr<HumanIntentionRecognizer> human_intention_recognizer_;
    std::unique_ptr<RobotAutonomySystem> robot_autonomy_;
    std::unique_ptr<AuthorityBalancer> authority_balancer_;
    std::unique_ptr<CollaborationManager> collaboration_manager_;
    
    // Shared control parameters
    double human_authority_weight_;
    double robot_authority_weight_;
    double adaptation_rate_;
    
    // Control fusion
    std::unique_ptr<ControlFusion> control_fusion_;

public:
    SharedAutonomySystem(double human_weight = 0.5, double robot_weight = 0.5,
                        double adaptation = 0.1)
        : human_authority_weight_(human_weight), robot_authority_weight_(robot_weight),
          adaptation_rate_(adaptation) {
        
        human_intention_recognizer_ = std::make_unique<IntentionRecognitionSystem>();
        robot_autonomy_ = std::make_unique<AutonomousPlanningSystem>();
        authority_balancer_ = std::make_unique<DynamicAuthorityBalancer>();
        collaboration_manager_ = std::make_unique<CollaborationManager>();
        control_fusion_ = std::make_unique<WeightedControlFusion>();
    }
    
    ControlCommand generateSharedControlCommand(
        const HumanInput& human_input,
        const RobotState& robot_state,
        const EnvironmentalContext& env_context) {
        
        // Recognize human intention
        auto human_intention = human_intention_recognizer_->predictIntention(
            human_input.state, env_context);
        
        // Generate robot autonomous plan
        auto robot_plan = robot_autonomy_->generatePlan(
            robot_state, env_context, human_intention);
        
        // Determine appropriate authority balance
        auto authority_balance = authority_balancer_->determineBalance(
            human_input, robot_state, human_intention);
        
        // Fuse human and robot control commands
        auto fused_command = control_fusion_->fuse(
            human_input.command, robot_plan.command, authority_balance);
        
        // Ensure safety constraints are met
        auto safe_command = applySafetyConstraints(fused_command, robot_state, env_context);
        
        return safe_command;
    }
    
    void adaptAuthorityBalance(const InteractionFeedback& feedback) {
        // Adapt authority balance based on interaction success
        if (feedback.efficiency > OPTIMAL_EFFICIENCY_THRESHOLD) {
            // Increase human authority if interaction is going well
            human_authority_weight_ = std::min(1.0, 
                                             human_authority_weight_ + adaptation_rate_);
        } else if (feedback.efficiency < MIN_EFFICIENCY_THRESHOLD) {
            // Increase robot authority if human is struggling
            robot_authority_weight_ = std::min(1.0, 
                                             robot_authority_weight_ + adaptation_rate_);
        }
    }

private:
    struct HumanInput {
        HumanState state;
        ControlCommand command;
        std::string intention;
        double confidence;
        rclcpp::Time timestamp;
    };
    
    struct InteractionFeedback {
        double efficiency;
        double safety_score;
        double user_satisfaction;
        double task_completion_rate;
        rclcpp::Time timestamp;
    };
    
    struct AuthorityBalance {
        double human_weight;
        double robot_weight;
        std::string control_mode;  // "human_lead", "robot_lead", "collaborative"
        rclcpp::Time timestamp;
    };
    
    ControlCommand applySafetyConstraints(const ControlCommand& command,
                                       const RobotState& state,
                                       const EnvironmentalContext& context) {
        
        ControlCommand safe_command = command;
        
        // Apply velocity limits
        double vel_magnitude = std::sqrt(
            command.linear_velocity.x * command.linear_velocity.x +
            command.linear_velocity.y * command.linear_velocity.y +
            command.linear_velocity.z * command.linear_velocity.z);
        
        if (vel_magnitude > MAX_LINEAR_VELOCITY) {
            double scale = MAX_LINEAR_VELOCITY / vel_magnitude;
            safe_command.linear_velocity.x *= scale;
            safe_command.linear_velocity.y *= scale;
            safe_command.linear_velocity.z *= scale;
        }
        
        // Check for collisions
        if (wouldCauseCollision(safe_command, state, context)) {
            // Reduce velocity or change direction to avoid collision
            safe_command.linear_velocity.x *= 0.5;
            safe_command.linear_velocity.y *= 0.5;
            safe_command.angular_velocity.z *= 0.5;
        }
        
        return safe_command;
    }
    
    bool wouldCauseCollision(const ControlCommand& command,
                           const RobotState& state,
                           const EnvironmentalContext& context) {
        // Predict robot position after applying command
        geometry_msgs::msg::Point predicted_position;
        predicted_position.x = state.pose.position.x + 
                              command.linear_velocity.x * PREDICTION_TIME_STEP;
        predicted_position.y = state.pose.position.y + 
                              command.linear_velocity.y * PREDICTION_TIME_STEP;
        
        // Check if predicted position is in collision
        for (const auto& obj : context.objects) {
            double distance = calculateDistance(predicted_position, obj.position);
            if (distance < ROBOT_SAFETY_RADIUS + obj.bounding_radius) {
                return true;
            }
        }
        
        return false;
    }
    
    double calculateDistance(const geometry_msgs::msg::Point& p1,
                           const geometry_msgs::msg::Point& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    class ControlFusion {
    public:
        ControlCommand fuse(const ControlCommand& human_cmd,
                           const ControlCommand& robot_cmd,
                           const AuthorityBalance& balance) {
            
            ControlCommand fused_cmd;
            
            // Weighted fusion of linear velocities
            fused_cmd.linear_velocity.x = 
                balance.human_weight * human_cmd.linear_velocity.x +
                balance.robot_weight * robot_cmd.linear_velocity.x;
            
            fused_cmd.linear_velocity.y = 
                balance.human_weight * human_cmd.linear_velocity.y +
                balance.robot_weight * robot_cmd.linear_velocity.y;
            
            fused_cmd.linear_velocity.z = 
                balance.human_weight * human_cmd.linear_velocity.z +
                balance.robot_weight * robot_cmd.linear_velocity.z;
            
            // Weighted fusion of angular velocities
            fused_cmd.angular_velocity.x = 
                balance.human_weight * human_cmd.angular_velocity.x +
                balance.robot_weight * robot_cmd.angular_velocity.x;
            
            fused_cmd.angular_velocity.y = 
                balance.human_weight * human_cmd.angular_velocity.y +
                balance.robot_weight * robot_cmd.angular_velocity.y;
            
            fused_cmd.angular_velocity.z = 
                balance.human_weight * human_cmd.angular_velocity.z +
                balance.robot_weight * robot_cmd.angular_velocity.z;
            
            return fused_cmd;
        }
    };
    
    static constexpr double MAX_LINEAR_VELOCITY = 0.5;  // m/s
    static constexpr double PREDICTION_TIME_STEP = 0.1;  // seconds
    static constexpr double ROBOT_SAFETY_RADIUS = 0.3;  // meters
    static constexpr double OPTIMAL_EFFICIENCY_THRESHOLD = 0.8;
    static constexpr double MIN_EFFICIENCY_THRESHOLD = 0.3;
};
```

### Learning from Demonstration

#### Imitation Learning for HRI
```cpp
class LearningFromDemonstration {
private:
    std::unique_ptr<DemonstrationRecorder> demo_recorder_;
    std::unique_ptr<BehaviorCloning> behavior_cloner_;
    std::unique_ptr<InverseRL> inverse_rl_;
    std::unique_ptr<SkillExtractor> skill_extractor_;
    
    // Demonstration database
    std::vector<Demonstration> demonstration_database_;
    
    // Skill models
    std::map<std::string, SkillModel> skill_models_;
    
    // Imitation learning parameters
    double learning_rate_;
    int max_training_epochs_;
    double validation_split_;

public:
    LearningFromDemonstration(double lr = 0.001, int epochs = 100, 
                            double val_split = 0.2)
        : learning_rate_(lr), max_training_epochs_(epochs),
          validation_split_(val_split) {
        
        demo_recorder_ = std::make_unique<DemonstrationRecorder>();
        behavior_cloner_ = std::make_unique<NeuralBehaviorCloner>();
        inverse_rl_ = std::make_unique<MaxCausalEntropyIRL>();
        skill_extractor_ = std::make_unique<TemporalSkillExtractor>();
    }
    
    void recordDemonstration(const std::string& skill_name,
                           const HumanDemonstration& demonstration) {
        
        Demonstration demo;
        demo.skill_name = skill_name;
        demo.human_trajectory = demonstration.trajectory;
        demo.robot_trajectory = demonstration.robot_trajectory;
        demo.environment_state = demonstration.environment;
        demo.timestamp = this->now();
        demo.quality_score = evaluateDemonstrationQuality(demonstration);
        
        demonstration_database_.push_back(demo);
        
        // Update skill model with new demonstration
        updateSkillModel(skill_name, demo);
    }
    
    RobotTrajectory imitateBehavior(const std::string& skill_name,
                                  const EnvironmentalContext& context) {
        
        if (skill_models_.find(skill_name) == skill_models_.end()) {
            RCLCPP_WARN(this->get_logger(), 
                       "No model found for skill: %s", skill_name.c_str());
            return RobotTrajectory();  // Return empty trajectory
        }
        
        // Use the learned skill model to generate robot trajectory
        auto trajectory = skill_models_[skill_name].generateTrajectory(context);
        
        // Adapt trajectory to current environment
        auto adapted_trajectory = adaptTrajectoryToEnvironment(trajectory, context);
        
        return adapted_trajectory;
    }
    
    void updateSkillModel(const std::string& skill_name, const Demonstration& demo) {
        if (skill_models_.find(skill_name) == skill_models_.end()) {
            skill_models_[skill_name] = SkillModel();
        }
        
        // Add demonstration to model's training data
        skill_models_[skill_name].addTrainingData(demo);
        
        // Retrain model with new data
        skill_models_[skill_name].retrain();
    }

private:
    struct Demonstration {
        std::string skill_name;
        std::vector<HumanState> human_trajectory;
        std::vector<RobotState> robot_trajectory;
        EnvironmentalContext environment_state;
        double quality_score;
        rclcpp::Time timestamp;
    };
    
    struct HumanDemonstration {
        std::vector<HumanState> trajectory;
        std::vector<RobotState> robot_trajectory;
        EnvironmentalContext environment;
        std::string description;
    };
    
    struct SkillModel {
        std::vector<Demonstration> training_data;
        std::unique_ptr<NeuralNetwork> network;
        std::string skill_name;
        double performance_score;
        
        void addTrainingData(const Demonstration& demo) {
            training_data.push_back(demo);
        }
        
        void retrain() {
            // Prepare training data
            auto [inputs, targets] = prepareTrainingData();
            
            // Train neural network
            // Implementation would train the network
        }
        
        RobotTrajectory generateTrajectory(const EnvironmentalContext& context) {
            // Generate trajectory based on environment context
            RobotTrajectory trajectory;
            // Implementation would generate trajectory using trained model
            return trajectory;
        }
        
        std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>>
        prepareTrainingData() {
            std::vector<std::vector<double>> inputs;
            std::vector<std::vector<double>> targets;
            
            for (const auto& demo : training_data) {
                // Convert demonstration to training format
                for (size_t i = 0; i < demo.human_trajectory.size(); i++) {
                    if (i < demo.robot_trajectory.size()) {
                        std::vector<double> input = extractFeatures(
                            demo.human_trajectory[i], demo.environment_state);
                        std::vector<double> target = extractAction(
                            demo.robot_trajectory[i]);
                        
                        inputs.push_back(input);
                        targets.push_back(target);
                    }
                }
            }
            
            return std::make_pair(inputs, targets);
        }
        
        std::vector<double> extractFeatures(const HumanState& human_state,
                                          const EnvironmentalContext& env) {
            std::vector<double> features;
            
            // Add human state features
            features.push_back(human_state.position.x);
            features.push_back(human_state.position.y);
            features.push_back(human_state.position.z);
            features.push_back(human_state.velocity.x);
            features.push_back(human_state.velocity.y);
            features.push_back(human_state.velocity.z);
            
            // Add environment features
            for (const auto& obj : env.objects) {
                features.push_back(obj.position.x);
                features.push_back(obj.position.y);
                features.push_back(obj.position.z);
            }
            
            return features;
        }
        
        std::vector<double> extractAction(const RobotState& robot_state) {
            std::vector<double> action;
            
            // Extract robot action (velocity, joint positions, etc.)
            action.push_back(robot_state.linear_velocity);
            action.push_back(robot_state.angular_velocity);
            
            return action;
        }
    };
    
    double evaluateDemonstrationQuality(const HumanDemonstration& demo) {
        // Evaluate demonstration quality based on:
        // - Smoothness of motion
        // - Achievement of goal
        // - Safety considerations
        // - Efficiency
        
        double quality = 0.0;
        
        // Check if demonstration achieves the intended goal
        if (demo.trajectory.size() > 0) {
            quality += 0.4;  // Base score for completing demonstration
            
            // Check smoothness (low jerk)
            double jerk_score = calculateSmoothnessScore(demo.trajectory);
            quality += 0.3 * jerk_score;
            
            // Check safety (maintains safe distances)
            double safety_score = calculateSafetyScore(demo.trajectory, demo.environment);
            quality += 0.3 * safety_score;
        }
        
        return std::min(1.0, quality);
    }
    
    double calculateSmoothnessScore(const std::vector<HumanState>& trajectory) {
        if (trajectory.size() < 3) return 1.0;
        
        double total_jerk = 0.0;
        for (size_t i = 1; i < trajectory.size() - 1; i++) {
            // Calculate jerk (derivative of acceleration)
            auto prev_vel = getVelocity(trajectory[i-1], trajectory[i]);
            auto curr_vel = getVelocity(trajectory[i], trajectory[i+1]);
            
            double jerk = std::abs(curr_vel - prev_vel);  // Simplified
            total_jerk += jerk;
        }
        
        // Lower jerk = higher smoothness score
        return 1.0 / (1.0 + total_jerk / trajectory.size());
    }
    
    double calculateSafetyScore(const std::vector<HumanState>& trajectory,
                              const EnvironmentalContext& env) {
        double safety_score = 1.0;
        
        for (const auto& state : trajectory) {
            for (const auto& obj : env.objects) {
                double distance = calculateDistance(state.position, obj.position);
                if (distance < MIN_SAFE_DISTANCE) {
                    safety_score -= (MIN_SAFE_DISTANCE - distance) * PENALTY_FACTOR;
                }
            }
        }
        
        return std::max(0.0, safety_score);
    }
    
    double getVelocity(const HumanState& s1, const HumanState& s2) {
        double dt = (s2.timestamp - s1.timestamp).seconds();
        if (dt < 0.001) return 0.0;  // Avoid division by zero
        
        double dx = s2.position.x - s1.position.x;
        double dy = s2.position.y - s1.position.y;
        double dz = s2.position.z - s1.position.z;
        
        return std::sqrt(dx*dx + dy*dy + dz*dz) / dt;
    }
    
    RobotTrajectory adaptTrajectoryToEnvironment(const RobotTrajectory& original,
                                              const EnvironmentalContext& context) {
        // Adapt trajectory to avoid obstacles in current environment
        RobotTrajectory adapted = original;
        
        for (auto& point : adapted.points) {
            // Check for collisions with current environment objects
            for (const auto& obj : context.objects) {
                double distance = calculateDistance(point.position, obj.position);
                if (distance < MIN_SAFE_DISTANCE) {
                    // Adjust point to maintain safe distance
                    auto direction = calculateRepulsionVector(point.position, obj.position);
                    point.position.x += direction.x * (MIN_SAFE_DISTANCE - distance);
                    point.position.y += direction.y * (MIN_SAFE_DISTANCE - distance);
                    point.position.z += direction.z * (MIN_SAFE_DISTANCE - distance);
                }
            }
        }
        
        return adapted;
    }
    
    geometry_msgs::msg::Vector3 calculateRepulsionVector(
        const geometry_msgs::msg::Point& robot_pos,
        const geometry_msgs::msg::Point& obj_pos) {
        
        geometry_msgs::msg::Vector3 repulsion;
        repulsion.x = robot_pos.x - obj_pos.x;
        repulsion.y = robot_pos.y - obj_pos.y;
        repulsion.z = robot_pos.z - obj_pos.z;
        
        double norm = std::sqrt(repulsion.x*repulsion.x + 
                               repulsion.y*repulsion.y + 
                               repulsion.z*repulsion.z);
        
        if (norm > 0.01) {
            repulsion.x /= norm;
            repulsion.y /= norm;
            repulsion.z /= norm;
        }
        
        return repulsion;
    }
    
    static constexpr double MIN_SAFE_DISTANCE = 0.5;  // meters
    static constexpr double PENALTY_FACTOR = 2.0;
};
```

## Performance Evaluation

### HRI Metrics and Evaluation

#### Quantitative Metrics
```cpp
class HRIPerformanceEvaluator {
private:
    // Interaction metrics
    std::vector<InteractionMetric> interaction_metrics_;
    
    // User experience metrics
    std::vector<UserExperienceMetric> user_experience_metrics_;
    
    // Task performance metrics
    std::vector<TaskPerformanceMetric> task_performance_metrics_;
    
    // Safety metrics
    std::vector<SafetyMetric> safety_metrics_;
    
    // Long-term relationship metrics
    std::vector<RelationshipMetric> relationship_metrics_;

public:
    HRIPerformanceEvaluator() {
        initializeMetrics();
    }
    
    HRIEvaluationResults evaluateInteraction(
        const std::vector<InteractionEvent>& interaction_log,
        const UserFeedback& user_feedback) {
        
        HRIEvaluationResults results;
        
        // Calculate interaction metrics
        results.interaction_metrics = calculateInteractionMetrics(interaction_log);
        
        // Calculate user experience metrics
        results.user_experience_metrics = calculateUserExperienceMetrics(
            interaction_log, user_feedback);
        
        // Calculate task performance metrics
        results.task_performance_metrics = calculateTaskPerformanceMetrics(interaction_log);
        
        // Calculate safety metrics
        results.safety_metrics = calculateSafetyMetrics(interaction_log);
        
        // Calculate relationship metrics (if long-term interaction)
        results.relationship_metrics = calculateRelationshipMetrics(interaction_log);
        
        // Generate overall evaluation
        results.overall_score = calculateOverallScore(results);
        results.recommendations = generateRecommendations(results);
        
        return results;
    }

private:
    struct HRIEvaluationResults {
        std::vector<InteractionMetric> interaction_metrics;
        std::vector<UserExperienceMetric> user_experience_metrics;
        std::vector<TaskPerformanceMetric> task_performance_metrics;
        std::vector<SafetyMetric> safety_metrics;
        std::vector<RelationshipMetric> relationship_metrics;
        double overall_score;
        std::vector<std::string> recommendations;
        rclcpp::Time evaluation_time;
    };
    
    struct InteractionMetric {
        std::string name;
        double value;
        double weight;
        std::string description;
    };
    
    struct UserExperienceMetric {
        std::string name;
        double value;
        std::string source;  // "user_feedback", "behavior_analysis", etc.
    };
    
    struct TaskPerformanceMetric {
        std::string name;
        double value;
        bool success;
    };
    
    struct SafetyMetric {
        std::string name;
        double value;
        bool safety_violation;
    };
    
    struct RelationshipMetric {
        std::string name;
        double value;
        std::string trend;  // "improving", "declining", "stable"
    };
    
    struct InteractionEvent {
        std::string type;  // "speech", "gesture", "navigation", etc.
        std::string content;
        int user_id;
        rclcpp::Time timestamp;
        double confidence;
    };
    
    struct UserFeedback {
        int satisfaction_rating;  // 1-5 scale
        std::string comments;
        std::vector<std::string> suggestions;
        rclcpp::Time timestamp;
    };
    
    std::vector<InteractionMetric> calculateInteractionMetrics(
        const std::vector<InteractionEvent>& events) {
        
        std::vector<InteractionMetric> metrics;
        
        // Calculate interaction frequency
        auto frequency_metric = calculateInteractionFrequency(events);
        metrics.push_back(frequency_metric);
        
        // Calculate response time
        auto response_metric = calculateResponseTime(events);
        metrics.push_back(response_metric);
        
        // Calculate communication success rate
        auto success_metric = calculateCommunicationSuccess(events);
        metrics.push_back(success_metric);
        
        // Calculate engagement level
        auto engagement_metric = calculateEngagementLevel(events);
        metrics.push_back(engagement_metric);
        
        return metrics;
    }
    
    InteractionMetric calculateInteractionFrequency(
        const std::vector<InteractionEvent>& events) {
        
        InteractionMetric metric;
        metric.name = "interaction_frequency";
        metric.description = "Number of interactions per time unit";
        
        if (events.empty()) {
            metric.value = 0.0;
            return metric;
        }
        
        auto start_time = events.front().timestamp;
        auto end_time = events.back().timestamp;
        double duration = (end_time - start_time).seconds();
        
        metric.value = static_cast<double>(events.size()) / std::max(1.0, duration / 3600.0);  // Per hour
        metric.weight = 0.2;
        
        return metric;
    }
    
    InteractionMetric calculateResponseTime(
        const std::vector<InteractionEvent>& events) {
        
        InteractionMetric metric;
        metric.name = "average_response_time";
        metric.description = "Average time robot takes to respond to user input";
        
        double total_response_time = 0.0;
        int response_count = 0;
        
        for (size_t i = 0; i < events.size() - 1; i++) {
            if (events[i].type == "user_input" && events[i+1].type == "robot_response") {
                double response_time = (events[i+1].timestamp - events[i].timestamp).seconds();
                total_response_time += response_time;
                response_count++;
            }
        }
        
        metric.value = response_count > 0 ? total_response_time / response_count : 0.0;
        metric.weight = 0.3;
        
        return metric;
    }
    
    InteractionMetric calculateCommunicationSuccess(
        const std::vector<InteractionEvent>& events) {
        
        InteractionMetric metric;
        metric.name = "communication_success_rate";
        metric.description = "Percentage of communications successfully understood";
        
        int total_attempts = 0;
        int successful_attempts = 0;
        
        for (const auto& event : events) {
            if (event.type == "speech_recognition" || event.type == "gesture_recognition") {
                total_attempts++;
                if (event.confidence > 0.7) {  // Threshold for success
                    successful_attempts++;
                }
            }
        }
        
        metric.value = total_attempts > 0 ? 
                      static_cast<double>(successful_attempts) / total_attempts : 0.0;
        metric.value *= 100.0;  // Convert to percentage
        metric.weight = 0.25;
        
        return metric;
    }
    
    std::vector<UserExperienceMetric> calculateUserExperienceMetrics(
        const std::vector<InteractionEvent>& events,
        const UserFeedback& feedback) {
        
        std::vector<UserExperienceMetric> metrics;
        
        // Satisfaction rating
        UserExperienceMetric satisfaction;
        satisfaction.name = "user_satisfaction";
        satisfaction.value = feedback.satisfaction_rating / 5.0;  // Normalize to 0-1
        satisfaction.source = "user_feedback";
        metrics.push_back(satisfaction);
        
        // Perceived naturalness
        UserExperienceMetric naturalness;
        naturalness.name = "perceived_naturalness";
        naturalness.value = calculatePerceivedNaturalness(events);
        naturalness.source = "behavior_analysis";
        metrics.push_back(naturalness);
        
        // Trust level
        UserExperienceMetric trust;
        trust.name = "trust_level";
        trust.value = calculateTrustLevel(events);
        trust.source = "behavior_analysis";
        metrics.push_back(trust);
        
        return metrics;
    }
    
    double calculatePerceivedNaturalness(const std::vector<InteractionEvent>& events) {
        // Analyze the naturalness of interaction based on:
        // - Turn-taking patterns
        // - Response appropriateness
        // - Social cue usage
        
        double naturalness_score = 0.5;  // Base score
        
        // Count appropriate use of social cues (greetings, politeness, etc.)
        int social_cue_count = 0;
        for (const auto& event : events) {
            if (event.content.find("please") != std::string::npos ||
                event.content.find("thank") != std::string::npos ||
                event.content.find("hello") != std::string::npos) {
                social_cue_count++;
            }
        }
        
        naturalness_score += std::min(0.3, 
                                   static_cast<double>(social_cue_count) / events.size() * 2.0);
        
        return std::min(1.0, naturalness_score);
    }
    
    double calculateTrustLevel(const std::vector<InteractionEvent>& events) {
        // Calculate trust based on:
        // - Reliability of robot actions
        // - Consistency of behavior
        // - Safety record
        
        double trust_score = 0.5;  // Base score
        
        // Count successful task completions
        int successful_completions = 0;
        int total_tasks = 0;
        
        for (const auto& event : events) {
            if (event.type == "task_completion") {
                total_tasks++;
                if (event.confidence > 0.9) {  // High confidence indicates success
                    successful_completions++;
                }
            }
        }
        
        if (total_tasks > 0) {
            trust_score = static_cast<double>(successful_completions) / total_tasks;
        }
        
        return trust_score;
    }
    
    std::vector<TaskPerformanceMetric> calculateTaskPerformanceMetrics(
        const std::vector<InteractionEvent>& events) {
        
        std::vector<TaskPerformanceMetric> metrics;
        
        // Task completion rate
        TaskPerformanceMetric completion_rate;
        completion_rate.name = "task_completion_rate";
        completion_rate.success = true;
        // Implementation would calculate actual completion rate
        completion_rate.value = 0.85;  // Example value
        metrics.push_back(completion_rate);
        
        // Task efficiency
        TaskPerformanceMetric efficiency;
        efficiency.name = "task_efficiency";
        efficiency.success = true;
        // Implementation would calculate actual efficiency
        efficiency.value = 0.78;  // Example value
        metrics.push_back(efficiency);
        
        return metrics;
    }
    
    std::vector<SafetyMetric> calculateSafetyMetrics(
        const std::vector<InteractionEvent>& events) {
        
        std::vector<SafetyMetric> metrics;
        
        // Safety violation rate
        SafetyMetric violation_rate;
        violation_rate.name = "safety_violation_rate";
        violation_rate.safety_violation = false;
        // Implementation would check for safety violations
        violation_rate.value = 0.0;  // No violations
        metrics.push_back(violation_rate);
        
        // Proximity safety
        SafetyMetric proximity_safety;
        proximity_safety.name = "proximity_safety";
        proximity_safety.safety_violation = false;
        // Implementation would calculate safe distance maintenance
        proximity_safety.value = 0.95;  // 95% of time maintaining safe distance
        metrics.push_back(proximity_safety);
        
        return metrics;
    }
    
    double calculateOverallScore(const HRIEvaluationResults& results) {
        double overall_score = 0.0;
        double total_weight = 0.0;
        
        // Weighted average of all metrics
        for (const auto& metric : results.interaction_metrics) {
            overall_score += metric.value * metric.weight;
            total_weight += metric.weight;
        }
        
        // Add user experience metrics (equal weight)
        for (const auto& metric : results.user_experience_metrics) {
            overall_score += metric.value * 0.1;  // Equal weight for UX metrics
            total_weight += 0.1;
        }
        
        // Add task performance metrics
        for (const auto& metric : results.task_performance_metrics) {
            overall_score += metric.value * 0.15;
            total_weight += 0.15;
        }
        
        // Add safety metrics (very important)
        for (const auto& metric : results.safety_metrics) {
            overall_score += metric.value * 0.25;  // High weight for safety
            total_weight += 0.25;
        }
        
        return total_weight > 0 ? overall_score / total_weight : 0.0;
    }
    
    std::vector<std::string> generateRecommendations(const HRIEvaluationResults& results) {
        std::vector<std::string> recommendations;
        
        // Check if response time is too high
        auto response_time_it = std::find_if(
            results.interaction_metrics.begin(),
            results.interaction_metrics.end(),
            [](const InteractionMetric& m) { return m.name == "average_response_time"; });
        
        if (response_time_it != results.interaction_metrics.end() && 
            response_time_it->value > MAX_ACCEPTABLE_RESPONSE_TIME) {
            recommendations.push_back("Improve system response time through optimization");
        }
        
        // Check if communication success rate is low
        auto success_rate_it = std::find_if(
            results.interaction_metrics.begin(),
            results.interaction_metrics.end(),
            [](const InteractionMetric& m) { return m.name == "communication_success_rate"; });
        
        if (success_rate_it != results.interaction_metrics.end() && 
            success_rate_it->value < MIN_ACCEPTABLE_SUCCESS_RATE) {
            recommendations.push_back("Improve speech/gesture recognition accuracy");
        }
        
        // Check if user satisfaction is low
        auto satisfaction_it = std::find_if(
            results.user_experience_metrics.begin(),
            results.user_experience_metrics.end(),
            [](const UserExperienceMetric& m) { return m.name == "user_satisfaction"; });
        
        if (satisfaction_it != results.user_experience_metrics.end() && 
            satisfaction_it->value < MIN_ACCEPTABLE_SATISFACTION) {
            recommendations.push_back("Investigate causes of low user satisfaction");
        }
        
        return recommendations;
    }
    
    void initializeMetrics() {
        // Initialize with baseline metrics
    }
    
    static constexpr double MAX_ACCEPTABLE_RESPONSE_TIME = 2.0;  // seconds
    static constexpr double MIN_ACCEPTABLE_SUCCESS_RATE = 80.0;  // percent
    static constexpr double MIN_ACCEPTABLE_SATISFACTION = 0.6;  // normalized 0-1
};
```

## Troubleshooting Common HRI Issues

### Communication Problems

#### Speech Recognition Issues
- **Symptoms**: Robot not understanding spoken commands, frequent recognition errors
- **Causes**: Background noise, speaker distance, accent differences, model limitations
- **Solutions**: Noise reduction, speaker adaptation, multimodal input, confirmation requests
- **Tools**: Audio preprocessing, ASR confidence thresholds, feedback mechanisms

#### Gesture Recognition Problems
- **Symptoms**: Robot misinterpreting gestures, failure to recognize common gestures
- **Causes**: Limited training data, lighting conditions, occlusion, cultural differences
- **Solutions**: Diverse training data, robust feature extraction, user calibration
- **Validation**: Gesture recognition accuracy metrics, user feedback collection

#### Natural Language Understanding Issues
- **Symptoms**: Robot misunderstanding user intent, inappropriate responses
- **Causes**: Ambiguous language, limited domain knowledge, context unawareness
- **Solutions**: Context-aware NLU, dialogue management, clarification requests
- **Monitoring**: Intent recognition accuracy, user satisfaction metrics

### Safety and Comfort Issues

#### Proxemic Violations
- **Symptoms**: Robot entering personal space, user discomfort, safety alerts
- **Causes**: Inadequate spatial reasoning, sensor errors, cultural misunderstandings
- **Solutions**: Proxemic rules implementation, cultural adaptation, user preference learning
- **Prevention**: Safety zone monitoring, distance maintenance algorithms

#### Collision Avoidance Problems
- **Symptoms**: Robot getting too close to humans, near-collision events
- **Causes**: Sensor limitations, prediction errors, high-speed navigation
- **Solutions**: Improved prediction, conservative safety margins, emergency stopping
- **Monitoring**: Distance tracking, collision prediction, safety violation logging

#### Social Navigation Issues
- **Symptoms**: Robot blocking pathways, interrupting human activities, unnatural movement
- **Causes**: Lack of social rules, inadequate human behavior prediction
- **Solutions**: Social force models, human-aware path planning, behavioral adaptation
- **Evaluation**: Social norm compliance, human interruption metrics

### Performance Issues

#### Real-time Performance Problems
- **Symptoms**: Delayed responses, missed interaction opportunities, poor tracking
- **Causes**: Computational complexity, resource contention, inefficient algorithms
- **Solutions**: Algorithm optimization, parallel processing, priority scheduling
- **Monitoring**: Processing time tracking, deadline miss rates

#### System Integration Issues
- **Symptoms**: Inconsistent behavior across components, communication failures
- **Causes**: Interface mismatches, timing issues, component failures
- **Solutions**: Robust interface design, error handling, fallback mechanisms
- **Testing**: Integration testing, fault injection, system monitoring

## Best Practices

### Design Principles

#### User-Centered Design
- **Understand Users**: Study target users' needs, preferences, and limitations
- **Iterative Design**: Develop through iterative cycles of design, testing, and refinement
- **Accessibility**: Ensure interaction is accessible to users with different abilities
- **Cultural Sensitivity**: Adapt to different cultural norms and expectations

#### Transparency and Trust
- **Clear Communication**: Robot should clearly communicate its capabilities and intentions
- **Explainable AI**: Provide explanations for robot decisions when appropriate
- **Consistent Behavior**: Maintain consistent and predictable behavior
- **Error Handling**: Gracefully handle and communicate errors

### Implementation Guidelines

#### Safety-First Approach
- **Conservative Defaults**: Design safety-critical systems with conservative defaults
- **Multiple Safeguards**: Implement multiple layers of safety protection
- **Emergency Procedures**: Include clear emergency stop and recovery procedures
- **Continuous Monitoring**: Monitor safety metrics continuously during operation

#### Privacy Protection
- **Data Minimization**: Collect only necessary data for interaction
- **Consent Management**: Obtain and manage user consent appropriately
- **Secure Storage**: Protect collected data with appropriate security measures
- **Transparency**: Be transparent about data collection and usage

### Evaluation and Validation

#### Comprehensive Testing
- **Laboratory Testing**: Test components and algorithms in controlled environments
- **User Studies**: Conduct studies with target users to evaluate effectiveness
- **Long-term Studies**: Assess long-term interaction quality and user acceptance
- **Edge Case Testing**: Test with unusual or challenging interaction scenarios

#### Continuous Improvement
- **User Feedback**: Actively collect and incorporate user feedback
- **Performance Monitoring**: Continuously monitor interaction performance
- **Adaptive Systems**: Implement systems that adapt to individual users
- **Regular Updates**: Regularly update models and algorithms based on new data

## Future Developments

### Emerging Technologies

#### Advanced AI for HRI
- **Large Language Models**: Using LLMs for more natural and contextual conversations
- **Multimodal AI**: Advanced integration of multiple sensory modalities
- **Affective Computing**: Better recognition and response to human emotions
- **Theory of Mind**: AI systems that understand human mental states and beliefs

#### Social Robotics Advances
- **Social Intelligence**: Robots with enhanced social reasoning capabilities
- **Cultural Adaptation**: Automatic adaptation to different cultural contexts
- **Group Interaction**: Managing interactions with multiple humans simultaneously
- **Social Learning**: Robots that learn social behaviors from human observation

### Integration Approaches

#### Seamless Integration
- **Ambient Intelligence**: Robots that blend into the environment
- **Wearable Integration**: Integration with wearable devices for enhanced interaction
- **IoT Integration**: Integration with smart environments and IoT devices
- **Cloud Robotics**: Leveraging cloud resources for enhanced capabilities

#### Personalized Interaction
- **Individual Adaptation**: Personalizing interaction to individual users
- **Relationship Building**: Developing long-term relationships with users
- **Context Awareness**: Understanding and adapting to situational context
- **Proactive Assistance**: Anticipating and offering help before being asked

## Conclusion

Human-Robot Interaction represents a critical frontier in Physical AI systems, requiring sophisticated integration of perception, reasoning, communication, and control systems. Effective HRI systems must understand human social cues, communicate naturally, and adapt their behavior to different users and contexts while maintaining safety and trust.

The field of HRI draws from multiple disciplines and continues to evolve with advances in AI, robotics, and human-computer interaction. The success of HRI systems depends on careful attention to user needs, safety considerations, and the social dynamics that govern human-robot partnerships.

As robots become increasingly integrated into human environments, the principles and techniques covered in this chapter provide the foundation for developing systems that can interact naturally, safely, and effectively with humans. The future of HRI lies in systems that can understand human intentions, adapt to individual users, and form meaningful collaborative relationships.

## Exercises

1. Implement a multimodal communication system that integrates speech, gesture, and facial expression recognition for a robot.
2. Design and implement a proxemic-aware navigation system that respects human spatial comfort zones.
3. Create a shared autonomy system that dynamically adjusts control authority between human and robot based on task requirements and user capabilities.

## Further Reading

- Goodrich, M. A., & Schultz, A. C. (2007). "Human-robot interaction: a survey." Foundations and Trends in Human-Computer Interaction.
- Breazeal, C. (2003). "Toward sociable robots." Robotics and Autonomous Systems.
- Mataric, M. J. (2007). "Socially assistive robotics." IEEE Robotics & Automation Magazine.
- Fong, T., Nourbakhsh, I., & Dautenhahn, K. (2003). "A survey of socially interactive robots." Robotics and Autonomous Systems.
- Argall, B. D., & Billard, A. G. (2011). "A survey of robot learning from demonstration." Robotics and Autonomous Systems.