---
sidebar_label: Research Methodologies and Experimental Design
title: Research Methodologies and Experimental Design - Scientific Methods for Robotics Research
description: Understanding research methodologies and experimental design for Physical AI and robotics research
keywords: [research methods, experimental design, robotics research, scientific method, hypothesis testing, validation, reproducibility, methodology]
---

# 11.2 Research Methodologies and Experimental Design

## Introduction

Research methodologies and experimental design form the foundation of rigorous scientific inquiry in Physical AI and robotics. As the field advances and becomes increasingly complex, the need for systematic, reproducible, and scientifically sound research approaches becomes more critical. This chapter explores the principles and practices of conducting high-quality research in robotics, from formulating hypotheses to designing experiments, collecting data, and drawing valid conclusions.

Physical AI research presents unique challenges compared to traditional AI research. The integration of computational algorithms with physical systems introduces additional variables, uncertainties, and safety considerations. Research in this domain must account for the complexities of real-world environments, the variability of physical systems, and the need for safety and reliability. The methodologies discussed in this chapter provide frameworks for addressing these challenges while maintaining scientific rigor.

The importance of proper research methodology in Physical AI cannot be overstated. Poor methodology can lead to invalid conclusions, irreproducible results, and wasted resources. Conversely, rigorous methodology ensures that findings are reliable, generalizable, and contribute meaningfully to the advancement of the field. This chapter provides practical guidance for conducting research that meets the highest scientific standards.

## Scientific Method in Robotics Research

### Hypothesis Formation and Testing

The scientific method in robotics research follows the traditional scientific approach but must be adapted to account for the unique characteristics of physical systems:

#### Problem Identification
- **Observation**: Careful observation of phenomena in physical systems
- **Question Formulation**: Developing clear, testable research questions
- **Literature Review**: Comprehensive review of existing research and methodologies
- **Gap Identification**: Identifying unaddressed problems or limitations in current approaches

#### Hypothesis Development
- **Specificity**: Hypotheses must be specific and testable
- **Measurability**: Variables must be measurable and quantifiable
- **Falsifiability**: Hypotheses must be structured to be potentially falsified
- **Relevance**: Hypotheses should address meaningful research questions

#### Example: Hypothesis Development in Physical AI
```cpp
// Example of a research hypothesis in Physical AI
/*
Hypothesis: "Using deep reinforcement learning for robotic manipulation tasks will 
result in higher success rates compared to classical control methods in unstructured 
environments."

Variables:
- Independent: Control method (DRL vs Classical)
- Dependent: Success rate of manipulation tasks
- Controlled: Robot platform, objects, environment conditions
- Constants: Task complexity, safety constraints, evaluation criteria
*/

class ManipulationHypothesisTester {
private:
    std::unique_ptr<RobotPlatform> robot_platform_;
    std::unique_ptr<EnvironmentManager> env_manager_;
    std::unique_ptr<ExperimentalDesign> exp_design_;
    std::unique_ptr<StatisticalAnalyzer> stats_analyzer_;
    
    // Control and experimental groups
    std::unique_ptr<ClassicalControlSystem> classical_controller_;
    std::unique_ptr<DeepRLControlSystem> drl_controller_;
    
    // Data collection and analysis
    std::vector<ExperimentTrial> trial_results_;
    int num_trials_per_condition_;
    double statistical_significance_level_;

public:
    ManipulationHypothesisTester(int trials_per_condition = 100, 
                               double significance_level = 0.05)
        : num_trials_per_condition_(trials_per_condition),
          statistical_significance_level_(significance_level) {
        
        robot_platform_ = std::make_unique<FrankaEmikaPanda>();
        env_manager_ = std::make_unique<UnstructuredEnvironmentManager>();
        exp_design_ = std::make_unique<RandomizedBlockDesign>();
        stats_analyzer_ = std::make_unique<StatisticalAnalyzer>();
        
        classical_controller_ = std::make_unique<PIDController>();
        drl_controller_ = std::make_unique<DeepRLController>();
    }
    
    ResearchResult testHypothesis() {
        ResearchResult result;
        
        // Block randomization to control for environmental variations
        auto experimental_blocks = exp_design_->createBlocks(
            num_trials_per_condition_ * 2);  // 2 conditions (classical, DRL)
        
        for (const auto& block : experimental_blocks) {
            // Randomize order within block
            auto randomized_trials = exp_design_->randomizeTrials(block);
            
            for (const auto& trial : randomized_trials) {
                ExperimentTrial trial_result;
                
                if (trial.condition == Condition::CLASSICAL) {
                    trial_result = runClassicalControlTrial(trial);
                } else if (trial.condition == Condition::DRL) {
                    trial_result = runDRLControlTrial(trial);
                }
                
                trial_results_.push_back(trial_result);
            }
        }
        
        // Analyze results
        result.statistical_analysis = stats_analyzer_->compareGroups(
            getTrialsByCondition(Condition::CLASSICAL),
            getTrialsByCondition(Condition::DRL));
        
        result.effect_size = calculateEffectSize();
        result.confidence_interval = calculateConfidenceInterval();
        result.p_value = calculatePValue();
        result.null_hypothesis_rejected = result.p_value < statistical_significance_level_;
        
        // Additional analyses
        result.power_analysis = performPowerAnalysis();
        result.confounding_factors = identifyConfoundingFactors();
        
        return result;
    }

private:
    enum class Condition {
        CLASSICAL,
        DRL
    };
    
    struct ExperimentTrial {
        Condition condition;
        bool success;
        double success_rate;
        double execution_time;
        double energy_consumption;
        double safety_metrics;
        rclcpp::Time timestamp;
        EnvironmentState environment_state;
        RobotState robot_state;
        ManipulationTask task;
    };
    
    struct ResearchResult {
        StatisticalAnalysis statistical_analysis;
        double effect_size;
        ConfidenceInterval confidence_interval;
        double p_value;
        bool null_hypothesis_rejected;
        PowerAnalysis power_analysis;
        std::vector<std::string> confounding_factors;
        rclcpp::Time completion_time;
    };
    
    struct StatisticalAnalysis {
        double mean_classical;
        double mean_drl;
        double std_dev_classical;
        double std_dev_drl;
        double t_statistic;
        double degrees_of_freedom;
        double cohens_d;  // Effect size
    };
    
    struct ConfidenceInterval {
        double lower_bound;
        double upper_bound;
        double confidence_level;
    };
    
    struct PowerAnalysis {
        double power;
        double beta_error;
        double required_sample_size;
        double achieved_power;
    };
    
    struct EnvironmentState {
        double clutter_level;
        lighting_condition;
        surface_friction;
        object_properties;
    };
    
    struct RobotState {
        joint_positions;
        joint_velocities;
        end_effector_pose;
        gripper_state;
    };
    
    struct ManipulationTask {
        std::string task_type;  // "pick_and_place", "assembly", etc.
        ObjectProperties target_object;
        std::string success_criteria;
    };
    
    struct ObjectProperties {
        std::string shape;
        double weight;
        coefficient_of_friction;
        center_of_mass;
        dimensions;
    };
    
    ExperimentTrial runClassicalControlTrial(const TrialDefinition& trial_def) {
        // Setup environment
        env_manager_->setupEnvironment(trial_def.environment_state);
        
        // Load manipulation task
        auto task = createManipulationTask(trial_def.task_definition);
        
        // Execute with classical controller
        auto start_time = this->now();
        bool success = classical_controller_->executeTask(task);
        auto end_time = this->now();
        
        // Collect metrics
        ExperimentTrial result;
        result.condition = Condition::CLASSICAL;
        result.success = success;
        result.execution_time = end_time - start_time;
        result.energy_consumption = calculateEnergyConsumption();
        result.safety_metrics = calculateSafetyMetrics();
        result.timestamp = this->now();
        result.environment_state = trial_def.environment_state;
        result.robot_state = getRobotState();
        result.task = task;
        
        return result;
    }
    
    ExperimentTrial runDRLControlTrial(const TrialDefinition& trial_def) {
        // Setup environment
        env_manager_->setupEnvironment(trial_def.environment_state);
        
        // Load manipulation task
        auto task = createManipulationTask(trial_def.task_definition);
        
        // Execute with DRL controller
        auto start_time = this->now();
        bool success = drl_controller_->executeTask(task);
        auto end_time = this->now();
        
        // Collect metrics
        ExperimentTrial result;
        result.condition = Condition::DRL;
        result.success = success;
        result.execution_time = end_time - start_time;
        result.energy_consumption = calculateEnergyConsumption();
        result.safety_metrics = calculateSafetyMetrics();
        result.timestamp = this->now();
        result.environment_state = trial_def.environment_state;
        result.robot_state = getRobotState();
        result.task = task;
        
        return result;
    }
    
    std::vector<ExperimentTrial> getTrialsByCondition(Condition cond) {
        std::vector<ExperimentTrial> filtered_trials;
        
        for (const auto& trial : trial_results_) {
            if (trial.condition == cond) {
                filtered_trials.push_back(trial);
            }
        }
        
        return filtered_trials;
    }
    
    double calculateEffectSize() {
        auto classical_trials = getTrialsByCondition(Condition::CLASSICAL);
        auto drl_trials = getTrialsByCondition(Condition::DRL);
        
        double mean_classical = calculateMeanSuccessRate(classical_trials);
        double mean_drl = calculateMeanSuccessRate(drl_trials);
        double pooled_std_dev = calculatePooledStandardDeviation(classical_trials, drl_trials);
        
        // Cohen's d
        return (mean_drl - mean_classical) / pooled_std_dev;
    }
    
    ConfidenceInterval calculateConfidenceInterval() {
        // Calculate 95% confidence interval for the difference in means
        auto classical_trials = getTrialsByCondition(Condition::CLASSICAL);
        auto drl_trials = getTrialsByCondition(Condition::DRL);
        
        double mean_diff = calculateMeanSuccessRate(drl_trials) - 
                          calculateMeanSuccessRate(classical_trials);
        double std_err = calculateStandardError(classical_trials, drl_trials);
        
        // For 95% CI, z-score ≈ 1.96
        double margin_error = 1.96 * std_err;
        
        ConfidenceInterval ci;
        ci.lower_bound = mean_diff - margin_error;
        ci.upper_bound = mean_diff + margin_error;
        ci.confidence_level = 0.95;
        
        return ci;
    }
    
    double calculatePValue() {
        // Perform two-sample t-test
        auto classical_trials = getTrialsByCondition(Condition::CLASSICAL);
        auto drl_trials = getTrialsByCondition(Condition::DRL);
        
        double t_stat = calculateTStatistic(classical_trials, drl_trials);
        double df = calculateDegreesOfFreedom(classical_trials, drl_trials);
        
        // Calculate p-value from t-distribution
        return calculatePFromT(t_stat, df);
    }
    
    PowerAnalysis performPowerAnalysis() {
        // Calculate statistical power of the experiment
        PowerAnalysis power;
        power.power = 0.8;  // Conventionally 0.8 is considered adequate
        power.beta_error = 1.0 - power.power;
        power.required_sample_size = calculateRequiredSampleSize();
        power.achieved_power = calculateAchievedPower();
        
        return power;
    }
    
    std::vector<std::string> identifyConfoundingFactors() {
        std::vector<std::string> confounders;
        
        // Check for potential confounding factors
        auto environment_factors = analyzeEnvironmentalFactors();
        auto robot_factors = analyzeRobotFactors();
        auto operator_factors = analyzeOperatorFactors();  // If human operators involved
        
        confounders.insert(confounders.end(), 
                          environment_factors.begin(), environment_factors.end());
        confounders.insert(confounders.end(), 
                          robot_factors.begin(), robot_factors.end());
        confounders.insert(confounders.end(), 
                          operator_factors.begin(), operator_factors.end());
        
        return confounders;
    }
    
    std::vector<std::string> analyzeEnvironmentalFactors() {
        std::vector<std::string> factors;
        
        // Analyze environmental variables that might affect results
        if (hasSignificantEnvironmentalVariations()) {
            factors.push_back("Environmental conditions (lighting, temperature, etc.)");
        }
        
        if (equipment_wear_variation()) {
            factors.push_back("Equipment wear and tear");
        }
        
        return factors;
    }
    
    std::vector<std::string> analyzeRobotFactors() {
        std::vector<std::string> factors;
        
        // Analyze robot-specific factors
        if (calibration_drift()) {
            factors.push_back("Robot calibration drift");
        }
        
        if (performance_degradation()) {
            factors.push_back("Robot performance degradation over time");
        }
        
        return factors;
    }
    
    std::vector<std::string> analyzeOperatorFactors() {
        std::vector<std::string> factors;
        
        // If human operators are involved
        return factors;  // Placeholder
    }
    
    bool hasSignificantEnvironmentalVariations() {
        // Check if environmental conditions varied significantly
        // Implementation would analyze environmental logs
        return false;  // Placeholder
    }
    
    bool equipment_wear_variation() {
        // Check if equipment wear varied between trials
        return false;  // Placeholder
    }
    
    bool calibration_drift() {
        // Check if robot calibration drifted during experiment
        return false;  // Placeholder
    }
    
    bool performance_degradation() {
        // Check for robot performance degradation
        return false;  // Placeholder
    }
    
    double calculateMeanSuccessRate(const std::vector<ExperimentTrial>& trials) {
        if (trials.empty()) return 0.0;
        
        int successes = 0;
        for (const auto& trial : trials) {
            if (trial.success) successes++;
        }
        
        return static_cast<double>(successes) / trials.size();
    }
    
    double calculatePooledStandardDeviation(const std::vector<ExperimentTrial>& group1,
                                          const std::vector<ExperimentTrial>& group2) {
        double var1 = calculateVariance(group1);
        double var2 = calculateVariance(group2);
        int n1 = group1.size();
        int n2 = group2.size();
        
        // Pooled standard deviation
        double pooled_var = ((n1 - 1) * var1 + (n2 - 1) * var2) / (n1 + n2 - 2);
        return std::sqrt(pooled_var);
    }
    
    double calculateVariance(const std::vector<ExperimentTrial>& trials) {
        if (trials.size() < 2) return 0.0;
        
        double mean = calculateMeanSuccessRate(trials);
        double sum_sq_diff = 0.0;
        
        for (const auto& trial : trials) {
            double diff = trial.success ? 1.0 : 0.0 - mean;
            sum_sq_diff += diff * diff;
        }
        
        return sum_sq_diff / (trials.size() - 1);
    }
    
    double calculateStandardError(const std::vector<ExperimentTrial>& group1,
                                 const std::vector<ExperimentTrial>& group2) {
        double var1 = calculateVariance(group1);
        double var2 = calculateVariance(group2);
        int n1 = group1.size();
        int n2 = group2.size();
        
        // Standard error of difference between means
        return std::sqrt(var1/n1 + var2/n2);
    }
    
    double calculateTStatistic(const std::vector<ExperimentTrial>& group1,
                              const std::vector<ExperimentTrial>& group2) {
        double mean1 = calculateMeanSuccessRate(group1);
        double mean2 = calculateMeanSuccessRate(group2);
        double se_diff = calculateStandardError(group1, group2);
        
        if (se_diff == 0.0) return 0.0;
        
        return (mean2 - mean1) / se_diff;
    }
    
    double calculateDegreesOfFreedom(const std::vector<ExperimentTrial>& group1,
                                   const std::vector<ExperimentTrial>& group2) {
        int n1 = group1.size();
        int n2 = group2.size();
        return n1 + n2 - 2;
    }
    
    double calculatePFromT(double t_stat, double df) {
        // Approximate p-value calculation
        // In practice, would use statistical library
        return 0.05;  // Placeholder
    }
    
    int calculateRequiredSampleSize() {
        // Calculate required sample size based on desired power and effect size
        return num_trials_per_condition_;  // Placeholder
    }
    
    double calculateAchievedPower() {
        // Calculate achieved power based on sample size and effect size
        return 0.8;  // Placeholder
    }
    
    struct TrialDefinition {
        Condition condition;
        EnvironmentState environment_state;
        ManipulationTask task_definition;
        rclcpp::Time scheduled_time;
    };
};

// Statistical analyzer for experimental results
class StatisticalAnalyzer {
public:
    StatisticalComparison compareGroups(const std::vector<ExperimentTrial>& group1,
                                     const std::vector<ExperimentTrial>& group2) {
        
        StatisticalComparison comparison;
        
        // Calculate descriptive statistics
        comparison.group1_stats = calculateDescriptiveStats(group1);
        comparison.group2_stats = calculateDescriptiveStats(group2);
        
        // Perform inferential statistics
        comparison.inferential_stats = performInferentialAnalysis(group1, group2);
        
        // Calculate effect sizes
        comparison.effect_sizes = calculateEffectSizes(group1, group2);
        
        return comparison;
    }

private:
    struct StatisticalComparison {
        DescriptiveStatistics group1_stats;
        DescriptiveStatistics group2_stats;
        InferentialStatistics inferential_stats;
        EffectSizes effect_sizes;
    };
    
    struct DescriptiveStatistics {
        double mean;
        double median;
        double std_dev;
        double variance;
        double min_val;
        double max_val;
        double confidence_interval_lower;
        double confidence_interval_upper;
    };
    
    struct InferentialStatistics {
        double t_statistic;
        double p_value;
        double degrees_of_freedom;
        bool significant_difference;
        double confidence_level;
    };
    
    struct EffectSizes {
        double cohens_d;
        double glass_delta;
        double hedges_g;
        double eta_squared;
        double odds_ratio;
    };
    
    DescriptiveStatistics calculateDescriptiveStats(const std::vector<ExperimentTrial>& group) {
        DescriptiveStatistics stats;
        
        // Calculate basic statistics
        std::vector<double> values;
        for (const auto& trial : group) {
            values.push_back(trial.success ? 1.0 : 0.0);  // Convert success/failure to 1/0
        }
        
        if (!values.empty()) {
            stats.mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
            
            // Calculate variance and std dev
            double sum_sq_diff = 0.0;
            for (double val : values) {
                sum_sq_diff += (val - stats.mean) * (val - stats.mean);
            }
            stats.variance = sum_sq_diff / (values.size() - 1);
            stats.std_dev = std::sqrt(stats.variance);
            
            // Calculate min/max
            auto minmax = std::minmax_element(values.begin(), values.end());
            stats.min_val = *minmax.first;
            stats.max_val = *minmax.second;
            
            // Calculate median
            std::sort(values.begin(), values.end());
            if (values.size() % 2 == 0) {
                stats.median = (values[values.size()/2 - 1] + values[values.size()/2]) / 2.0;
            } else {
                stats.median = values[values.size()/2];
            }
            
            // Calculate 95% confidence interval
            double std_err = stats.std_dev / std::sqrt(values.size());
            stats.confidence_interval_lower = stats.mean - 1.96 * std_err;
            stats.confidence_interval_upper = stats.mean + 1.96 * std_err;
        }
        
        return stats;
    }
    
    InferentialStatistics performInferentialAnalysis(
        const std::vector<ExperimentTrial>& group1,
        const std::vector<ExperimentTrial>& group2) {
        
        InferentialStatistics stats;
        
        // Two-sample t-test
        auto [t_stat, p_value, df] = performTwoSampleTTest(group1, group2);
        
        stats.t_statistic = t_stat;
        stats.p_value = p_value;
        stats.degrees_of_freedom = df;
        stats.significant_difference = p_value < 0.05;  // Alpha level
        stats.confidence_level = 0.95;
        
        return stats;
    }
    
    std::tuple<double, double, double> performTwoSampleTTest(
        const std::vector<ExperimentTrial>& group1,
        const std::vector<ExperimentTrial>& group2) {
        
        // Calculate sample statistics
        auto stats1 = calculateDescriptiveStats(group1);
        auto stats2 = calculateDescriptiveStats(group2);
        
        int n1 = group1.size();
        int n2 = group2.size();
        
        // Calculate t-statistic
        double t_stat = (stats2.mean - stats1.mean) / 
                       std::sqrt(stats1.variance/n1 + stats2.variance/n2);
        
        // Calculate degrees of freedom (Welch's t-test)
        double df = std::pow(stats1.variance/n1 + stats2.variance/n2, 2) /
                   (std::pow(stats1.variance/n1, 2)/(n1-1) + 
                    std::pow(stats2.variance/n2, 2)/(n2-1));
        
        // Calculate p-value (approximate)
        double p_value = calculatePValueFromT(t_stat, df);
        
        return std::make_tuple(t_stat, p_value, df);
    }
    
    EffectSizes calculateEffectSizes(const std::vector<ExperimentTrial>& group1,
                                   const std::vector<ExperimentTrial>& group2) {
        
        EffectSizes effects;
        
        auto stats1 = calculateDescriptiveStats(group1);
        auto stats2 = calculateDescriptiveStats(group2);
        
        int n1 = group1.size();
        int n2 = group2.size();
        
        // Cohen's d
        double pooled_std = std::sqrt(((n1-1)*stats1.variance + (n2-1)*stats2.variance) / 
                                     (n1 + n2 - 2));
        effects.cohens_d = (stats2.mean - stats1.mean) / pooled_std;
        
        // Glass's delta (using group1's std dev)
        effects.glass_delta = (stats2.mean - stats1.mean) / stats1.std_dev;
        
        // Hedges' g (corrected for bias)
        double correction_factor = 1.0 - 3.0 / (4*(n1 + n2) - 9);
        effects.hedges_g = effects.cohens_d * correction_factor;
        
        // Eta-squared (for ANOVA context)
        // This is a simplified version
        effects.eta_squared = std::pow(effects.cohens_d, 2) / 
                             (std::pow(effects.cohens_d, 2) + (n1 + n2) / (n1 * n2));
        
        // Odds ratio (for binary outcomes)
        int successes1 = countSuccesses(group1);
        int failures1 = group1.size() - successes1;
        int successes2 = countSuccesses(group2);
        int failures2 = group2.size() - successes2;
        
        double odds1 = static_cast<double>(successes1) / std::max(1, failures1);
        double odds2 = static_cast<double>(successes2) / std::max(1, failures2);
        effects.odds_ratio = odds2 / odds1;
        
        return effects;
    }
    
    int countSuccesses(const std::vector<ExperimentTrial>& group) {
        int successes = 0;
        for (const auto& trial : group) {
            if (trial.success) successes++;
        }
        return successes;
    }
    
    double calculatePValueFromT(double t_stat, double df) {
        // Approximate p-value calculation
        // In practice, would use statistical library like Boost or GSL
        return 0.05;  // Placeholder
    }
};
```

### Reproducibility in Robotics Research

Reproducibility is a cornerstone of scientific research, but it presents unique challenges in robotics due to the integration of hardware, software, and real-world environments:

#### Hardware Variability
- **Component Differences**: Different robot platforms, sensors, and actuators
- **Calibration Variations**: Differences in calibration procedures and accuracy
- **Wear and Tear**: Physical degradation of components over time
- **Environmental Factors**: Temperature, humidity, lighting conditions

#### Software Complexity
- **Dependencies**: Complex software stacks with multiple dependencies
- **Configuration**: System configuration affecting performance
- **Randomness**: Stochastic algorithms requiring fixed seeds
- **Version Control**: Software versioning and updates

#### Experimental Design Considerations
```cpp
class ReproducibleExperimentFramework {
private:
    std::unique_ptr<ExperimentLogger> logger_;
    std::unique_ptr<ConfigurationManager> config_manager_;
    std::unique_ptr<HardwareAbstractionLayer> hal_;
    std::unique_ptr<RandomnessManager> randomness_manager_;
    
    // Metadata and provenance tracking
    std::unique_ptr<MetadataTracker> metadata_tracker_;
    std::unique_ptr<ProvenanceRecorder> provenance_recorder_;

public:
    ReproducibleExperimentFramework() {
        logger_ = std::make_unique<StructuredExperimentLogger>();
        config_manager_ = std::make_unique<YAMLConfigurationManager>();
        hal_ = std::make_unique<StandardizedHAL>();
        randomness_manager_ = std::make_unique<FixedSeedManager>();
        metadata_tracker_ = std::make_unique<ComprehensiveMetadataTracker>();
        provenance_recorder_ = std::make_unique<ProvenanceRecorder>();
    }
    
    ExperimentResult runReproducibleExperiment(const ExperimentProtocol& protocol) {
        // Log experiment setup
        logger_->logSetup(protocol);
        
        // Record hardware configuration
        auto hardware_config = hal_->getConfiguration();
        metadata_tracker_->recordHardwareConfig(hardware_config);
        
        // Set random seeds for reproducibility
        randomness_manager_->initializeSeeds(protocol.random_seeds);
        
        // Record software configuration
        auto software_config = config_manager_->getConfiguration();
        metadata_tracker_->recordSoftwareConfig(software_config);
        
        // Execute experiment protocol
        ExperimentResult result = executeProtocol(protocol);
        
        // Record environmental conditions
        auto env_conditions = measureEnvironment();
        metadata_tracker_->recordEnvironment(env_conditions);
        
        // Create comprehensive experiment record
        auto experiment_record = provenance_recorder_->createRecord(
            protocol, result, hardware_config, software_config, env_conditions);
        
        // Save complete experiment data
        logger_->saveCompleteExperiment(experiment_record);
        
        return result;
    }
    
    bool verifyReproducibility(const ExperimentResult& result1,
                             const ExperimentResult& result2,
                             double tolerance = 0.01) {
        
        // Compare key metrics within tolerance
        bool success_rates_match = std::abs(result1.success_rate - result2.success_rate) < tolerance;
        bool execution_times_match = std::abs(result1.mean_execution_time - result2.mean_execution_time) < tolerance;
        bool energy_consumption_match = std::abs(result1.mean_energy_consumption - result2.mean_energy_consumption) < tolerance;
        
        return success_rates_match && execution_times_match && energy_consumption_match;
    }

private:
    ExperimentResult executeProtocol(const ExperimentProtocol& protocol) {
        ExperimentResult result;
        
        for (const auto& trial : protocol.trials) {
            TrialResult trial_result = executeTrial(trial);
            result.trial_results.push_back(trial_result);
        }
        
        // Calculate aggregate metrics
        result.success_rate = calculateSuccessRate(result.trial_results);
        result.mean_execution_time = calculateMeanExecutionTime(result.trial_results);
        result.mean_energy_consumption = calculateMeanEnergyConsumption(result.trial_results);
        result.variance = calculateVariance(result.trial_results);
        
        return result;
    }
    
    TrialResult executeTrial(const TrialDefinition& trial) {
        // Execute single trial according to definition
        TrialResult result;
        
        // Setup trial conditions
        setupTrialEnvironment(trial.environment);
        
        // Execute task
        auto start_time = this->now();
        bool success = executeTask(trial.task);
        auto end_time = this->now();
        
        // Measure energy consumption
        double energy_used = measureEnergyConsumption();
        
        // Record trial result
        result.success = success;
        result.execution_time = end_time - start_time;
        result.energy_consumption = energy_used;
        result.task_complexity = trial.task.complexity;
        result.environment_challenges = trial.environment.challenges;
        
        return result;
    }
    
    EnvironmentState measureEnvironment() {
        EnvironmentState env;
        env.temperature = hal_->getTemperature();
        env.humidity = hal_->getHumidity();
        env.lighting_level = hal_->getLightingLevel();
        env.acoustic_noise = hal_->getAcousticNoise();
        env.magnetic_interference = hal_->getMagneticInterference();
        env.air_pressure = hal_->getAirPressure();
        env.floor_flatness = hal_->getFloorFlatness();
        env.surface_friction = hal_->getSurfaceFriction();
        
        return env;
    }
    
    double calculateSuccessRate(const std::vector<TrialResult>& results) {
        if (results.empty()) return 0.0;
        
        int successes = 0;
        for (const auto& result : results) {
            if (result.success) successes++;
        }
        
        return static_cast<double>(successes) / results.size();
    }
    
    double calculateMeanExecutionTime(const std::vector<TrialResult>& results) {
        if (results.empty()) return 0.0;
        
        double total_time = 0.0;
        for (const auto& result : results) {
            total_time += result.execution_time.seconds();
        }
        
        return total_time / results.size();
    }
    
    double calculateMeanEnergyConsumption(const std::vector<TrialResult>& results) {
        if (results.empty()) return 0.0;
        
        double total_energy = 0.0;
        for (const auto& result : results) {
            total_energy += result.energy_consumption;
        }
        
        return total_energy / results.size();
    }
    
    double calculateVariance(const std::vector<TrialResult>& results) {
        if (results.size() < 2) return 0.0;
        
        double mean = calculateSuccessRate(results);
        double sum_sq_diff = 0.0;
        
        for (const auto& result : results) {
            double value = result.success ? 1.0 : 0.0;
            double diff = value - mean;
            sum_sq_diff += diff * diff;
        }
        
        return sum_sq_diff / (results.size() - 1);
    }
    
    struct ExperimentProtocol {
        std::string name;
        std::string description;
        std::vector<TrialDefinition> trials;
        std::vector<int> random_seeds;
        std::string researcher;
        rclcpp::Time scheduled_time;
        std::vector<std::string> controlled_variables;
        std::vector<std::string> measured_variables;
        std::vector<std::string> potential_confounds;
    };
    
    struct TrialDefinition {
        TaskDefinition task;
        EnvironmentDefinition environment;
        RobotConfiguration robot_config;
        rclcpp::Time scheduled_time;
        std::vector<std::string> specific_conditions;
    };
    
    struct TaskDefinition {
        std::string type;
        std::string description;
        std::vector<std::string> success_criteria;
        double complexity;
        std::vector<std::string> required_capabilities;
    };
    
    struct EnvironmentDefinition {
        std::string type;
        std::vector<ObstacleDescription> obstacles;
        std::vector<ObjectDescription> objects;
        std::vector<std::string> challenges;
        LightingCondition lighting;
        double clutter_level;
    };
    
    struct RobotConfiguration {
        std::string platform;
        std::vector<SensorSpecification> sensors;
        std::vector<ActuatorSpecification> actuators;
        ControlAlgorithm control_algorithm;
        std::vector<std::string> software_versions;
    };
    
    struct ExperimentResult {
        std::vector<TrialResult> trial_results;
        double success_rate;
        double mean_execution_time;
        double mean_energy_consumption;
        double variance;
        rclcpp::Time completion_time;
        std::string notes;
    };
    
    struct TrialResult {
        bool success;
        rclcpp::Duration execution_time;
        double energy_consumption;
        double task_complexity;
        std::vector<std::string> environment_challenges;
        rclcpp::Time timestamp;
        std::string error_message;
    };
    
    struct EnvironmentState {
        double temperature;
        double humidity;
        double lighting_level;
        double acoustic_noise;
        double magnetic_interference;
        double air_pressure;
        double floor_flatness;
        double surface_friction;
        rclcpp::Time measurement_time;
    };
    
    struct HardwareConfiguration {
        std::string robot_model;
        std::string controller_firmware_version;
        std::vector<SensorInfo> sensors;
        std::vector<ActuatorInfo> actuators;
        std::vector<CalibrationInfo> calibrations;
        rclcpp::Time calibration_time;
    };
    
    struct SoftwareConfiguration {
        std::string os_version;
        std::string ros_version;
        std::vector<PackageInfo> packages;
        std::vector<LibraryInfo> libraries;
        rclcpp::Time snapshot_time;
    };
    
    struct ExperimentRecord {
        ExperimentProtocol protocol;
        ExperimentResult result;
        HardwareConfiguration hardware_config;
        SoftwareConfiguration software_config;
        EnvironmentState environment_state;
        rclcpp::Time start_time;
        rclcpp::Time end_time;
        std::string researcher_id;
        std::string lab_location;
        std::vector<std::string> notes;
    };
    
    struct ObstacleDescription {
        std::string type;
        geometry_msgs::msg::Pose pose;
        std::vector<double> dimensions;
    };
    
    struct ObjectDescription {
        std::string type;
        std::string material;
        double weight;
        std::vector<double> dimensions;
        geometry_msgs::msg::Pose pose;
    };
    
    struct LightingCondition {
        double intensity;
        double color_temperature;
        std::string type;
    };
    
    struct SensorSpecification {
        std::string type;
        std::string model;
        double accuracy;
        std::vector<double> field_of_view;
        double update_rate;
    };
    
    struct ActuatorSpecification {
        std::string type;
        std::string model;
        double max_force_torque;
        precision;
        max_velocity;
    };
    
    struct ControlAlgorithm {
        std::string name;
        std::string version;
        std::vector<Parameter> parameters;
    };
    
    struct Parameter {
        std::string name;
        std::string value;
        std::string type;
    };
    
    struct SensorInfo {
        std::string name;
        std::string type;
        std::string serial_number;
        std::string firmware_version;
    };
    
    struct ActuatorInfo {
        std::string name;
        std::string type;
        std::string serial_number;
        std::string firmware_version;
    };
    
    struct CalibrationInfo {
        std::string component_name;
        rclcpp::Time calibration_time;
        std::vector<double> parameters;
        double accuracy_after_calibration;
    };
    
    struct PackageInfo {
        std::string name;
        std::string version;
        std::string repository;
    };
    
    struct LibraryInfo {
        std::string name;
        std::string version;
        std::string license;
    };
};

// Hardware abstraction layer for reproducibility
class StandardizedHAL {
public:
    HardwareConfiguration getConfiguration() {
        HardwareConfiguration config;
        config.robot_model = getRobotModel();
        config.controller_firmware_version = getFirmwareVersion();
        config.sensors = getSensors();
        config.actuators = getActuators();
        config.calibrations = getCalibrations();
        config.calibration_time = getLastCalibrationTime();
        
        return config;
    }
    
    double getTemperature() { /* Implementation */ return 22.5; }
    double getHumidity() { /* Implementation */ return 45.0; }
    double getLightingLevel() { /* Implementation */ return 500.0; }
    double getAcousticNoise() { /* Implementation */ return 35.0; }
    double getMagneticInterference() { /* Implementation */ return 0.1; }
    double getAirPressure() { /* Implementation */ return 1013.25; }
    double getFloorFlatness() { /* Implementation */ return 0.98; }
    double getSurfaceFriction() { /* Implementation */ return 0.6; }

private:
    std::string getRobotModel() { /* Implementation */ return "Franka Emika Panda"; }
    std::string getFirmwareVersion() { /* Implementation */ return "4.1.0"; }
    
    std::vector<SensorInfo> getSensors() { 
        /* Implementation */ 
        return std::vector<SensorInfo>(); 
    }
    
    std::vector<ActuatorInfo> getActuators() { 
        /* Implementation */ 
        return std::vector<ActuatorInfo>(); 
    }
    
    std::vector<CalibrationInfo> getCalibrations() { 
        /* Implementation */ 
        return std::vector<CalibrationInfo>(); 
    }
    
    rclcpp::Time getLastCalibrationTime() { 
        /* Implementation */ 
        return this->now(); 
    }
};
```

## Experimental Design Principles

### Controlled Experiments

#### Randomization and Blocking
```cpp
class ExperimentalDesignManager {
private:
    std::unique_ptr<RandomizationScheme> randomizer_;
    std::unique_ptr<BlockDesign> block_designer_;
    std::unique_ptr<FactorialDesigner> factorial_designer_;
    std::unique_ptr<CovariateManager> covariate_manager_;

public:
    ExperimentalDesignManager() {
        randomizer_ = std::make_unique<StratifiedRandomizer>();
        block_designer_ = std::make_unique<RandomizedBlockDesigner>();
        factorial_designer_ = std::make_unique<FactorialExperimentDesigner>();
        covariate_manager_ = std::make_unique<CovariateControlManager>();
    }
    
    std::vector<TrialAssignment> designExperiment(const ExperimentalParameters& params) {
        std::vector<TrialAssignment> assignments;
        
        // Create blocks to control for environmental variations
        auto blocks = block_designer_->createBlocks(params);
        
        for (auto& block : blocks) {
            // Randomize treatment assignment within each block
            auto randomized_block = randomizer_->randomizeBlock(block, params);
            
            // Add to assignments
            assignments.insert(assignments.end(), 
                             randomized_block.begin(), randomized_block.end());
        }
        
        // Apply factorial design if multiple factors
        if (params.factorial_design) {
            assignments = factorial_designer_->applyFactorialDesign(assignments, params);
        }
        
        // Control for covariates
        assignments = covariate_manager_->controlForCovariates(assignments, params);
        
        return assignments;
    }

private:
    struct ExperimentalParameters {
        int total_trials;
        std::vector<std::string> treatments;
        std::vector<std::string> blocking_factors;
        std::vector<std::string> covariates;
        std::vector<std::string> factors;  // For factorial design
        bool factorial_design;
        double significance_level;
        double power;
        std::string randomization_scheme;
        std::string blocking_scheme;
    };
    
    struct TrialAssignment {
        int trial_id;
        std::string treatment;
        std::vector<std::string> blocking_levels;
        std::vector<std::string> factor_levels;
        rclcpp::Time scheduled_time;
        std::string notes;
    };
    
    class RandomizationScheme {
    public:
        virtual std::vector<TrialAssignment> randomizeBlock(
            const std::vector<TrialAssignment>& block,
            const ExperimentalParameters& params) = 0;
    };
    
    class StratifiedRandomizer : public RandomizationScheme {
    public:
        std::vector<TrialAssignment> randomizeBlock(
            const std::vector<TrialAssignment>& block,
            const ExperimentalParameters& params) override {
            
            std::vector<TrialAssignment> randomized_block = block;
            
            // Ensure equal allocation of treatments within block
            std::vector<std::string> treatments = params.treatments;
            
            // Shuffle treatments for assignment
            std::shuffle(treatments.begin(), treatments.end(), 
                        std::mt19937(std::random_device{}()));
            
            // Assign treatments cyclically to ensure balance
            for (size_t i = 0; i < randomized_block.size(); i++) {
                randomized_block[i].treatment = 
                    treatments[i % treatments.size()];
            }
            
            return randomized_block;
        }
    };
    
    class BlockDesign {
    public:
        virtual std::vector<std::vector<TrialAssignment>> createBlocks(
            const ExperimentalParameters& params) = 0;
    };
    
    class RandomizedBlockDesigner : public BlockDesign {
    public:
        std::vector<std::vector<TrialAssignment>> createBlocks(
            const ExperimentalParameters& params) override {
            
            std::vector<std::vector<TrialAssignment>> blocks;
            
            // Calculate number of blocks based on blocking factors
            int num_blocks = calculateNumberOfBlocks(params);
            int trials_per_block = params.total_trials / num_blocks;
            
            // Create blocks
            for (int block_id = 0; block_id < num_blocks; block_id++) {
                std::vector<TrialAssignment> block;
                
                for (int i = 0; i < trials_per_block; i++) {
                    TrialAssignment assignment;
                    assignment.trial_id = block_id * trials_per_block + i;
                    assignment.blocking_levels = getBlockingLevelsForBlock(block_id, params);
                    assignment.scheduled_time = calculateScheduledTime(block_id, i);
                    
                    block.push_back(assignment);
                }
                
                blocks.push_back(block);
            }
            
            return blocks;
        }

    private:
        int calculateNumberOfBlocks(const ExperimentalParameters& params) {
            // Calculate optimal number of blocks
            // This is a simplified approach
            return std::max(1, params.total_trials / 10);  // Max 10 trials per block
        }
        
        std::vector<std::string> getBlockingLevelsForBlock(
            int block_id, const ExperimentalParameters& params) {
            
            std::vector<std::string> levels;
            
            // Assign blocking factor levels to each block
            for (const auto& factor : params.blocking_factors) {
                // In practice, this would map block_id to specific factor levels
                levels.push_back("level_" + std::to_string(block_id % 3));  // Example
            }
            
            return levels;
        }
        
        rclcpp::Time calculateScheduledTime(int block_id, int trial_in_block) {
            // Calculate scheduled time based on block and trial position
            // This helps control for time-based confounds
            return rclcpp::Time(0) + 
                   rclcpp::Duration::from_seconds(block_id * 3600 + trial_in_block * 300);
        }
    };
    
    class FactorialDesigner {
    public:
        virtual std::vector<TrialAssignment> applyFactorialDesign(
            const std::vector<TrialAssignment>& assignments,
            const ExperimentalParameters& params) = 0;
    };
    
    class FactorialExperimentDesigner : public FactorialDesigner {
    public:
        std::vector<TrialAssignment> applyFactorialDesign(
            const std::vector<TrialAssignment>& assignments,
            const ExperimentalParameters& params) override {
            
            std::vector<TrialAssignment> factorial_assignments;
            
            // Generate all combinations of factor levels
            auto factor_combinations = generateFactorCombinations(params.factors);
            
            // Assign factor combinations to trials
            for (size_t i = 0; i < assignments.size(); i++) {
                TrialAssignment assignment = assignments[i];
                
                // Cycle through factor combinations
                assignment.factor_levels = factor_combinations[
                    i % factor_combinations.size()];
                
                factorial_assignments.push_back(assignment);
            }
            
            return factorial_assignments;
        }

    private:
        std::vector<std::vector<std::string>> generateFactorCombinations(
            const std::vector<std::string>& factors) {
            
            std::vector<std::vector<std::string>> combinations;
            
            // For simplicity, assuming 2 levels per factor (high/low)
            // In practice, this would handle arbitrary numbers of levels
            std::vector<std::vector<int>> level_indices = 
                generateBinaryCombinations(factors.size());
            
            for (const auto& indices : level_indices) {
                std::vector<std::string> combination;
                for (size_t i = 0; i < factors.size(); i++) {
                    combination.push_back(
                        indices[i] == 0 ? "low" : "high");
                }
                combinations.push_back(combination);
            }
            
            return combinations;
        }
        
        std::vector<std::vector<int>> generateBinaryCombinations(int num_factors) {
            std::vector<std::vector<int>> combinations;
            int total_combinations = 1 << num_factors;  // 2^num_factors
            
            for (int i = 0; i < total_combinations; i++) {
                std::vector<int> combination;
                for (int j = 0; j < num_factors; j++) {
                    combination.push_back((i >> j) & 1);
                }
                combinations.push_back(combination);
            }
            
            return combinations;
        }
    };
    
    class CovariateManager {
    public:
        virtual std::vector<TrialAssignment> controlForCovariates(
            const std::vector<TrialAssignment>& assignments,
            const ExperimentalParameters& params) = 0;
    };
    
    class CovariateControlManager : public CovariateManager {
    public:
        std::vector<TrialAssignment> controlForCovariates(
            const std::vector<TrialAssignment>& assignments,
            const ExperimentalParameters& params) override {
            
            // Implement covariate control strategies
            // This could include matching, stratification, or statistical control
            
            std::vector<TrialAssignment> controlled_assignments = assignments;
            
            // For example, ensure covariates are balanced across treatment groups
            balanceCovariates(controlled_assignments, params.covariates);
            
            return controlled_assignments;
        }

    private:
        void balanceCovariates(std::vector<TrialAssignment>& assignments,
                              const std::vector<std::string>& covariates) {
            
            // Implementation would ensure covariates are balanced
            // across treatment groups through re-randomization or other methods
        }
    };
};
```

### Cross-Validation in Physical AI

Cross-validation in Physical AI requires special considerations due to the physical nature of experiments:

```cpp
class PhysicalAICrossValidator {
private:
    std::unique_ptr<EnvironmentVariator> env_variator_;
    std::unique_ptr<RobotVariator> robot_variator_;
    std::unique_ptr<TaskVariator> task_variator_;
    std::unique_ptr<StatisticalAnalyzer> stats_analyzer_;

public:
    PhysicalAICrossValidator() {
        env_variator_ = std::make_unique<EnvironmentCrossValidator>();
        robot_variator_ = std::make_unique<RobotPlatformCrossValidator>();
        task_variator_ = std::make_unique<TaskCrossValidator>();
        stats_analyzer_ = std::make_unique<RobustStatisticalAnalyzer>();
    }
    
    CrossValidationResult performPhysicalCrossValidation(
        const RobotAlgorithm& algorithm,
        const std::vector<Environment>& environments,
        const std::vector<RobotPlatform>& robot_platforms,
        const std::vector<Task>& tasks) {
        
        CrossValidationResult result;
        
        // Environment-wise cross-validation
        auto env_results = env_variator_->validateAcrossEnvironments(
            algorithm, environments);
        result.environment_validation = env_results;
        
        // Robot-platform-wise cross-validation
        auto robot_results = robot_variator_->validateAcrossPlatforms(
            algorithm, robot_platforms);
        result.robot_validation = robot_results;
        
        // Task-wise cross-validation
        auto task_results = task_variator_->validateAcrossTasks(
            algorithm, tasks);
        result.task_validation = task_results;
        
        // Combined analysis
        result.overall_performance = calculateOverallPerformance(
            env_results, robot_results, task_results);
        result.robustness_metrics = calculateRobustnessMetrics(
            env_results, robot_results, task_results);
        
        return result;
    }

private:
    struct CrossValidationResult {
        ValidationResults environment_validation;
        ValidationResults robot_validation;
        ValidationResults task_validation;
        PerformanceMetrics overall_performance;
        RobustnessMetrics robustness_metrics;
        rclcpp::Time completion_time;
    };
    
    struct ValidationResults {
        std::vector<TrialResult> fold_results;
        double mean_performance;
        double std_dev;
        double confidence_interval_lower;
        double confidence_interval_upper;
        std::vector<std::string> failure_modes;
        std::vector<std::string> success_factors;
    };
    
    struct PerformanceMetrics {
        double mean_success_rate;
        double mean_execution_time;
        double mean_energy_efficiency;
        double generalization_score;
        double robustness_index;
    };
    
    struct RobustnessMetrics {
        double environment_robustness;
        double platform_robustness;
        double task_robustness;
        double overall_robustness;
        std::vector<RobustnessFactor> contributing_factors;
    };
    
    struct RobustnessFactor {
        std::string factor_name;
        double contribution_to_robustness;
        double sensitivity_measure;
    };
    
    class EnvironmentCrossValidator {
    public:
        ValidationResults validateAcrossEnvironments(
            const RobotAlgorithm& algorithm,
            const std::vector<Environment>& environments) {
            
            ValidationResults results;
            
            for (size_t i = 0; i < environments.size(); i++) {
                // Use environment i as test set, others as training
                std::vector<Environment> training_envs;
                for (size_t j = 0; j < environments.size(); j++) {
                    if (i != j) training_envs.push_back(environments[j]);
                }
                
                // Train algorithm on training environments
                auto trained_algorithm = trainOnEnvironments(algorithm, training_envs);
                
                // Test on held-out environment
                auto test_result = testOnEnvironment(trained_algorithm, environments[i]);
                results.fold_results.push_back(test_result);
            }
            
            // Calculate aggregate metrics
            results.mean_performance = calculateMeanPerformance(results.fold_results);
            results.std_dev = calculateStdDev(results.fold_results);
            results.confidence_interval_lower = calculateLowerCI(results.fold_results);
            results.confidence_interval_upper = calculateUpperCI(results.fold_results);
            
            return results;
        }

    private:
        RobotAlgorithm trainOnEnvironments(const RobotAlgorithm& base_algorithm,
                                          const std::vector<Environment>& environments) {
            
            RobotAlgorithm trained_algorithm = base_algorithm;
            
            // Train algorithm on multiple environments
            for (const auto& env : environments) {
                auto training_data = generateTrainingData(env);
                trained_algorithm.train(training_data);
            }
            
            return trained_algorithm;
        }
        
        TrialResult testOnEnvironment(const RobotAlgorithm& algorithm,
                                     const Environment& environment) {
            
            TrialResult result;
            
            // Run algorithm in environment
            int total_trials = 100;  // Example
            int successes = 0;
            double total_time = 0.0;
            
            for (int i = 0; i < total_trials; i++) {
                auto task_result = runSingleTrial(algorithm, environment);
                if (task_result.success) successes++;
                total_time += task_result.execution_time.seconds();
            }
            
            result.success = static_cast<double>(successes) / total_trials;
            result.execution_time = rclcpp::Duration::from_seconds(
                total_time / total_trials);
            
            return result;
        }
        
        std::vector<TrialData> generateTrainingData(const Environment& environment) {
            // Generate training data for the environment
            return std::vector<TrialData>();  // Placeholder
        }
        
        TrialResult runSingleTrial(const RobotAlgorithm& algorithm,
                                  const Environment& environment) {
            // Run a single trial in the environment
            return TrialResult();  // Placeholder
        }
        
        double calculateMeanPerformance(const std::vector<TrialResult>& results) {
            if (results.empty()) return 0.0;
            
            double sum = 0.0;
            for (const auto& result : results) {
                sum += result.success ? 1.0 : 0.0;
            }
            return sum / results.size();
        }
        
        double calculateStdDev(const std::vector<TrialResult>& results) {
            if (results.size() < 2) return 0.0;
            
            double mean = calculateMeanPerformance(results);
            double sum_sq_diff = 0.0;
            
            for (const auto& result : results) {
                double value = result.success ? 1.0 : 0.0;
                double diff = value - mean;
                sum_sq_diff += diff * diff;
            }
            
            return std::sqrt(sum_sq_diff / (results.size() - 1));
        }
        
        double calculateLowerCI(const std::vector<TrialResult>& results) {
            // Calculate lower bound of confidence interval
            double mean = calculateMeanPerformance(results);
            double std_err = calculateStdDev(results) / std::sqrt(results.size());
            return mean - 1.96 * std_err;  // 95% CI
        }
        
        double calculateUpperCI(const std::vector<TrialResult>& results) {
            // Calculate upper bound of confidence interval
            double mean = calculateMeanPerformance(results);
            double std_err = calculateStdDev(results) / std::sqrt(results.size());
            return mean + 1.96 * std_err;  // 95% CI
        }
    };
    
    class RobotPlatformCrossValidator {
    public:
        ValidationResults validateAcrossPlatforms(
            const RobotAlgorithm& algorithm,
            const std::vector<RobotPlatform>& platforms) {
            
            ValidationResults results;
            
            for (size_t i = 0; i < platforms.size(); i++) {
                // Use platform i as test set, others as training
                std::vector<RobotPlatform> training_platforms;
                for (size_t j = 0; j < platforms.size(); j++) {
                    if (i != j) training_platforms.push_back(platforms[j]);
                }
                
                // Train algorithm on training platforms
                auto trained_algorithm = trainOnPlatforms(algorithm, training_platforms);
                
                // Test on held-out platform
                auto test_result = testOnPlatform(trained_algorithm, platforms[i]);
                results.fold_results.push_back(test_result);
            }
            
            // Calculate aggregate metrics
            results.mean_performance = calculateMeanPerformance(results.fold_results);
            results.std_dev = calculateStdDev(results.fold_results);
            results.confidence_interval_lower = calculateLowerCI(results.fold_results);
            results.confidence_interval_upper = calculateUpperCI(results.fold_results);
            
            return results;
        }

    private:
        RobotAlgorithm trainOnPlatforms(const RobotAlgorithm& base_algorithm,
                                       const std::vector<RobotPlatform>& platforms) {
            
            RobotAlgorithm trained_algorithm = base_algorithm;
            
            // Train algorithm on multiple platforms
            for (const auto& platform : platforms) {
                auto training_data = generateTrainingData(platform);
                trained_algorithm.train(training_data);
            }
            
            return trained_algorithm;
        }
        
        TrialResult testOnPlatform(const RobotAlgorithm& algorithm,
                                  const RobotPlatform& platform) {
            
            TrialResult result;
            
            // Run algorithm on platform
            int total_trials = 100;  // Example
            int successes = 0;
            double total_time = 0.0;
            
            for (int i = 0; i < total_trials; i++) {
                auto task_result = runSingleTrial(algorithm, platform);
                if (task_result.success) successes++;
                total_time += task_result.execution_time.seconds();
            }
            
            result.success = static_cast<double>(successes) / total_trials;
            result.execution_time = rclcpp::Duration::from_seconds(
                total_time / total_trials);
            
            return result;
        }
        
        std::vector<TrialData> generateTrainingData(const RobotPlatform& platform) {
            // Generate training data for the platform
            return std::vector<TrialData>();  // Placeholder
        }
        
        TrialResult runSingleTrial(const RobotAlgorithm& algorithm,
                                  const RobotPlatform& platform) {
            // Run a single trial on the platform
            return TrialResult();  // Placeholder
        }
        
        double calculateMeanPerformance(const std::vector<TrialResult>& results) {
            if (results.empty()) return 0.0;
            
            double sum = 0.0;
            for (const auto& result : results) {
                sum += result.success ? 1.0 : 0.0;
            }
            return sum / results.size();
        }
        
        double calculateStdDev(const std::vector<TrialResult>& results) {
            if (results.size() < 2) return 0.0;
            
            double mean = calculateMeanPerformance(results);
            double sum_sq_diff = 0.0;
            
            for (const auto& result : results) {
                double value = result.success ? 1.0 : 0.0;
                double diff = value - mean;
                sum_sq_diff += diff * diff;
            }
            
            return std::sqrt(sum_sq_diff / (results.size() - 1));
        }
        
        double calculateLowerCI(const std::vector<TrialResult>& results) {
            double mean = calculateMeanPerformance(results);
            double std_err = calculateStdDev(results) / std::sqrt(results.size());
            return mean - 1.96 * std_err;  // 95% CI
        }
        
        double calculateUpperCI(const std::vector<TrialResult>& results) {
            double mean = calculateMeanPerformance(results);
            double std_err = calculateStdDev(results) / std::sqrt(results.size());
            return mean + 1.96 * std_err;  // 95% CI
        }
    };
    
    class TaskCrossValidator {
    public:
        ValidationResults validateAcrossTasks(
            const RobotAlgorithm& algorithm,
            const std::vector<Task>& tasks) {
            
            ValidationResults results;
            
            for (size_t i = 0; i < tasks.size(); i++) {
                // Use task i as test set, others as training
                std::vector<Task> training_tasks;
                for (size_t j = 0; j < tasks.size(); j++) {
                    if (i != j) training_tasks.push_back(tasks[j]);
                }
                
                // Train algorithm on training tasks
                auto trained_algorithm = trainOnTasks(algorithm, training_tasks);
                
                // Test on held-out task
                auto test_result = testOnTask(trained_algorithm, tasks[i]);
                results.fold_results.push_back(test_result);
            }
            
            // Calculate aggregate metrics
            results.mean_performance = calculateMeanPerformance(results.fold_results);
            results.std_dev = calculateStdDev(results.fold_results);
            results.confidence_interval_lower = calculateLowerCI(results.fold_results);
            results.confidence_interval_upper = calculateUpperCI(results.fold_results);
            
            return results;
        }

    private:
        RobotAlgorithm trainOnTasks(const RobotAlgorithm& base_algorithm,
                                   const std::vector<Task>& tasks) {
            
            RobotAlgorithm trained_algorithm = base_algorithm;
            
            // Train algorithm on multiple tasks
            for (const auto& task : tasks) {
                auto training_data = generateTrainingData(task);
                trained_algorithm.train(training_data);
            }
            
            return trained_algorithm;
        }
        
        TrialResult testOnTask(const RobotAlgorithm& algorithm,
                              const Task& task) {
            
            TrialResult result;
            
            // Run algorithm on task
            int total_trials = 100;  // Example
            int successes = 0;
            double total_time = 0.0;
            
            for (int i = 0; i < total_trials; i++) {
                auto task_result = runSingleTrial(algorithm, task);
                if (task_result.success) successes++;
                total_time += task_result.execution_time.seconds();
            }
            
            result.success = static_cast<double>(successes) / total_trials;
            result.execution_time = rclcpp::Duration::from_seconds(
                total_time / total_trials);
            
            return result;
        }
        
        std::vector<TrialData> generateTrainingData(const Task& task) {
            // Generate training data for the task
            return std::vector<TrialData>();  // Placeholder
        }
        
        TrialResult runSingleTrial(const RobotAlgorithm& algorithm,
                                  const Task& task) {
            // Run a single trial for the task
            return TrialResult();  // Placeholder
        }
        
        double calculateMeanPerformance(const std::vector<TrialResult>& results) {
            if (results.empty()) return 0.0;
            
            double sum = 0.0;
            for (const auto& result : results) {
                sum += result.success ? 1.0 : 0.0;
            }
            return sum / results.size();
        }
        
        double calculateStdDev(const std::vector<TrialResult>& results) {
            if (results.size() < 2) return 0.0;
            
            double mean = calculateMeanPerformance(results);
            double sum_sq_diff = 0.0;
            
            for (const auto& result : results) {
                double value = result.success ? 1.0 : 0.0;
                double diff = value - mean;
                sum_sq_diff += diff * diff;
            }
            
            return std::sqrt(sum_sq_diff / (results.size() - 1));
        }
        
        double calculateLowerCI(const std::vector<TrialResult>& results) {
            double mean = calculateMeanPerformance(results);
            double std_err = calculateStdDev(results) / std::sqrt(results.size());
            return mean - 1.96 * std_err;  // 95% CI
        }
        
        double calculateUpperCI(const std::vector<TrialResult>& results) {
            double mean = calculateMeanPerformance(results);
            double std_err = calculateStdDev(results) / std::sqrt(results.size());
            return mean + 1.96 * std_err;  // 95% CI
        }
    };
    
    struct RobotAlgorithm {
        std::string name;
        std::string version;
        std::vector<Hyperparameter> hyperparameters;
        
        virtual void train(const std::vector<TrialData>& training_data) = 0;
        virtual bool executeTask(const Task& task) = 0;
    };
    
    struct Environment {
        std::string name;
        std::string description;
        std::vector<Obstacle> obstacles;
        std::vector<Object> objects;
        EnvironmentalConditions conditions;
    };
    
    struct RobotPlatform {
        std::string name;
        std::string model;
        std::vector<SensorSpec> sensors;
        std::vector<ActuatorSpec> actuators;
        PhysicalSpecifications specifications;
    };
    
    struct Task {
        std::string name;
        std::string description;
        std::vector<std::string> success_criteria;
        TaskComplexity complexity;
        std::vector<SkillRequirement> required_skills;
    };
    
    struct TrialData {
        RobotState robot_state;
        SensorData sensor_data;
        Action taken_action;
        Reward received_reward;
        bool success;
        rclcpp::Time timestamp;
    };
    
    struct Hyperparameter {
        std::string name;
        std::string value;
        std::string type;
        std::string description;
    };
    
    struct Obstacle {
        std::string type;
        geometry_msgs::msg::Pose pose;
        std::vector<double> dimensions;
    };
    
    struct Object {
        std::string type;
        std::string material;
        double weight;
        std::vector<double> dimensions;
        geometry_msgs::msg::Pose pose;
    };
    
    struct EnvironmentalConditions {
        double temperature;
        double humidity;
        double lighting_level;
        double acoustic_noise;
        double floor_friction;
        std::string floor_type;
    };
    
    struct SensorSpec {
        std::string type;
        std::string model;
        double accuracy;
        double range;
        double resolution;
    };
    
    struct ActuatorSpec {
        std::string type;
        std::string model;
        double max_force_torque;
        double max_velocity;
        precision;
    };
    
    struct PhysicalSpecifications {
        double max_payload;
        double reach;
        std::vector<double> joint_limits;
        double max_velocity;
        double max_acceleration;
    };
    
    struct TaskComplexity {
        int difficulty_level;
        std::vector<Skill> required_skills;
        std::vector<Constraint> constraints;
        double estimated_time;
    };
    
    struct SkillRequirement {
        std::string skill_name;
        int proficiency_level;
        std::vector<std::string> dependencies;
    };
    
    struct RobotState {
        std::vector<double> joint_positions;
        std::vector<double> joint_velocities;
        geometry_msgs::msg::Pose end_effector_pose;
        std::vector<double> cartesian_velocities;
    };
    
    struct SensorData {
        std::vector<sensor_msgs::msg::JointState> joint_states;
        std::vector<sensor_msgs::msg::Image> images;
        std::vector<sensor_msgs::msg::LaserScan> laser_scans;
        std::vector<sensor_msgs::msg::Imu> imu_data;
        std::vector<geometry_msgs::msg::Wrench> force_torque;
    };
    
    struct Action {
        std::string type;  // "move_to", "grasp", "navigate", etc.
        std::vector<double> parameters;
        geometry_msgs::msg::Pose target_pose;
        std::vector<double> joint_targets;
    };
    
    struct Reward {
        double immediate_reward;
        double cumulative_reward;
        std::vector<std::string> reward_components;
        bool terminal_state;
    };
    
    struct Skill {
        std::string name;
        double proficiency;
        std::vector<std::string> dependencies;
    };
    
    struct Constraint {
        std::string type;  // "kinematic", "dynamic", "environmental", etc.
        std::string description;
        std::vector<double> parameters;
    };
};
```

## Data Collection and Management

### Systematic Data Collection

#### Structured Data Collection Framework
```cpp
class StructuredDataCollectionFramework {
private:
    std::unique_ptr<DataCollector> primary_collector_;
    std::unique_ptr<DataCollector> secondary_collector_;
    std::unique_ptr<DataValidator> validator_;
    std::unique_ptr<DataFormatter> formatter_;
    std::unique_ptr<DataStorageManager> storage_manager_;
    std::unique_ptr<MetadataManager> metadata_manager_;

public:
    StructuredDataCollectionFramework() {
        primary_collector_ = std::make_unique<PrimaryDataCollector>();
        secondary_collector_ = std::make_unique<SecondaryDataCollector>();
        validator_ = std::make_unique<DataIntegrityValidator>();
        formatter_ = std::make_unique<StandardizedDataFormatter>();
        storage_manager_ = std::make_unique<RobustStorageManager>();
        metadata_manager_ = std::make_unique<ComprehensiveMetadataManager>();
    }
    
    CollectionResult collectExperimentalData(const ExperimentProtocol& protocol) {
        CollectionResult result;
        
        // Start data collection streams
        auto primary_stream = primary_collector_->startStream(protocol.primary_data_schema);
        auto secondary_stream = secondary_collector_->startStream(protocol.secondary_data_schema);
        
        // Collect primary data (robot state, sensor readings, actions)
        auto primary_data = collectPrimaryData(primary_stream, protocol.duration);
        
        // Collect secondary data (environmental conditions, timestamps, annotations)
        auto secondary_data = collectSecondaryData(secondary_stream, protocol.duration);
        
        // Validate collected data
        auto validation_result = validator_->validateData(primary_data, secondary_data);
        result.validation_passed = validation_result.success;
        result.validation_issues = validation_result.issues;
        
        if (!validation_result.success) {
            result.error_message = "Data validation failed: " + 
                                 joinStrings(validation_result.issues, "; ");
            return result;
        }
        
        // Format data according to standard schema
        auto formatted_primary = formatter_->formatPrimaryData(primary_data);
        auto formatted_secondary = formatter_->formatSecondaryData(secondary_data);
        
        // Collect metadata
        auto metadata = metadata_manager_->collectMetadata(protocol);
        
        // Store data
        auto storage_result = storage_manager_->storeData(
            formatted_primary, formatted_secondary, metadata);
        
        result.storage_success = storage_result.success;
        result.data_location = storage_result.location;
        result.collection_summary = createCollectionSummary(
            primary_data, secondary_data, metadata);
        
        return result;
    }

private:
    struct CollectionResult {
        bool validation_passed;
        std::vector<std::string> validation_issues;
        bool storage_success;
        std::string data_location;
        CollectionSummary collection_summary;
        std::string error_message;
        rclcpp::Time completion_time;
    };
    
    struct CollectionSummary {
        int total_samples;
        int valid_samples;
        int discarded_samples;
        double collection_rate_hz;
        double data_volume_mb;
        std::vector<DataQualityMetric> quality_metrics;
        rclcpp::Duration collection_duration;
    };
    
    struct DataQualityMetric {
        std::string metric_name;
        double value;
        double threshold;
        bool within_acceptable_range;
    };
    
    std::vector<PrimaryDataPoint> collectPrimaryData(
        const DataStream& stream, const rclcpp::Duration& duration) {
        
        std::vector<PrimaryDataPoint> data_points;
        auto start_time = this->now();
        
        while (this->now() - start_time < duration) {
            auto data_point = stream.read();
            if (data_point.valid) {
                data_points.push_back(data_point.data);
            }
            
            // Check for timeout or other termination conditions
            if (data_points.size() > MAX_DATA_POINTS) {
                break;  // Prevent memory issues
            }
        }
        
        return data_points;
    }
    
    std::vector<SecondaryDataPoint> collectSecondaryData(
        const DataStream& stream, const rclcpp::Duration& duration) {
        
        std::vector<SecondaryDataPoint> data_points;
        auto start_time = this->now();
        
        while (this->now() - start_time < duration) {
            auto data_point = stream.read();
            if (data_point.valid) {
                data_points.push_back(data_point.data);
            }
        }
        
        return data_points;
    }
    
    CollectionSummary createCollectionSummary(
        const std::vector<PrimaryDataPoint>& primary_data,
        const std::vector<SecondaryDataPoint>& secondary_data,
        const Metadata& metadata) {
        
        CollectionSummary summary;
        summary.total_samples = primary_data.size() + secondary_data.size();
        summary.valid_samples = summary.total_samples;  // Assuming all validated
        summary.discarded_samples = 0;  // After validation
        summary.collection_duration = metadata.experiment_duration;
        
        if (summary.collection_duration.seconds() > 0) {
            summary.collection_rate_hz = summary.total_samples / 
                                       summary.collection_duration.seconds();
        }
        
        // Estimate data volume
        summary.data_volume_mb = estimateDataVolume(primary_data, secondary_data);
        
        // Calculate quality metrics
        summary.quality_metrics = calculateQualityMetrics(primary_data, secondary_data);
        
        return summary;
    }
    
    double estimateDataVolume(const std::vector<PrimaryDataPoint>& primary,
                             const std::vector<SecondaryDataPoint>& secondary) {
        
        double total_size = 0.0;
        
        for (const auto& point : primary) {
            total_size += sizeof(point);  // Simplified size calculation
        }
        
        for (const auto& point : secondary) {
            total_size += sizeof(point);  // Simplified size calculation
        }
        
        return total_size / (1024.0 * 1024.0);  // Convert to MB
    }
    
    std::vector<DataQualityMetric> calculateQualityMetrics(
        const std::vector<PrimaryDataPoint>& primary,
        const std::vector<SecondaryDataPoint>& secondary) {
        
        std::vector<DataQualityMetric> metrics;
        
        // Completeness metric
        DataQualityMetric completeness;
        completeness.metric_name = "Completeness";
        completeness.value = 1.0;  // Assuming all data collected
        completeness.threshold = 0.95;  // 95% threshold
        completeness.within_acceptable_range = completeness.value >= completeness.threshold;
        metrics.push_back(completeness);
        
        // Timeliness metric
        DataQualityMetric timeliness;
        timeliness.metric_name = "Timeliness";
        timeliness.value = calculateTimelinessScore(primary);
        timeliness.threshold = 0.98;  // 98% threshold
        timeliness.within_acceptable_range = timeliness.value >= timeliness.threshold;
        metrics.push_back(timeliness);
        
        // Consistency metric
        DataQualityMetric consistency;
        consistency.metric_name = "Consistency";
        consistency.value = calculateConsistencyScore(primary);
        consistency.threshold = 0.99;  // 99% threshold
        consistency.within_acceptable_range = consistency.value >= consistency.threshold;
        metrics.push_back(consistency);
        
        return metrics;
    }
    
    double calculateTimelinessScore(const std::vector<PrimaryDataPoint>& data) {
        // Calculate score based on regularity of data collection
        if (data.size() < 2) return 1.0;
        
        // Calculate time intervals between samples
        std::vector<double> intervals;
        for (size_t i = 1; i < data.size(); i++) {
            double interval = (data[i].timestamp - data[i-1].timestamp).seconds();
            intervals.push_back(interval);
        }
        
        // Calculate regularity (inverse of variance)
        double mean_interval = std::accumulate(intervals.begin(), intervals.end(), 0.0) / 
                              intervals.size();
        double variance = 0.0;
        for (double interval : intervals) {
            variance += (interval - mean_interval) * (interval - mean_interval);
        }
        variance /= intervals.size();
        
        // Higher regularity = higher score
        double regularity_score = 1.0 / (1.0 + variance);
        return std::min(1.0, regularity_score);
    }
    
    double calculateConsistencyScore(const std::vector<PrimaryDataPoint>& data) {
        // Calculate score based on consistency of values
        if (data.empty()) return 1.0;
        
        // For simplicity, checking joint position consistency
        // In practice, this would check multiple consistency measures
        double consistency_score = 0.0;
        
        for (size_t i = 1; i < data.size(); i++) {
            // Check if joint positions are physically plausible
            double position_change = calculatePositionChange(data[i-1], data[i]);
            if (position_change < MAX_PHYSICAL_CHANGE) {
                consistency_score += 1.0;
            }
        }
        
        return consistency_score / data.size();
    }
    
    double calculatePositionChange(const PrimaryDataPoint& p1, 
                                  const PrimaryDataPoint& p2) {
        double total_change = 0.0;
        
        for (size_t i = 0; i < std::min(p1.joint_positions.size(), 
                                        p2.joint_positions.size()); i++) {
            total_change += std::abs(p2.joint_positions[i] - p1.joint_positions[i]);
        }
        
        return total_change;
    }
    
    std::string joinStrings(const std::vector<std::string>& strings, 
                           const std::string& delimiter) {
        if (strings.empty()) return "";
        
        std::string result = strings[0];
        for (size_t i = 1; i < strings.size(); i++) {
            result += delimiter + strings[i];
        }
        
        return result;
    }
    
    static constexpr int MAX_DATA_POINTS = 1000000;  // 1M points max
    static constexpr double MAX_PHYSICAL_CHANGE = 1.0;  // Radians

    struct PrimaryDataPoint {
        std::vector<double> joint_positions;
        std::vector<double> joint_velocities;
        std::vector<double> joint_efforts;
        geometry_msgs::msg::Pose end_effector_pose;
        std::vector<double> cartesian_velocities;
        std::vector<double> cartesian_forces;
        std::vector<sensor_msgs::msg::Image> camera_images;
        std::vector<sensor_msgs::msg::LaserScan> laser_scans;
        sensor_msgs::msg::Imu imu_data;
        std::vector<double> control_commands;
        rclcpp::Time timestamp;
        bool valid = true;
    };
    
    struct SecondaryDataPoint {
        double ambient_temperature;
        double humidity;
        double lighting_level;
        double acoustic_noise;
        double magnetic_field_strength;
        std::vector<double> floor_vibration;
        std::vector<std::string> environmental_annotations;
        std::vector<std::string> operator_notes;
        std::vector<std::string> system_alerts;
        rclcpp::Time timestamp;
        bool valid = true;
    };
    
    struct DataStream {
        virtual DataPoint read() = 0;
        virtual bool isOpen() = 0;
        virtual void close() = 0;
    };
    
    struct DataPoint {
        PrimaryDataPoint data;
        bool valid;
    };
    
    struct ValidationResult {
        bool success;
        std::vector<std::string> issues;
        rclcpp::Time validation_time;
    };
    
    struct StorageResult {
        bool success;
        std::string location;
        rclcpp::Time storage_time;
    };
    
    class DataCollector {
    public:
        virtual DataStream startStream(const DataSchema& schema) = 0;
    };
    
    class PrimaryDataCollector : public DataCollector {
    public:
        DataStream startStream(const DataSchema& schema) override {
            return DataStream();  // Placeholder
        }
    };
    
    class SecondaryDataCollector : public DataCollector {
    public:
        DataStream startStream(const DataSchema& schema) override {
            return DataStream();  // Placeholder
        }
    };
    
    class DataValidator {
    public:
        virtual ValidationResult validateData(
            const std::vector<PrimaryDataPoint>& primary,
            const std::vector<SecondaryDataPoint>& secondary) = 0;
    };
    
    class DataIntegrityValidator : public DataValidator {
    public:
        ValidationResult validateData(
            const std::vector<PrimaryDataPoint>& primary,
            const std::vector<SecondaryDataPoint>& secondary) override {
            
            ValidationResult result;
            result.success = true;
            result.validation_time = this->now();
            
            // Check for missing values
            for (size_t i = 0; i < primary.size(); i++) {
                if (!isDataComplete(primary[i])) {
                    result.issues.push_back("Incomplete primary data at index " + 
                                          std::to_string(i));
                    result.success = false;
                }
            }
            
            // Check for out-of-range values
            for (size_t i = 0; i < primary.size(); i++) {
                if (!isDataInRange(primary[i])) {
                    result.issues.push_back("Out-of-range primary data at index " + 
                                          std::to_string(i));
                    result.success = false;
                }
            }
            
            // Check temporal consistency
            if (!isTemporalConsistent(primary)) {
                result.issues.push_back("Temporal inconsistency in primary data");
                result.success = false;
            }
            
            return result;
        }

    private:
        bool isDataComplete(const PrimaryDataPoint& point) {
            // Check if required fields are populated
            return !point.joint_positions.empty() && 
                   point.joint_positions.size() == EXPECTED_JOINT_COUNT;
        }
        
        bool isDataInRange(const PrimaryDataPoint& point) {
            // Check if values are within expected ranges
            for (double pos : point.joint_positions) {
                if (std::abs(pos) > MAX_JOINT_POSITION) {
                    return false;
                }
            }
            
            for (double vel : point.joint_velocities) {
                if (std::abs(vel) > MAX_JOINT_VELOCITY) {
                    return false;
                }
            }
            
            return true;
        }
        
        bool isTemporalConsistent(const std::vector<PrimaryDataPoint>& data) {
            // Check if timestamps are monotonically increasing
            for (size_t i = 1; i < data.size(); i++) {
                if (data[i].timestamp < data[i-1].timestamp) {
                    return false;
                }
            }
            return true;
        }
        
        static constexpr int EXPECTED_JOINT_COUNT = 7;
        static constexpr double MAX_JOINT_POSITION = 10.0;  // Radians
        static constexpr double MAX_JOINT_VELOCITY = 5.0;  // Rad/s
    };
    
    class DataFormatter {
    public:
        virtual FormattedData formatPrimaryData(const std::vector<PrimaryDataPoint>& data) = 0;
        virtual FormattedData formatSecondaryData(const std::vector<SecondaryDataPoint>& data) = 0;
    };
    
    class StandardizedDataFormatter : public DataFormatter {
    public:
        FormattedData formatPrimaryData(const std::vector<PrimaryDataPoint>& data) override {
            // Format according to standard schema
            FormattedData formatted;
            // Implementation would convert to standard format
            return formatted;
        }
        
        FormattedData formatSecondaryData(const std::vector<SecondaryDataPoint>& data) override {
            // Format according to standard schema
            FormattedData formatted;
            // Implementation would convert to standard format
            return formatted;
        }
    };
    
    class DataStorageManager {
    public:
        virtual StorageResult storeData(const FormattedData& primary,
                                      const FormattedData& secondary,
                                      const Metadata& metadata) = 0;
    };
    
    class RobustStorageManager : public DataStorageManager {
    public:
        StorageResult storeData(const FormattedData& primary,
                              const FormattedData& secondary,
                              const Metadata& metadata) override {
            
            StorageResult result;
            result.success = true;
            result.storage_time = this->now();
            
            try {
                // Store primary data
                std::string primary_location = storeToFile(primary, "primary");
                
                // Store secondary data
                std::string secondary_location = storeToFile(secondary, "secondary");
                
                // Store metadata
                std::string metadata_location = storeMetadata(metadata);
                
                // Create composite location reference
                result.location = "primary:" + primary_location + 
                                ",secondary:" + secondary_location + 
                                ",metadata:" + metadata_location;
                                
            } catch (const std::exception& e) {
                result.success = false;
                result.location = "Error: " + std::string(e.what());
            }
            
            return result;
        }

    private:
        std::string storeToFile(const FormattedData& data, 
                               const std::string& prefix) {
            // Store data to file with standardized naming
            std::string filename = prefix + "_" + 
                                  std::to_string(this->now().nanoseconds()) + ".dat";
            // Implementation would write data to file
            return filename;  // Placeholder
        }
        
        std::string storeMetadata(const Metadata& metadata) {
            // Store metadata in standardized format
            std::string filename = "metadata_" + 
                                  std::to_string(this->now().nanoseconds()) + ".json";
            // Implementation would write metadata to file
            return filename;  // Placeholder
        }
    };
    
    struct FormattedData {
        std::string format_type;  // "protobuf", "json", "csv", etc.
        std::string schema_version;
        std::vector<uint8_t> data;
        rclcpp::Time creation_time;
    };
    
    struct DataSchema {
        std::string schema_name;
        std::string version;
        std::vector<FieldDefinition> fields;
        std::string format;
    };
    
    struct FieldDefinition {
        std::string name;
        std::string type;  // "double", "string", "array", etc.
        std::string description;
        double min_value;
        double max_value;
        bool required;
    };
};
```

### Data Quality Assurance

#### Quality Control Procedures
```cpp
class DataQualityAssuranceSystem {
private:
    std::unique_ptr<PreCollectionValidator> pre_validator_;
    std::unique_ptr<InCollectionMonitor> in_monitor_;
    std::unique_ptr<PostCollectionAnalyzer> post_analyzer_;
    std::unique_ptr<QualityReporter> reporter_;
    std::unique_ptr<ArchiveManager> archive_manager_;

public:
    DataQualityAssuranceSystem() {
        pre_validator_ = std::make_unique<PreCollectionValidator>();
        in_monitor_ = std::make_unique<RealTimeDataMonitor>();
        post_analyzer_ = std::make_unique<PostCollectionAnalyzer>();
        reporter_ = std::make_unique<AutomatedQualityReporter>();
        archive_manager_ = std::make_unique<DataArchiveManager>();
    }
    
    QualityAssessmentResult assessDataQuality(const RawDataCollection& raw_data,
                                            const CollectionMetadata& metadata) {
        
        QualityAssessmentResult result;
        
        // Pre-collection validation
        auto pre_validation = pre_validator_->validateSetup(metadata);
        result.pre_collection_validation = pre_validation;
        
        if (!pre_validation.passed) {
            result.overall_quality_score = 0.0;
            result.status = DataQualityStatus::SETUP_ERROR;
            return result;
        }
        
        // Real-time monitoring during collection
        auto monitoring_results = in_monitor_->monitorCollection(raw_data);
        result.real_time_monitoring = monitoring_results;
        
        // Post-collection analysis
        auto post_analysis = post_analyzer_->analyze(raw_data);
        result.post_collection_analysis = post_analysis;
        
        // Calculate overall quality score
        result.overall_quality_score = calculateQualityScore(
            pre_validation, monitoring_results, post_analysis);
        
        // Determine data quality status
        result.status = determineQualityStatus(result.overall_quality_score);
        
        // Generate quality report
        auto report = reporter_->generateReport(result, raw_data, metadata);
        result.quality_report = report;
        
        // Archive results
        archive_manager_->archiveAssessment(result, raw_data, metadata);
        
        return result;
    }

private:
    struct QualityAssessmentResult {
        PreCollectionValidationResult pre_collection_validation;
        RealTimeMonitoringResult real_time_monitoring;
        PostCollectionAnalysisResult post_collection_analysis;
        double overall_quality_score;
        DataQualityStatus status;
        QualityReport quality_report;
        rclcpp::Time assessment_time;
    };
    
    enum class DataQualityStatus {
        EXCELLENT,
        GOOD,
        ACCEPTABLE,
        POOR,
        UNUSABLE,
        SETUP_ERROR
    };
    
    struct PreCollectionValidationResult {
        bool passed;
        std::vector<std::string> setup_issues;
        std::vector<std::string> recommendations;
        rclcpp::Time validation_time;
    };
    
    struct RealTimeMonitoringResult {
        std::vector<DataIntegrityAlert> alerts;
        std::vector<DataQualityMetric> metrics_over_time;
        bool collection_interrupted;
        std::string interruption_reason;
        rclcpp::Time monitoring_duration;
    };
    
    struct PostCollectionAnalysisResult {
        std::vector<DataAnomaly> detected_anomalies;
        StatisticalSummary statistical_summary;
        CompletenessAssessment completeness;
        ConsistencyAssessment consistency;
        AccuracyAssessment accuracy;
        rclcpp::Time analysis_time;
    };
    
    struct QualityReport {
        std::string overall_assessment;
        std::vector<IssueSummary> critical_issues;
        std::vector<Recommendation> recommendations;
        std::vector<StatisticalSummary> summary_statistics;
        std::string confidence_level;
        std::vector<std::string> data_usability_notes;
        rclcpp::Time report_generation_time;
    };
    
    struct DataIntegrityAlert {
        AlertType type;
        std::string description;
        rclcpp::Time timestamp;
        SeverityLevel severity;
        std::string component_affected;
        std::string suggested_action;
    };
    
    struct DataQualityMetric {
        std::string metric_name;
        std::vector<double> values_over_time;
        std::vector<rclcpp::Time> timestamps;
        double current_value;
        double threshold;
        bool within_acceptable_range;
    };
    
    struct DataAnomaly {
        AnomalyType type;
        std::string description;
        rclcpp::Time timestamp;
        double severity_score;
        std::string confidence_level;
        std::vector<std::string> potential_causes;
        std::vector<std::string> recommended_actions;
    };
    
    struct StatisticalSummary {
        double mean;
        double median;
        double std_dev;
        double variance;
        double min_val;
        double max_val;
        double quartile_25;
        double quartile_75;
        double skewness;
        double kurtosis;
        int sample_size;
    };
    
    struct CompletenessAssessment {
        double data_completeness_ratio;
        std::vector<std::string> missing_data_fields;
        int total_expected_samples;
        int actual_samples_collected;
        double temporal_coverage;
        std::vector<rclcpp::Time> gaps_identified;
    };
    
    struct ConsistencyAssessment {
        double consistency_score;
        std::vector<std::string> consistency_issues;
        std::vector<ConsistencyCheckResult> individual_checks;
        double physical_plausibility_score;
        double temporal_coherence_score;
    };
    
    struct AccuracyAssessment {
        double accuracy_score;
        std::vector<CalibrationCheckResult> calibration_results;
        double precision_score;
        double recall_score;
        double f1_score;
        std::vector<ErrorAnalysisResult> error_analysis;
    };
    
    struct ConsistencyCheckResult {
        std::string check_name;
        bool passed;
        double score;
        std::string details;
    };
    
    struct CalibrationCheckResult {
        std::string component_name;
        bool calibrated;
        double accuracy_before_calibration;
        double accuracy_after_calibration;
        rclcpp::Time calibration_time;
        std::string calibration_method;
    };
    
    struct ErrorAnalysisResult {
        std::string error_type;
        double frequency;
        double impact_score;
        std::string root_cause;
        std::string suggested_correction;
    };
    
    struct IssueSummary {
        std::string issue_category;
        std::string description;
        int occurrence_count;
        double impact_score;
        std::string severity_level;
    };
    
    struct Recommendation {
        std::string category;
        std::string description;
        std::string priority_level;
        std::string implementation_complexity;
        std::string expected_benefit;
    };
    
    enum class AlertType {
        MISSING_DATA,
        OUT_OF_RANGE,
        TIMING_ERROR,
        CALIBRATION_ERROR,
        HARDWARE_FAULT,
        ENVIRONMENTAL_CHANGE
    };
    
    enum class SeverityLevel {
        INFO,
        WARNING,
        ERROR,
        CRITICAL
    };
    
    enum class AnomalyType {
        SENSOR_SPIKE,
        TEMPORAL_GAP,
        PHYSICAL_IMPOSSIBILITY,
        CALIBRATION_DRIFT,
        ENVIRONMENTAL_ANOMALY,
        ALGORITHM_ERROR
    };
    
    double calculateQualityScore(const PreCollectionValidationResult& pre,
                                const RealTimeMonitoringResult& real_time,
                                const PostCollectionAnalysisResult& post) {
        
        // Weighted combination of different quality aspects
        double pre_weight = 0.2;    // 20% for setup validation
        double real_time_weight = 0.3;  // 30% for real-time monitoring
        double post_weight = 0.5;   // 50% for post-analysis
        
        double pre_score = pre.passed ? 1.0 : 0.0;
        double real_time_score = calculateRealTimeScore(real_time);
        double post_score = calculatePostScore(post);
        
        return pre_weight * pre_score + 
               real_time_weight * real_time_score + 
               post_weight * post_score;
    }
    
    double calculateRealTimeScore(const RealTimeMonitoringResult& monitoring) {
        // Calculate score based on real-time monitoring results
        if (monitoring.alerts.empty()) {
            return 1.0;  // No alerts = perfect
        }
        
        // Deduct points for alerts based on severity
        double score = 1.0;
        for (const auto& alert : monitoring.alerts) {
            switch (alert.severity) {
                case SeverityLevel::WARNING:
                    score -= 0.05;  // Small deduction for warnings
                    break;
                case SeverityLevel::ERROR:
                    score -= 0.1;   // Medium deduction for errors
                    break;
                case SeverityLevel::CRITICAL:
                    score -= 0.3;   // Large deduction for critical issues
                    break;
                default:
                    break;  // Info level doesn't affect score
            }
        }
        
        // Ensure score doesn't go below 0
        return std::max(0.0, score);
    }
    
    double calculatePostScore(const PostCollectionAnalysisResult& post) {
        // Calculate score based on post-collection analysis
        double completeness_score = post.completeness.data_completeness_ratio;
        double consistency_score = post.consistency.consistency_score;
        double accuracy_score = post.accuracy.accuracy_score;
        
        // Weighted average
        return 0.4 * completeness_score + 
               0.4 * consistency_score + 
               0.2 * accuracy_score;
    }
    
    DataQualityStatus determineQualityStatus(double quality_score) {
        if (quality_score >= 0.9) return DataQualityStatus::EXCELLENT;
        if (quality_score >= 0.8) return DataQualityStatus::GOOD;
        if (quality_score >= 0.7) return DataQualityStatus::ACCEPTABLE;
        if (quality_score >= 0.5) return DataQualityStatus::POOR;
        return DataQualityStatus::UNUSABLE;
    }
    
    class PreCollectionValidator {
    public:
        PreCollectionValidationResult validateSetup(const CollectionMetadata& metadata) {
            PreCollectionValidationResult result;
            result.passed = true;
            result.validation_time = this->now();
            
            // Check robot calibration
            if (!isRobotCalibrated()) {
                result.setup_issues.push_back("Robot not calibrated");
                result.passed = false;
            }
            
            // Check sensor functionality
            if (!areSensorsFunctional()) {
                result.setup_issues.push_back("Sensors not functioning properly");
                result.passed = false;
            }
            
            // Check environmental conditions
            if (!isEnvironmentSuitable()) {
                result.setup_issues.push_back("Environmental conditions not suitable");
                result.passed = false;
            }
            
            // Check software versions
            if (!areSoftwareVersionsCorrect()) {
                result.setup_issues.push_back("Incorrect software versions");
                result.passed = false;
            }
            
            // Add recommendations for issues
            if (!result.setup_issues.empty()) {
                result.recommendations = generateRecommendations(result.setup_issues);
            }
            
            return result;
        }

    private:
        bool isRobotCalibrated() { /* Implementation */ return true; }
        bool areSensorsFunctional() { /* Implementation */ return true; }
        bool isEnvironmentSuitable() { /* Implementation */ return true; }
        bool areSoftwareVersionsCorrect() { /* Implementation */ return true; }
        
        std::vector<std::string> generateRecommendations(
            const std::vector<std::string>& issues) {
            
            std::vector<std::string> recommendations;
            
            for (const auto& issue : issues) {
                if (issue.find("calibration") != std::string::npos) {
                    recommendations.push_back("Perform robot calibration procedure");
                } else if (issue.find("sensors") != std::string::npos) {
                    recommendations.push_back("Check sensor connections and configurations");
                } else if (issue.find("environment") != std::string::npos) {
                    recommendations.push_back("Adjust environmental conditions or reschedule");
                } else if (issue.find("software") != std::string::npos) {
                    recommendations.push_back("Update software to required versions");
                }
            }
            
            return recommendations;
        }
    };
    
    class RealTimeMonitor {
    public:
        virtual RealTimeMonitoringResult monitorCollection(const RawDataCollection& data) = 0;
    };
    
    class RealTimeDataMonitor : public RealTimeMonitor {
    public:
        RealTimeMonitoringResult monitorCollection(const RawDataCollection& data) override {
            RealTimeMonitoringResult result;
            result.monitoring_duration = rclcpp::Duration::from_seconds(data.duration);
            
            // Monitor for common issues during collection
            for (size_t i = 0; i < data.data_points.size(); i++) {
                auto& point = data.data_points[i];
                
                // Check for missing data
                if (isMissingData(point)) {
                    DataIntegrityAlert alert;
                    alert.type = AlertType::MISSING_DATA;
                    alert.description = "Missing sensor data at time " + 
                                      std::to_string(point.timestamp.nanoseconds());
                    alert.timestamp = point.timestamp;
                    alert.severity = SeverityLevel::WARNING;
                    alert.component_affected = "sensor_system";
                    alert.suggested_action = "Check sensor connections";
                    
                    result.alerts.push_back(alert);
                }
                
                // Check for out-of-range values
                if (hasOutOfRangeValues(point)) {
                    DataIntegrityAlert alert;
                    alert.type = AlertType::OUT_OF_RANGE;
                    alert.description = "Out-of-range values detected";
                    alert.timestamp = point.timestamp;
                    alert.severity = SeverityLevel::ERROR;
                    alert.component_affected = "sensor_readings";
                    alert.suggested_action = "Verify sensor calibration";
                    
                    result.alerts.push_back(alert);
                }
                
                // Track quality metrics over time
                DataQualityMetric metric;
                metric.metric_name = "data_integrity";
                metric.current_value = calculateIntegrityScore(point);
                metric.threshold = 0.95;
                metric.within_acceptable_range = metric.current_value >= metric.threshold;
                metric.timestamps.push_back(point.timestamp);
                
                result.metrics_over_time.push_back(metric);
            }
            
            return result;
        }

    private:
        bool isMissingData(const RawDataPoint& point) {
            // Check if required data fields are missing
            return point.joint_positions.empty() || 
                   point.joint_positions.size() != EXPECTED_JOINT_COUNT;
        }
        
        bool hasOutOfRangeValues(const RawDataPoint& point) {
            // Check if any values are outside expected ranges
            for (double pos : point.joint_positions) {
                if (std::abs(pos) > MAX_JOINT_POSITION) {
                    return true;
                }
            }
            
            for (double vel : point.joint_velocities) {
                if (std::abs(vel) > MAX_JOINT_VELOCITY) {
                    return true;
                }
            }
            
            return false;
        }
        
        double calculateIntegrityScore(const RawDataPoint& point) {
            // Calculate data integrity score
            int valid_fields = 0;
            int total_fields = 0;
            
            if (!point.joint_positions.empty()) { valid_fields++; }
            total_fields++;
            
            if (!point.joint_velocities.empty()) { valid_fields++; }
            total_fields++;
            
            if (!point.end_effector_pose.position.x == 0.0 || 
                !point.end_effector_pose.position.y == 0.0) { valid_fields++; }
            total_fields++;
            
            return static_cast<double>(valid_fields) / total_fields;
        }
        
        static constexpr int EXPECTED_JOINT_COUNT = 7;
        static constexpr double MAX_JOINT_POSITION = 10.0;
        static constexpr double MAX_JOINT_VELOCITY = 5.0;
    };
    
    class PostCollectionAnalyzer {
    public:
        virtual PostCollectionAnalysisResult analyze(const RawDataCollection& data) = 0;
    };
    
    class PostCollectionAnalyzerImpl : public PostCollectionAnalyzer {
    public:
        PostCollectionAnalysisResult analyze(const RawDataCollection& data) override {
            PostCollectionAnalysisResult result;
            result.analysis_time = this->now();
            
            // Detect anomalies
            result.detected_anomalies = detectAnomalies(data);
            
            // Calculate statistical summary
            result.statistical_summary = calculateStatisticalSummary(data);
            
            // Assess completeness
            result.completeness = assessCompleteness(data);
            
            // Assess consistency
            result.consistency = assessConsistency(data);
            
            // Assess accuracy
            result.accuracy = assessAccuracy(data);
            
            return result;
        }

    private:
        std::vector<DataAnomaly> detectAnomalies(const RawDataCollection& data) {
            std::vector<DataAnomaly> anomalies;
            
            // Detect sensor spikes
            for (size_t i = 1; i < data.data_points.size() - 1; i++) {
                if (isSensorSpike(data.data_points[i], 
                                 data.data_points[i-1], 
                                 data.data_points[i+1])) {
                    
                    DataAnomaly anomaly;
                    anomaly.type = AnomalyType::SENSOR_SPIKE;
                    anomaly.description = "Sensor spike detected at index " + 
                                        std::to_string(i);
                    anomaly.timestamp = data.data_points[i].timestamp;
                    anomaly.severity_score = 0.8;  // High severity
                    anomaly.confidence_level = "high";
                    
                    anomalies.push_back(anomaly);
                }
            }
            
            return anomalies;
        }
        
        StatisticalSummary calculateStatisticalSummary(const RawDataCollection& data) {
            StatisticalSummary summary;
            
            // For simplicity, calculating statistics for joint positions
            std::vector<double> all_positions;
            for (const auto& point : data.data_points) {
                all_positions.insert(all_positions.end(),
                                   point.joint_positions.begin(),
                                   point.joint_positions.end());
            }
            
            if (!all_positions.empty()) {
                summary.mean = std::accumulate(all_positions.begin(), 
                                             all_positions.end(), 0.0) / 
                             all_positions.size();
                
                // Calculate variance and std dev
                double variance = 0.0;
                for (double val : all_positions) {
                    variance += (val - summary.mean) * (val - summary.mean);
                }
                variance /= all_positions.size();
                summary.variance = variance;
                summary.std_dev = std::sqrt(variance);
                
                // Calculate min/max
                auto minmax = std::minmax_element(all_positions.begin(), 
                                                all_positions.end());
                summary.min_val = *minmax.first;
                summary.max_val = *minmax.second;
                
                // Calculate quartiles
                std::sort(all_positions.begin(), all_positions.end());
                summary.quartile_25 = all_positions[all_positions.size() * 0.25];
                summary.quartile_75 = all_positions[all_positions.size() * 0.75];
                
                summary.sample_size = all_positions.size();
            }
            
            return summary;
        }
        
        CompletenessAssessment assessCompleteness(const RawDataCollection& data) {
            CompletenessAssessment assessment;
            
            // Calculate completeness ratio
            int expected_samples = data.expected_sample_count;
            int actual_samples = data.data_points.size();
            assessment.total_expected_samples = expected_samples;
            assessment.actual_samples_collected = actual_samples;
            
            if (expected_samples > 0) {
                assessment.data_completeness_ratio = 
                    static_cast<double>(actual_samples) / expected_samples;
            } else {
                assessment.data_completeness_ratio = 1.0;  // Assume complete if no expectation
            }
            
            // Identify temporal gaps
            for (size_t i = 1; i < data.data_points.size(); i++) {
                auto time_diff = data.data_points[i].timestamp - 
                               data.data_points[i-1].timestamp;
                
                if (time_diff.seconds() > MAX_ACCEPTABLE_GAP_SECONDS) {
                    assessment.gaps_identified.push_back(
                        data.data_points[i-1].timestamp);
                }
            }
            
            return assessment;
        }
        
        ConsistencyAssessment assessConsistency(const RawDataCollection& data) {
            ConsistencyAssessment assessment;
            
            // Check physical plausibility (movement speeds, accelerations)
            int plausible_count = 0;
            int total_checks = 0;
            
            for (size_t i = 1; i < data.data_points.size(); i++) {
                if (isMovementPhysicallyPlausible(
                    data.data_points[i-1], data.data_points[i])) {
                    plausible_count++;
                }
                total_checks++;
            }
            
            if (total_checks > 0) {
                assessment.physical_plausibility_score = 
                    static_cast<double>(plausible_count) / total_checks;
            }
            
            // Overall consistency score
            assessment.consistency_score = assessment.physical_plausibility_score;
            
            return assessment;
        }
        
        AccuracyAssessment assessAccuracy(const RawDataCollection& data) {
            AccuracyAssessment assessment;
            
            // For accuracy assessment, we would typically compare against
            // ground truth data, which may not be available in all cases
            // This is a simplified version
            assessment.accuracy_score = 0.95;  // Placeholder
            assessment.precision_score = 0.92; // Placeholder
            assessment.recall_score = 0.94;    // Placeholder
            assessment.f1_score = 0.93;        // Placeholder
            
            return assessment;
        }
        
        bool isSensorSpike(const RawDataPoint& current,
                          const RawDataPoint& previous,
                          const RawDataPoint& next) {
            
            // Check if current value is significantly different from neighbors
            for (size_t i = 0; i < current.joint_positions.size(); i++) {
                double prev_diff = std::abs(current.joint_positions[i] - 
                                          previous.joint_positions[i]);
                double next_diff = std::abs(current.joint_positions[i] - 
                                          next.joint_positions[i]);
                
                if (prev_diff > SPIKE_THRESHOLD && next_diff > SPIKE_THRESHOLD) {
                    return true;
                }
            }
            
            return false;
        }
        
        bool isMovementPhysicallyPlausible(const RawDataPoint& prev,
                                          const RawDataPoint& curr) {
            
            auto time_diff = (curr.timestamp - prev.timestamp).seconds();
            if (time_diff <= 0) return true;  // Cannot determine
            
            // Check joint velocity limits
            for (size_t i = 0; i < curr.joint_positions.size(); i++) {
                double velocity = std::abs(curr.joint_positions[i] - 
                                         prev.joint_positions[i]) / time_diff;
                
                if (velocity > MAX_JOINT_VELOCITY) {
                    return false;  // Movement too fast
                }
            }
            
            return true;
        }
        
        static constexpr double MAX_ACCEPTABLE_GAP_SECONDS = 0.1;  // 100ms
        static constexpr double SPIKE_THRESHOLD = 1.0;  // Radians
        static constexpr double MAX_JOINT_VELOCITY = 5.0;  // Rad/s
    };
    
    struct RawDataCollection {
        std::vector<RawDataPoint> data_points;
        int expected_sample_count;
        double duration;  // seconds
        std::string collection_method;
        rclcpp::Time start_time;
        rclcpp::Time end_time;
    };
    
    struct RawDataPoint {
        std::vector<double> joint_positions;
        std::vector<double> joint_velocities;
        std::vector<double> joint_efforts;
        geometry_msgs::msg::Pose end_effector_pose;
        std::vector<double> control_commands;
        rclcpp::Time timestamp;
    };
    
    struct CollectionMetadata {
        std::string experiment_name;
        std::string researcher_name;
        std::string robot_platform;
        std::string environment_description;
        rclcpp::Time scheduled_start_time;
        rclcpp::Duration expected_duration;
        std::vector<std::string> controlled_variables;
        std::vector<std::string> measured_variables;
        std::vector<std::string> potential_confounds;
        std::string protocol_version;
        std::string data_schema_version;
    };
};
```

## Statistical Analysis in Robotics Research

### Hypothesis Testing Framework

#### Statistical Testing Procedures
```cpp
class StatisticalTestingFramework {
private:
    std::unique_ptr<NormalityTester> normality_tester_;
    std::unique_ptr<VarianceHomogeneityTester> variance_tester_;
    std::unique_ptr<ParametricTester> parametric_tester_;
    std::unique_ptr<NonParametricTester> non_parametric_tester_;
    std::unique_ptr<EffectSizeCalculator> effect_calculator_;
    std::unique_ptr<PowerAnalyzer> power_analyzer_;

public:
    StatisticalTestingFramework() {
        normality_tester_ = std::make_unique<ShapiroWilkTester>();
        variance_tester_ = std::make_unique<LevenesVarianceTester>();
        parametric_tester_ = std::make_unique<ParametricStatisticalTester>();
        non_parametric_tester_ = std::make_unique<NonParametricStatisticalTester>();
        effect_calculator_ = std::make_unique<EffectSizeCalculatorImpl>();
        power_analyzer_ = std::make_unique<PowerAnalysisCalculator>();
    }
    
    StatisticalTestResult performStatisticalTest(
        const std::vector<double>& group1_data,
        const std::vector<double>& group2_data,
        const StatisticalTestParameters& params) {
        
        StatisticalTestResult result;
        
        // Check normality assumption
        auto normality_result = normality_tester_->test(
            group1_data, params.alpha_level);
        result.normality_test = normality_result;
        
        // Check variance homogeneity
        auto variance_result = variance_tester_->test(
            group1_data, group2_data, params.alpha_level);
        result.variance_homogeneity_test = variance_result;
        
        // Choose appropriate test based on assumptions
        if (normality_result.passed && variance_result.passed) {
            // Parametric test (e.g., t-test)
            result.parametric_result = parametric_tester_->performTest(
                group1_data, group2_data, params);
            result.test_type_used = "parametric";
        } else {
            // Non-parametric test (e.g., Mann-Whitney U)
            result.non_parametric_result = non_parametric_tester_->performTest(
                group1_data, group2_data, params);
            result.test_type_used = "non_parametric";
        }
        
        // Calculate effect size
        result.effect_size = effect_calculator_->calculate(
            group1_data, group2_data);
        
        // Perform power analysis
        result.power_analysis = power_analyzer_->analyze(
            group1_data, group2_data, params);
        
        // Calculate confidence intervals
        result.confidence_intervals = calculateConfidenceIntervals(
            group1_data, group2_data, params.confidence_level);
        
        // Generate interpretation
        result.interpretation = generateInterpretation(result, params);
        
        return result;
    }

private:
    struct StatisticalTestResult {
        NormalityTestResult normality_test;
        VarianceHomogeneityTestResult variance_homogeneity_test;
        ParametricTestResult parametric_result;
        NonParametricTestResult non_parametric_result;
        EffectSizeResult effect_size;
        PowerAnalysisResult power_analysis;
        ConfidenceIntervalResult confidence_intervals;
        std::string test_type_used;
        StatisticalInterpretation interpretation;
        rclcpp::Time analysis_time;
    };
    
    struct StatisticalTestParameters {
        double alpha_level;          // Significance level (e.g., 0.05)
        double confidence_level;     // Confidence level (e.g., 0.95)
        std::string test_type;       // "t_test", "mann_whitney", etc.
        int min_sample_size;         // Minimum sample size for validity
        bool paired_test;            // Whether to use paired test
        std::string alternative_hypothesis;  // "two_sided", "greater", "less"
    };
    
    struct NormalityTestResult {
        bool passed;
        double p_value;
        double test_statistic;
        std::string test_name;
        rclcpp::Time test_time;
    };
    
    struct VarianceHomogeneityTestResult {
        bool passed;
        double p_value;
        double test_statistic;
        std::string test_name;
        rclcpp::Time test_time;
    };
    
    struct ParametricTestResult {
        double test_statistic;
        double p_value;
        double degrees_of_freedom;
        bool significant_difference;
        rclcpp::Time test_time;
    };
    
    struct NonParametricTestResult {
        double test_statistic;
        double p_value;
        bool significant_difference;
        rclcpp::Time test_time;
    };
    
    struct EffectSizeResult {
        double cohens_d;
        double glass_delta;
        double hedges_g;
        double cliff_delta;
        std::string interpretation;
        rclcpp::Time calculation_time;
    };
    
    struct PowerAnalysisResult {
        double achieved_power;
        double required_sample_size;
        double type_ii_error_probability;
        rclcpp::Time analysis_time;
    };
    
    struct ConfidenceIntervalResult {
        double lower_bound;
        double upper_bound;
        double confidence_level;
        rclcpp::Time calculation_time;
    };
    
    struct StatisticalInterpretation {
        std::string overall_conclusion;
        std::string practical_significance;
        std::string statistical_significance;
        std::string effect_size_interpretation;
        std::vector<std::string> limitations;
        std::vector<std::string> recommendations;
        rclcpp::Time interpretation_time;
    };
    
    class NormalityTester {
    public:
        virtual NormalityTestResult test(const std::vector<double>& data,
                                       double alpha_level) = 0;
    };
    
    class ShapiroWilkTester : public NormalityTester {
    public:
        NormalityTestResult test(const std::vector<double>& data,
                               double alpha_level) override {
            
            NormalityTestResult result;
            result.test_name = "Shapiro-Wilk";
            result.test_time = this->now();
            
            if (data.size() < 3) {
                result.passed = false;
                result.p_value = 1.0;  // Cannot test
                result.test_statistic = 0.0;
                return result;
            }
            
            if (data.size() > 5000) {
                // For large samples, Shapiro-Wilk may not be appropriate
                // Consider alternative tests
                result.passed = false;
                result.p_value = 1.0;  // Use alternative
                result.test_statistic = 0.0;
                return result;
            }
            
            // Calculate Shapiro-Wilk statistic
            // This is a simplified placeholder
            result.test_statistic = calculateShapiroWilkStatistic(data);
            
            // Calculate p-value (in practice, would use statistical tables)
            result.p_value = calculatePValueFromSW(result.test_statistic, data.size());
            
            result.passed = result.p_value > alpha_level;
            
            return result;
        }

    private:
        double calculateShapiroWilkStatistic(const std::vector<double>& data) {
            // Simplified calculation - in practice, this would be complex
            // The actual Shapiro-Wilk test involves ordered statistics and
            // coefficients from normal distribution
            return 0.95;  // Placeholder
        }
        
        double calculatePValueFromSW(double statistic, int n) {
            // In practice, would use statistical tables or approximations
            // For now, return a reasonable placeholder
            return 0.10;  // Placeholder
        }
    };
    
    class VarianceHomogeneityTester {
    public:
        virtual VarianceHomogeneityTestResult test(
            const std::vector<double>& group1,
            const std::vector<double>& group2,
            double alpha_level) = 0;
    };
    
    class LevenesVarianceTester : public VarianceHomogeneityTester {
    public:
        VarianceHomogeneityTestResult test(
            const std::vector<double>& group1,
            const std::vector<double>& group2,
            double alpha_level) override {
            
            VarianceHomogeneityTestResult result;
            result.test_name = "Levene's Test";
            result.test_time = this->now();
            
            if (group1.empty() || group2.empty()) {
                result.passed = false;
                result.p_value = 1.0;
                result.test_statistic = 0.0;
                return result;
            }
            
            // Calculate Levene's test statistic
            result.test_statistic = calculateLevenesStatistic(group1, group2);
            
            // Calculate p-value (placeholder)
            result.p_value = calculatePValueFromLevene(result.test_statistic, 
                                                     group1.size(), group2.size());
            
            result.passed = result.p_value > alpha_level;
            
            return result;
        }

    private:
        double calculateLevenesStatistic(const std::vector<double>& group1,
                                       const std::vector<double>& group2) {
            // Calculate Levene's test using median (more robust)
            double median1 = calculateMedian(group1);
            double median2 = calculateMedian(group2);
            
            // Calculate absolute deviations from median
            std::vector<double> abs_devs1, abs_devs2;
            for (double val : group1) {
                abs_devs1.push_back(std::abs(val - median1));
            }
            for (double val : group2) {
                abs_devs2.push_back(std::abs(val - median2));
            }
            
            // Perform ANOVA on absolute deviations
            double mean_abs_dev1 = calculateMean(abs_devs1);
            double mean_abs_dev2 = calculateMean(abs_devs2);
            double overall_mean = (mean_abs_dev1 * group1.size() + 
                                 mean_abs_dev2 * group2.size()) / 
                                (group1.size() + group2.size());
            
            // Calculate between-group and within-group sums of squares
            double ss_between = group1.size() * std::pow(mean_abs_dev1 - overall_mean, 2) +
                               group2.size() * std::pow(mean_abs_dev2 - overall_mean, 2);
            double ss_within = 0.0;
            
            for (double dev : abs_devs1) {
                ss_within += std::pow(dev - mean_abs_dev1, 2);
            }
            for (double dev : abs_devs2) {
                ss_within += std::pow(dev - mean_abs_dev2, 2);
            }
            
            // Calculate F-statistic
            double df1 = 1.0;  // Between groups
            double df2 = group1.size() + group2.size() - 2.0;  // Within groups
            double ms_between = ss_between / df1;
            double ms_within = ss_within / df2;
            
            return ms_between / ms_within;
        }
        
        double calculatePValueFromLevene(double f_stat, int n1, int n2) {
            // In practice, would use F-distribution
            // Placeholder for demonstration
            return 0.15;  // Placeholder
        }
        
        double calculateMedian(const std::vector<double>& data) {
            if (data.empty()) return 0.0;
            
            std::vector<double> sorted_data = data;
            std::sort(sorted_data.begin(), sorted_data.end());
            
            size_t n = sorted_data.size();
            if (n % 2 == 0) {
                return (sorted_data[n/2 - 1] + sorted_data[n/2]) / 2.0;
            } else {
                return sorted_data[n/2];
            }
        }
        
        double calculateMean(const std::vector<double>& data) {
            if (data.empty()) return 0.0;
            
            double sum = std::accumulate(data.begin(), data.end(), 0.0);
            return sum / data.size();
        }
    };
    
    class ParametricTester {
    public:
        virtual ParametricTestResult performTest(
            const std::vector<double>& group1,
            const std::vector<double>& group2,
            const StatisticalTestParameters& params) = 0;
    };
    
    class ParametricStatisticalTester : public ParametricTester {
    public:
        ParametricTestResult performTest(
            const std::vector<double>& group1,
            const std::vector<double>& group2,
            const StatisticalTestParameters& params) override {
            
            ParametricTestResult result;
            result.test_time = this->now();
            
            if (params.paired_test) {
                // Paired t-test
                result = performPairedTTest(group1, group2, params);
            } else {
                // Independent t-test (Welch's t-test for unequal variances)
                result = performWelchsTTest(group1, group2, params);
            }
            
            // Determine significance
            result.significant_difference = result.p_value < params.alpha_level;
            
            return result;
        }

    private:
        ParametricTestResult performWelchsTTest(
            const std::vector<double>& group1,
            const std::vector<double>& group2,
            const StatisticalTestParameters& params) {
            
            ParametricTestResult result;
            
            // Calculate means
            double mean1 = calculateMean(group1);
            double mean2 = calculateMean(group2);
            
            // Calculate variances
            double var1 = calculateVariance(group1, mean1);
            double var2 = calculateVariance(group2, mean2);
            
            // Calculate Welch's t-statistic
            double se_diff = std::sqrt(var1/group1.size() + var2/group2.size());
            result.test_statistic = (mean2 - mean1) / se_diff;
            
            // Calculate degrees of freedom (Welch-Satterthwaite equation)
            double numerator = std::pow(var1/group1.size() + var2/group2.size(), 2);
            double denominator = std::pow(var1/group1.size(), 2)/(group1.size()-1) +
                               std::pow(var2/group2.size(), 2)/(group2.size()-1);
            result.degrees_of_freedom = numerator / denominator;
            
            // Calculate p-value (would use t-distribution in practice)
            result.p_value = calculatePValueFromT(result.test_statistic, 
                                                result.degrees_of_freedom);
            
            return result;
        }
        
        ParametricTestResult performPairedTTest(
            const std::vector<double>& group1,
            const std::vector<double>& group2,
            const StatisticalTestParameters& params) {
            
            ParametricTestResult result;
            
            // Calculate differences
            std::vector<double> differences;
            size_t n = std::min(group1.size(), group2.size());
            
            for (size_t i = 0; i < n; i++) {
                differences.push_back(group2[i] - group1[i]);
            }
            
            // Calculate mean and std dev of differences
            double mean_diff = calculateMean(differences);
            double std_diff = std::sqrt(calculateVariance(differences, mean_diff));
            
            // Calculate t-statistic
            double std_error = std_diff / std::sqrt(n);
            result.test_statistic = mean_diff / std_error;
            result.degrees_of_freedom = n - 1;
            
            // Calculate p-value
            result.p_value = calculatePValueFromT(std::abs(result.test_statistic), 
                                                result.degrees_of_freedom);
            
            // For two-sided test, multiply by 2
            if (params.alternative_hypothesis == "two_sided") {
                result.p_value *= 2;
            }
            
            return result;
        }
        
        double calculateMean(const std::vector<double>& data) {
            if (data.empty()) return 0.0;
            double sum = std::accumulate(data.begin(), data.end(), 0.0);
            return sum / data.size();
        }
        
        double calculateVariance(const std::vector<double>& data, double mean) {
            if (data.size() < 2) return 0.0;
            
            double sum_sq_diff = 0.0;
            for (double val : data) {
                sum_sq_diff += std::pow(val - mean, 2);
            }
            
            return sum_sq_diff / (data.size() - 1);  // Sample variance
        }
        
        double calculatePValueFromT(double t_stat, double df) {
            // In practice, would use t-distribution
            // This is a simplified approximation
            return 0.05;  // Placeholder
        }
    };
    
    class NonParametricTester {
    public:
        virtual NonParametricTestResult performTest(
            const std::vector<double>& group1,
            const std::vector<double>& group2,
            const StatisticalTestParameters& params) = 0;
    };
    
    class NonParametricStatisticalTester : public NonParametricTester {
    public:
        NonParametricTestResult performTest(
            const std::vector<double>& group1,
            const std::vector<double>& group2,
            const StatisticalTestParameters& params) override {
            
            NonParametricTestResult result;
            result.test_time = this->now();
            
            if (params.paired_test) {
                // Wilcoxon signed-rank test
                result = performWilcoxonTest(group1, group2, params);
            } else {
                // Mann-Whitney U test (Wilcoxon rank-sum test)
                result = performMannWhitneyTest(group1, group2, params);
            }
            
            // Determine significance
            result.significant_difference = result.p_value < params.alpha_level;
            
            return result;
        }

    private:
        NonParametricTestResult performMannWhitneyTest(
            const std::vector<double>& group1,
            const std::vector<double>& group2,
            const StatisticalTestParameters& params) {
            
            NonParametricTestResult result;
            
            // Combine and rank all observations
            std::vector<std::pair<double, int>> combined;  // (value, group_id)
            
            for (double val : group1) {
                combined.push_back({val, 1});  // Group 1
            }
            for (double val : group2) {
                combined.push_back({val, 2});  // Group 2
            }
            
            // Sort by value
            std::sort(combined.begin(), combined.end());
            
            // Assign ranks (handling ties)
            std::vector<double> ranks = assignRanks(combined);
            
            // Calculate sum of ranks for each group
            double rank_sum1 = 0.0, rank_sum2 = 0.0;
            for (size_t i = 0; i < combined.size(); i++) {
                if (combined[i].second == 1) {
                    rank_sum1 += ranks[i];
                } else {
                    rank_sum2 += ranks[i];
                }
            }
            
            // Calculate U statistics
            double n1 = group1.size();
            double n2 = group2.size();
            double u1 = n1 * n2 + n1 * (n1 + 1) / 2.0 - rank_sum1;
            double u2 = n1 * n2 + n2 * (n2 + 1) / 2.0 - rank_sum2;
            result.test_statistic = std::min(u1, u2);
            
            // Calculate p-value (would use Mann-Whitney U distribution)
            result.p_value = calculatePValueFromMW(result.test_statistic, n1, n2);
            
            return result;
        }
        
        NonParametricTestResult performWilcoxonTest(
            const std::vector<double>& group1,
            const std::vector<double>& group2,
            const StatisticalTestParameters& params) {
            
            NonParametricTestResult result;
            
            // Calculate differences
            std::vector<double> differences;
            size_t n = std::min(group1.size(), group2.size());
            
            for (size_t i = 0; i < n; i++) {
                differences.push_back(group2[i] - group1[i]);
            }
            
            // Remove zeros
            differences.erase(
                std::remove_if(differences.begin(), differences.end(),
                             [](double x) { return std::abs(x) < 1e-10; }),
                differences.end());
            
            if (differences.empty()) {
                result.test_statistic = 0.0;
                result.p_value = 1.0;
                return result;
            }
            
            // Calculate absolute values and signs
            std::vector<std::pair<double, int>> abs_signed;  // (abs_value, sign)
            for (double diff : differences) {
                abs_signed.push_back({std::abs(diff), diff > 0 ? 1 : -1});
            }
            
            // Sort by absolute value
            std::sort(abs_signed.begin(), abs_signed.end());
            
            // Assign ranks
            std::vector<double> ranks = assignRanks(abs_signed);
            
            // Calculate test statistic (sum of positive ranks)
            double w_plus = 0.0;
            for (size_t i = 0; i < abs_signed.size(); i++) {
                if (abs_signed[i].second == 1) {  // Positive difference
                    w_plus += ranks[i];
                }
            }
            
            result.test_statistic = w_plus;
            
            // Calculate p-value (would use Wilcoxon signed-rank distribution)
            result.p_value = calculatePValueFromWSR(result.test_statistic, 
                                                  abs_signed.size());
            
            return result;
        }
        
        std::vector<double> assignRanks(const std::vector<std::pair<double, int>>& values) {
            std::vector<double> ranks(values.size());
            
            // Assign preliminary ranks
            for (size_t i = 0; i < values.size(); i++) {
                ranks[i] = i + 1;
            }
            
            // Handle ties by averaging ranks
            for (size_t i = 0; i < values.size(); ) {
                size_t j = i;
                // Find all tied values
                while (j < values.size() && 
                       std::abs(values[j].first - values[i].first) < 1e-10) {
                    j++;
                }
                
                if (j > i + 1) {
                    // Average the ranks for tied values
                    double avg_rank = (i + 1 + j) / 2.0;
                    for (size_t k = i; k < j; k++) {
                        ranks[k] = avg_rank;
                    }
                }
                
                i = j;
            }
            
            return ranks;
        }
        
        double calculatePValueFromMW(double u_stat, double n1, double n2) {
            // In practice, would use Mann-Whitney U distribution
            return 0.05;  // Placeholder
        }
        
        double calculatePValueFromWSR(double w_stat, double n) {
            // In practice, would use Wilcoxon signed-rank distribution
            return 0.05;  // Placeholder
        }
    };
    
    class EffectSizeCalculator {
    public:
        virtual EffectSizeResult calculate(const std::vector<double>& group1,
                                        const std::vector<double>& group2) = 0;
    };
    
    class EffectSizeCalculatorImpl : public EffectSizeCalculator {
    public:
        EffectSizeResult calculate(const std::vector<double>& group1,
                                 const std::vector<double>& group2) override {
            
            EffectSizeResult result;
            result.calculation_time = this->now();
            
            double mean1 = calculateMean(group1);
            double mean2 = calculateMean(group2);
            double var1 = calculateVariance(group1, mean1);
            double var2 = calculateVariance(group2, mean2);
            
            int n1 = group1.size();
            int n2 = group2.size();
            
            // Cohen's d
            double pooled_std = std::sqrt(((n1-1)*var1 + (n2-1)*var2) / (n1 + n2 - 2));
            result.cohens_d = (mean2 - mean1) / pooled_std;
            
            // Glass's delta (using group 1's std dev)
            result.glass_delta = (mean2 - mean1) / std::sqrt(var1);
            
            // Hedges' g (bias-corrected)
            double correction_factor = 1.0 - 3.0 / (4*(n1 + n2) - 9);
            result.hedges_g = result.cohens_d * correction_factor;
            
            // Cliff's delta (non-parametric)
            result.cliff_delta = calculateCliffsDelta(group1, group2);
            
            // Interpret effect size
            result.interpretation = interpretEffectSize(result.cohens_d);
            
            return result;
        }

    private:
        double calculateMean(const std::vector<double>& data) {
            if (data.empty()) return 0.0;
            double sum = std::accumulate(data.begin(), data.end(), 0.0);
            return sum / data.size();
        }
        
        double calculateVariance(const std::vector<double>& data, double mean) {
            if (data.size() < 2) return 0.0;
            
            double sum_sq_diff = 0.0;
            for (double val : data) {
                sum_sq_diff += std::pow(val - mean, 2);
            }
            
            return sum_sq_diff / (data.size() - 1);
        }
        
        double calculateCliffsDelta(const std::vector<double>& group1,
                                  const std::vector<double>& group2) {
            
            int wins = 0;
            int losses = 0;
            int ties = 0;
            
            for (double x : group1) {
                for (double y : group2) {
                    if (x > y) wins++;
                    else if (x < y) losses++;
                    else ties++;
                }
            }
            
            int total_comparisons = wins + losses + ties;
            if (total_comparisons == 0) return 0.0;
            
            return static_cast<double>(wins - losses) / total_comparisons;
        }
        
        std::string interpretEffectSize(double cohen_d) {
            double abs_d = std::abs(cohen_d);
            
            if (abs_d < 0.2) return "negligible";
            else if (abs_d < 0.5) return "small";
            else if (abs_d < 0.8) return "medium";
            else return "large";
        }
    };
    
    class PowerAnalyzer {
    public:
        virtual PowerAnalysisResult analyze(const std::vector<double>& group1,
                                          const std::vector<double>& group2,
                                          const StatisticalTestParameters& params) = 0;
    };
    
    class PowerAnalysisCalculator : public PowerAnalyzer {
    public:
        PowerAnalysisResult analyze(const std::vector<double>& group1,
                                  const StatisticalTestParameters& params) override {
            
            PowerAnalysisResult result;
            result.analysis_time = this->now();
            
            // Calculate achieved power based on effect size and sample size
            double effect_size = calculateEffectSize(group1, group2);
            int sample_size = std::min(group1.size(), group2.size());
            
            result.achieved_power = calculatePower(effect_size, sample_size, 
                                                 params.alpha_level);
            result.type_ii_error_probability = 1.0 - result.achieved_power;
            
            // Calculate required sample size for desired power
            result.required_sample_size = calculateRequiredSampleSize(
                effect_size, params.alpha_level, DESIRED_POWER);
            
            return result;
        }

    private:
        double calculateEffectSize(const std::vector<double>& group1,
                                 const std::vector<double>& group2) {
            // Calculate Cohen's d
            double mean1 = calculateMean(group1);
            double mean2 = calculateMean(group2);
            double var1 = calculateVariance(group1, mean1);
            double var2 = calculateVariance(group2, mean2);
            
            int n1 = group1.size();
            int n2 = group2.size();
            
            double pooled_std = std::sqrt(((n1-1)*var1 + (n2-1)*var2) / (n1 + n2 - 2));
            return std::abs(mean2 - mean1) / pooled_std;
        }
        
        double calculatePower(double effect_size, int sample_size, double alpha) {
            // Simplified power calculation
            // In practice, would use non-central t-distribution
            if (sample_size < 2) return 0.05;  // Minimum power
            
            // Approximate power calculation
            double non_centrality = effect_size * std::sqrt(sample_size / 2.0);
            // This is a simplified approximation
            return std::min(1.0, 0.05 + 0.95 * (non_centrality / (non_centrality + 2.0)));
        }
        
        int calculateRequiredSampleSize(double effect_size, 
                                      double alpha_level, 
                                      double desired_power) {
            // Iteratively find required sample size
            int n = 2;
            while (n < 10000) {  // Prevent infinite loop
                double power = calculatePower(effect_size, n, alpha_level);
                if (power >= desired_power) {
                    return n;
                }
                n++;
            }
            return n;  // Return maximum tried value
        }
        
        double calculateMean(const std::vector<double>& data) {
            if (data.empty()) return 0.0;
            double sum = std::accumulate(data.begin(), data.end(), 0.0);
            return sum / data.size();
        }
        
        double calculateVariance(const std::vector<double>& data, double mean) {
            if (data.size() < 2) return 0.0;
            
            double sum_sq_diff = 0.0;
            for (double val : data) {
                sum_sq_diff += std::pow(val - mean, 2);
            }
            
            return sum_sq_diff / (data.size() - 1);
        }
        
        static constexpr double DESIRED_POWER = 0.80;  // 80% power
    };
    
    ConfidenceIntervalResult calculateConfidenceIntervals(
        const std::vector<double>& group1,
        const std::vector<double>& group2,
        double confidence_level) {
        
        ConfidenceIntervalResult result;
        result.confidence_level = confidence_level;
        result.calculation_time = this->now();
        
        // Calculate confidence interval for difference in means
        double mean1 = calculateMean(group1);
        double mean2 = calculateMean(group2);
        double var1 = calculateVariance(group1, mean1);
        double var2 = calculateVariance(group2, mean2);
        
        int n1 = group1.size();
        int n2 = group2.size();
        
        // Standard error of difference
        double se_diff = std::sqrt(var1/n1 + var2/n2);
        
        // Critical value (would use t-distribution in practice)
        double critical_value = getCriticalValue(confidence_level, 
                                               n1 + n2 - 2);  // Welch's df
        
        double diff = mean2 - mean1;
        double margin_error = critical_value * se_diff;
        
        result.lower_bound = diff - margin_error;
        result.upper_bound = diff + margin_error;
        
        return result;
    }
    
    StatisticalInterpretation generateInterpretation(
        const StatisticalTestResult& result,
        const StatisticalTestParameters& params) {
        
        StatisticalInterpretation interpretation;
        interpretation.interpretation_time = this->now();
        
        // Overall conclusion
        if (result.parametric_result.significant_difference || 
            result.non_parametric_result.significant_difference) {
            interpretation.overall_conclusion = "Statistically significant difference detected";
        } else {
            interpretation.overall_conclusion = "No statistically significant difference detected";
        }
        
        // Statistical significance
        double p_value = (result.test_type_used == "parametric") ? 
                        result.parametric_result.p_value : 
                        result.non_parametric_result.p_value;
        
        interpretation.statistical_significance = 
            "p-value = " + std::to_string(p_value) + 
            (p_value < params.alpha_level ? " (< α, significant)" : " (≥ α, not significant)");
        
        // Effect size interpretation
        interpretation.effect_size_interpretation = 
            "Effect size (Cohen's d): " + std::to_string(result.effect_size.cohens_d) +
            " (" + result.effect_size.interpretation + ")";
        
        // Practical significance
        if (std::abs(result.effect_size.cohens_d) > PRACTICAL_SIGNIFICANCE_THRESHOLD) {
            interpretation.practical_significance = "Effect appears practically significant";
        } else {
            interpretation.practical_significance = "Effect may not be practically significant";
        }
        
        // Limitations
        if (!result.normality_test.passed) {
            interpretation.limitations.push_back("Data did not meet normality assumption");
        }
        if (!result.variance_homogeneity_test.passed) {
            interpretation.limitations.push_back("Groups did not have equal variances");
        }
        if (result.power_analysis.achieved_power < MIN_ACCEPTABLE_POWER) {
            interpretation.limitations.push_back("Study may be underpowered");
        }
        
        // Recommendations
        if (result.power_analysis.achieved_power < MIN_ACCEPTABLE_POWER) {
            interpretation.recommendations.push_back(
                "Consider increasing sample size to achieve adequate power");
        }
        if (!result.normality_test.passed) {
            interpretation.recommendations.push_back(
                "Consider using non-parametric tests or data transformation");
        }
        
        return interpretation;
    }
    
    double calculateMean(const std::vector<double>& data) {
        if (data.empty()) return 0.0;
        double sum = std::accumulate(data.begin(), data.end(), 0.0);
        return sum / data.size();
    }
    
    double calculateVariance(const std::vector<double>& data, double mean) {
        if (data.size() < 2) return 0.0;
        
        double sum_sq_diff = 0.0;
        for (double val : data) {
            sum_sq_diff += std::pow(val - mean, 2);
        }
        
        return sum_sq_diff / (data.size() - 1);
    }
    
    double getCriticalValue(double confidence_level, int degrees_of_freedom) {
        // In practice, would use t-distribution table
        // This is a simplified approximation
        if (confidence_level == 0.95) return 1.96;  // Approximate for large df
        else if (confidence_level == 0.99) return 2.58;
        else return 1.96;  // Default
    }
    
    static constexpr double PRACTICAL_SIGNIFICANCE_THRESHOLD = 0.2;
    static constexpr double MIN_ACCEPTABLE_POWER = 0.80;
};
```

### Bayesian Analysis in Robotics

Bayesian analysis provides an alternative framework for statistical inference that is particularly useful in robotics research:

```cpp
class BayesianAnalysisFramework {
private:
    std::unique_ptr<PriorSpecification> prior_specifier_;
    std::unique_ptr<LikelihoodCalculator> likelihood_calculator_;
    std::unique_ptr<PosteriorSampler> posterior_sampler_;
    std::unique_ptr<BayesianInferenceEngine> inference_engine_;
    std::unique_ptr<ModelComparison> model_comparison_;

public:
    BayesianAnalysisFramework() {
        prior_specifier_ = std::make_unique<InformativePriorSpecifier>();
        likelihood_calculator_ = std::make_unique<RoboticsLikelihoodCalculator>();
        posterior_sampler_ = std::make_unique<MarkovChainMonteCarloSampler>();
        inference_engine_ = std::make_unique<BayesianInferenceEngineImpl>();
        model_comparison_ = std::make_unique<BayesianModelComparison>();
    }
    
    BayesianAnalysisResult performBayesianAnalysis(
        const std::vector<RoboticsDataPoint>& data,
        const BayesianAnalysisParameters& params) {
        
        BayesianAnalysisResult result;
        
        // Specify prior distributions
        auto priors = prior_specifier_->specifyPriors(params);
        result.prior_distributions = priors;
        
        // Calculate likelihood
        auto likelihood = likelihood_calculator_->calculateLikelihood(data, params);
        result.likelihood = likelihood;
        
        // Sample from posterior distribution
        auto posterior_samples = posterior_sampler_->sample(
            priors, likelihood, params.n_mcmc_samples);
        result.posterior_samples = posterior_samples;
        
        // Perform Bayesian inference
        auto inference_results = inference_engine_->infer(posterior_samples, params);
        result.inference_results = inference_results;
        
        // Compare models if specified
        if (params.model_comparison_enabled) {
            auto model_comparison = model_comparison_->compare(
                data, params.models);
            result.model_comparison = model_comparison;
        }
        
        // Calculate credible intervals
        result.credible_intervals = calculateCredibleIntervals(
            posterior_samples, params.credible_interval_prob);
        
        // Generate interpretation
        result.interpretation = generateBayesianInterpretation(
            inference_results, params);
        
        return result;
    }

private:
    struct BayesianAnalysisResult {
        PriorDistributions prior_distributions;
        LikelihoodResult likelihood;
        std::vector<PosteriorSample> posterior_samples;
        BayesianInferenceResult inference_results;
        ModelComparisonResult model_comparison;
        CredibleIntervalResult credible_intervals;
        BayesianInterpretation interpretation;
        rclcpp::Time analysis_time;
    };
    
    struct BayesianAnalysisParameters {
        std::vector<std::string> parameters_of_interest;
        int n_mcmc_samples;
        double credible_interval_prob;
        double mcmc_thinning;
        int mcmc_burn_in;
        bool model_comparison_enabled;
        std::vector<ModelSpecification> models;
        PriorSpecification prior_specs;
        rclcpp::Time analysis_deadline;
    };
    
    struct PriorDistributions {
        std::map<std::string, Distribution> priors;
        std::string specification_method;
        rclcpp::Time specification_time;
    };
    
    struct LikelihoodResult {
        std::vector<double> likelihood_values;
        std::string likelihood_function;
        rclcpp::Time calculation_time;
    };
    
    struct PosteriorSample {
        std::map<std::string, double> parameter_values;
        double log_posterior_density;
        int iteration;
        rclcpp::Time sample_time;
    };
    
    struct BayesianInferenceResult {
        std::map<std::string, ParameterEstimate> parameter_estimates;
        std::map<std::string, std::vector<double>> marginal_distributions;
        std::vector<HypothesisTestResult> hypothesis_tests;
        rclcpp::Time inference_time;
    };
    
    struct ParameterEstimate {
        double mean;
        double median;
        double mode;
        double std_dev;
        double variance;
    };
    
    struct HypothesisTestResult {
        std::string hypothesis;
        double bayes_factor;
        double posterior_probability;
        std::string conclusion;
    };
    
    struct ModelComparisonResult {
        std::vector<ModelEvidence> model_evidences;
        std::vector<BayesFactor> bayes_factors;
        std::vector<ModelProbability> model_probabilities;
        std::string best_model;
        rclcpp::Time comparison_time;
    };
    
    struct CredibleIntervalResult {
        std::map<std::string, std::pair<double, double>> intervals;  // lower, upper
        double probability;
        rclcpp::Time calculation_time;
    };
    
    struct BayesianInterpretation {
        std::string overall_conclusion;
        std::vector<std::string> key_findings;
        std::vector<std::string> uncertainties;
        std::vector<std::string> recommendations;
        std::string confidence_statement;
        rclcpp::Time interpretation_time;
    };
    
    struct Distribution {
        std::string type;  // "normal", "beta", "gamma", etc.
        std::vector<double> parameters;
        std::string description;
    };
    
    struct ModelSpecification {
        std::string name;
        std::string formula;
        std::vector<std::string> parameters;
        std::string description;
    };
    
    struct ModelEvidence {
        std::string model_name;
        double evidence;
        double log_evidence;
        rclcpp::Time calculation_time;
    };
    
    struct BayesFactor {
        std::string model1;
        std::string model2;
        double bayes_factor;
        std::string interpretation;
    };
    
    struct ModelProbability {
        std::string model_name;
        double probability;
        double posterior_odds;
    };
    
    struct RoboticsDataPoint {
        std::vector<double> joint_positions;
        std::vector<double> joint_velocities;
        std::vector<double> joint_efforts;
        geometry_msgs::msg::Pose end_effector_pose;
        bool success_indicator;
        double performance_metric;
        rclcpp::Time timestamp;
    };
    
    class PriorSpecification {
    public:
        virtual PriorDistributions specifyPriors(
            const BayesianAnalysisParameters& params) = 0;
    };
    
    class InformativePriorSpecifier : public PriorSpecification {
    public:
        PriorDistributions specifyPriors(
            const BayesianAnalysisParameters& params) override {
            
            PriorDistributions priors;
            priors.specification_method = "informative";
            priors.specification_time = this->now();
            
            // Use domain knowledge to specify informative priors
            for (const auto& param : params.parameters_of_interest) {
                if (param == "success_rate") {
                    // Beta prior for success rate (between 0 and 1)
                    Distribution dist;
                    dist.type = "beta";
                    dist.parameters = {2.0, 2.0};  // Weakly informative Beta(2,2)
                    dist.description = "Weakly informative beta prior for success rate";
                    priors.priors[param] = dist;
                } else if (param == "execution_time") {
                    // Gamma prior for positive continuous variable
                    Distribution dist;
                    dist.type = "gamma";
                    dist.parameters = {2.0, 0.5};  // Shape=2, Rate=0.5
                    dist.description = "Gamma prior for execution time";
                    priors.priors[param] = dist;
                } else if (param == "energy_consumption") {
                    // Normal prior for energy consumption
                    Distribution dist;
                    dist.type = "normal";
                    dist.parameters = {50.0, 10.0};  // Mean=50, Std=10
                    dist.description = "Normal prior for energy consumption (J)";
                    priors.priors[param] = dist;
                }
            }
            
            return priors;
        }
    };
    
    class LikelihoodCalculator {
    public:
        virtual LikelihoodResult calculateLikelihood(
            const std::vector<RoboticsDataPoint>& data,
            const BayesianAnalysisParameters& params) = 0;
    };
    
    class RoboticsLikelihoodCalculator : public LikelihoodCalculator {
    public:
        LikelihoodResult calculateLikelihood(
            const std::vector<RoboticsDataPoint>& data,
            const BayesianAnalysisParameters& params) override {
            
            LikelihoodResult result;
            result.likelihood_function = "custom_robotics_likelihood";
            result.calculation_time = this->now();
            
            // Calculate likelihood for each data point
            for (const auto& datapoint : data) {
                double likelihood = calculateDatapointLikelihood(datapoint, params);
                result.likelihood_values.push_back(likelihood);
            }
            
            return result;
        }

    private:
        double calculateDatapointLikelihood(const RoboticsDataPoint& datapoint,
                                          const BayesianAnalysisParameters& params) {
            
            // Calculate likelihood based on robotics-specific models
            // This would depend on the specific parameters being estimated
            
            double likelihood = 1.0;
            
            // Example: likelihood for success/failure outcome
            if (std::find(params.parameters_of_interest.begin(), 
                         params.parameters_of_interest.end(), 
                         "success_rate") != params.parameters_of_interest.end()) {
                
                // Bernoulli likelihood for success indicator
                double success_prob = estimateSuccessProbability(datapoint);
                likelihood *= datapoint.success_indicator ? success_prob : (1 - success_prob);
            }
            
            // Example: likelihood for continuous performance metric
            if (std::find(params.parameters_of_interest.begin(), 
                         params.parameters_of_interest.end(), 
                         "performance") != params.parameters_of_interest.end()) {
                
                // Normal likelihood for performance metric
                double expected_performance = calculateExpectedPerformance(datapoint);
                double performance_variance = 1.0;  // Assumed variance
                double performance_residual = datapoint.performance_metric - expected_performance;
                
                double normal_likelihood = (1.0 / std::sqrt(2 * M_PI * performance_variance)) *
                                          std::exp(-0.5 * performance_residual * performance_residual / performance_variance);
                
                likelihood *= normal_likelihood;
            }
            
            return likelihood;
        }
        
        double estimateSuccessProbability(const RoboticsDataPoint& datapoint) {
            // Estimate success probability based on robot state
            // This would use domain knowledge or preliminary analysis
            return 0.8;  // Placeholder
        }
        
        double calculateExpectedPerformance(const RoboticsDataPoint& datapoint) {
            // Calculate expected performance based on robot state
            return 0.0;  // Placeholder
        }
    };
    
    class PosteriorSampler {
    public:
        virtual std::vector<PosteriorSample> sample(
            const PriorDistributions& priors,
            const LikelihoodResult& likelihood,
            int n_samples) = 0;
    };
    
    class MarkovChainMonteCarloSampler : public PosteriorSampler {
    public:
        std::vector<PosteriorSample> sample(
            const PriorDistributions& priors,
            const LikelihoodResult& likelihood,
            int n_samples) override {
            
            std::vector<PosteriorSample> samples;
            
            // Initialize chain
            PosteriorSample current_sample = initializeChain(priors);
            
            // MCMC sampling
            for (int i = 0; i < n_samples; i++) {
                PosteriorSample proposed_sample = proposeNewState(current_sample, priors);
                
                // Calculate acceptance probability
                double acceptance_prob = calculateAcceptanceProbability(
                    current_sample, proposed_sample, priors, likelihood);
                
                // Accept or reject
                if (std::rand() / double(RAND_MAX) < acceptance_prob) {
                    current_sample = proposed_sample;
                }
                
                // Store sample (after burn-in and thinning)
                if (i >= MCMC_BURN_IN && i % MCMC_THINNING_INTERVAL == 0) {
                    samples.push_back(current_sample);
                }
            }
            
            return samples;
        }

    private:
        PosteriorSample initializeChain(const PriorDistributions& priors) {
            PosteriorSample sample;
            sample.iteration = 0;
            sample.sample_time = this->now();
            
            // Initialize parameters from prior distributions
            for (const auto& [param_name, dist] : priors.priors) {
                double init_value = sampleFromDistribution(dist);
                sample.parameter_values[param_name] = init_value;
            }
            
            return sample;
        }
        
        PosteriorSample proposeNewState(const PosteriorSample& current,
                                      const PriorDistributions& priors) {
            PosteriorSample proposed = current;
            proposed.iteration = current.iteration + 1;
            proposed.sample_time = this->now();
            
            // Propose new values using random walk
            for (auto& [param_name, value] : proposed.parameter_values) {
                double proposal_std = getProposalStdDev(param_name);
                double noise = generateNormalNoise(0.0, proposal_std);
                proposed.parameter_values[param_name] = value + noise;
                
                // Ensure parameter stays within reasonable bounds
                proposed.parameter_values[param_name] = 
                    constrainParameter(param_name, proposed.parameter_values[param_name]);
            }
            
            return proposed;
        }
        
        double calculateAcceptanceProbability(const PosteriorSample& current,
                                            const PosteriorSample& proposed,
                                            const PriorDistributions& priors,
                                            const LikelihoodResult& likelihood) {
            
            // Calculate posterior densities
            double current_posterior = calculatePosteriorDensity(current, priors, likelihood);
            double proposed_posterior = calculatePosteriorDensity(proposed, priors, likelihood);
            
            // Metropolis-Hastings acceptance probability
            double acceptance_prob = std::min(1.0, proposed_posterior / current_posterior);
            
            return acceptance_prob;
        }
        
        double calculatePosteriorDensity(const PosteriorSample& sample,
                                       const PriorDistributions& priors,
                                       const LikelihoodResult& likelihood) {
            
            // Calculate prior density
            double prior_density = 1.0;
            for (const auto& [param_name, value] : sample.parameter_values) {
                if (priors.priors.count(param_name)) {
                    double param_prior = evaluatePriorDensity(
                        priors.priors.at(param_name), value);
                    prior_density *= param_prior;
                }
            }
            
            // Calculate likelihood (simplified - would normally be data-dependent)
            double likelihood_value = evaluateLikelihood(sample, likelihood);
            
            // Posterior is proportional to prior * likelihood
            return prior_density * likelihood_value;
        }
        
        double evaluatePriorDensity(const Distribution& dist, double value) {
            if (dist.type == "beta") {
                // Beta distribution PDF
                double alpha = dist.parameters[0];
                double beta = dist.parameters[1];
                // Simplified evaluation
                return 1.0;  // Placeholder
            } else if (dist.type == "gamma") {
                // Gamma distribution PDF
                double shape = dist.parameters[0];
                double rate = dist.parameters[1];
                // Simplified evaluation
                return 1.0;  // Placeholder
            } else if (dist.type == "normal") {
                // Normal distribution PDF
                double mean = dist.parameters[0];
                double std_dev = dist.parameters[1];
                // Simplified evaluation
                return 1.0;  // Placeholder
            }
            
            return 1.0;  // Uniform if unknown
        }
        
        double evaluateLikelihood(const PosteriorSample& sample,
                                const LikelihoodResult& likelihood) {
            // Simplified likelihood evaluation
            // In practice, this would depend on the specific model and data
            return 1.0;  // Placeholder
        }
        
        double sampleFromDistribution(const Distribution& dist) {
            // Sample from specified distribution
            if (dist.type == "beta") {
                // Sample from beta distribution
                return 0.5;  // Placeholder
            } else if (dist.type == "gamma") {
                // Sample from gamma distribution
                return 1.0;  // Placeholder
            } else if (dist.type == "normal") {
                // Sample from normal distribution
                return 0.0;  // Placeholder
            }
            
            return 0.0;  // Default
        }
        
        double getProposalStdDev(const std::string& param_name) {
            // Return appropriate proposal standard deviation for parameter
            return 0.1;  // Placeholder
        }
        
        double generateNormalNoise(double mean, double std_dev) {
            // Generate normal random noise
            static std::random_device rd;
            static std::mt19937 gen(rd());
            static std::normal_distribution<> dis(mean, std_dev);
            return dis(gen);
        }
        
        double constrainParameter(const std::string& param_name, double value) {
            // Apply parameter constraints based on type
            if (param_name == "success_rate") {
                return std::clamp(value, 0.0, 1.0);
            }
            return value;
        }
        
        static constexpr int MCMC_BURN_IN = 1000;
        static constexpr int MCMC_THINNING_INTERVAL = 10;
    };
    
    class BayesianInferenceEngine {
    public:
        virtual BayesianInferenceResult infer(
            const std::vector<PosteriorSample>& samples,
            const BayesianAnalysisParameters& params) = 0;
    };
    
    class BayesianInferenceEngineImpl : public BayesianInferenceEngine {
    public:
        BayesianInferenceResult infer(
            const std::vector<PosteriorSample>& samples,
            const BayesianAnalysisParameters& params) override {
            
            BayesianInferenceResult result;
            result.inference_time = this->now();
            
            if (samples.empty()) {
                return result;
            }
            
            // Calculate parameter estimates for each parameter of interest
            for (const auto& param : params.parameters_of_interest) {
                auto estimates = calculateParameterEstimates(samples, param);
                result.parameter_estimates[param] = estimates;
                
                // Calculate marginal distribution
                auto marginal = calculateMarginalDistribution(samples, param);
                result.marginal_distributions[param] = marginal;
            }
            
            // Perform hypothesis tests
            for (const auto& hypothesis : params.hypotheses) {
                auto test_result = performHypothesisTest(samples, hypothesis);
                result.hypothesis_tests.push_back(test_result);
            }
            
            return result;
        }

    private:
        ParameterEstimate calculateParameterEstimates(
            const std::vector<PosteriorSample>& samples,
            const std::string& param_name) {
            
            ParameterEstimate estimates;
            
            if (samples.empty()) {
                return estimates;
            }
            
            // Extract parameter values
            std::vector<double> values;
            for (const auto& sample : samples) {
                if (sample.parameter_values.count(param_name)) {
                    values.push_back(sample.parameter_values.at(param_name));
                }
            }
            
            if (values.empty()) {
                return estimates;
            }
            
            // Calculate statistics
            estimates.mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
            
            // Calculate median
            std::vector<double> sorted_values = values;
            std::sort(sorted_values.begin(), sorted_values.end());
            size_t n = sorted_values.size();
            if (n % 2 == 0) {
                estimates.median = (sorted_values[n/2 - 1] + sorted_values[n/2]) / 2.0;
            } else {
                estimates.median = sorted_values[n/2];
            }
            
            // Calculate variance and std dev
            double variance = 0.0;
            for (double val : values) {
                variance += (val - estimates.mean) * (val - estimates.mean);
            }
            variance /= values.size();
            estimates.variance = variance;
            estimates.std_dev = std::sqrt(variance);
            
            // For mode, we'd typically use kernel density estimation
            // For simplicity, using the value closest to mean
            double closest_to_mean = values[0];
            double min_diff = std::abs(values[0] - estimates.mean);
            for (double val : values) {
                double diff = std::abs(val - estimates.mean);
                if (diff < min_diff) {
                    min_diff = diff;
                    closest_to_mean = val;
                }
            }
            estimates.mode = closest_to_mean;
            
            return estimates;
        }
        
        std::vector<double> calculateMarginalDistribution(
            const std::vector<PosteriorSample>& samples,
            const std::string& param_name) {
            
            std::vector<double> marginal;
            
            for (const auto& sample : samples) {
                if (sample.parameter_values.count(param_name)) {
                    marginal.push_back(sample.parameter_values.at(param_name));
                }
            }
            
            return marginal;
        }
        
        HypothesisTestResult performHypothesisTest(
            const std::vector<PosteriorSample>& samples,
            const HypothesisSpecification& hypothesis) {
            
            HypothesisTestResult result;
            result.hypothesis = hypothesis.description;
            
            // Calculate posterior probability of hypothesis
            int hypothesis_support_count = 0;
            for (const auto& sample : samples) {
                if (evaluateHypothesis(sample, hypothesis)) {
                    hypothesis_support_count++;
                }
            }
            
            result.posterior_probability = 
                static_cast<double>(hypothesis_support_count) / samples.size();
            
            // Calculate Bayes factor against null hypothesis
            // This is a simplified calculation
            result.bayes_factor = calculateSimpleBayesFactor(
                result.posterior_probability, hypothesis.prior_probability);
            
            // Determine conclusion
            if (result.posterior_probability > HYPOTHESIS_THRESHOLD) {
                result.conclusion = "Hypothesis supported";
            } else {
                result.conclusion = "Hypothesis not supported";
            }
            
            return result;
        }
        
        bool evaluateHypothesis(const PosteriorSample& sample,
                              const HypothesisSpecification& hypothesis) {
            // Evaluate whether sample supports the hypothesis
            // This would depend on the specific hypothesis
            return true;  // Placeholder
        }
        
        double calculateSimpleBayesFactor(double posterior_prob, 
                                        double prior_prob) {
            if (prior_prob == 0.0) return std::numeric_limits<double>::infinity();
            return posterior_prob / prior_prob;
        }
        
        struct HypothesisSpecification {
            std::string description;
            std::string condition;
            double prior_probability;
        };
        
        static constexpr double HYPOTHESIS_THRESHOLD = 0.95;
    };
    
    CredibleIntervalResult calculateCredibleIntervals(
        const std::vector<PosteriorSample>& samples,
        double probability) {
        
        CredibleIntervalResult result;
        result.probability = probability;
        result.calculation_time = this->now();
        
        if (samples.empty()) {
            return result;
        }
        
        // Calculate credible interval for each parameter
        if (!samples.empty() && !samples[0].parameter_values.empty()) {
            for (const auto& [param_name, dummy_value] : samples[0].parameter_values) {
                // Extract all samples for this parameter
                std::vector<double> param_samples;
                for (const auto& sample : samples) {
                    if (sample.parameter_values.count(param_name)) {
                        param_samples.push_back(sample.parameter_values.at(param_name));
                    }
                }
                
                if (!param_samples.empty()) {
                    // Calculate credible interval
                    auto interval = calculateParameterCredibleInterval(
                        param_samples, probability);
                    result.intervals[param_name] = interval;
                }
            }
        }
        
        return result;
    }
    
    std::pair<double, double> calculateParameterCredibleInterval(
        const std::vector<double>& samples,
        double probability) {
        
        if (samples.size() < 2) {
            return {0.0, 0.0};
        }
        
        // Sort samples
        std::vector<double> sorted_samples = samples;
        std::sort(sorted_samples.begin(), sorted_samples.end());
        
        // Calculate tail probabilities
        double alpha = 1.0 - probability;
        double lower_tail = alpha / 2.0;
        double upper_tail = 1.0 - alpha / 2.0;
        
        // Calculate indices
        int lower_idx = static_cast<int>(lower_tail * sorted_samples.size());
        int upper_idx = static_cast<int>(upper_tail * sorted_samples.size()) - 1;
        
        // Ensure bounds
        lower_idx = std::max(0, lower_idx);
        upper_idx = std::min(static_cast<int>(sorted_samples.size()) - 1, upper_idx);
        
        return {sorted_samples[lower_idx], sorted_samples[upper_idx]};
    }
    
    BayesianInterpretation generateBayesianInterpretation(
        const BayesianInferenceResult& inference,
        const BayesianAnalysisParameters& params) {
        
        BayesianInterpretation interpretation;
        interpretation.interpretation_time = this->now();
        
        // Overall conclusion based on parameter estimates
        interpretation.overall_conclusion = "Bayesian analysis completed";
        
        // Key findings
        for (const auto& [param_name, estimates] : inference.parameter_estimates) {
            std::string finding = param_name + " estimated as " + 
                                std::to_string(estimates.mean) + " ± " + 
                                std::to_string(estimates.std_dev);
            interpretation.key_findings.push_back(finding);
        }
        
        // Uncertainties (represented by standard deviations)
        for (const auto& [param_name, estimates] : inference.parameter_estimates) {
            if (estimates.std_dev > UNCERTAINTY_THRESHOLD) {
                interpretation.uncertainties.push_back(
                    param_name + " shows high uncertainty (std dev: " + 
                    std::to_string(estimates.std_dev) + ")");
            }
        }
        
        // Recommendations
        if (interpretation.uncertainties.size() > 0) {
            interpretation.recommendations.push_back(
                "Consider collecting more data to reduce parameter uncertainty");
        }
        
        // Confidence statement
        interpretation.confidence_statement = 
            "Results based on " + std::to_string(params.n_mcmc_samples) + 
            " MCMC samples with " + std::to_string(params.credible_interval_prob*100) + 
            "% credible intervals";
        
        return interpretation;
    }
    
    static constexpr double UNCERTAINTY_THRESHOLD = 0.1;
};
```

## Reporting and Documentation

### Research Paper Structure

#### Scientific Writing for Robotics Research
```cpp
class ResearchPaperStructure {
public:
    ResearchPaperStructure() {
        paper_sections_ = initializePaperStructure();
    }
    
    PaperOutline generateOutline(const ResearchType& research_type) {
        PaperOutline outline;
        
        // Add standard sections
        outline.sections.push_back(createAbstractSection());
        outline.sections.push_back(createIntroductionSection());
        outline.sections.push_back(createRelatedWorkSection());
        
        // Add research-type-specific sections
        switch (research_type) {
            case ResearchType::ALGORITHMIC:
                outline.sections.push_back(createAlgorithmSection());
                outline.sections.push_back(createComplexityAnalysisSection());
                break;
                
            case ResearchType::HARDWARE:
                outline.sections.push_back(createHardwareDesignSection());
                outline.sections.push_back(createValidationSection());
                break;
                
            case ResearchType::SYSTEMS:
                outline.sections.push_back(createSystemArchitectureSection());
                outline.sections.push_back(createIntegrationSection());
                break;
                
            case ResearchType::APPLICATION:
                outline.sections.push_back(createUseCaseSection());
                outline.sections.push_back(createImpactAnalysisSection());
                break;
        }
        
        outline.sections.push_back(createMethodologySection());
        outline.sections.push_back(createResultsSection());
        outline.sections.push_back(createDiscussionSection());
        outline.sections.push_back(createConclusionSection());
        outline.sections.push_back(createReferencesSection());
        
        return outline;
    }
    
    WritingGuidelines getWritingGuidelines(const ResearchType& research_type) {
        WritingGuidelines guidelines;
        
        // General guidelines
        guidelines.style_guidelines = getGeneralStyleGuidelines();
        guidelines.figure_guidelines = getFigureGuidelines();
        guidelines.table_guidelines = getTableGuidelines();
        guidelines.equation_guidelines = getEquationGuidelines();
        
        // Research-type-specific guidelines
        switch (research_type) {
            case ResearchType::ALGORITHMIC:
                guidelines.content_guidelines = getAlgorithmicGuidelines();
                break;
            case ResearchType::HARDWARE:
                guidelines.content_guidelines = getHardwareGuidelines();
                break;
            case ResearchType::SYSTEMS:
                guidelines.content_guidelines = getSystemsGuidelines();
                break;
            case ResearchType::APPLICATION:
                guidelines.content_guidelines = getApplicationGuidelines();
                break;
        }
        
        return guidelines;
    }

private:
    std::vector<PaperSection> paper_sections_;
    
    enum class ResearchType {
        ALGORITHMIC,
        HARDWARE,
        SYSTEMS,
        APPLICATION
    };
    
    struct PaperOutline {
        std::vector<PaperSection> sections;
        std::string title;
        std::vector<std::string> authors;
        std::string abstract;
        rclcpp::Time creation_time;
    };
    
    struct PaperSection {
        std::string title;
        std::string section_id;
        std::vector<std::string> subsections;
        std::vector<Figure> figures;
        std::vector<Table> tables;
        std::vector<Equation> equations;
        std::string content_guidelines;
        int estimated_word_count;
    };
    
    struct Figure {
        std::string caption;
        std::string label;
        std::string file_path;
        std::string type;  // "graph", "photo", "diagram", etc.
        std::vector<std::string> referenced_in;
    };
    
    struct Table {
        std::string caption;
        std::string label;
        std::vector<std::vector<std::string>> data;
        std::vector<std::string> headers;
        std::vector<std::string> referenced_in;
    };
    
    struct Equation {
        std::string latex;
        std::string label;
        std::string description;
        std::vector<std::string> referenced_in;
    };
    
    struct WritingGuidelines {
        std::vector<std::string> style_guidelines;
        std::vector<std::string> figure_guidelines;
        std::vector<std::string> table_guidelines;
        std::vector<std::string> equation_guidelines;
        std::vector<std::string> content_guidelines;
    };
    
    std::vector<PaperSection> initializePaperStructure() {
        std::vector<PaperSection> structure;
        
        // Define standard sections
        PaperSection abstract_section;
        abstract_section.title = "Abstract";
        abstract_section.section_id = "abstract";
        abstract_section.estimated_word_count = 150-250;
        
        PaperSection intro_section;
        intro_section.title = "Introduction";
        intro_section.section_id = "intro";
        intro_section.estimated_word_count = 800-1200;
        
        PaperSection related_work_section;
        related_work_section.title = "Related Work";
        related_work_section.section_id = "related-work";
        related_work_section.estimated_word_count = 600-1000;
        
        PaperSection methodology_section;
        methodology_section.title = "Methodology";
        methodology_section.section_id = "methodology";
        methodology_section.estimated_word_count = 800-1500;
        
        PaperSection results_section;
        results_section.title = "Results and Discussion";
        results_section.section_id = "results";
        results_section.estimated_word_count = 1000-2000;
        
        PaperSection conclusion_section;
        conclusion_section.title = "Conclusion";
        conclusion_section.section_id = "conclusion";
        conclusion_section.estimated_word_count = 400-600;
        
        structure.push_back(abstract_section);
        structure.push_back(intro_section);
        structure.push_back(related_work_section);
        structure.push_back(methodology_section);
        structure.push_back(results_section);
        structure.push_back(conclusion_section);
        
        return structure;
    }
    
    PaperSection createAbstractSection() {
        PaperSection section;
        section.title = "Abstract";
        section.section_id = "abstract";
        section.estimated_word_count = 150-250;
        section.content_guidelines = 
            "Provide concise summary of problem, approach, results, and implications. "
            "Avoid citations and abbreviations. Focus on key contributions.";
        
        return section;
    }
    
    PaperSection createIntroductionSection() {
        PaperSection section;
        section.title = "Introduction";
        section.section_id = "intro";
        section.estimated_word_count = 800-1200;
        section.content_guidelines = 
            "Motivate the problem, state the research question, summarize contributions, "
            "outline approach, and preview results. End with paper organization.";
        
        return section;
    }
    
    PaperSection createRelatedWorkSection() {
        PaperSection section;
        section.title = "Related Work";
        section.section_id = "related-work";
        section.estimated_word_count = 600-1000;
        section.content_guidelines = 
            "Survey relevant literature chronologically and/or thematically. "
            "Identify gaps and position your work. Avoid simple laundry lists.";
        
        return section;
    }
    
    PaperSection createAlgorithmSection() {
        PaperSection section;
        section.title = "Algorithm Design";
        section.section_id = "algorithm";
        section.estimated_word_count = 600-1000;
        section.content_guidelines = 
            "Present algorithm pseudocode, explain key steps, prove correctness if possible, "
            "analyze complexity, and discuss design choices.";
        
        return section;
    }
    
    PaperSection createComplexityAnalysisSection() {
        PaperSection section;
        section.title = "Complexity Analysis";
        section.section_id = "complexity";
        section.estimated_word_count = 400-600;
        section.content_guidelines = 
            "Analyze time and space complexity. Consider best, worst, and average cases. "
            "Compare with existing approaches theoretically.";
        
        return section;
    }
    
    PaperSection createHardwareDesignSection() {
        PaperSection section;
        section.title = "Hardware Design";
        section.section_id = "hardware-design";
        section.estimated_word_count = 800-1200;
        section.content_guidelines = 
            "Describe design requirements, design process, components, assembly, "
            "and rationale for design choices. Include CAD drawings and specifications.";
        
        return section;
    }
    
    PaperSection createValidationSection() {
        PaperSection section;
        section.title = "Validation";
        section.section_id = "validation";
        section.estimated_word_count = 600-1000;
        section.content_guidelines = 
            "Describe validation methodology, experimental setup, results, and discussion. "
            "Include statistical analysis and uncertainty quantification.";
        
        return section;
    }
    
    PaperSection createSystemArchitectureSection() {
        PaperSection section;
        section.title = "System Architecture";
        section.section_id = "architecture";
        section.estimated_word_count = 600-1000;
        section.content_guidelines = 
            "Present system design, component interactions, data flows, and interfaces. "
            "Include architectural diagrams and design rationale.";
        
        return section;
    }
    
    PaperSection createIntegrationSection() {
        PaperSection section;
        section.title = "Integration and Testing";
        section.section_id = "integration";
        section.estimated_word_count = 600-1000;
        section.content_guidelines = 
            "Describe integration process, testing methodology, challenges faced, "
            "and lessons learned. Include performance benchmarks.";
        
        return section;
    }
    
    PaperSection createUseCaseSection() {
        PaperSection section;
        section.title = "Use Case Study";
        section.section_id = "use-case";
        section.estimated_word_count = 800-1200;
        section.content_guidelines = 
            "Describe real-world application, deployment scenario, implementation details, "
            "and practical considerations. Include user feedback if available.";
        
        return section;
    }
    
    PaperSection createImpactAnalysisSection() {
        PaperSection section;
        section.title = "Impact Analysis";
        section.section_id = "impact";
        section.estimated_word_count = 600-1000;
        section.content_guidelines = 
            "Analyze practical impact, benefits, limitations, and broader implications. "
            "Discuss scalability and transferability.";
        
        return section;
    }
    
    PaperSection createMethodologySection() {
        PaperSection section;
        section.title = "Methodology";
        section.section_id = "methodology";
        section.estimated_word_count = 800-1500;
        section.content_guidelines = 
            "Describe experimental design, data collection procedures, "
            "analysis methods, and validation approach. Be reproducible.";
        
        return section;
    }
    
    PaperSection createResultsSection() {
        PaperSection section;
        section.title = "Results and Discussion";
        section.section_id = "results";
        section.estimated_word_count = 1000-2000;
        section.content_guidelines = 
            "Present results clearly with appropriate figures and tables. "
            "Discuss implications, limitations, and compare with baselines.";
        
        return section;
    }
    
    PaperSection createDiscussionSection() {
        PaperSection section;
        section.title = "Discussion";
        section.section_id = "discussion";
        section.estimated_word_count = 400-800;
        section.content_guidelines = 
            "Interpret results, discuss limitations, address threats to validity, "
            "and suggest future work. Connect back to research questions.";
        
        return section;
    }
    
    PaperSection createConclusionSection() {
        PaperSection section;
        section.title = "Conclusion";
        section.section_id = "conclusion";
        section.estimated_word_count = 400-600;
        section.content_guidelines = 
            "Summarize key contributions, results, and implications. "
            "Avoid introducing new information. Suggest future directions.";
        
        return section;
    }
    
    PaperSection createReferencesSection() {
        PaperSection section;
        section.title = "References";
        section.section_id = "references";
        section.estimated_word_count = 0;  // Counted differently
        section.content_guidelines = 
            "Follow venue-specific citation style. Ensure all cited works are referenced "
            "and all references are cited in the text. Include DOIs when available.";
        
        return section;
    }
    
    std::vector<std::string> getGeneralStyleGuidelines() {
        return {
            "Use active voice when possible",
            "Define all abbreviations at first use",
            "Use consistent terminology throughout",
            "Write in past tense for completed work",
            "Avoid idioms and colloquialisms",
            "Use gender-neutral language",
            "Maintain parallel structure in lists",
            "Keep sentences concise and focused"
        };
    }
    
    std::vector<std::string> getFigureGuidelines() {
        return {
            "Use vector graphics when possible (PDF, EPS, SVG)",
            "Ensure figures are legible at publication size",
            "Include axis labels and units",
            "Use colorblind-friendly palettes",
            "Maintain consistent font sizes across figures",
            "Provide captions that stand alone",
            "Number figures consecutively",
            "Refer to all figures in the text"
        };
    }
    
    std::vector<std::string> getTableGuidelines() {
        return {
            "Use landscape orientation for wide tables",
            "Include units in column headers",
            "Highlight important values appropriately",
            "Use horizontal lines sparingly",
            "Align numbers on decimal points",
            "Provide titles that stand alone",
            "Number tables consecutively",
            "Refer to all tables in the text"
        };
    }
    
    std::vector<std::string> getEquationGuidelines() {
        return {
            "Number all displayed equations",
            "Use consistent notation throughout",
            "Define all variables after equations",
            "Use appropriate mathematical fonts",
            "Center equations on page",
            "Refer to numbered equations in text",
            "Break complex equations across lines appropriately",
            "Use punctuation after equations when grammatically required"
        };
    }
    
    std::vector<std::string> getAlgorithmicGuidelines() {
        return {
            "Provide algorithm pseudocode with clear structure",
            "Include initialization steps explicitly",
            "Use consistent naming conventions",
            "Add comments explaining key steps",
            "Specify input/output requirements",
            "Analyze time and space complexity",
            "Discuss optimality and correctness",
            "Compare with existing algorithms theoretically"
        };
    }
    
    std::vector<std::string> getHardwareGuidelines() {
        return {
            "Include detailed technical specifications",
            "Provide CAD drawings and assembly instructions",
            "Document design decisions and trade-offs",
            "Include bill of materials with sources",
            "Validate performance against requirements",
            "Discuss manufacturing and cost considerations",
            "Address safety and regulatory requirements",
            "Provide maintenance and troubleshooting guides"
        };
    }
    
    std::vector<std::string> getSystemsGuidelines() {
        return {
            "Present complete system architecture diagram",
            "Document component interfaces and protocols",
            "Describe data flow and processing pipelines",
            "Address system integration challenges",
            "Validate system performance holistically",
            "Discuss scalability and maintainability",
            "Address security and privacy considerations",
            "Provide system evaluation with real-world scenarios"
        };
    }
    
    std::vector<std::string> getApplicationGuidelines() {
        return {
            "Describe real-world deployment context",
            "Address practical implementation challenges",
            "Validate effectiveness in target environment",
            "Discuss user experience and adoption",
            "Analyze economic and social impact",
            "Address ethical considerations",
            "Discuss generalizability to other domains",
            "Provide recommendations for deployment"
        };
    }
};
```

### Reproducibility Documentation

#### Experimental Protocol Documentation
```cpp
class ReproducibilityDocumentation {
private:
    std::unique_ptr<ProtocolTemplate> protocol_template_;
    std::unique_ptr<DocumentationGenerator> doc_generator_;
    std::unique_ptr<VerificationSystem> verification_system_;
    std::unique_ptr<ArchiveManager> archive_manager_;

public:
    ReproducibilityDocumentation() {
        protocol_template_ = std::make_unique<StandardizedProtocolTemplate>();
        doc_generator_ = std::make_unique<AutomatedDocumentationGenerator>();
        verification_system_ = std::make_unique<ReproducibilityVerifier>();
        archive_manager_ = std::make_unique<ResearchArchiveManager>();
    }
    
    ExperimentalProtocolDocument generateProtocolDocumentation(
        const ExperimentalDesign& design,
        const RobotConfiguration& config,
        const EnvironmentalSetup& env_setup) {
        
        ExperimentalProtocolDocument document;
        
        // Generate standardized protocol sections
        document.protocol_header = createProtocolHeader(design);
        document.robot_configuration = config;
        document.environmental_setup = env_setup;
        document.procedure_steps = generateProcedureSteps(design);
        document.safety_protocols = generateSafetyProtocols(design);
        document.data_collection_plan = generateDataCollectionPlan(design);
        document.analysis_plan = generateAnalysisPlan(design);
        document.troubleshooting_guide = generateTroubleshootingGuide(design);
        
        // Validate protocol for completeness
        auto validation_result = validateProtocol(document);
        document.validation_status = validation_result;
        
        if (!validation_result.complete) {
            document.notes.push_back("Protocol requires additional information: " + 
                                   joinStrings(validation_result.missing_elements, ", "));
        }
        
        // Generate verification checklist
        document.verification_checklist = generateVerificationChecklist(document);
        
        // Archive protocol
        archive_manager_->archiveProtocol(document, design.experiment_id);
        
        return document;
    }
    
    bool verifyReproducibility(const ExperimentalProtocolDocument& protocol,
                             const ReplicationAttempt& replication) {
        return verification_system_->verifyReproduction(protocol, replication);
    }

private:
    struct ExperimentalProtocolDocument {
        ProtocolHeader protocol_header;
        RobotConfiguration robot_configuration;
        EnvironmentalSetup environmental_setup;
        std::vector<ProcedureStep> procedure_steps;
        std::vector<SafetyProtocol> safety_protocols;
        DataCollectionPlan data_collection_plan;
        AnalysisPlan analysis_plan;
        TroubleshootingGuide troubleshooting_guide;
        ProtocolValidationResult validation_status;
        std::vector<VerificationItem> verification_checklist;
        std::vector<std::string> notes;
        rclcpp::Time creation_time;
        std::string document_version;
    };
    
    struct ProtocolHeader {
        std::string title;
        std::string experiment_id;
        std::string version;
        std::vector<std::string> authors;
        std::string institution;
        rclcpp::Time creation_date;
        std::string abstract;
        std::vector<std::string> keywords;
        std::string license;
    };
    
    struct ProcedureStep {
        int step_number;
        std::string title;
        std::string description;
        std::vector<std::string> prerequisites;
        std::vector<std::string> actions;
        std::vector<std::string> expected_outcomes;
        std::vector<std::string> safety_considerations;
        std::vector<std::string> required_equipment;
        std::vector<Figure> illustrations;
        rclcpp::Duration estimated_duration;
    };
    
    struct SafetyProtocol {
        std::string hazard_type;
        std::string risk_level;
        std::vector<std::string> preventive_measures;
        std::vector<std::string> protective_equipment;
        std::vector<std::string> emergency_procedures;
        std::vector<std::string> personnel_requirements;
    };
    
    struct DataCollectionPlan {
        std::vector<DataChannel> data_channels;
        std::vector<CalibrationProcedure> calibration_procedures;
        std::vector<QualityControlMeasure> quality_control_measures;
        std::string storage_format;
        std::string backup_procedure;
        std::vector<std::string> metadata_requirements;
    };
    
    struct AnalysisPlan {
        std::vector<StatisticalTest> statistical_tests;
        std::vector<VisualizationRequirement> visualization_requirements;
        std::vector<ReportingRequirement> reporting_requirements;
        std::string significance_level;
        std::string confidence_level;
        std::vector<std::string> assumptions;
    };
    
    struct TroubleshootingGuide {
        std::vector<TroubleshootingEntry> entries;
        std::string contact_information;
        std::string support_resources;
    };
    
    struct TroubleshootingEntry {
        std::string symptom;
        std::string possible_causes;
        std::vector<std::string> diagnostic_steps;
        std::vector<std::string> solutions;
        std::string difficulty_level;
    };
    
    struct ProtocolValidationResult {
        bool complete;
        std::vector<std::string> missing_elements;
        std::vector<std::string> inconsistencies;
        std::vector<std::string> recommendations;
        rclcpp::Time validation_time;
    };
    
    struct VerificationItem {
        std::string item_description;
        std::string verification_method;
        std::string expected_result;
        bool verified;
        std::string notes;
    };
    
    struct DataChannel {
        std::string name;
        std::string type;
        std::string source;
        std::string format;
        std::string frequency;
        std::string units;
        std::string accuracy;
        std::string notes;
    };
    
    struct CalibrationProcedure {
        std::string equipment_name;
        std::string procedure_description;
        std::vector<std::string> required_tools;
        std::vector<std::string> steps;
        std::string acceptance_criteria;
        std::string frequency;
        std::string responsible_person;
    };
    
    struct QualityControlMeasure {
        std::string measure_type;
        std::string description;
        std::string frequency;
        std::string responsible_person;
        std::string acceptance_criteria;
    };
    
    struct StatisticalTest {
        std::string test_name;
        std::string purpose;
        std::string assumptions;
        std::string formula;
        std::string interpretation;
        std::string effect_size_measure;
    };
    
    struct VisualizationRequirement {
        std::string chart_type;
        std::string purpose;
        std::string required_data;
        std::string style_requirements;
        std::string accessibility_requirements;
    };
    
    struct ReportingRequirement {
        std::string report_type;
        std::string required_content;
        std::string format;
        std::string audience;
        std::string frequency;
    };
    
    struct ReplicationAttempt {
        std::string replicator_id;
        std::string environment;
        std::vector<std::string> deviations_from_protocol;
        std::vector<std::string> challenges_encountered;
        std::vector<std::string> solutions_applied;
        rclcpp::Time start_time;
        rclcpp::Time end_time;
        std::string overall_assessment;
        std::vector<std::string> recommendations_for_protocol_improvement;
    };
    
    ProtocolHeader createProtocolHeader(const ExperimentalDesign& design) {
        ProtocolHeader header;
        header.title = design.objective + ": Experimental Protocol";
        header.experiment_id = design.experiment_id;
        header.version = "1.0";
        header.authors = design.researchers;
        header.institution = design.institution;
        header.creation_date = this->now();
        header.abstract = design.abstract;
        header.keywords = design.keywords;
        header.license = "CC BY 4.0";  // Standard for reproducible research
        
        return header;
    }
    
    std::vector<ProcedureStep> generateProcedureSteps(const ExperimentalDesign& design) {
        std::vector<ProcedureStep> steps;
        
        // Step 1: Setup
        ProcedureStep setup_step;
        setup_step.step_number = 1;
        setup_step.title = "Experimental Setup";
        setup_step.description = "Prepare the experimental environment and equipment";
        setup_step.prerequisites = {"Robot calibrated", "Environment prepared", "Software loaded"};
        setup_step.actions = {
            "Verify robot calibration parameters",
            "Check environmental conditions",
            "Load experimental software",
            "Initialize data collection systems"
        };
        setup_step.expected_outcomes = {"All systems operational", "Calibration verified"};
        setup_step.safety_considerations = {"Ensure safety zones established", "Emergency stop accessible"};
        setup_step.required_equipment = {"Robot platform", "Calibration tools", "Computer"};
        setup_step.estimated_duration = rclcpp::Duration::from_seconds(30 * 60);  // 30 minutes
        
        steps.push_back(setup_step);
        
        // Step 2: Calibration
        ProcedureStep calibration_step;
        calibration_step.step_number = 2;
        calibration_step.title = "System Calibration";
        calibration_step.description = "Calibrate all sensors and systems";
        calibration_step.prerequisites = {"Setup complete"};
        calibration_step.actions = {
            "Run robot self-calibration routine",
            "Calibrate external sensors",
            "Verify sensor fusion algorithms",
            "Test safety systems"
        };
        calibration_step.expected_outcomes = {"All calibrations pass", "Accuracy verified"};
        calibration_step.safety_considerations = {"Keep area clear during calibration"};
        calibration_step.required_equipment = {"Calibration targets", "Measuring tools"};
        calibration_step.estimated_duration = rclcpp::Duration::from_seconds(45 * 60);  // 45 minutes
        
        steps.push_back(calibration_step);
        
        // Step 3: Baseline Measurement
        ProcedureStep baseline_step;
        baseline_step.step_number = 3;
        baseline_step.title = "Baseline Measurements";
        baseline_step.description = "Collect baseline performance data";
        baseline_step.prerequisites = {"Calibration complete"};
        baseline_step.actions = {
            "Execute baseline tasks",
            "Record performance metrics",
            "Verify system behavior",
            "Adjust parameters if needed"
        };
        baseline_step.expected_outcomes = {"Baseline performance established", "Systems behaving as expected"};
        baseline_step.safety_considerations = {"Monitor for unexpected behavior"};
        baseline_step.required_equipment = {"Data collection software"};
        baseline_step.estimated_duration = rclcpp::Duration::from_seconds(20 * 60);  // 20 minutes
        
        steps.push_back(baseline_step);
        
        // Step 4: Main Experiment
        ProcedureStep experiment_step;
        experiment_step.step_number = 4;
        experiment_step.title = "Main Experiment";
        experiment_step.description = "Execute the main experimental protocol";
        experiment_step.prerequisites = {"Baselines established"};
        experiment_step.actions = design.experimental_procedures;  // Use design-specific procedures
        experiment_step.expected_outcomes = {"Experimental data collected", "All conditions tested"};
        experiment_step.safety_considerations = design.safety_considerations;
        experiment_step.required_equipment = design.required_equipment;
        experiment_step.estimated_duration = design.estimated_duration;
        
        steps.push_back(experiment_step);
        
        // Step 5: Cleanup
        ProcedureStep cleanup_step;
        cleanup_step.step_number = 5;
        cleanup_step.title = "Experiment Cleanup";
        cleanup_step.description = "Safely conclude experiment and secure equipment";
        cleanup_step.prerequisites = {"Main experiment complete"};
        cleanup_step.actions = {
            "Stop all robot motion",
            "Save all data",
            "Clean equipment",
            "Restore environment to original state"
        };
        cleanup_step.expected_outcomes = {"Equipment secured", "Data safely stored"};
        cleanup_step.safety_considerations = {"Ensure robot is stationary", "Disconnect power safely"};
        cleanup_step.required_equipment = {};
        cleanup_step.estimated_duration = rclcpp::Duration::from_seconds(15 * 60);  // 15 minutes
        
        steps.push_back(cleanup_step);
        
        return steps;
    }
    
    std::vector<SafetyProtocol> generateSafetyProtocols(const ExperimentalDesign& design) {
        std::vector<SafetyProtocol> protocols;
        
        // General safety protocol
        SafetyProtocol general_protocol;
        general_protocol.hazard_type = "General";
        general_protocol.risk_level = "Medium";
        general_protocol.preventive_measures = {
            "Establish safety zones around robot workspace",
            "Ensure emergency stop buttons are accessible",
            "Brief all personnel on safety procedures",
            "Conduct pre-experiment safety check"
        };
        general_protocol.protective_equipment = {"Safety glasses", "Closed-toe shoes"};
        general_protocol.emergency_procedures = {
            "Press emergency stop button immediately",
            "Evacuate the area if necessary",
            "Contact emergency services if needed"
        };
        general_protocol.personnel_requirements = {"Trained supervisor must be present"};
        
        protocols.push_back(general_protocol);
        
        // Equipment-specific safety protocol
        SafetyProtocol equipment_protocol;
        equipment_protocol.hazard_type = "Equipment Failure";
        equipment_protocol.risk_level = "High";
        equipment_protocol.preventive_measures = {
            "Inspect equipment before use",
            "Verify all connections secure",
            "Test safety systems before experiment",
            "Have backup equipment available"
        };
        equipment_protocol.protective_equipment = {"Inspection gloves", "Tool kit"};
        equipment_protocol.emergency_procedures = {
            "Disconnect power immediately",
            "Isolate faulty equipment",
            "Notify supervisor",
            "Document incident"
        };
        equipment_protocol.personnel_requirements = {"Certified technician required for repairs"};
        
        protocols.push_back(equipment_protocol);
        
        // Environment-specific safety protocol
        SafetyProtocol environment_protocol;
        environment_protocol.hazard_type = "Environmental";
        environment_protocol.risk_level = "Low to Medium";
        environment_protocol.preventive_measures = {
            "Check environmental conditions",
            "Ensure adequate lighting",
            "Verify floor stability and cleanliness",
            "Remove trip hazards"
        };
        environment_protocol.protective_equipment = {"Non-slip footwear"};
        environment_protocol.emergency_procedures = {
            "Address environmental hazard immediately",
            "Suspend experiment if unsafe",
            "Clean up spills promptly"
        };
        environment_protocol.personnel_requirements = {"All personnel must be aware of exits"};
        
        protocols.push_back(environment_protocol);
        
        return protocols;
    }
    
    DataCollectionPlan generateDataCollectionPlan(const ExperimentalDesign& design) {
        DataCollectionPlan plan;
        
        // Define data channels based on experiment design
        for (const auto& channel_desc : design.data_requirements) {
            DataChannel channel;
            channel.name = channel_desc.name;
            channel.type = channel_desc.type;
            channel.source = channel_desc.source;
            channel.format = channel_desc.format;
            channel.frequency = channel_desc.frequency;
            channel.units = channel_desc.units;
            channel.accuracy = channel_desc.accuracy;
            channel.notes = channel_desc.notes;
            
            plan.data_channels.push_back(channel);
        }
        
        // Add calibration procedures
        CalibrationProcedure robot_calibration;
        robot_calibration.equipment_name = "Robot Platform";
        robot_calibration.procedure_description = "Standard robot self-calibration";
        robot_calibration.steps = {
            "Power on robot",
            "Execute calibration routine",
            "Verify joint zero positions",
            "Test range of motion"
        };
        robot_calibration.acceptance_criteria = "All joints within 0.1mm tolerance";
        robot_calibration.frequency = "Before each experiment";
        robot_calibration.responsible_person = "Experimenter";
        
        plan.calibration_procedures.push_back(robot_calibration);
        
        // Add quality control measures
        QualityControlMeasure data_verification;
        data_verification.measure_type = "Data Integrity";
        data_verification.description = "Verify data completeness and consistency";
        data_verification.frequency = "Continuous during collection";
        data_verification.responsible_person = "Data Collection System";
        data_verification.acceptance_criteria = "No missing packets, consistent timestamps";
        
        plan.quality_control_measures.push_back(data_verification);
        
        // Set storage format
        plan.storage_format = "Standardized HDF5 format with metadata";
        plan.backup_procedure = "Automatic backup to cloud storage";
        plan.metadata_requirements = {
            "Timestamp for each data point",
            "Calibration parameters",
            "Environmental conditions",
            "Software versions",
            "Operator ID"
        };
        
        return plan;
    }
    
    AnalysisPlan generateAnalysisPlan(const ExperimentalDesign& design) {
        AnalysisPlan plan;
        
        // Add statistical tests based on hypothesis
        StatisticalTest comparison_test;
        comparison_test.test_name = "Two-sample t-test";
        comparison_test.purpose = "Compare performance between conditions";
        comparison_test.assumptions = "Normal distribution, equal variances";
        comparison_test.formula = "t = (mean1 - mean2) / sqrt(var1/n1 + var2/n2)";
        comparison_test.interpretation = "p < 0.05 indicates significant difference";
        comparison_test.effect_size_measure = "Cohen's d";
        
        plan.statistical_tests.push_back(comparison_test);
        
        // Add visualization requirements
        VisualizationRequirement performance_plot;
        performance_plot.chart_type = "Bar chart with error bars";
        performance_plot.purpose = "Show mean performance with confidence intervals";
        performance_plot.required_data = "Performance metrics by condition";
        performance_plot.style_requirements = "Colorblind-friendly palette, clear labels";
        performance_plot.accessibility_requirements = "Alternative text for screen readers";
        
        plan.visualization_requirements.push_back(performance_plot);
        
        // Add reporting requirements
        ReportingRequirement summary_report;
        summary_report.report_type = "Executive Summary";
        summary_report.required_content = {
            "Key findings",
            "Statistical significance",
            "Effect sizes",
            "Practical implications"
        };
        summary_report.format = "LaTeX document";
        summary_report.audience = "Research team and collaborators";
        summary_report.frequency = "Upon completion";
        
        plan.reporting_requirements.push_back(summary_report);
        
        // Set analysis parameters
        plan.significance_level = "0.05";
        plan.confidence_level = "0.95";
        plan.assumptions = {
            "Data points are independent",
            "Samples are randomly selected",
            "Equal variances between groups"
        };
        
        return plan;
    }
    
    TroubleshootingGuide generateTroubleshootingGuide(const ExperimentalDesign& design) {
        TroubleshootingGuide guide;
        
        // Add common robot issues
        TroubleshootingEntry calibration_failure;
        calibration_failure.symptom = "Robot fails to calibrate";
        calibration_failure.possible_causes = "Loose connections, power issues, software bugs";
        calibration_failure.diagnostic_steps = {
            "Check all cable connections",
            "Verify power supply voltage",
            "Restart robot controller",
            "Update software if outdated"
        };
        calibration_failure.solutions = {
            "Secure loose connections",
            "Replace faulty cables",
            "Reinstall software",
            "Contact manufacturer support"
        };
        calibration_failure.difficulty_level = "Medium";
        
        guide.entries.push_back(calibration_failure);
        
        // Add sensor issues
        TroubleshootingEntry sensor_noise;
        sensor_noise.symptom = "High sensor noise or inaccurate readings";
        sensor_noise.possible_causes = "Electromagnetic interference, calibration drift, hardware failure";
        sensor_noise.diagnostic_steps = {
            "Check for nearby electromagnetic sources",
            "Re-calibrate sensor",
            "Test with known reference",
            "Inspect sensor physically"
        };
        sensor_noise.solutions = {
            "Move sources of interference",
            "Perform recalibration",
            "Replace sensor if damaged",
            "Update sensor firmware"
        };
        sensor_noise.difficulty_level = "Medium";
        
        guide.entries.push_back(sensor_noise);
        
        // Add software issues
        TroubleshootingEntry software_crash;
        software_crash.symptom = "Control software crashes during experiment";
        software_crash.possible_causes = "Memory leaks, race conditions, hardware errors";
        software_crash.diagnostic_steps = {
            "Check error logs",
            "Monitor system resources",
            "Verify hardware connections",
            "Test with simplified code"
        };
        software_crash.solutions = {
            "Apply software patches",
            "Optimize memory usage",
            "Fix race conditions",
            "Replace faulty hardware"
        };
        software_crash.difficulty_level = "High";
        
        guide.entries.push_back(software_crash);
        
        // Add environmental issues
        TroubleshootingEntry environment_change;
        environment_change.symptom = "Unexpected environmental changes affect experiment";
        environment_change.possible_causes = "Lighting changes, temperature fluctuations, people movement";
        environment_change.diagnostic_steps = {
            "Monitor environmental sensors",
            "Document changes",
            "Assess impact on results",
            "Consider pausing experiment"
        };
        environment_change.solutions = {
            "Control lighting conditions",
            "Maintain temperature stability",
            "Limit access during experiment",
            "Account for changes in analysis"
        };
        environment_change.difficulty_level = "Low";
        
        guide.entries.push_back(environment_change);
        
        // Add contact information
        guide.contact_information = "Primary: Dr. Jane Smith (jane.smith@university.edu, +1-555-0123)\n"
                                  "Technical Support: robotics-lab@university.edu\n"
                                  "Emergency: Campus Security (555-0000)";
        
        // Add support resources
        guide.support_resources = {
            "Robot User Manual: http://robot.university.edu/manual",
            "Software Documentation: http://software.university.edu/docs",
            "Video Tutorials: http://videos.university.edu/robotics",
            "FAQ: http://faq.university.edu/robotics"
        };
        
        return guide;
    }
    
    ProtocolValidationResult validateProtocol(const ExperimentalProtocolDocument& document) {
        ProtocolValidationResult result;
        result.complete = true;
        result.validation_time = this->now();
        
        // Check for completeness
        if (document.protocol_header.title.empty()) {
            result.missing_elements.push_back("Protocol title");
            result.complete = false;
        }
        
        if (document.robot_configuration.platform.empty()) {
            result.missing_elements.push_back("Robot configuration");
            result.complete = false;
        }
        
        if (document.environmental_setup.description.empty()) {
            result.missing_elements.push_back("Environmental setup");
            result.complete = false;
        }
        
        if (document.procedure_steps.empty()) {
            result.missing_elements.push_back("Procedure steps");
            result.complete = false;
        }
        
        if (document.safety_protocols.empty()) {
            result.missing_elements.push_back("Safety protocols");
            result.complete = false;
        }
        
        if (document.data_collection_plan.data_channels.empty()) {
            result.missing_elements.push_back("Data collection plan");
            result.complete = false;
        }
        
        if (document.analysis_plan.statistical_tests.empty()) {
            result.missing_elements.push_back("Analysis plan");
            result.complete = false;
        }
        
        if (document.troubleshooting_guide.entries.empty()) {
            result.missing_elements.push_back("Troubleshooting guide");
            result.complete = false;
        }
        
        // Check for consistency issues
        std::set<std::string> required_equipment;
        for (const auto& step : document.procedure_steps) {
            required_equipment.insert(step.required_equipment.begin(), 
                                   step.required_equipment.end());
        }
        
        std::set<std::string> available_equipment;
        for (const auto& channel : document.data_collection_plan.data_channels) {
            available_equipment.insert(channel.source);
        }
        
        // Check if required equipment is available
        for (const auto& req_eq : required_equipment) {
            if (req_eq != "" && available_equipment.find(req_eq) == available_equipment.end()) {
                result.inconsistencies.push_back("Required equipment '" + req_eq + 
                                               "' not available in data collection plan");
            }
        }
        
        // Generate recommendations
        if (result.missing_elements.size() > 3) {
            result.recommendations.push_back("Consider using protocol template to ensure completeness");
        }
        
        if (!result.inconsistencies.empty()) {
            result.recommendations.push_back("Resolve equipment availability inconsistencies");
        }
        
        return result;
    }
    
    std::vector<VerificationItem> generateVerificationChecklist(
        const ExperimentalProtocolDocument& document) {
        
        std::vector<VerificationItem> checklist;
        
        // Equipment verification
        VerificationItem equipment_check;
        equipment_check.item_description = "All required equipment is available and functional";
        equipment_check.verification_method = "Physical inspection and testing";
        equipment_check.expected_result = "All items on equipment list are present and working";
        equipment_check.verified = false;
        
        checklist.push_back(equipment_check);
        
        // Calibration verification
        VerificationItem calibration_check;
        calibration_check.item_description = "All systems are properly calibrated";
        calibration_check.verification_method = "Run calibration routines and verify parameters";
        calibration_check.expected_result = "Calibration values within specified tolerances";
        calibration_check.verified = false;
        
        checklist.push_back(calibration_check);
        
        // Safety verification
        VerificationItem safety_check;
        safety_check.item_description = "Safety systems are operational and safety zones established";
        safety_check.verification_method = "Test emergency stops and verify safety boundaries";
        safety_check.expected_result = "All safety systems responsive and boundaries clear";
        safety_check.verified = false;
        
        checklist.push_back(safety_check);
        
        // Software verification
        VerificationItem software_check;
        software_check.item_description = "All software is correctly loaded and configured";
        software_check.verification_method = "Execute test programs and verify functionality";
        software_check.expected_result = "Software runs without errors and produces expected outputs";
        software_check.verified = false;
        
        checklist.push_back(software_check);
        
        // Environmental verification
        VerificationItem environment_check;
        environment_check.item_description = "Experimental environment meets specified conditions";
        environment_check.verification_method = "Measure environmental parameters";
        environment_check.expected_result = "Temperature, lighting, and other parameters within ranges";
        environment_check.verified = false;
        
        checklist.push_back(environment_check);
        
        // Data collection verification
        VerificationItem data_check;
        data_check.item_description = "Data collection systems are properly configured";
        data_check.verification_method = "Collect test data and verify storage";
        data_check.expected_result = "Data is being collected and stored correctly";
        data_check.verified = false;
        
        checklist.push_back(data_check);
        
        return checklist;
    }
    
    std::string joinStrings(const std::vector<std::string>& strings, 
                           const std::string& delimiter) {
        if (strings.empty()) return "";
        
        std::string result = strings[0];
        for (size_t i = 1; i < strings.size(); i++) {
            result += delimiter + strings[i];
        }
        
        return result;
    }
    
    struct ExperimentalDesign {
        std::string experiment_id;
        std::string objective;
        std::vector<std::string> researchers;
        std::string institution;
        std::string abstract;
        std::vector<std::string> keywords;
        std::vector<std::string> experimental_procedures;
        std::vector<SafetyConsideration> safety_considerations;
        std::vector<EquipmentRequirement> required_equipment;
        std::vector<DataRequirement> data_requirements;
        rclcpp::Duration estimated_duration;
        rclcpp::Time deadline;
    };
    
    struct SafetyConsideration {
        std::string hazard_type;
        std::string risk_level;
        std::vector<std::string> mitigation_strategies;
    };
    
    struct EquipmentRequirement {
        std::string name;
        std::string model;
        std::string quantity;
        std::string specifications;
    };
    
    struct DataRequirement {
        std::string name;
        std::string type;
        std::string source;
        std::string format;
        std::string frequency;
        std::string units;
        std::string accuracy;
        std::string notes;
    };
    
    struct RobotConfiguration {
        std::string platform;
        std::string controller_version;
        std::vector<SensorConfiguration> sensors;
        std::vector<ActuatorConfiguration> actuators;
        std::vector<SoftwareConfiguration> software_stack;
        std::string calibration_date;
        std::string operational_limits;
    };
    
    struct EnvironmentalSetup {
        std::string description;
        std::vector<EnvironmentalParameter> parameters;
        std::string layout_diagram;
        std::string safety_zone_definition;
        std::string lighting_requirements;
        std::string acoustic_requirements;
    };
    
    struct SensorConfiguration {
        std::string type;
        std::string model;
        std::string accuracy;
        std::string mounting_position;
        std::string calibration_parameters;
    };
    
    struct ActuatorConfiguration {
        std::string type;
        std::string model;
        std::string specifications;
        std::string control_parameters;
    };
    
    struct SoftwareConfiguration {
        std::string name;
        std::string version;
        std::string dependencies;
        std::string configuration_parameters;
    };
    
    struct EnvironmentalParameter {
        std::string name;
        std::string unit;
        std::string target_value;
        std::string tolerance;
        std::string measurement_method;
    };
};
```

## Troubleshooting Common Research Issues

### Experimental Design Problems

#### Confounding Variables
- **Symptoms**: Unexpected correlations, spurious results, inability to draw clear conclusions
- **Causes**: Poor experimental design, uncontrolled variables, selection bias
- **Solutions**: Randomization, blocking, statistical control, proper controls
- **Prevention**: Careful variable identification, pilot studies, literature review

#### Sample Size Issues
- **Symptoms**: Underpowered studies, non-significant results, wide confidence intervals
- **Causes**: Insufficient power analysis, budget constraints, time limitations
- **Solutions**: Power analysis, effect size estimation, meta-analysis
- **Planning**: A-priori power calculations, feasibility assessments

#### Replication Challenges
- **Symptoms**: Inconsistent results across replications, irreproducible findings
- **Causes**: Hardware variations, environmental changes, protocol ambiguities
- **Solutions**: Standardized protocols, detailed documentation, robust designs
- **Mitigation**: Multi-site studies, hardware standardization, environmental controls

### Data Collection Issues

#### Sensor Drift and Calibration
- **Symptoms**: Gradual changes in measurements, decreased accuracy over time
- **Causes**: Component aging, environmental effects, mechanical wear
- **Solutions**: Regular calibration, drift detection algorithms, redundancy
- **Monitoring**: Continuous monitoring, automatic alerts, scheduled maintenance

#### Missing Data
- **Symptoms**: Gaps in time series, incomplete records, reduced statistical power
- **Causes**: Sensor failures, communication issues, system crashes
- **Solutions**: Imputation methods, robust statistical techniques, backup systems
- **Prevention**: Redundant sensors, robust communication, proper error handling

### Analysis Problems

#### Statistical Assumption Violations
- **Symptoms**: Invalid p-values, incorrect confidence intervals, misleading conclusions
- **Causes**: Non-normal data, heteroscedasticity, dependence, outliers
- **Solutions**: Non-parametric tests, data transformation, robust methods
- **Diagnosis**: Diagnostic plots, assumption testing, sensitivity analysis

#### Multiple Comparisons
- **Symptoms**: Inflated Type I error rate, spurious significant findings
- **Causes**: Many statistical tests, data dredging, p-hacking
- **Solutions**: Correction methods (Bonferroni, FDR), pre-registration, selective testing
- **Prevention**: Planned comparisons, family-wise error rate control

## Best Practices

### Research Ethics

#### Human Subjects Research
- **Informed Consent**: Ensure participants understand the research
- **Privacy Protection**: Safeguard participant data and identities
- **Risk Assessment**: Evaluate and minimize potential harms
- **IRB Approval**: Obtain institutional review board approval when required

#### Animal Research
- **Justification**: Demonstrate necessity and scientific value
- **Minimization**: Minimize animal use and suffering
- **Alternatives**: Consider non-animal alternatives
- **IACUC Approval**: Obtain animal care and use committee approval

### Open Science Practices

#### Data Sharing
- **Public Repositories**: Share data in appropriate repositories
- **Metadata Standards**: Provide comprehensive metadata
- **Access Control**: Balance openness with privacy
- **Citation**: Ensure data is citable and credited

#### Code Sharing
- **Version Control**: Use Git or similar systems
- **Documentation**: Provide clear documentation
- **Licensing**: Choose appropriate open source licenses
- **Testing**: Include tests and examples

### Quality Assurance

#### Peer Review Preparation
- **Replicability**: Ensure others can reproduce results
- **Clarity**: Write clearly and completely
- **Transparency**: Disclose limitations and conflicts
- **Standards**: Follow journal/conference guidelines

#### Continuous Validation
- **Pilot Studies**: Test methods on small scale first
- **Negative Controls**: Include appropriate controls
- **Blinding**: Where appropriate, blind evaluators
- **Monitoring**: Continuous monitoring during data collection

## Future Developments

### Automated Research Systems

#### AI-Assisted Research Design
- **Hypothesis Generation**: AI for generating research hypotheses
- **Protocol Optimization**: AI for optimizing experimental protocols
- **Resource Allocation**: AI for efficient resource use
- **Predictive Analytics**: AI for predicting experiment outcomes

#### Autonomous Experimentation
- **Robotic Scientists**: Fully autonomous experimental systems
- **Adaptive Design**: Systems that adapt protocols in real-time
- **Self-Improvement**: Systems that learn from results
- **Multi-objective Optimization**: Balancing multiple research goals

### Advanced Analysis Techniques

#### Causal Inference
- **Causal Models**: Understanding cause-effect relationships
- **Counterfactual Reasoning**: Understanding what-if scenarios
- **Structural Equation Models**: Complex relationship modeling
- **Instrumental Variables**: Addressing confounding in observational data

#### Machine Learning Integration
- **Automated Feature Discovery**: ML for finding relevant variables
- **Anomaly Detection**: ML for identifying unusual patterns
- **Predictive Modeling**: ML for forecasting outcomes
- **Uncertainty Quantification**: ML with uncertainty estimates

## Conclusion

Research methodologies and experimental design are fundamental to advancing the field of Physical AI and robotics. The complexity of integrating computational algorithms with physical systems requires rigorous scientific approaches that account for the unique challenges of real-world experimentation. Proper methodology ensures that findings are reliable, generalizable, and contribute meaningfully to the scientific understanding of Physical AI systems.

The principles outlined in this chapter provide a framework for conducting high-quality research in robotics, from hypothesis formation to result interpretation. As the field continues to evolve, these methodological foundations will remain essential for producing research that stands up to scientific scrutiny and drives genuine progress in Physical AI.

The integration of traditional scientific methods with modern computational tools and AI techniques offers exciting opportunities for advancing research quality and efficiency. However, the fundamental principles of scientific rigor, reproducibility, and ethical conduct must remain paramount in all research endeavors.

## Exercises

1. Design and implement a controlled experiment to compare two robotic manipulation algorithms, including proper randomization, controls, and statistical analysis.
2. Create a comprehensive experimental protocol document for a robotics study, including safety considerations and reproducibility measures.
3. Analyze a published robotics paper to identify potential methodological issues and suggest improvements.

## Further Reading

- Judd, C. M., & McClelland, G. H. (2001). "Data Analysis: A Model Comparison Approach." Harcourt Brace.
- Shadish, W. R., Cook, T. D., & Campbell, D. T. (2002). "Experimental and Quasi-Experimental Designs for Generalized Causal Inference." Houghton Mifflin.
- Etz, A., Haaf, J. M., Rouder, J. N., & Vandekerckhove, J. (2018). "Why Batch Effects Matter in Omics Data, and How to Avoid Them." Biochemical Society Transactions.
- Peng, R. D. (2011). "Reproducible Research in Computational Science." Science.
- National Academies of Sciences, Engineering, and Medicine. (2019). "Reproducibility and Replicability in Science." The National Academies Press.