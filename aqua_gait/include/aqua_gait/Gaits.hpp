#ifndef GAITS_HPP_
#define GAITS_HPP_


#include <cmath>
#include <cassert>
#include <chrono>
#include <functional>


struct MotorTarget_t {
  double pos;
  double vel;
  double acc;

  MotorTarget_t() : pos(0), vel(0), acc(0) {};
};


//
/**
 * LEG INDICES:
 * 0: front-left
 * 1: mid-left
 * 2: hind-left
 * 3: front-right
 * 4: mid-right
 * 5: hind-right
 *
 * LEG ORIENTATIONS:
 * 0: pointing backwards
 * pi: pointing downwards
 * 2*pi: pointing forwards
 *
 * leg_angle = amplitude * cos(2*pi * frequency * (tsin-FStsinstart) + phase_offfset) + leg_offset
 */
struct PeriodicLegState_t {
  float amplitudes[6]; // unit: radian; saturated within [0, pi/2] bounds
  float frequencies[6]; // unit: 1/sec; saturated within [0, 4] bounds
  float phase_offsets[6]; // unit: radian
  float leg_offsets[6]; // unit: radian
  float leg_velocities[6]; //unit: radian/sec; saturated within [-4*pi, 4*pi] bounds

  PeriodicLegState_t() {
    for (int i = 0; i < 6; i++) {
      amplitudes[i] = 0;
      frequencies[i] = 0;
      phase_offsets[i] = 0;
      leg_offsets[i] = 0;
      leg_velocities[i] = 0;
    }
  };
};


/**
 * HOW TO USE UnderwaterSwimmerGait?
 *
 * - create object of type UnderwaterSwimmerGait
 *   - pass in configuration parameters
 *
 * - call obj.activate()
 *   - pass in current leg positions, in radians
 *
 * - whenever a command is issued, call rollControl(), heaveControl(), ...
 *
 * - inside a thread, repeatedly call updateSineCmd(PeriodicLegState_t& legCmd),
 *   setPeriodicLegCmd(const PeriodicLegState_t& legCmd), and
 *   updateMotorTarget(MotorTarget_t (&target)[6])
 *   - resulting per-leg motor commands are computed and stored in 'target' buffer, which is passed as input argument
 *
 *
 *
 * WHY DOES THIS CLASS NOT SUPPORT start_angle / pauseSR() / unpauseSR() / timeUnpause smooth initial leg transition?
 * - in short, we assume that the RoboDevel variant that this code talks to,
 *   always filters incoming its generated motor targets, to ensure safe behaviors
 *
 * - assume start_angle[i] == 0 upon activate(), and expect that RD's filter will catch up
 *   to desired sine pos in ROS, within the first (few) sine periods
 *
 * - whenever the real robot receives a pause, the sine generator will still run, but
 *   its outputs will be ignored in RD
 *
 * - upon unpausing, the situation is similar to upon activate(), and expect that RD's filter
 *   will catch up to desired sine pos in ROS, within the first (few) sine waves
 */
class UnderwaterSwimmerGait {
public:
  constexpr static double two_pi = 2*M_PI;


  constexpr static bool FS_ZERO_CROSSING = false; // If true, only update amplitude, frequency, and leg offset at zero crossing
  // If true, when a frequency change occurs, tsinstart will be recomputed based on the phase of the previous command.
  // WARNING: this will give a discrepancy from ideal case only when frequency changes
  constexpr static bool FS_ZERO_CROSSING_UPDATE_TSINSTART_TO_MATCH_PHASE = false;

  constexpr static bool FS_PHASE_OFFSET_USE_STATIC = false;

  constexpr static double FS_MAX_FREQUENCY_HZ = 4.0;

  // NOTE: empirically, adding a acceleration cap, whether via saturation or
  // sigmoid squashing, adds a temporal lag when switching frequencies. Thus,
  // suggest first we try no acceleration limiting, and only if that still
  // causes the robot to surge, then we apply saturation (which seems to give
  // less oscillatory response in velocity)
  constexpr static bool FS_ALL_MOTORS_ACCELERATION_SQUASH_MAX = false; // if false, saturate
  constexpr static double FS_ALL_MOTORS_ACCELERATION_SQUASH_GAIN = 2.0; // this gain is chosen so that the squashing function behaves near-linearly within [-maxThresh, maxThresh] range
  constexpr static double FS_ALL_MOTORS_ACCELERATION_MAX = -1; // disable saturation and squashing
  //constexpr static double FS_ALL_MOTORS_ACCELERATION_MAX = 6.0 * (20.0/180.0*M_PI)*(two_pi*2.5)*(two_pi*2.5)*2.0; // 20', 2.5Hz, hover-midoff case
  //constexpr static double FS_ALL_MOTORS_ACCELERATION_MAX = 6.0 * (20.0/180.0*M_PI)*(two_pi*4.0)*(two_pi*4.0)*1.0; // 20', 4Hz, slightly conservative worst case
  //constexpr static double FS_ALL_MOTORS_ACCELERATION_MAX = 6.0 * (35.0/180.0*M_PI)*(two_pi*4.0)*(two_pi*4.0)*1.0; // 35', 4Hz, worst case

  constexpr static bool FS_ALL_MOTORS_VELOCITY_SQUASH_MAX = true; // if false, saturate
  constexpr static double FS_ALL_MOTORS_VELOCITY_SQUASH_GAIN = 2.0; // this gain is chosen so that the squashing function behaves near-linearly within [-maxThresh, maxThresh] range
  //constexpr static double FS_ALL_MOTORS_VELOCITY_MAX = -1; // disable saturation and squashing
  constexpr static double FS_ALL_MOTORS_VELOCITY_MAX = 6.0 * (20.0/180.0*M_PI)*(two_pi*2.5)*2.0; // 20', 2.5Hz, hover-midoff case
  //constexpr static double FS_ALL_MOTORS_VELOCITY_MAX = 6.0 * (20.0/180.0*M_PI)*(two_pi*4.0)*1.0; // 20', 4Hz, slighty conservative worst case
  //constexpr static double FS_ALL_MOTORS_VELOCITY_MAX = 6.0 * (35.0/180.0*M_PI)*(two_pi*4.0)*1.0; // 35', 4Hz, worst case


  // Analogous to b - a, but handles angular wrap-arounds
  inline static double FSAngularMag(double bRad, double aRad) {
    double dRad = bRad - aRad + M_PI;
    return (dRad > 0) ? dRad - floor(dRad/two_pi)*two_pi - M_PI : dRad - (floor(dRad/two_pi) + 1)*two_pi + M_PI;
  };


  inline static float FSPiAdjust(float rad) {
    float wrap = (rad - std::floor(rad/two_pi)*two_pi);
    return (wrap > M_PI) ? wrap - two_pi : wrap;
  };


  inline static double FSSigmoidSquashGain(double val, double thresh, double gain) {
    double downscaleGain = 0.0;
    if (fabs(val) > 2e-10) { // Prevent numerical instabilities
      downscaleGain = thresh/val*(2.0/(1.0 + exp(-gain/thresh*val)) - 1.0);
    }
    return downscaleGain;
  }


  inline static double saturate(const double in, const double low, const double high) {
    if (in < low) return low;
    else if (in > high) return high;
    return in;
  };


  // NOTE: RoboDevel's MMReadTime() actually starts counting @ program start, whereas this implementation starts counting at epoch
  inline static double MMReadTime() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1.0e9;
  };


  /**
   * timeFn should be a function that ONLY returns non-negative and
   * monotonically increasing values. The behavior is not defined if
   * these assumptions are not met.
   *
   * NOTE: all default values were obtained by reading from within RoboDevel's
   * update() while Ramius was swimming in HOVER_MIDOFF mode
   */
  UnderwaterSwimmerGait(
      std::function<double(void)> timeFn = std::function<double(void)>(nullptr),
      float defaultMaxAmplitudeRad = (20.0/180.0*M_PI),
      float defaultFrequency = 2.5,
      float defaultHoverParkAngleRad = M_PI/4,
      float defaultHoverIIRTimeConstant = 0.03, // Number of seconds needed for low pass filter to reduce original error down to ~36% (i.e. exp(-1)*100 %)
      float defaultLinerizationFactor = 0.0
  ) :
    time((timeFn) ? timeFn : UnderwaterSwimmerGait::MMReadTime),
    legsEnabled(false),
    foreAngle(0),
    aftAngle(M_PI),
    linearizationFactor(defaultLinerizationFactor),
    speedCmd(0.0),
    hoverPitchCommand(0),
    hoverRollCommand(0),
    hoverYawCommand(0),
    hoverHeaveCommand(0),
    swimBackwards(false),
    maxAmplitudeRad(defaultMaxAmplitudeRad),
    frequencyHz(defaultFrequency),
    hoverSlewRate(0.01),
    hoverParkAngle(defaultHoverParkAngleRad),
    hoverIIRValue(exp(-0.001/saturate(defaultHoverIIRTimeConstant, 0.001, 1))),
    hoverKneePoint(M_PI/4),
    latestUpdateSineCmdTime(-1), // -1: invalid time
    FSLatestUpdateMotorTargetsTime(-1) // -1: invalid time
      {
    for (int i = 0; i < 6; i++) {
      leg_offset[i] = desired_leg_offset[i] = 0;
      phase_offset[i] = 0;
      hoverAmp[i] = 0;
      hover_offset[i] = 0;

      FSCurrPeriodicLegState.amplitudes[i] = FSTargetPeriodicLegState.amplitudes[i] = 0;
      FSCurrPeriodicLegState.frequencies[i] = FSTargetPeriodicLegState.frequencies[i] = defaultFrequency;
      FSCurrPeriodicLegState.phase_offsets[i] = FSTargetPeriodicLegState.phase_offsets[i] = 0;
      FSCurrPeriodicLegState.leg_offsets[i] = FSTargetPeriodicLegState.leg_offsets[i] = 0;
      if (i == 1 || i == 4) {
        FSCurrPeriodicLegState.phase_offsets[i] = FSTargetPeriodicLegState.phase_offsets[i] = M_PI;
      }

      FStsinstart[i] = 0;

      FSLatestMotorTargets[i].pos = 0;
      FSLatestMotorTargets[i].vel = 0;
      FSLatestMotorTargets[i].acc = 0;
    }

    setDefaultPhaseOffset();

    // Disable all legs by default, until activate() is called
    legsEnabled = false;
  };


  ~UnderwaterSwimmerGait() {};


  void activate();


  void foreaftControl(int direction) {
    bool newSwimBackwards = (direction < 0);
    if (swimBackwards != newSwimBackwards) {
      swimBackwards = newSwimBackwards;

      // Flip the sign of hover_offset, since ComputeOrientedThrusters() does not
      // know about swimBackwards, and we are correcting for hover_offset's phase
      // after that fn call
      for (int i = 0; i < 6; i++) { hover_offset[i] = -hover_offset[i]; }
    }
  };
  int getForeAftDirection() { return (swimBackwards) ? -1 : 1; };


  void setMaxAmplitudeRad(float rad) { // Only used to translate from speed/heave/roll/pitch/yaw to sine signal
    maxAmplitudeRad = saturate(rad, 0, M_PI);
  };
  float getMaxAmplitudeRad() { return maxAmplitudeRad; };


  void setFrequency(float hz) { // Only used to translate from speed/heave/roll/pitch/yaw to sine signal
    frequencyHz = saturate(hz, 0, 4);
  };
  float getFrequency() { return frequencyHz; };


  inline void setBodyCmd(float speed, float heave, float roll, float pitch, float yaw) {
    setSpeedCmd(speed);
    heaveControl(heave);
    rollControl(roll);
    pitchControl(pitch);
    yawControl(yaw);
  };


  /** The following are legacy functions from RoboDevel::UnderwaterSwimmer */
  inline void setSpeedCmd(float cmd) {
    speedCmd = saturate(cmd, 0.0, 1.0);
  };
  float getSpeedCmd() { return speedCmd; };
  inline void heaveControl(float direction) {
    hoverHeaveCommand = saturate(direction, -1.0, 1.0);
  };
  float getHeaveCmd() { return hoverHeaveCommand; };
  inline void rollControl(float direction) {
    hoverRollCommand = saturate(direction, -1.0, 1.0); //Sets leg offsets to roll based on fraction of analogue tilt
  };
  float getRollCmd() { return hoverRollCommand; };
  inline void pitchControl(float direction) {
    hoverPitchCommand = saturate(direction, -1.0, 1.0); //Sets leg offsets to pitch based on fraction of analogue tilt
  };
  float getPitchCmd() { return hoverPitchCommand; };
  inline void yawControl(float direction) {
    hoverYawCommand = saturate(direction, -1.0, 1.0);
  };
  float getYawCmd() { return hoverYawCommand; };

  void updateSineCmd(PeriodicLegState_t& legsCmd);


  const PeriodicLegState_t& setPeriodicLegCmd(const PeriodicLegState_t& legsCmd) {
    for (int i = 0; i < 6; i++) {
      FSTargetPeriodicLegState.frequencies[i] = saturate(legsCmd.frequencies[i], 0.0, FS_MAX_FREQUENCY_HZ);
      FSTargetPeriodicLegState.amplitudes[i] = saturate(legsCmd.amplitudes[i], 0, M_PI/2); // a preliminary filter on amplitudes
      if (!FS_PHASE_OFFSET_USE_STATIC) {
        FSTargetPeriodicLegState.phase_offsets[i] = FSPiAdjust(legsCmd.phase_offsets[i]);
      }
      FSTargetPeriodicLegState.leg_offsets[i] = FSPiAdjust(legsCmd.leg_offsets[i]);
      FSTargetPeriodicLegState.leg_velocities[i] = saturate(legsCmd.leg_velocities[i], -4*M_PI, 4*M_PI);
    }

    return FSTargetPeriodicLegState;
  };
  void updateMotorTarget(MotorTarget_t (&target)[6]);


protected:
  /** Sets individual leg phase offsets for hover-midoff gait */
  inline void setDefaultPhaseOffset() {
    for(int i = 0; i < 6; i++) {
      if ((i == 1) || (i == 4)) {
        phase_offset[i] = M_PI;
      } else {
        phase_offset[i] = 0.0;
      }

      FSCurrPeriodicLegState.phase_offsets[i] = phase_offset[i];
      FSTargetPeriodicLegState.phase_offsets[i] = phase_offset[i];
    }
  };


  const std::function<double(void)> time; // For use to generate commands with arbitrary time constraints, rather than depending on real clock

  bool legsEnabled;
  float leg_offset[6], desired_leg_offset[6], phase_offset[6];

  const float foreAngle;
  const float aftAngle;

  // New IIR value
  const float linearizationFactor;

  // Hovering-related variables
  float speedCmd; // desired speed, within [0, 1] range
  float hoverPitchCommand;
  float hoverRollCommand;
  float hoverYawCommand;
  float hoverHeaveCommand;
  bool swimBackwards;
  float maxAmplitudeRad;
  float frequencyHz;

  float hover_offset[6];
  float hoverAmp[6];

  const float hoverSlewRate;  // Angle rate at which flipper re-orientation happens.
  const float hoverParkAngle;
  const float hoverIIRValue;
  const float hoverKneePoint;

  double latestUpdateSineCmdTime;
  double FSLatestUpdateMotorTargetsTime;

  double FStsinstart[6]; // latest time of start of a sinusoid; used as reference to compute tsin in updateMotorTargets(); set in activate() and when sine finished 1 period in updateMotorTargets()

  MotorTarget_t FSLatestMotorTargets[6];

  PeriodicLegState_t FSCurrPeriodicLegState;
  PeriodicLegState_t FSTargetPeriodicLegState;
};


#endif // #ifndef GAITS_HPP_
