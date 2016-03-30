#include "aqua_gait/Gaits.hpp"


#define FLATTEN_FLIPPERS_ON_ZERO_BODY_COMMAND


inline bool FSZeroCrossing( float time, float freq ) {
  if (fabs(time - 3.0/4/freq) <= 0.005) {
    return true;
  } else if (fabs(time - 1.0/4/freq) <= 0.005) {
    return true;
  } else {
    return false;
  }
};


inline float FSIIR(float old, float in, float gain, unsigned int ms = 1) {
  if (ms > 1) {
    return UnderwaterSwimmerGait::FSPiAdjust(in - pow(gain, ms)*UnderwaterSwimmerGait::FSPiAdjust(in-old));
  } else if (ms <= 0) {
    return old;
  }
  return UnderwaterSwimmerGait::FSPiAdjust(in - gain*UnderwaterSwimmerGait::FSPiAdjust(in-old));
};


inline double sign(double a) {
  if (a > 0) return 1;
  else if (a < 0) return -1;
  return 0;
};


inline float ratelinearization(float x, float alpha) {
  return 2.0*sign(x)*alpha*(sqrt(1.0 + fabs(x)/alpha) - 1.0);
};


/*******************************************************************************
 *                            ComputeOrientedThruster
 * This function will compute what the individual action of a flipper should
 * be in order to generate the proper forces. The basic idea is that the forces
 * can be generated either by moving the surface in the proper direction (thus
 * generating pressure drag in the somewhat right direction) or oscillating
 * the surface to generate thrust. The change of behavior from one to the other
 * is decided when the surface is at 45 deg of the demanded thrust direction.
 * When no force is asked, the surface is "parked" slowly to parkAngle, in
 * order to increase the reaction time if an opposite command is demanded.
 * The variable slewRate will dictate how fast the surface is moved to generate
 * the pressure drag, and therefore should be selected so it generate a drag
 * compatible with the oscillation characteristics.
 * ****************************************************************************/
inline void ComputeOrientedThruster(float Xthrust, float Ythrust,
    float offset, float *pLegAngle, float *pAmp,
    float slewRate, float parkAngle, float kneePoint) {
  float targetAngle, Magthrust;
  float ratio, deltaAngle;

  float nonSaturatedMagThrust = sqrt(Xthrust*Xthrust + Ythrust*Ythrust);

  Xthrust = UnderwaterSwimmerGait::saturate(Xthrust,-1.5,1.5);
  Ythrust = UnderwaterSwimmerGait::saturate(Ythrust,0.0,1.0);
  targetAngle = atan2(Xthrust,Ythrust) + offset;
  Magthrust = sqrt(Xthrust*Xthrust + Ythrust*Ythrust);
  Magthrust = UnderwaterSwimmerGait::saturate(Magthrust,0.0,1.5);
  if (Magthrust < 0.05) {
    // We want to bring the flippers to a +-45 deg parking position, so when
    // a new command is required we have some margins.
    if (nonSaturatedMagThrust > 0.1) {
      // If there is a net command for other flippers, we should retract our flippers
      targetAngle = 0.0;
      slewRate = slewRate*0.2;
    } else { // nonSaturatedMagThrust <= 0.1
      targetAngle = UnderwaterSwimmerGait::saturate((*pLegAngle), -parkAngle, parkAngle);
      slewRate = slewRate*0.05;
    }
  } else {
    slewRate = slewRate * Magthrust; // To make motion somewhat prop to demanded force
  }

  // If we are more than 45 deg away from target position, we cut-off
  // the amplitude of the oscillation
  //
  // NOTE: the following LINEAR difference works only since targetAngle is
  //       being saturated to -pi/pi (assuming offset = 0), and pLegAngle
  //       follows targetAngle while limiting its rate of change (by +/- slewRate).
  //       Ideally, we should be using an ANGULAR difference instead, although
  //       that would add slightly more computation:
  //
  // deltaAngle = targetAngle - (*pLegAngle) + two_pi/2;
  // deltaAngle = (deltaAngle > 0) ? deltaAngle - floor(deltaAngle/two_pi)*two_pi - two_pi/2 : deltaAngle - (floor(deltaAngle/two_pi) + 1)*two_pi + two_pi/2;
  //
  // a.k.a. deltaAngle = UnderwaterSwimmerGait::FSAngularMag(targetAngle, (*pLegAngle));
  deltaAngle = targetAngle - (*pLegAngle);
  ratio = UnderwaterSwimmerGait::saturate((1 - fabs(deltaAngle) / (kneePoint)),0.0,1.0);
  (*pAmp) = Magthrust * ratio;
  (*pLegAngle) += UnderwaterSwimmerGait::saturate(deltaAngle,-slewRate,slewRate);
};


void UnderwaterSwimmerGait::activate() {
  double now = time();

  speedCmd = 0;
  hoverPitchCommand = 0;
  hoverRollCommand = 0;
  hoverYawCommand = 0;
  hoverHeaveCommand = 0;
  //swimBackwards = false;
  latestUpdateSineCmdTime = -1;
  FSLatestUpdateMotorTargetsTime = -1;

  for(int i = 0; i < 6; i++) {
    // Reset variables
    hoverAmp[i] = 0.0;
    hover_offset[i] = 0.0;
    phase_offset[i] = 0;
    leg_offset[i] = desired_leg_offset[i] = 0.0;

    FStsinstart[i] = now;
  }
  
  setDefaultPhaseOffset();

  legsEnabled = true;
};


void UnderwaterSwimmerGait::updateSineCmd(PeriodicLegState_t& outputLegsCmdBuffer) {
  // NOTE: if this section is called AFTER filtering leg_offset but BEFORE
  //       ComputeOrientedThruster, then upon pause the legs slowly filter
  //       towards the last computed hover_offset angle.
  //       Since we have currently removed the pause feature, it can be placed
  //       at the start of the function
  if (!legsEnabled) {
    for (int i = 0; i < 6; i++) {
      outputLegsCmdBuffer.amplitudes[i] = 0.0;
      outputLegsCmdBuffer.frequencies[i] = 0.0;
      outputLegsCmdBuffer.phase_offsets[i] = phase_offset[i];
      outputLegsCmdBuffer.leg_offsets[i] = leg_offset[i];
      outputLegsCmdBuffer.leg_velocities[i] = 0.0;
    }
    return;
  }

  // Determine time since last call
  double dt = 0;
  double now = time();
  if (latestUpdateSineCmdTime >= 0) {
    dt = now - latestUpdateSineCmdTime;
  }
  latestUpdateSineCmdTime = now;
  unsigned int dtMSEC = floor(dt/0.001); // 1 msec is the atomic rate in RoboDevel

  if (dt > 0) { // Do not update commands unless time has progressed
    // Compute desired leg offsets (i.e. hover_offset[i]) and amplitudes
    //
    // NOTE: hover_offset updated slowly towards the combined target angle computed
    //       from all X and Y forced desired. This is achieved by capping the
    //       change in hover_offset per update() call to by slew_rate param

    // This is another hack to quickly get the second hovering gate in place.
    // Ideally we need to rewrite all this code to get it working easily.
    // The input variables are:
    //   hoverRollCommand   -> Roll Command
    //   hoverPitchCommand  -> Pitch Command
    //   hoverYawCommand    -> Yaw Command
    //   hoverHeaveCommand  -> Heave Command
    //   hoverSurgeCommand  -> Surge Command
    // The output variables are:
    //   ampl[]        -> Amplitude of the oscillation
    //   leg_offset[]  -> leg offset
    //
    // For each "thruster", we have a sum of the thrust in the X and Y direction.
    //   Pitch, Roll, Heave  -> Y axis
    //   Yaw, Surge          -> X axis
    // By computing the sum over each axis, we can determine the angle of the thruster and its magnitude.
    // The min(..) max(...) is used to allow only certain commands directions.
    float Xthrust, Ythrust, targetAmp;
    const float timeNormalizedHoverSlewRate = hoverSlewRate * dt * 1.0e3;

    // Front Legs
    if (swimBackwards) {
      Xthrust = hoverRollCommand - hoverPitchCommand - hoverHeaveCommand;
      Ythrust = -hoverYawCommand + speedCmd;
    } else {
      Xthrust = hoverRollCommand - hoverPitchCommand + hoverHeaveCommand;
      Ythrust =  hoverYawCommand + speedCmd;
    }
    ComputeOrientedThruster(Xthrust, Ythrust, 0.0, &hover_offset[0], &targetAmp,
        timeNormalizedHoverSlewRate, hoverParkAngle, hoverKneePoint);
    targetAmp *= maxAmplitudeRad; // NOTE: ComputeOrientedThruster returns a normalized amplitude; we depend on amplitudeGUI to scale up to amplitude in leg angle radian space
    hoverAmp[0] = FSIIR(hoverAmp[0], targetAmp, hoverIIRValue, dtMSEC);

    if (swimBackwards) {
      Xthrust = -hoverRollCommand - hoverPitchCommand - hoverHeaveCommand;
      Ythrust =  hoverYawCommand + speedCmd;
    } else {
      Xthrust = -hoverRollCommand - hoverPitchCommand + hoverHeaveCommand;
      Ythrust = -hoverYawCommand + speedCmd;
    }
    ComputeOrientedThruster(Xthrust, Ythrust,0.0, &hover_offset[3], &targetAmp,
        timeNormalizedHoverSlewRate, hoverParkAngle, hoverKneePoint);
    targetAmp *= maxAmplitudeRad;
    hoverAmp[3] = FSIIR(hoverAmp[3], targetAmp, hoverIIRValue, dtMSEC);

    // Back Legs
    if (swimBackwards) {
      Xthrust = hoverRollCommand + hoverPitchCommand - hoverHeaveCommand;
      Ythrust = -hoverYawCommand + speedCmd;
    } else {
      Xthrust = hoverRollCommand + hoverPitchCommand + hoverHeaveCommand;
      Ythrust =  hoverYawCommand + speedCmd;
    }
    ComputeOrientedThruster(Xthrust, Ythrust, 0.0, &hover_offset[2], &targetAmp,
        timeNormalizedHoverSlewRate, hoverParkAngle, hoverKneePoint);
    targetAmp *= maxAmplitudeRad;
    hoverAmp[2] = FSIIR(hoverAmp[2], targetAmp, hoverIIRValue, dtMSEC);

    if (swimBackwards) {
      Xthrust = -hoverRollCommand + hoverPitchCommand - hoverHeaveCommand;
      Ythrust =  hoverYawCommand + speedCmd;
    } else {
      Xthrust = -hoverRollCommand + hoverPitchCommand + hoverHeaveCommand;
      Ythrust = -hoverYawCommand + speedCmd;
    }
    ComputeOrientedThruster(Xthrust, Ythrust, 0.0, &hover_offset[5], &targetAmp,
        timeNormalizedHoverSlewRate, hoverParkAngle, hoverKneePoint);
    targetAmp *= maxAmplitudeRad;
    hoverAmp[5] = FSIIR(hoverAmp[5], targetAmp, hoverIIRValue, dtMSEC);

    // Middle Legs
    if (swimBackwards) {
      Xthrust = hoverRollCommand - hoverHeaveCommand;
      Ythrust = -hoverYawCommand + speedCmd;
    } else {
      Xthrust = hoverRollCommand + hoverHeaveCommand;
      Ythrust =  hoverYawCommand + speedCmd;
    }
    ComputeOrientedThruster(Xthrust, Ythrust, 0.0, &hover_offset[1], &targetAmp,
        timeNormalizedHoverSlewRate, hoverParkAngle, hoverKneePoint);
    targetAmp *= maxAmplitudeRad;
    hoverAmp[1] = FSIIR(hoverAmp[1], targetAmp, hoverIIRValue, dtMSEC);

    if (swimBackwards) {
      Xthrust = -hoverRollCommand - hoverHeaveCommand;
      Ythrust =  hoverYawCommand + speedCmd;
    } else {
      Xthrust = -hoverRollCommand + hoverHeaveCommand;
      Ythrust = -hoverYawCommand + speedCmd;
    }
    ComputeOrientedThruster(Xthrust, Ythrust, 0.0, &hover_offset[4], &targetAmp,
        timeNormalizedHoverSlewRate, hoverParkAngle, hoverKneePoint);
    targetAmp *= maxAmplitudeRad;
    hoverAmp[4] = FSIIR(hoverAmp[4], targetAmp, hoverIIRValue, dtMSEC);

    bool flattenFlippers = false;
#ifdef FLATTEN_FLIPPERS_ON_ZERO_BODY_COMMAND
    if (speedCmd == 0 && hoverHeaveCommand == 0 && hoverRollCommand == 0 &&
        hoverPitchCommand == 0 && hoverYawCommand == 0) {
      flattenFlippers = true;
    }
#endif

    //Filter leg offset angle
    for (int i = 0; i < 6; i++) {
      desired_leg_offset[i] = hover_offset[i] + ((swimBackwards) ? aftAngle : foreAngle);

      if (flattenFlippers) {
        desired_leg_offset[i] = 0;
      }

      //IIR filter of leg offset given a step command

      /* 26 July 2005 P. Giguere: I will be trying a slightly different filter
       * than the regular low-pass filter. This is done in order to linearize
       * the moments generated by the flippers, considering that a significant
       * portion comes from the squared angular velocity of the flipper.
       * The idea is to have the filter behave normally for small commands,
       * but to have the effect of increased time-constant for larger value,
       * in order to reduce the velocity by square root, roughly. The linearizationFactor parameter
       * indicates roughly where to switch from normal to reduced behavior. */
      if (linearizationFactor > 0.001) {
        for (unsigned int i = 0; i < dtMSEC; i++) {
          leg_offset[i] = leg_offset[i] + (1.0-hoverIIRValue) *
              ratelinearization((desired_leg_offset[i] - leg_offset[i]), linearizationFactor);
        }
      } else {
        leg_offset[i] = FSIIR(leg_offset[i], desired_leg_offset[i], hoverIIRValue, dtMSEC);
      }

      leg_offset[i] = FSPiAdjust(leg_offset[i]);
    }
  }

  // Store resulting commands
  for (int i = 0; i < 6; i++) {
    outputLegsCmdBuffer.amplitudes[i] = hoverAmp[i];
    outputLegsCmdBuffer.frequencies[i] = frequencyHz;
    outputLegsCmdBuffer.phase_offsets[i] = phase_offset[i];
    outputLegsCmdBuffer.leg_offsets[i] = leg_offset[i];
    outputLegsCmdBuffer.leg_velocities[i] = 0;
  }
};


void UnderwaterSwimmerGait::updateMotorTarget(MotorTarget_t (&tar)[6]) {
  // Disable acceleration terms
  for (int i = 0; i < 6; i++) tar[i].acc = 0;

  // If legs are currently not enabled, then immediately return desired leg offset
  if (!legsEnabled) { // For now, consider this as an abnormal exception, rather than a true function call [since we removed the pause feature]
    for (int i = 0; i < 6; i++) {
      FSLatestMotorTargets[i].pos = FSCurrPeriodicLegState.leg_offsets[i];
      FSLatestMotorTargets[i].vel = 0;
      FSLatestMotorTargets[i].acc = 0;
      tar[i].pos = FSCurrPeriodicLegState.leg_offsets[i];
      tar[i].vel = 0;
    }
    return;
  }


  // 0. Determine time since last call
  double dt = 0;
  double now = time();
  if (FSLatestUpdateMotorTargetsTime >= 0) {
    dt = now - FSLatestUpdateMotorTargetsTime;
  }
  FSLatestUpdateMotorTargetsTime = now;
  // Do not update motor targets if time has not progressed
  if (dt <= 0) {
    for (int i = 0; i < 6; i++) {
      tar[i].pos = FSLatestMotorTargets[i].pos;
      tar[i].vel = FSLatestMotorTargets[i].vel;
      tar[i].acc = 0;
    }
    return;
  }


  // 1. Update current leg commands with target commands
  bool phaseOffsetChanged;
  for (int i = 0; i < 6; i++) {
    // Update leg velocities
    FSCurrPeriodicLegState.leg_velocities[i] = FSTargetPeriodicLegState.leg_velocities[i];

    // Update phase offset
    phaseOffsetChanged = false;
    if (!FS_PHASE_OFFSET_USE_STATIC &&
        (FSCurrPeriodicLegState.phase_offsets[i] != FSTargetPeriodicLegState.phase_offsets[i])) {
      FSCurrPeriodicLegState.phase_offsets[i] = FSTargetPeriodicLegState.phase_offsets[i];
      phaseOffsetChanged = true;
    }

    // Update amplitude, frequency, and leg offset
    if (!FS_ZERO_CROSSING ||
        (FSCurrPeriodicLegState.amplitudes[i] == 0) ||
        (FSCurrPeriodicLegState.frequencies[i] == 0)) {
      FSCurrPeriodicLegState.amplitudes[i] = FSTargetPeriodicLegState.amplitudes[i];
      FSCurrPeriodicLegState.frequencies[i] = FSTargetPeriodicLegState.frequencies[i];

      if (FSCurrPeriodicLegState.leg_velocities[i] != 0) {
        // Integrate velocity
        if (dt > 0) {
          FSCurrPeriodicLegState.leg_offsets[i] += FSCurrPeriodicLegState.leg_velocities[i]*dt;
        }
      } else { // Static leg offset
        FSCurrPeriodicLegState.leg_offsets[i] = FSTargetPeriodicLegState.leg_offsets[i];
      }
    } else {
      // WARNING: The following logic probably does not work when phase_offsets change,
      //          since we either need to compute new FStsinstart[i] based on matching
      //          cos(params1) == cos(params2) & sign(sin(params1)) == sign(sin(params2));
      //          or we immediately accept new phase offset, but then the analytical
      //          velocity would be arbitrarily different from the true velocity
      //          needed to implement the phase shift.
      //
      //          To see this, consider a simple example where amplitude and frequency
      //          does not change, and phase offset changes by M_PI between 2 consecutive
      //          calls:
      //          - for any given tsin, what is the proper velocity?
      //          - do we need to update FStsinstart? if so, to what?
      double tsin = now - FStsinstart[i];
      // WARNING: FSZeroCrossing() only work properly if updateMotorTarget()
      //          is called faster than 200Hz, and ideally faster than 400Hz.
      //          This is because FSZeroCrossing() checks for approximate
      //          equality of time with a fudge factor of +/- 5ms
      //          (which came from RoboDevel's UnderwaterSwimmer())
      if (!phaseOffsetChanged &&
          FSZeroCrossing(tsin, FSCurrPeriodicLegState.frequencies[i])) {
        // May need to shift new FStsinstart if frequency changes, in order to match frequencies
        // WARNING: this may in fact destroy phase offsets between legs...
        if (FS_ZERO_CROSSING_UPDATE_TSINSTART_TO_MATCH_PHASE) {
          double newDTSin = tsin * FSCurrPeriodicLegState.frequencies[i] /
              FSTargetPeriodicLegState.frequencies[i];
          FStsinstart[i] = now - newDTSin;
        }

        FSCurrPeriodicLegState.amplitudes[i] = FSTargetPeriodicLegState.amplitudes[i];
        FSCurrPeriodicLegState.frequencies[i] = FSTargetPeriodicLegState.frequencies[i];
        if (FSCurrPeriodicLegState.leg_velocities[i] != 0) {
          // Integrate velocity
          if (dt > 0) {
            FSCurrPeriodicLegState.leg_offsets[i] += FSCurrPeriodicLegState.leg_velocities[i]*dt;
          }
        } else { // Static leg offset
          FSCurrPeriodicLegState.leg_offsets[i] = FSTargetPeriodicLegState.leg_offsets[i];
        }
      }
    }
  }


  // 2. Compute sinusoidal motion, and check for end-of-sine-period condition
  now = time();
  for (int i = 0; i < 6; i++) {
    // Generate target pose and velocity based on ideal sinusoidal pattern
    double tsin = now - FStsinstart[i];
    const float omega = two_pi*FSCurrPeriodicLegState.frequencies[i];
    if (omega == 0) {
      tar[i].pos = FSCurrPeriodicLegState.leg_offsets[i];
      tar[i].vel = 0;
    } else {
      double currPhase = omega * tsin + FSCurrPeriodicLegState.phase_offsets[i];
      tar[i].pos = FSCurrPeriodicLegState.amplitudes[i] *
          cos(currPhase) + FSCurrPeriodicLegState.leg_offsets[i];
      tar[i].vel = -FSCurrPeriodicLegState.amplitudes[i] * omega *
          sin(currPhase);
    }

    // WARNING: this logic only works if updateMotorTarget() is called at a much faster rate than the period of the sine!

    if (omega == 0) {
      FStsinstart[i] = now;
    } else if (tsin > 1.0/FSCurrPeriodicLegState.frequencies[i]) {
      double currPeriod = 1.0/FSCurrPeriodicLegState.frequencies[i];
      double tsinExtra = tsin - floor(tsin/currPeriod)*currPeriod;
      FStsinstart[i] = now - tsinExtra; // Reset counter

      // WARNING: this is the old way of resetting tsinstart based on RoboDevel;
      // this leaks time ever slightly and causes the sinusoidal pattern to
      // stretch slowly in time, since the start of each subsequent sine period
      // is reset at the time the next update() is called
      // FStsinstart[i] = now;
    }
  }


  // 3. Filter motor targets (pos and vel) to ensure safe current-limited
  //   operations
  /*
   * RATIONALE:
   * - velocity is computed as max((leg_pos - prev_leg_pos)/dt, vel_sine)
   *   - while implementing a fixed sine command, vel_sine > empirical estimate
   *     (check for yourself)
   *   - however, this may no longer be true when we allow arbitrary changes to
   *     phase offsets or leg offsets between two consecutive sine commands
   *
   * - velocity is saturated by maximum value, analogous to acceleration (*SEE BELOW*)
   *
   * - acceleration is computed as vel_sine - prev_vel_sine
   *   - this means that we ignore the analytical acceleration of a sine command,
   *     and only focus on limiting the change in empirically estimated acceleration
   *     when changing from one sine command to another arbitrary sine command
   *
   * - the main problem that these saturation filters are trying to address is
   *   excessive current draw, which is hypothesized to crash the entire robot.
   *   - for a DC motor, current draw is proportional to torque
   *   - torque is proportional to (angular) acceleration
   *   - hence if we want to limit maximum current draw, we need to limit maximum
   *     acceleration
   *
   * - whenever a new sine command comes in, we first saturate its amplitude
   *   and frequency based on current designated maximum acceleration, for all legs:
   *   - first assume that frequency is saturated to a fixed value, e.g. 4Hz
   *   - the analytical acceleration of angle = ampl*cos(2*pi*freq*t) is
   *     -ampl*(2*pi*freq)*(2*pi*freq)*cos(2*pi*freq*t)
   *   - therefore the total current requested for all legs is:
   *     acc_tot = (sum_{i=1:6} ampl_i) * (2*pi*freq_max)^2
   *   - if acc_tot exceeds acc_max, then all amplitudes are scaled down by
   *     the ratio (acc_tot/acc_max)
   *
   * - regardless of the filtering at the amplitude and frequency level,
   *   there is still the concern that the leg angles can change very fast,
   *   e.g. when the leg_offset of 2 consecutive sine commands are off by M_PI
   *   - thus, after we compute the analytical sinusoidal leg poses and velocities,
   *     we must filter them
   *   - first, vel_curr = max(vel_sine, vel_empirical) is saturated by vel_max
   *   - then we compute acc_empirical, and saturate that by acc_max
   *   - then we re-compute vel_curr based on the saturated acceleration
   *     (which must be smaller than the un-filtered value)
   *   - finally we re-compute pos_curr based on the saturated velocity
   *
   * - since we don't know the scalar constant between the (empirical)
   *   acceleration and the total current draw of all legs, the maximum
   *   acceleration threshold is chosen heuristically:
   *   - in HOVER_MIDOFF, the robot is expected to be able to swim smoothly,
   *     without any filtering, at ampl=20/180*pi and freq=2.5.
   *   - we shall target ampl=30/180*pi and freq=3 as the most aggressively
   *     allowed behavior for a typical scenario (although we've seen non-crashing
   *     motions of ampl=35/180*pi and freq=4 on Ramius)
   *   - thus acc_max = (sum_{i=1:6} ampl_i) * (2*pi*freq_i)^2 = (pi) * (2*pi*3)^2 = 36*pi^3
   *     (stored in variable: FSAllMotorsMaxAcceleration)
   *   - (*FROM ABOVE*) we will pick vel_max to match these settings:
   *     vel_max = (sum_{i=1:6} ampl_i) * (2*pi*freq_i) = 6*pi^2
   *     (stored in variable: FSAllMotorsMaxVelocity)
   */

  // 3.1 Switch to empirical velocity during transient state between
  // two different periodic leg commands
  //
  // In particular, accept empirical velocity and ignore sinusoidal
  // velocity when the former has larger magnitude, and when the
  // former asks the leg position to move in the opposite direction
  // of the latter
  double velEmp[6];
  bool filteredVelUseEmpirical[6];
  double speedAllLegs = 0;
  for (int i = 0; i < 6; i++) {
    filteredVelUseEmpirical[i] = false;
    velEmp[i] = UnderwaterSwimmerGait::FSAngularMag(tar[i].pos, FSLatestMotorTargets[i].pos)/dt;
    if ((fabs(velEmp[i]) > fabs(tar[i].vel)) || (sign(velEmp[i]) != sign(tar[i].vel))) {
      tar[i].vel = velEmp[i];
      filteredVelUseEmpirical[i] = true;
    }

    // Also compute the total speed, in preparation for 3.2
    speedAllLegs += fabs(tar[i].vel);
  }

  // 3.2 Limit maximum velocity via saturation or sigmoid squashing
  double velDownscaleFactor = 1.0;
  if (FS_ALL_MOTORS_VELOCITY_MAX >= 0) { // allow saturation or squashing
    if (FS_ALL_MOTORS_VELOCITY_SQUASH_MAX) { // Squash velocity
      velDownscaleFactor = FSSigmoidSquashGain(speedAllLegs,
          FS_ALL_MOTORS_VELOCITY_MAX,
          FS_ALL_MOTORS_VELOCITY_SQUASH_GAIN);
    } else { // Saturate velocity
      if (speedAllLegs > FS_ALL_MOTORS_VELOCITY_MAX) {
        velDownscaleFactor = FS_ALL_MOTORS_VELOCITY_MAX / speedAllLegs;
      }
    }

    for (int i = 0; i < 6; i++) {
      tar[i].vel *= velDownscaleFactor;
    }
  }

  // 3.3 Limit maximum acceleration via saturation or sigmoid squashing
  double accDownscaleFactor = 1.0;
  double accMagnAllLegs = 0;
  double accLeg[6];
  for (int i = 0; i < 6; i++) {
    accLeg[i] = (tar[i].vel - FSLatestMotorTargets[i].vel)/dt;
    accMagnAllLegs += fabs(accLeg[i]);
  }
  if (FS_ALL_MOTORS_ACCELERATION_MAX >= 0) { // allow saturation or squashing
    if (FS_ALL_MOTORS_ACCELERATION_SQUASH_MAX) { // Squash acceleration
      accDownscaleFactor = FSSigmoidSquashGain(accMagnAllLegs,
          FS_ALL_MOTORS_ACCELERATION_MAX,
          FS_ALL_MOTORS_ACCELERATION_SQUASH_GAIN);
    } else { // Saturate acceleration
      if (accMagnAllLegs > FS_ALL_MOTORS_ACCELERATION_MAX) {
        accDownscaleFactor = FS_ALL_MOTORS_ACCELERATION_MAX/accMagnAllLegs;
      }
    }
  }
  for (int i = 0; i < 6; i++) {
    accLeg[i] *= accDownscaleFactor;
    tar[i].vel = FSLatestMotorTargets[i].vel + accLeg[i]*dt;
  }

  // 3.4 Filter change in target position using PD feed-forward control
  // NOTE: only proceed if either:
  //   A) during transient period when switching to a new periodic leg
  //      command, empirical velocity is used rather than sinuosidal velocity;
  //   B) at the start and end of transient phase, the acceleration
  //      is saturated to prevent spiking
  for (int i = 0; i < 6; i++) {
    if (filteredVelUseEmpirical[i] || accDownscaleFactor < 1) {
      tar[i].pos = FSLatestMotorTargets[i].pos + tar[i].vel*dt;
    }
  }


  // 4. Save filtered motor targets
  for (int i = 0; i < 6; i++) {
    FSLatestMotorTargets[i].pos = tar[i].pos;
    FSLatestMotorTargets[i].vel = tar[i].vel;
    FSLatestMotorTargets[i].acc = tar[i].acc;
  }
};
