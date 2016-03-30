#ifndef QCLASSES_
#define QCLASSES_


#include <QtGui>
#include <QMainWindow>
#include <QWidget>
#include <QPainter>
#include <QPen>
#include <QSize>
#include <QGridLayout>
#include <QLineEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QString>
#include <QTimer>
#include <QElapsedTimer>
#include <QGroupBox>
#include <QMutex>
#include <QSocketNotifier>

#include <sys/socket.h>
#include <algorithm>

#include "aqua_gait/Gaits.hpp"
#include <ros/ros.h>
#include <aquacore/Command.h>
#include <aquacore/PeriodicLegCommand.h>
#include <std_srvs/Empty.h>
#include <aquacore/SetGait.h>
#include <aquacore/SetPauseMode.h>


#define ROS_PUBLISHER_WAIT_FOR_SERVICES


enum {MODE_ARBT_BODY_CMD = 0, MODE_ARBT_SINE_CMD, MODE_SUB_BODY_CMD, MODE_SUB_SINE_CMD, NUM_MODES};


inline double angularMag(
    double a,
    double b,
    double range = 360.0) {
  double d = b - a + range/2;
  d = (d > 0) ? d - floor(d/range)*range - range/2 : d - (floor(d/range) + 1)*range + range/2;
  return d;
};


/**
 * Computes angular (and general modulo-'range') distance
 */
inline double angularDist(double a, double b, double range = 2*M_PI) { return fabs(angularMag(a, b, range)); };


/** Group box with single widget */
class QWidgetBox : public QGroupBox {
Q_OBJECT
public:
  QWidgetBox(const QString& title, QWidget* child, QWidget* parent = 0) :
      QGroupBox(title, parent) {
    layout = new QGridLayout(this);
    layout->addWidget(child);
    setLayout(layout);
    setFlat(true);
  };
  virtual ~QWidgetBox() {};

protected:
  QGridLayout* layout;
};


class LegWidget : public QWidget {
Q_OBJECT
public:
  double angleRad;

  LegWidget(QWidget* parent = NULL) : QWidget(parent), angleRad(0) {
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
  };
  virtual ~LegWidget() {};

  virtual QSize sizeHint() const { return QSize(50, 50); };

public slots:
  void setAngle(double radians) {
    angleRad = radians;
    emit update();
  };

protected:
  void paintEvent(QPaintEvent* event) {
    QPainter painter(this);
    QPen pen(Qt::black, 3, Qt::SolidLine);
    double minwh = std::min(width(), height());
    double mid = minwh/2;
    double radius = mid*0.9;
    painter.drawEllipse(mid-radius, mid-radius, radius*2, radius*2);
    painter.drawLine(mid-radius, mid, mid-0.8*radius, mid);
    painter.drawLine(mid, mid-radius, mid, mid-0.8*radius);
    double xtip = mid + radius*cos(angleRad);
    double ytip = mid + radius*sin(angleRad);
    painter.drawLine(mid, mid, xtip, ytip);
  };
};


class LegWithTextWidget : public QWidget {
Q_OBJECT
public:
  LegWithTextWidget(QWidget* parent = NULL) : QWidget(parent) {
    legImage = new LegWidget(this);
    legValue = new QLineEdit("0", this);
    legValue->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
    connect(legValue, SIGNAL(textEdited(const QString&)), this, SLOT(updateFromText(const QString&)));
    layout = new QGridLayout(this);
    layout->addWidget(legImage, 0, 0);
    layout->addWidget(legValue, 1, 0);
    this->setLayout(layout);
  };

public slots:
  void updateFromText(const QString& newvalstr) {
    bool ok = false;
    double newval = newvalstr.toDouble(&ok);
    if (ok) legImage->setAngle(newval);
  };


  void setAngle(float rad) {
    legImage->setAngle(rad);
    legValue->setText(QString::number(rad, 'f', 4));
  };


protected:
  LegWidget* legImage;
  QLineEdit* legValue;
  QGridLayout* layout;
};


class LegsWidget : public QWidget {
Q_OBJECT
public:
  LegsWidget(QWidget* parent = NULL) : QWidget(parent) {
    leftSideLabel = new QLabel("Left F/M/B:", this);
    rightSideLabel = new QLabel("Right F/M/B:", this);
    timeLabel = new QLabel("time: 0", this);

    for (int i = 0; i < 6; i++) {
      legs[i] = new LegWithTextWidget(this);
    }

    layout = new QGridLayout(this);
    layout->addWidget(leftSideLabel, 0, 0);
    layout->addWidget(legs[0], 0, 1);
    layout->addWidget(legs[1], 0, 2);
    layout->addWidget(legs[2], 0, 3);
    layout->addWidget(rightSideLabel, 1, 0);
    layout->addWidget(legs[3], 1, 1);
    layout->addWidget(legs[4], 1, 2);
    layout->addWidget(legs[5], 1, 3);
    layout->addWidget(timeLabel, 2, 0);
    setLayout(layout);
  };


public slots:
  void setTime(double t) {
    timeLabel->setText(QString("time: ") + QString::number(t, 'f', 3));
  };


  void setLegAngles(float* anglesRad) {
    for (int i = 0; i < 6; i++) {
      legs[i]->setAngle(anglesRad[i]);
    }
  };


protected:
  LegWithTextWidget* legs[6];
  QLabel* leftSideLabel;
  QLabel* rightSideLabel;
  QLabel* timeLabel;
  QGridLayout* layout;
};


class PeriodicLegStateWidget : public QWidget {
Q_OBJECT
public:
  constexpr static double radian = M_PI/180.0;


  PeriodicLegStateWidget(QWidget* parent = NULL) : QWidget(parent) {
    resetBtn = new QPushButton("Zero Sine Cmd", this);
    connect(resetBtn, SIGNAL(pressed()), this, SLOT(zeroCmd()));

    map = new QSignalMapper(this);
    connect(map, SIGNAL(mapped(int)), this, SLOT(updateFromText(int)));

    layout = new QGridLayout(this);

    leg_offsetLabel = new QLabel("Leg Offset (deg):", this);
    layout->addWidget(leg_offsetLabel, 0, 0);
    leg_velocityLabel = new QLabel("Leg Velocity (deg/s):", this);
    layout->addWidget(leg_velocityLabel, 1, 0);
    amplitudeLabel = new QLabel("Amplitude (deg):", this);
    layout->addWidget(amplitudeLabel, 2, 0);
    frequencyLabel = new QLabel("Frequency (Hz):", this);
    layout->addWidget(frequencyLabel, 3, 0);
    phase_offsetLabel = new QLabel("Phase Offset (deg):", this);
    layout->addWidget(phase_offsetLabel, 4, 0);

    for (int i = 0; i < 6; i++) {
      leg_offset[i] = new QLineEdit(this);
      layout->addWidget(leg_offset[i], 0, i+1);
      map->setMapping(leg_offset[i], i);
      connect(leg_offset[i], SIGNAL(textEdited(const QString&)), map, SLOT(map()));

      leg_velocity[i] = new QLineEdit(this);
      layout->addWidget(leg_velocity[i], 1, i+1);
      map->setMapping(leg_velocity[i], i+6);
      connect(leg_velocity[i], SIGNAL(textEdited(const QString&)), map, SLOT(map()));

      amplitude[i] = new QLineEdit(this);
      layout->addWidget(amplitude[i], 2, i+1);
      map->setMapping(amplitude[i], i+12);
      connect(amplitude[i], SIGNAL(textEdited(const QString&)), map, SLOT(map()));

      frequency[i] = new QLineEdit(this);
      layout->addWidget(frequency[i], 3, i+1);
      map->setMapping(frequency[i], i+18);
      connect(frequency[i], SIGNAL(textEdited(const QString&)), map, SLOT(map()));

      phase_offset[i] = new QLineEdit(this);
      layout->addWidget(phase_offset[i], 4, i+1);
      map->setMapping(phase_offset[i], i+24);
      connect(phase_offset[i], SIGNAL(textEdited(const QString&)), map, SLOT(map()));
    }

    layout->addWidget(resetBtn, 5, 0);

    setLayout(layout);
  };


  const PeriodicLegState_t& getState() { return state; };


  void updateStateDisplay(PeriodicLegState_t& newState) {
    state = newState;
    refreshDisplay();
  };


public slots:
  void zeroCmd() {
    for (int i = 0; i < 6; i++) {
      state.leg_offsets[i] = 0;
      state.leg_velocities[i] = 0;
      state.amplitudes[i] = 0;
      state.frequencies[i] = 0;
      state.phase_offsets[i] = 0;
    }
    refreshDisplay();
    emit stateUpdated();
  };


  void updateFromText(int ID) {
    int field = ID / 6;
    bool ok = false;
    if (field == 0) { // leg_offset
      ID -= field*6;
      double newval = leg_offset[ID]->text().toDouble(&ok);
      if (ok) state.leg_offsets[ID] = UnderwaterSwimmerGait::FSPiAdjust(newval*radian);
    } else if (field == 1) { // leg_velocity
      ID -= field*6;
      double newval = leg_velocity[ID]->text().toDouble(&ok);
      if (ok) state.leg_velocities[ID] = UnderwaterSwimmerGait::saturate(newval*radian, -4*M_PI, 4*M_PI);
    } else if (field == 2) { // amplitude
      ID -= field*6;
      double newval = amplitude[ID]->text().toDouble(&ok);
      if (ok) state.amplitudes[ID] = UnderwaterSwimmerGait::saturate(newval*radian, 0, M_PI/2);
    } else if (field == 3) { // frequency
      ID -= field*6;
      double newval = frequency[ID]->text().toDouble(&ok);
      if (ok) state.frequencies[ID] = UnderwaterSwimmerGait::saturate(newval, 0, 4);
    } else if (field == 4) { // phase_offset
      ID -= field*6;
      double newval = phase_offset[ID]->text().toDouble(&ok);
      if (ok) state.phase_offsets[ID] = UnderwaterSwimmerGait::FSPiAdjust(newval*radian);
    }

    if (!ok) {
      refreshDisplay(); // reset to correct values
    } else {
      emit stateUpdated();
    }
  };


  void setReadOnly(bool RO) {
    for (int i = 0; i < 6; i++) {
      leg_offset[i]->setReadOnly(RO);
      leg_velocity[i]->setReadOnly(RO);
      amplitude[i]->setReadOnly(RO);
      frequency[i]->setReadOnly(RO);
      phase_offset[i]->setReadOnly(RO);
    }
    resetBtn->setEnabled(!RO);
  };

signals:
  void stateUpdated();


protected:
  void refreshDisplay() {
    for (int i = 0; i < 6; i++) {
      leg_offset[i]->setText(QString::number(state.leg_offsets[i]/M_PI*180.0, 'f', 4));
      leg_velocity[i]->setText(QString::number(state.leg_velocities[i]/M_PI*180.0, 'f', 4));
      amplitude[i]->setText(QString::number(state.amplitudes[i]/M_PI*180.0, 'f', 4));
      frequency[i]->setText(QString::number(state.frequencies[i], 'f', 4));
      phase_offset[i]->setText(QString::number(state.phase_offsets[i]/M_PI*180.0, 'f', 4));
    }
  };


  PeriodicLegState_t state;

  QLabel* leg_offsetLabel;
  QLabel* leg_velocityLabel;
  QLabel* amplitudeLabel;
  QLabel* frequencyLabel;
  QLabel* phase_offsetLabel;

  QLineEdit* leg_offset[6];
  QLineEdit* leg_velocity[6];
  QLineEdit* amplitude[6];
  QLineEdit* frequency[6];
  QLineEdit* phase_offset[6];

  QPushButton* resetBtn;

  QSignalMapper* map;

  QGridLayout* layout;
};


class ROSWidget : public QWidget {
Q_OBJECT
  public:
    ROSWidget(QWidget* parent = NULL) : QWidget(parent),
        mode(MODE_SUB_BODY_CMD), nh(), local_nh("~") {
      rateLabel = new QLabel("Pub Rate:", this);
      rateValue = new QLineEdit("0", this);
      rateValue->setFixedWidth(120);
      connect(rateValue, SIGNAL(textEdited(const QString&)), this, SLOT(updatePubRate(const QString&)));

      toggleGaitLabel = new QLabel("Curr Gait:", this);
      toggleGaitBtn = new QCheckBox("HOVER_MIDOFF", this);
      connect(toggleGaitBtn, SIGNAL(stateChanged(int)), this, SLOT(requestSineGait(int)));

      calibrateBtn = new QPushButton("Calibrate Rbt", this);
      connect(calibrateBtn, SIGNAL(released()), this, SLOT(calibrate()));

      pauseRobotBtn = new QPushButton("Pause Rbt", this);
      connect(pauseRobotBtn, SIGNAL(released()), this, SLOT(pauseRobot()));

      resumeRobotBtn = new QPushButton("Resume Rbt", this);
      connect(resumeRobotBtn, SIGNAL(released()), this, SLOT(resumeRobot()));

      layout = new QGridLayout(this);
      layout->addWidget(rateLabel, 0, 0);
      layout->addWidget(rateValue, 0, 1);
      layout->addWidget(toggleGaitLabel, 1, 0);
      layout->addWidget(toggleGaitBtn, 1, 1);
      layout->addWidget(calibrateBtn, 1, 2);
      layout->addWidget(pauseRobotBtn, 1, 3);
      layout->addWidget(resumeRobotBtn, 1, 4);
      setLayout(layout);

      pubTimer = new QTimer(this);
      connect(pubTimer, SIGNAL(timeout()), this, SLOT(publishMsg()));

#ifdef ROS_PUBLISHER_WAIT_FOR_SERVICES
      ROS_INFO_STREAM("Waiting for /aqua/calibrate service...");
      if (ros::service::waitForService("/aqua/calibrate")) {
        calibrateCln = nh.serviceClient<std_srvs::Empty>("/aqua/calibrate");
      }
      ROS_INFO_STREAM("... /aqua/calibrate service found!");

      ROS_INFO_STREAM("Waiting for /aqua/set_gait service...");
      if (ros::service::waitForService("/aqua/set_gait")) {
        setGaitCln = nh.serviceClient<aquacore::SetGait>("/aqua/set_gait");
      }
      ROS_INFO_STREAM("... /aqua/set_gait service found!");

      ROS_INFO_STREAM("Waiting for /aqua/pause service...");
      if (ros::service::waitForService("/aqua/pause")) {
        pauseCln = nh.serviceClient<aquacore::SetPauseMode>("/aqua/pause");
      }
      ROS_INFO_STREAM("... /aqua/pause service found!");
#else
      calibrateCln = nh.serviceClient<std_srvs::Empty>("/aqua/calibrate");
      setGaitCln = nh.serviceClient<aquacore::SetGait>("/aqua/set_gait");
      pauseCln = nh.serviceClient<aquacore::SetPauseMode>("/aqua/pause");
#endif

      spinTimer = new QTimer(this);
      connect(spinTimer, SIGNAL(timeout()), this, SLOT(spinOnce()));
      spinTimer->start(10);
    };


    const aquacore::PeriodicLegCommand& getSineCmdBuffer() { return sineCmd; };


    void handleBodyCmdFromROS(const aquacore::Command::ConstPtr& msg) {
      if (mode == MODE_SUB_BODY_CMD) {
        emit receivedNewBodyCmd(msg->speed, msg->heave, msg->roll, msg->pitch, msg->yaw);
      } else {
        ROS_WARN_STREAM("Received command on /aqua/command while in unexpected mode: " << mode);
      }
    };


    void handleSineCmdFromROS(const aquacore::PeriodicLegCommand::ConstPtr& msg) {
      if (mode == MODE_SUB_SINE_CMD) {
        for (int i = 0; i < 6; i++) {
          sineCmd.amplitudes[i] = msg->amplitudes[i];
          sineCmd.frequencies[i] = msg->frequencies[i];
          sineCmd.phase_offsets[i] = msg->phase_offsets[i];
          sineCmd.leg_offsets[i] = msg->leg_offsets[i];
          sineCmd.leg_velocities[i] = msg->leg_velocities[i];
        }
        emit receivedNewSineCmd();
      } else {
        ROS_WARN_STREAM("Received command on /aqua/periodic_leg_command while in unexpected mode: " << mode);
      }
    };


  public slots:
    void setMode(int newMode) {
      if (newMode < 0 || newMode >= NUM_MODES) {
        ROS_WARN_STREAM("Cannot set rosgui setMode to unknown mode: " << newMode);
        return;
      }

      int oldMode = mode;

      // Unhook existing publisher/subscriber
      if (oldMode == MODE_ARBT_SINE_CMD || oldMode == MODE_ARBT_BODY_CMD) {
        pubTimer->stop();
        rateValue->setText(QString::number(0, 'f', 1));
      }
      switch (oldMode) {
      case MODE_ARBT_BODY_CMD:
        bodyCmdPub.shutdown();
        break;
      case MODE_ARBT_SINE_CMD:
        sineCmdPub.shutdown();
        break;
      case MODE_SUB_BODY_CMD:
        bodyCmdSub.shutdown();
        break;
      case MODE_SUB_SINE_CMD:
        sineCmdSub.shutdown();
        break;
      default:
        break;
      }

      // Hook up new publisher/subscriber
      switch (newMode) {
      case MODE_ARBT_BODY_CMD:
        bodyCmdPub = nh.advertise<aquacore::Command>("/aqua/command", 1);
        break;
      case MODE_ARBT_SINE_CMD:
        sineCmdPub = nh.advertise<aquacore::PeriodicLegCommand>("/aqua/periodic_leg_command", 1);
        break;
      case MODE_SUB_BODY_CMD:
        bodyCmdSub = nh.subscribe("/aqua/command", 1, &ROSWidget::handleBodyCmdFromROS, this);
        break;
      case MODE_SUB_SINE_CMD:
        sineCmdSub = nh.subscribe("/aqua/periodic_leg_command", 1, &ROSWidget::handleSineCmdFromROS, this);
        break;
      default:
        ROS_ERROR_STREAM("rosgui attempting to set to unknown mode: " << newMode);
        break;
      }

      mode = newMode;
    };

    void setBodyCmd(double speed, double heave, double roll, double pitch, double yaw) {
      bodyCmd.speed = speed;
      bodyCmd.heave = heave;
      bodyCmd.roll = roll;
      bodyCmd.pitch = pitch;
      bodyCmd.yaw = yaw;
    };

    void newSineCmd(const PeriodicLegState_t& cmd) {
      for (int i = 0; i < 6; i++) {
        sineCmd.amplitudes[i] = cmd.amplitudes[i];
        sineCmd.frequencies[i] = cmd.frequencies[i];
        sineCmd.phase_offsets[i] = cmd.phase_offsets[i];
        sineCmd.leg_offsets[i] = cmd.leg_offsets[i];
        sineCmd.leg_velocities[i] = cmd.leg_velocities[i];
      }
    };

    void spinOnce() {
      ros::spinOnce();
    };

    void publishMsg() {
      if (mode == MODE_ARBT_SINE_CMD) { // FLEXIBLE_SINE
        sineCmd.header.stamp = ros::Time::now();
        sineCmdPub.publish(sineCmd);
      } else if (mode == MODE_ARBT_BODY_CMD) { // HOVER_MIDOFF
        bodyCmdPub.publish(bodyCmd);
      } else {
        ROS_WARN_STREAM("ros gui cannot publishMsg() in mode " << mode);
      }
    };

    void updatePubRate(const QString& newvalstr) {
      bool ok = false;
      double newval = newvalstr.toDouble(&ok);
      if (ok) {
        if (newval <= 0) {
          newval = 0;
          pubTimer->stop();
        } else {
          if (newval > 100) {
            newval = 100;
          }
          pubTimer->start(1000.0/newval);
        }
      } else {
        newval = 0;
        pubTimer->stop();
      }
      rateValue->setText(QString::number(newval, 'f', 1));
    };

    void requestSineGait(int sineGaitChecked) {
#ifdef SAFE_ROS_PUBLISHER
      if (pubTimer->isActive()) {
        ROS_WARN_STREAM("Cannot change gait while publishing commands");
        toggleGaitBtn->setChecked(!sineGaitChecked);
        return;
      }
#endif

      aquacore::SetGait srv;
      if (sineGaitChecked != 0) {
        srv.request.gait = "flexible-sine";
        toggleGaitBtn->setText("FLEXIBLE-SINE");
      } else {
        srv.request.gait = "hover-midoff";
        toggleGaitBtn->setText("HOVER-MIDOFF");
      }
      setGaitCln.call(srv);
    };

    void calibrate() {
#ifdef SAFE_ROS_PUBLISHER
      if (pubTimer->isActive()) {
        ROS_WARN_STREAM("Cannot calibrate while publishing commands");
        return;
      }
#endif

      std_srvs::Empty srv;
      calibrateCln.call(srv);
    };

    void pauseRobot() {
      requestSetPauseMode(true);
    };

    void resumeRobot() {
      requestSetPauseMode(false);
    };

    void requestSetPauseMode(bool pause) {
      aquacore::SetPauseMode srv;
      srv.request.value = pause;
      pauseCln.call(srv);
    };


signals:
  void receivedNewBodyCmd(double speed, double heave, double roll, double pitch, double yaw);
  void receivedNewSineCmd();


  protected:
    QLabel* rateLabel;
    QLineEdit* rateValue;

    QLabel* toggleGaitLabel;
    QCheckBox* toggleGaitBtn;

    QPushButton* calibrateBtn;

    QPushButton* pauseRobotBtn;
    QPushButton* resumeRobotBtn;

    QGridLayout* layout;

    QTimer* pubTimer;
    QTimer* spinTimer;

    int mode;

    ros::NodeHandle nh, local_nh;

    ros::Publisher bodyCmdPub;
    ros::Publisher sineCmdPub;
    ros::Subscriber bodyCmdSub;
    ros::Subscriber sineCmdSub;
    ros::ServiceClient calibrateCln;
    ros::ServiceClient setGaitCln;
    ros::ServiceClient pauseCln;

    aquacore::Command bodyCmd;
    aquacore::PeriodicLegCommand sineCmd;
};


class TestGaitsGUI : public QWidget {
Q_OBJECT
public:
  constexpr static bool USE_ANGULAR_DIST_CHECK = true;
  static double angleDiff(double a, double b) {
    double dist = (USE_ANGULAR_DIST_CHECK) ? angularDist(a, b) : fabs(a - b);
    if (dist > M_PI) { ROS_INFO_STREAM("angularMag(" << a << ", " << b << ") = " << angularMag(a, b)); }
    return dist;
  };

  constexpr static double UPDATE_RATE_MSEC = 20;

  constexpr static double TICK_MAX_LEG_POS_CHANGE = 10*2*M_PI; // rad/s
  constexpr static double TICK_MAX_LEG_VEL_CHANGE = TICK_MAX_LEG_POS_CHANGE*2*M_PI*4; // rad/s^2


  static double getSimTime() { return simTimeNow; };
  static double simTimeNow;


  static int sigintFd[2];
  static void sigintHandler(int unused) {
    char a = 1;
    ::write(sigintFd[0], &a, sizeof(a));
  };


  TestGaitsGUI() : gait(NULL), useSimTime(false), latestTickTime(0), mode(MODE_ARBT_BODY_CMD) {
    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, sigintFd)) {
      qFatal("Could not create SIGINT socketpair for Qt");
    }
    snSigint = new QSocketNotifier(sigintFd[1], QSocketNotifier::Read, this);
    connect(snSigint, SIGNAL(activated(int)), this, SLOT(handleSigint()));

    for (int i = 0; i < 6; i++) {
      latestTickLegAngles[i] = 0;
    }

    legs = new LegsWidget(this);
    legsBox = new QWidgetBox("Current Leg States", legs, this);

    sine = new PeriodicLegStateWidget(this);
    connect(sine, SIGNAL(stateUpdated()), this, SLOT(processUserSineCmd()));
    connect(this, SIGNAL(sineParamsReadOnly(bool)), sine, SLOT(setReadOnly(bool)));
    sineBox = new QWidgetBox("Target Periodic Leg Command", sine, this);

    speedValue = new QLineEdit("0", this);
    heaveValue = new QLineEdit("0", this);
    rollValue = new QLineEdit("0", this);
    pitchValue = new QLineEdit("0", this);
    yawValue = new QLineEdit("0", this);
    connect(speedValue, SIGNAL(textEdited(const QString&)), this, SLOT(updateSpeed(const QString&)));
    connect(heaveValue, SIGNAL(textEdited(const QString&)), this, SLOT(updateHeave(const QString&)));
    connect(rollValue, SIGNAL(textEdited(const QString&)), this, SLOT(updateRoll(const QString&)));
    connect(pitchValue, SIGNAL(textEdited(const QString&)), this, SLOT(updatePitch(const QString&)));
    connect(yawValue, SIGNAL(textEdited(const QString&)), this, SLOT(updateYaw(const QString&)));

    resetBodyCmdBtn = new QPushButton("Zero Body Cmd", this);
    connect(resetBodyCmdBtn, SIGNAL(pressed()), this, SLOT(resetBodyCmd()));

    amplitudeValue = new QLineEdit("20", this);
    frequencyValue = new QLineEdit("2.5", this);
    connect(amplitudeValue, SIGNAL(textEdited(const QString&)), this, SLOT(updateAmplitude(const QString&)));
    connect(frequencyValue, SIGNAL(textEdited(const QString&)), this, SLOT(updateFrequency(const QString&)));

    swimBackwards = new QCheckBox("Backwards", this);
    connect(swimBackwards, SIGNAL(stateChanged(int)), this, SLOT(updateForeAft(int)));

    bodyCmdBox = new QGroupBox("Target Body Command", this);
    bodyCmdLayout = new QGridLayout(bodyCmdBox);

    bodyCmdLayout->addWidget(new QLabel("Speed:", this), 0, 0);
    bodyCmdLayout->addWidget(new QLabel("Heave:", this), 0, 1);
    bodyCmdLayout->addWidget(new QLabel("Roll:", this),  0, 2);
    bodyCmdLayout->addWidget(new QLabel("Pitch:", this), 0, 3);
    bodyCmdLayout->addWidget(new QLabel("Yaw:", this),   0, 4);
    bodyCmdLayout->addWidget(speedValue, 1, 0);
    bodyCmdLayout->addWidget(heaveValue, 1, 1);
    bodyCmdLayout->addWidget(rollValue,  1, 2);
    bodyCmdLayout->addWidget(pitchValue, 1, 3);
    bodyCmdLayout->addWidget(yawValue,   1, 4);
    bodyCmdLayout->addWidget(new QLabel("Ampl (deg):", this), 2, 0);
    bodyCmdLayout->addWidget(new QLabel("Freq (Hz):", this), 2, 1);
    bodyCmdLayout->addWidget(swimBackwards, 2, 2);
    bodyCmdLayout->addWidget(resetBodyCmdBtn, 2, 4);
    bodyCmdLayout->addWidget(amplitudeValue, 3, 0);
    bodyCmdLayout->addWidget(frequencyValue, 3, 1);

    bodyCmdBox->setLayout(bodyCmdLayout);

    modeBox = new QComboBox(this);
    modeBox->addItem("Arbt. Body Cmd", MODE_ARBT_BODY_CMD);
    modeBox->addItem("Arbt. Sine Cmd", MODE_ARBT_SINE_CMD);
    modeBox->addItem("Sub /aqua/command", MODE_SUB_BODY_CMD);
    modeBox->addItem("Sub /aqua/[PLC]", MODE_SUB_SINE_CMD);
    modeBox->setCurrentIndex(mode);
    connect(modeBox, SIGNAL(currentIndexChanged(int)), this, SLOT(updateMode(int)));

    modeNextBtn = new QPushButton("Next Mode", this);
    connect(modeNextBtn, SIGNAL(pressed()), this, SLOT(nextMode()));

    simTimeChk = new QCheckBox("Sim Time", this);
    connect(simTimeChk, SIGNAL(stateChanged(int)), this, SLOT(toggleSimTime(int)));

    simTimeDelayValue = new QLineEdit("0.001", this);

    tickSimTimeBtn = new QPushButton("Step Sim", this);
    connect(tickSimTimeBtn, SIGNAL(pressed()), this, SLOT(tickSimTime()));

    resetSimTimeNowBtn = new QPushButton("Reset Sim Time", this);
    connect(resetSimTimeNowBtn, SIGNAL(pressed()), this, SLOT(resetSimTimeNow()));

    testGaitsBox = new QGroupBox("LOCAL CONTROLS", this);
    testGaitsLayout = new QGridLayout(testGaitsBox);

    testGaitsLayout->addWidget(modeBox, 0, 0);
    testGaitsLayout->addWidget(modeNextBtn, 0, 1);

    testGaitsLayout->addWidget(simTimeChk, 1, 0);
    testGaitsLayout->addWidget(new QLabel("Sim Time Delay:", this), 1, 1);
    testGaitsLayout->addWidget(simTimeDelayValue, 1, 2);
    testGaitsLayout->addWidget(tickSimTimeBtn, 1, 3);
    testGaitsLayout->addWidget(resetSimTimeNowBtn, 1, 4);

    testGaitsBox->setLayout(testGaitsLayout);

    rosgui = new ROSWidget(this);
    connect(this, SIGNAL(refreshBodyCmd(double, double, double, double, double)),
        rosgui, SLOT(setBodyCmd(double, double, double, double, double)));
    connect(rosgui, SIGNAL(receivedNewBodyCmd(double, double, double, double, double)),
        this, SLOT(handleNewBodyCmd(double, double, double, double, double)));
    connect(rosgui, SIGNAL(receivedNewSineCmd()),
        this, SLOT(handleNewSineCmdFromROS()));
    rosgui->newSineCmd(latestTickLegsCmd);
    rosguiBox = new QWidgetBox("ROS Publisher", rosgui, this);

    layout = new QGridLayout(this);
    layout->addWidget(legsBox, 0, 0);
    layout->addWidget(sineBox, 1, 0);
    layout->addWidget(bodyCmdBox, 2, 0);
    layout->addWidget(testGaitsBox, 3, 0);
    layout->addWidget(rosguiBox, 4, 0);
    setLayout(layout);

    resize(500, 800);

    ticker = new QTimer(this);
    connect(ticker, SIGNAL(timeout()), this, SLOT(tick()));

    toggleSimTime(useSimTime);
    mode = MODE_SUB_BODY_CMD;
    updateMode(MODE_ARBT_BODY_CMD);
  };


  virtual ~TestGaitsGUI() {};


public slots:
  void handleSigint() { close(); };

  void processUserSineCmd() {
    if (mode == MODE_ARBT_SINE_CMD) {
      userSineCmd = sine->getState();
      rosgui->newSineCmd(userSineCmd);
      gaitMutex.lock();
      gait->setPeriodicLegCmd(userSineCmd);
      gaitMutex.unlock();
    }
  };

  void resetBodyCmd() {
    gait->setBodyCmd(0, 0, 0, 0, 0);
    gait->setMaxAmplitudeRad(20.0/180.0*M_PI);
    gait->setFrequency(2.5);
    gait->foreaftControl(1.0);
    float speed = gait->getSpeedCmd();
    float heave = gait->getHeaveCmd();
    float roll = gait->getRollCmd();
    float pitch = gait->getPitchCmd();
    float yaw = gait->getYawCmd();
    speedValue->setText(QString::number(speed, 'f', 4));
    heaveValue->setText(QString::number(heave, 'f', 4));
    rollValue->setText(QString::number(roll, 'f', 4));
    pitchValue->setText(QString::number(pitch, 'f', 4));
    yawValue->setText(QString::number(yaw, 'f', 4));
    emit refreshBodyCmd(speed, heave, roll, pitch, yaw);
    amplitudeValue->setText(QString::number(gait->getMaxAmplitudeRad()/M_PI*180.0, 'f', 4));
    frequencyValue->setText(QString::number(gait->getFrequency(), 'f', 4));
    swimBackwards->setChecked((gait->getForeAftDirection() == -1));
  };

  void handleNewSineCmdFromROS() {
    if (mode != MODE_SUB_SINE_CMD) {
      ROS_WARN_STREAM("handleNewSineCmdFromROS() called on unexpected mode: " << mode);
      return;
    }

    const aquacore::PeriodicLegCommand& roscmd = rosgui->getSineCmdBuffer();
    for (int i = 0; i < 6; i++) {
      userSineCmd.amplitudes[i] = roscmd.amplitudes[i];
      userSineCmd.frequencies[i] = roscmd.frequencies[i];
      userSineCmd.phase_offsets[i] = roscmd.phase_offsets[i];
      userSineCmd.leg_offsets[i] = roscmd.leg_offsets[i];
      userSineCmd.leg_velocities[i] = roscmd.leg_velocities[i];
    }

    rosgui->newSineCmd(userSineCmd);
    sine->updateStateDisplay(userSineCmd);

    gaitMutex.lock();
    gait->setPeriodicLegCmd(userSineCmd);
    gaitMutex.unlock();
  };

  void handleNewBodyCmd(double speed, double heave, double roll, double pitch, double yaw) {
    if (mode != MODE_SUB_BODY_CMD) {
      ROS_WARN_STREAM("handleNewBodyCmd() called on unexpected mode: " << mode);
      return;
    }

    gaitMutex.lock();
    gait->setBodyCmd(speed, heave, roll, pitch, yaw);
    speed = gait->getSpeedCmd();
    heave = gait->getHeaveCmd();
    roll = gait->getRollCmd();
    pitch = gait->getPitchCmd();
    yaw = gait->getYawCmd();
    gaitMutex.unlock();

    speedValue->setText(QString::number(speed, 'f', 4));
    heaveValue->setText(QString::number(heave, 'f', 4));
    rollValue->setText(QString::number(roll, 'f', 4));
    pitchValue->setText(QString::number(pitch, 'f', 4));
    yawValue->setText(QString::number(yaw, 'f', 4));
  };

  void updateSpeed(const QString& newvalstr) {
    bool ok = false;
    double newval = newvalstr.toDouble(&ok);
    if (ok) {
      gaitMutex.lock();
      gait->setSpeedCmd(newval);
      gaitMutex.unlock();
      announceNewBodyCmd();
    }
  };

  void updateHeave(const QString& newvalstr) {
    bool ok = false;
    double newval = newvalstr.toDouble(&ok);
    if (ok) {
      gaitMutex.lock();
      gait->heaveControl(newval);
      gaitMutex.unlock();
      announceNewBodyCmd();
    }
  };

  void updateRoll(const QString& newvalstr) {
    bool ok = false;
    double newval = newvalstr.toDouble(&ok);
    if (ok) {
      gaitMutex.lock();
      gait->rollControl(newval);
      gaitMutex.unlock();
      announceNewBodyCmd();
    }
  };

  void updatePitch(const QString& newvalstr) {
    bool ok = false;
    double newval = newvalstr.toDouble(&ok);
    if (ok) {
      gaitMutex.lock();
      gait->pitchControl(newval);
      gaitMutex.unlock();
      announceNewBodyCmd();
    }
  };

  void updateYaw(const QString& newvalstr) {
    bool ok = false;
    double newval = newvalstr.toDouble(&ok);
    if (ok) {
      gaitMutex.lock();
      gait->yawControl(newval);
      gaitMutex.unlock();
      announceNewBodyCmd();
    }
  };

  void updateAmplitude(const QString& newvalstr) {
    bool ok = false;
    double newval = newvalstr.toDouble(&ok);
    if (ok) {
      gaitMutex.lock();
      gait->setMaxAmplitudeRad(newval/180.0*M_PI);
      gaitMutex.unlock();
    }
  };

  void updateFrequency(const QString& newvalstr) {
    bool ok = false;
    double newval = newvalstr.toDouble(&ok);
    if (ok) {
      gaitMutex.lock();
      gait->setFrequency(newval);
      gaitMutex.unlock();
    }
  };

  void updateForeAft(int backwardsChecked) {
    gaitMutex.lock();
    gait->foreaftControl((backwardsChecked != 0) ? -1 : 1);
    gaitMutex.unlock();
  };

  void nextMode() {
    modeBox->setCurrentIndex((mode+1) % NUM_MODES);
  };

  void updateMode(int newMode) {
    if (newMode < 0 || newMode > NUM_MODES) {
      modeBox->setCurrentIndex(mode);
      return;
    }
    if (mode == newMode) return;

    int oldMode = mode;
    bool arbtBodyCmdMode = (newMode == MODE_ARBT_BODY_CMD);
    bool arbtSineCmdMode = (newMode == MODE_ARBT_SINE_CMD);
    bool oldModeArbt = ((oldMode == MODE_ARBT_BODY_CMD) || (oldMode == MODE_ARBT_SINE_CMD));
    bool newModeArbt = ((newMode == MODE_ARBT_BODY_CMD) || (newMode == MODE_ARBT_SINE_CMD));

    if (oldModeArbt && !newModeArbt) {
      simTimeDelayValue->setEnabled(false);
      tickSimTimeBtn->setEnabled(false);
      resetSimTimeNowBtn->setEnabled(false);
    }
    spinMutex.lock();

    speedValue->setReadOnly(!arbtBodyCmdMode);
    heaveValue->setReadOnly(!arbtBodyCmdMode);
    rollValue->setReadOnly(!arbtBodyCmdMode);
    pitchValue->setReadOnly(!arbtBodyCmdMode);
    yawValue->setReadOnly(!arbtBodyCmdMode);
    amplitudeValue->setReadOnly(!arbtBodyCmdMode);
    frequencyValue->setReadOnly(!arbtBodyCmdMode);
    swimBackwards->setEnabled(arbtBodyCmdMode);

    emit sineParamsReadOnly(!arbtSineCmdMode);

    if (arbtBodyCmdMode) {
      updateSpeed(speedValue->text());
      updateHeave(heaveValue->text());
      updateRoll(rollValue->text());
      updatePitch(pitchValue->text());
      updateYaw(yawValue->text());
      updateAmplitude(amplitudeValue->text());
      updateFrequency(frequencyValue->text());
      updateForeAft(swimBackwards->isChecked());
    }

    if (arbtSineCmdMode) {
      userSineCmd = sine->getState();
    }

    mode = newMode;

    spinMutex.unlock();
    if (!oldModeArbt && newModeArbt) {
      simTimeDelayValue->setEnabled(true);
      tickSimTimeBtn->setEnabled(true);
      resetSimTimeNowBtn->setEnabled(true);
    }

    if (newMode == MODE_SUB_BODY_CMD) {
      // Reset local fields
      gaitMutex.lock();
      gait->setBodyCmd(0, 0, 0, 0, 0);
      gaitMutex.unlock();
      float speed = gait->getSpeedCmd();
      float heave = gait->getHeaveCmd();
      float roll = gait->getRollCmd();
      float pitch = gait->getPitchCmd();
      float yaw = gait->getYawCmd();
      speedValue->setText(QString::number(speed, 'f', 4));
      heaveValue->setText(QString::number(heave, 'f', 4));
      rollValue->setText(QString::number(roll, 'f', 4));
      pitchValue->setText(QString::number(pitch, 'f', 4));
      yawValue->setText(QString::number(yaw, 'f', 4));
      emit refreshBodyCmd(speed, heave, roll, pitch, yaw);
    }

    if (!newModeArbt) {
      PeriodicLegState_t nullSineCmd;
      rosgui->newSineCmd(nullSineCmd);
      sine->updateStateDisplay(nullSineCmd);
      gaitMutex.lock();
      gait->setPeriodicLegCmd(nullSineCmd);
      gaitMutex.unlock();
    }

    if (newMode == MODE_SUB_SINE_CMD || newMode == MODE_SUB_BODY_CMD) {
      rosSubModeStartTime = ros::Time::now();
      latestTickTime = -1;
      toggleSimTime(0);
    }

    modeBox->setCurrentIndex(mode);
    rosgui->setMode(mode);

    // TODO: fix bug: in arbt body cmd mode, when going into sim time, thne step sim, legs do not move
  };

  inline void announceNewBodyCmd() {
    gaitMutex.lock();
    float speed = gait->getSpeedCmd();
    float heave = gait->getHeaveCmd();
    float roll = gait->getRollCmd();
    float pitch = gait->getPitchCmd();
    float yaw = gait->getYawCmd();
    gaitMutex.unlock();

    emit refreshBodyCmd(speed, heave, roll, pitch, yaw);
  };

  void toggleSimTime(int simTimeChecked) {
    bool recreateGait = true;
    if (!(mode == MODE_ARBT_SINE_CMD || mode == MODE_ARBT_BODY_CMD)) {
      recreateGait = false;
    }

    // Stop current spinner, and wait to have sole access to gait
    ticker->stop();
    spinMutex.lock();
    gaitMutex.lock();
    useSimTime = (simTimeChecked != 0);

    // Re-create new gait object with either our own time fn, or use the default time fn (for real time)
    if (recreateGait) {
      if (gait != NULL) {
        delete gait;
      }
      if (useSimTime) {
        simTimeNow = latestTickTime;
        legs->setTime(latestTickTime);
        gait = new UnderwaterSwimmerGait(&TestGaitsGUI::getSimTime);
      } else {
        gait = new UnderwaterSwimmerGait();
      }
      gait->activate();

      // Configure newly created gait object with current GUI states, then update GUI to reflect gait's state
      gait->setBodyCmd(
          speedValue->text().toDouble(),
          heaveValue->text().toDouble(),
          rollValue->text().toDouble(),
          pitchValue->text().toDouble(),
          yawValue->text().toDouble());

      gait->setMaxAmplitudeRad(amplitudeValue->text().toDouble()/180.0*M_PI);
      gait->setFrequency(frequencyValue->text().toDouble());
      gait->foreaftControl(swimBackwards->isChecked() ? -1 : 1);
    }

    float speed = gait->getSpeedCmd();
    float heave = gait->getHeaveCmd();
    float roll = gait->getRollCmd();
    float pitch = gait->getPitchCmd();
    float yaw = gait->getYawCmd();
    speedValue->setText(QString::number(speed, 'f', 4));
    heaveValue->setText(QString::number(heave, 'f', 4));
    rollValue->setText(QString::number(roll, 'f', 4));
    pitchValue->setText(QString::number(pitch, 'f', 4));
    yawValue->setText(QString::number(yaw, 'f', 4));
    emit refreshBodyCmd(speed, heave, roll, pitch, yaw);

    amplitudeValue->setText(QString::number(gait->getMaxAmplitudeRad()/M_PI*180.0, 'f', 4));
    frequencyValue->setText(QString::number(gait->getFrequency(), 'f', 4));
    swimBackwards->setChecked((gait->getForeAftDirection() == -1));

    // Release lock on gait object and spin thread, and restart spin thread if in real time
    gaitMutex.unlock();
    spinMutex.unlock();
    simTimeDelayValue->setEnabled(useSimTime);
    tickSimTimeBtn->setEnabled(useSimTime);
    resetSimTimeNowBtn->setEnabled(useSimTime);
    if (!useSimTime) {
      dt.start();
      ticker->start(UPDATE_RATE_MSEC);
    }

    // Update checkbox
    disconnect(simTimeChk, 0, 0, 0);
    simTimeChk->setChecked(useSimTime);
    connect(simTimeChk, SIGNAL(stateChanged(int)), this, SLOT(toggleSimTime(int)));
  };

  void resetSimTimeNow() {
    if (useSimTime) {
      simTimeNow = 0;
      latestTickTime = 0;
      legs->setTime(latestTickTime);
    }
  };

  void tickSimTime() {
    if (!useSimTime) {
      ROS_WARN_STREAM("tickSimTime() called when not in sim time mode!");
      return;
    }

    bool ok;
    double simDT = simTimeDelayValue->text().toDouble(&ok);
    if (!ok || simDT <= 0) {
      simDT = 0.001;
      simTimeDelayValue->setText(QString::number(simDT, 'f', 4));
    }
    simTimeNow += simDT;

    tick();
  };

  void tick() {
    spinMutex.lock();

    if (mode == MODE_ARBT_SINE_CMD || mode == MODE_SUB_SINE_CMD) {
      // Check sanity of currLegsCmd
      for (int i = 0; i < 6; i++) {
        if (fabs(userSineCmd.amplitudes[i]) > 10) {
          ROS_WARN_STREAM("Abnormal amplitude requested from user!\n- leg " <<
              i << ":\n - amplitude = " << userSineCmd.amplitudes[i]);
        }
        if (fabs(userSineCmd.leg_offsets[i]) > 10) {
          ROS_WARN_STREAM("Abnormal leg offset requested from user!\n- leg " <<
              i << ":\n - leg_offset = " << userSineCmd.leg_offsets[i]);
        }
      }

      // Issue current target command from user
      gaitMutex.lock();
      gait->setPeriodicLegCmd(userSineCmd);
      gaitMutex.unlock();
    } else if (mode == MODE_ARBT_BODY_CMD || mode == MODE_SUB_BODY_CMD) {
      // Obtain updated sinusoidal command
      gaitMutex.lock();
      gait->updateSineCmd(latestTickLegsCmd);
      gaitMutex.unlock();

      // Check sanity of currLegsCmd
      for (int i = 0; i < 6; i++) {
        if (fabs(latestTickLegsCmd.amplitudes[i]) > 10) {
          ROS_WARN_STREAM("Abnormal amplitude requested!\n- leg " <<
              i << ":\n - amplitude = " << latestTickLegsCmd.amplitudes[i]);
        }
        if (fabs(latestTickLegsCmd.leg_offsets[i]) > 10) {
          ROS_WARN_STREAM("Abnormal leg offset requested!\n- leg " <<
              i << ":\n - leg_offset = " << latestTickLegsCmd.leg_offsets[i]);
        }
      }

      // Issue new target command
      sine->updateStateDisplay(latestTickLegsCmd);
      rosgui->newSineCmd(latestTickLegsCmd);
      gaitMutex.lock();
      gait->setPeriodicLegCmd(latestTickLegsCmd);
      gaitMutex.unlock();
    } else {
      ROS_WARN_STREAM("tick() running in unexpected mode: " << mode);
    }

    // Obtain updated motor target command
    gaitMutex.lock();
    gait->updateMotorTarget(currMotorTargets);
    gaitMutex.unlock();

    // Check sanity of currMotorTargets
    double newTickTime = simTimeNow;
    if (!useSimTime) {
      if (mode == MODE_ARBT_BODY_CMD || mode == MODE_ARBT_SINE_CMD) {
        newTickTime = dt.elapsed()/1000.0;
      } else {
        newTickTime = (ros::Time::now() - rosSubModeStartTime).toSec();
      }
    }
    double dtCheck = newTickTime - latestTickTime;
    if (dtCheck == 0) { dtCheck = 0.001; } // We know that it's impossible that tick() twice with no delay; this is probably because Qt Timer's resolution is too poor
    for (int i = 0; i < 6; i++) {
      // Check for excessive motor movement
      double estLegPosRate = angleDiff(currMotorTargets[i].pos, latestTickLegAngles[i])/dtCheck;
      if (estLegPosRate > TICK_MAX_LEG_POS_CHANGE) {
        if (currMotorTargets[i].vel > TICK_MAX_LEG_POS_CHANGE) {
          ROS_ERROR_STREAM("Abnormally high leg position change & target velocity requested!\n- leg " <<
              i << ":\n  - from " << latestTickLegAngles[i] << "\n  - to " <<
              currMotorTargets[i].pos << "\n  - within dt = " << dtCheck <<
              "\n  - ang diff: " << angleDiff(currMotorTargets[i].pos, latestTickLegAngles[i]) <<
              "\n  - est. velocity: " << estLegPosRate << "\n  - thresh: " << TICK_MAX_LEG_POS_CHANGE);
        } else {
          ROS_WARN_STREAM("Abnormally high leg position change requested. (target velocity still within bounds)\n- leg " <<
              i << ":\n  - from " << latestTickLegAngles[i] << "\n  - to " <<
              currMotorTargets[i].pos << "\n  - within dt = " << dtCheck <<
              "\n  - ang diff: " << angleDiff(currMotorTargets[i].pos, latestTickLegAngles[i]) <<
              "\n  - est. velocity: " << estLegPosRate << "\n  - thresh: " << TICK_MAX_LEG_POS_CHANGE);
        }
      }

      // Check for excessive motor acceleration
      double estLegVelRate = fabs(currMotorTargets[i].vel - latestMotorTargets[i].vel)/dtCheck;
      if (estLegVelRate > TICK_MAX_LEG_VEL_CHANGE) {
        ROS_ERROR_STREAM("Abnormally high leg velocity change requested!\n- leg " <<
            i << ":\n  - from " << latestTickLegAngles[i] << "\n  - to " <<
            currMotorTargets[i].pos << "\n  - w/ old vel " << latestMotorTargets[i].vel <<
            "\n  - to new vel " << currMotorTargets[i].vel <<
            "\n  - within dt = " << dtCheck <<
            "\n  - vel diff: " << fabs(currMotorTargets[i].vel - latestMotorTargets[i].vel) <<
            "\n  - est. acceleration: " << estLegVelRate << "\n  - thres: " << TICK_MAX_LEG_VEL_CHANGE);
      }

      latestTickLegAngles[i] = currMotorTargets[i].pos;
      latestMotorTargets[i] = currMotorTargets[i];
    }

    // Implement latest motor target commands
    latestTickTime = newTickTime;
    legs->setTime(latestTickTime);
    legs->setLegAngles(latestTickLegAngles);

    spinMutex.unlock();
  };


signals:
  void sineParamsReadOnly(bool sineMode);
  void refreshBodyCmd(double speed, double heave, double roll, double pitch, double yaw);

protected:
  LegsWidget* legs;
  QWidgetBox* legsBox;

  PeriodicLegStateWidget* sine;
  QWidgetBox* sineBox;

  ROSWidget* rosgui;
  QWidgetBox* rosguiBox;

  QLineEdit* speedValue;
  QLineEdit* heaveValue;
  QLineEdit* rollValue;
  QLineEdit* pitchValue;
  QLineEdit* yawValue;
  QPushButton* resetBodyCmdBtn;

  QGroupBox* bodyCmdBox;
  QGridLayout* bodyCmdLayout;

  QLineEdit* amplitudeValue;
  QLineEdit* frequencyValue;
  QCheckBox* swimBackwards;

  QComboBox* modeBox;
  QPushButton* modeNextBtn;

  QCheckBox* simTimeChk;
  QLineEdit* simTimeDelayValue;
  QPushButton* tickSimTimeBtn;
  QPushButton* resetSimTimeNowBtn;

  QGroupBox* testGaitsBox;
  QGridLayout* testGaitsLayout;

  QGridLayout* layout;

  UnderwaterSwimmerGait* gait;
  QMutex gaitMutex;
  MotorTarget_t currMotorTargets[6];
  MotorTarget_t latestMotorTargets[6];

  QMutex spinMutex;
  QTimer* ticker;
  QElapsedTimer dt;
  bool useSimTime;

  PeriodicLegState_t latestTickLegsCmd;
  PeriodicLegState_t userSineCmd;
  float latestTickLegAngles[6];
  double latestTickTime;

  int mode;
  ros::Time rosSubModeStartTime;

  QSocketNotifier* snSigint;
};


#endif // QCLASSES_
