#include "aqua_gait/QTestGaitsGUI.hpp"
#include <QtGui>
#include <QApplication>
#include <csignal>


double TestGaitsGUI::simTimeNow = 0;
int TestGaitsGUI::sigintFd[2] = {0, 0};


using namespace std;


int main(int argc, char** argv) {
  ros::init(argc, argv, "test_gaits_gui", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  QApplication app( argc, argv );
  TestGaitsGUI gui;
  gui.show();
  struct sigaction sigint;
  sigint.sa_handler = TestGaitsGUI::sigintHandler;
  sigemptyset(&sigint.sa_mask);
  sigint.sa_flags = 0;
  sigint.sa_flags |= SA_RESTART;
  sigaction(SIGINT, &sigint, 0);

  return app.exec();
};
