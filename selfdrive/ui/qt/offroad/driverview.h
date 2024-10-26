#pragma once

#include "selfdrive/ui/qt/widgets/cameraview.h"
#include "selfdrive/ui/qt/onroad/driver_monitoring.h"
#include "selfdrive/ui/qt/widgets/input.h"

class DriverViewWindow : public CameraWidget {
  Q_OBJECT
public:
  explicit DriverViewWindow(QWidget *parent);
  void paintGL() override;
  mat4 calcFrameMatrix() override;
  DriverMonitorRenderer driver_monitor;

  void mini_knightScanner(QPainter &p);

public:
  DriverViewWindow(QWidget* parent , int myMethod);

signals:
  void done();

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  int my_method;
  Params params;
};

class DriverViewDialog : public DialogBase {
  Q_OBJECT
public:
  DriverViewDialog(QWidget *parent);
  void done(int r) override;
};
