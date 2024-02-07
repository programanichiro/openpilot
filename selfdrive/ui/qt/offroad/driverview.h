#pragma once

#include "selfdrive/ui/qt/widgets/cameraview.h"

class DriverViewWindow : public CameraWidget {
  Q_OBJECT

public:
  explicit DriverViewWindow(QWidget *parent);

signals:
  void done();
  void done2();

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  void paintGL() override;
  void mini_knightScanner(QPainter &p);

  Params params;
  QPixmap face_img;
};
