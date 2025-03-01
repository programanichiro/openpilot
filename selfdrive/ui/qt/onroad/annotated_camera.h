#pragma once

#include <QVBoxLayout>
#include <memory>
#include "selfdrive/ui/qt/onroad/hud.h"
#include "selfdrive/ui/qt/onroad/buttons.h"
#include "selfdrive/ui/qt/onroad/driver_monitoring.h"
#include "selfdrive/ui/qt/onroad/model.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"

class AnnotatedCameraWidget : public CameraWidget {
  Q_OBJECT

public:
  explicit AnnotatedCameraWidget(VisionStreamType type, QWidget* parent = 0);
  void updateState(const UIState &s);
  MapSettingsButton *map_settings_btn;

private:
  QVBoxLayout *main_layout;
  ExperimentalButton *experimental_btn;
  DriverMonitorRenderer dmon;
  HudRenderer hud;
  ButtonsWindow *buttons;
  ModelRenderer model;
  std::unique_ptr<PubMaster> pm;

  int skip_frame_count = 0;
  bool wide_cam_requested = false;
  int drawTextLeft(QPainter &p, int x, int y, const QString &text, int alpha = 255 , bool brakeLight = false , int red=255, int blu=255, int grn=255 , int bk_red=0, int bk_blu=0, int bk_grn=0, int bk_alp=0, int bk_yofs=0, int bk_corner_r=0 , int bk_add_w=0, int bk_xofs=0 , int bk_add_h=0);

protected:
  void paintGL() override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  mat4 calcFrameMatrix() override;

  double prev_draw_t = 0;
  FirstOrderFilter fps_filter;
};
