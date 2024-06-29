#pragma once

#include <QVBoxLayout>
#include <memory>

#include "selfdrive/ui/qt/onroad/buttons.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"

class AnnotatedCameraWidget : public CameraWidget {
  Q_OBJECT

public:
  explicit AnnotatedCameraWidget(VisionStreamType type, QWidget* parent = 0);
  void updateState(const UIState &s);

  MapSettingsButton *map_settings_btn;

private:
  int drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255 , bool brakeLight = false);
  int drawText(QPainter &p, int x, int y, const QString &text, const QColor &col);
  int drawTextCircleCenter(QPainter &p, int x, int y, const QString &text, const QColor &col); //円の中央とxを共有したいときに使いやすいので残す。
  void my_drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity , float ang);
  int drawTextLeft(QPainter &p, int x, int y, const QString &text, int alpha = 255 , bool brakeLight = false , int red=255, int blu=255, int grn=255 , int bk_red=0, int bk_blu=0, int bk_grn=0, int bk_alp=0, int bk_yofs=0, int bk_corner_r=0 , int bk_add_w=0, int bk_xofs=0 , int bk_add_h=0);
  int drawTextRight(QPainter &p, int x, int y, const QString &text, int alpha = 255 , bool brakeLight = false , int red=255, int blu=255, int grn=255 , int bk_red=0, int bk_blu=0, int bk_grn=0, int bk_alp=0, int bk_yofs=0, int bk_corner_r=0 , int bk_add_w=0, int bk_xofs=0);
  int drawTextCenter(QPainter &p, int x, int y, const QString &text, int alpha = 255 , bool brakeLight = false , int red=255, int blu=255, int grn=255 , int bk_red=0, int bk_blu=0, int bk_grn=0, int bk_alp=0, int bk_yofs=0, int bk_corner_r=0 , int bk_add_w=0, int bk_xofs=0);

  ButtonsWindow *buttons;
  QPixmap engage_img;
  QPixmap experimental_img;
  QVBoxLayout *main_layout;
  ExperimentalButton *experimental_btn;
  QPixmap dm_img;
  float speed;
  QString speedUnit;
  QString maxSpeed;
  float setSpeed;
  float speedLimit;
  bool is_cruise_set = false;
  bool is_metric = false;
  bool dmActive = false;
  bool hideBottomIcons = false;
  bool rightHandDM = false;
  float dm_fade_state = 1.0;
  bool has_us_speed_limit = false;
  bool has_eu_speed_limit = false;
  bool v_ego_cluster_seen = false;
  int status = STATUS_DISENGAGED;
  std::unique_ptr<PubMaster> pm;

  int skip_frame_count = 0;
  bool wide_cam_requested = false;

protected:
  void paintGL() override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  void updateFrameMat() override;
  void drawLaneLines(QPainter &painter, const UIState *s);
  void drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd , int num);
  void drawLockon(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd , int num  /*使っていない, size_t leads_num , const cereal::RadarState::LeadData::Reader &lead0, const cereal::RadarState::LeadData::Reader &lead1 */);
  void drawHud(QPainter &p);
  void drawDriverState(QPainter &painter, const UIState *s);
  inline QColor redColor(int alpha = 255) { return QColor(201, 34, 49, alpha); }
  inline QColor whiteColor(int alpha = 255) { return QColor(255, 255, 255, alpha); }
  inline QColor blackColor(int alpha = 255) { return QColor(0, 0, 0, alpha); }

  double prev_draw_t = 0;
  FirstOrderFilter fps_filter;
  void knightScanner(QPainter &p);
};
