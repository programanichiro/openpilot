#pragma once

#include <memory>

#include <QPushButton>
#include <QStackedLayout>
#include <QWidget>
#include <QPushButton>

#include "common/util.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"


const int btn_size = 192;
const int img_size = (btn_size / 4) * 3;


// ***** onroad widgets *****
class OnroadAlerts : public QWidget {
  Q_OBJECT

public:
  OnroadAlerts(QWidget *parent = 0) : QWidget(parent) {}
  void updateAlert(const Alert &a);

protected:
  void paintEvent(QPaintEvent*) override;

private:
  QColor bg;
  Alert alert = {};
};

class MapSettingsButton;
class ButtonsWindow : public QWidget {
  Q_OBJECT

public:
  ButtonsWindow(QWidget *parent = 0 , MapSettingsButton *map_settings_btn = 0);
  void psn_update();
  void MAX_touch();

private:
  QPushButton *lockOnButton;
  QPushButton *accelCtrlButton;
  QPushButton *decelCtrlButton;
  QPushButton *accelEngagedButton;
  QPushButton *LTA_EnableButton;
  QPushButton *startAccelPowerUpButton;
  QPushButton *useDynmicExpButton;
  QPushButton *T1_Button; //⚫︎⚪︎⬇︎
  QPushButton *T3_Button; //⬆︎⬆︎⬆︎

  // int dfStatus = -1;  // always initialize style sheet and send msg
  // const QStringList dfButtonColors = {"#044389", "#24a8bc", "#fcff4b", "#37b868"};

  // int lsStatus = -1;  // always initialize style sheet and send msg
  // const QStringList lsButtonColors = {"#ff3737", "#37b868", "#044389"};

  const QStringList mButtonColors = {"#909090", "#37b868"};

  // model long button
  bool mLockOnButton = true;  // triggers initialization
  bool mAccelCtrlButton = true;  // triggers initialization
  bool mDecelCtrlButton = true;  // triggers initialization
  int mAccelEngagedButton = 0;  // triggers initialization
  int mLTA_EnableButton = 0;  // triggers initialization
  bool mStartAccelPowerUpButton = false;  // triggers initialization
  int mUseDynmicExpButton = 0;  // triggers initialization

public slots:
  void updateState(const UIState &s);
};

class ExperimentalButton : public QPushButton {
  Q_OBJECT

public:
  explicit ExperimentalButton(QWidget *parent = 0);
  void updateState(const UIState &s);

private:
  void paintEvent(QPaintEvent *event) override;
  void changeMode();

  Params params;
  QPixmap engage_img;
  QPixmap experimental_img;
  bool experimental_mode;
  bool engageable;
};


class MapSettingsButton : public QPushButton {
  Q_OBJECT

public:
  explicit MapSettingsButton(QWidget *parent = 0);

private:
  void paintEvent(QPaintEvent *event) override;

  QPixmap settings_img;
};

// container window for the NVG UI
class AnnotatedCameraWidget : public CameraWidget {
  Q_OBJECT

public:
  explicit AnnotatedCameraWidget(VisionStreamType type, QWidget* parent = 0);
  void updateState(const UIState &s);

  MapSettingsButton *map_settings_btn;

private:
  void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255 , bool brakeLight = false);
  void drawText(QPainter &p, int x, int y, const QString &text, const QColor &col);
  void my_drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity , float ang);
  int drawTextLeft(QPainter &p, int x, int y, const QString &text, int alpha = 255 , bool brakeLight = false , int red=255, int blu=255, int grn=255);
  int drawTextRight(QPainter &p, int x, int y, const QString &text, int alpha = 255 , bool brakeLight = false , int red=255, int blu=255, int grn=255 , int bk_red=0, int bk_blu=0, int bk_grn=0, int bk_alp=0, int bk_yofs=0/*塗りつぶしを縦にずらす*/);

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

//signals:
//  void resizeSignal(int w);
};

// container for all onroad widgets
class OnroadWindow : public QWidget {
  Q_OBJECT

public:
  OnroadWindow(QWidget* parent = 0);
  bool isMapVisible() const { return map && map->isVisible(); }
  void showMapPanel(bool show) { if (map) map->setVisible(show); }

signals:
  void mapPanelRequested();
  void driverViewOn();
  void driverViewOff();

private:
  void paintEvent(QPaintEvent *event);
  void mousePressEvent(QMouseEvent* e) override;
  OnroadAlerts *alerts;
  AnnotatedCameraWidget *nvg;
  float mb_width_rate = 0;
  QColor bg = bg_colors[STATUS_DISENGAGED];
  QWidget *map = nullptr;
  QHBoxLayout* split;

private slots:
  void offroadTransition(bool offroad);
  void primeChanged(bool prime);
  void updateState(const UIState &s);
};
