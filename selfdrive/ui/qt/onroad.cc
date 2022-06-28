#include "selfdrive/ui/qt/onroad.h"

#include <cmath>

#include <QDebug>

#include "common/timing.h"
#include "selfdrive/ui/qt/util.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#endif

#define PI0_DEBUG false

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(bdr_s);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  nvg = new NvgWindow(VISION_STREAM_ROAD, this);

  buttons = new ButtonsWindow(this);
  QObject::connect(uiState(), &UIState::uiUpdate, buttons, &ButtonsWindow::updateState);
  QObject::connect(nvg, &NvgWindow::resizeSignal, [=](int w){
    buttons->setFixedWidth(w);
  });
  stacked_layout->addWidget(buttons);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addWidget(nvg);

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);
}

void OnroadWindow::updateState(const UIState &s) {
  QColor bgColor = bg_colors[s.status];
  Alert alert = Alert::get(*(s.sm), s.scene.started_frame);
  if (s.sm->updated("controlsState") || !alert.equal({})) {
    if (alert.type == "controlsUnresponsive") {
      bgColor = bg_colors[STATUS_ALERT];
    } else if (alert.type == "controlsUnresponsivePermanent") {
      bgColor = bg_colors[STATUS_DISENGAGED];
    }
    alerts->updateAlert(alert, bgColor);
  }

  nvg->updateState(s);

  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
  if (map != nullptr) {
    bool sidebarVisible = geometry().x() > 0;
    map->setVisible(!sidebarVisible && !map->isVisible());
  }
  // propagation event to parent(HomeWindow)
  QWidget::mousePressEvent(e);
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    if (map == nullptr && (uiState()->prime_type || !MAPBOX_TOKEN.isEmpty())) {
      MapWindow * m = new MapWindow(get_mapbox_settings());
      map = m;

      QObject::connect(uiState(), &UIState::offroadTransition, m, &MapWindow::offroadTransition);

      m->setFixedWidth(topWidget(this)->width() / 2);
      split->addWidget(m, 0, Qt::AlignRight);

      // Make map visible after adding to split
      m->offroadTransition(offroad);
    }
  }
#endif

  alerts->updateAlert({}, bg);

  // update stream type
  bool wide_cam = Hardware::TICI() && Params().getBool("EnableWideCamera");
  nvg->setStreamType(wide_cam ? VISION_STREAM_WIDE_ROAD : VISION_STREAM_ROAD);
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));
}

// ***** onroad widgets *****
bool getButtonEnabled(const char*fn){ //fn="../manager/lockon_disp_disable.txt"など、このファイルが無かったらtrueのニュアンスで。
  std::string txt = util::read_file(fn);
  if(txt.empty() == false){
    if ( txt == "0" ) {
      return true; //ファイルが無効値なのでtrue
    } else {
      return false; //ファイルが有効値なのでfalse
    }
  } else {
    return true; //ファイルがなければtrue
  }
}

bool getButtonEnabled0(const char*fn){ //旧fn="../manager/accel_engaged.txt"など、このファイルが無かったらfalseのニュアンスで。
  std::string txt = util::read_file(fn);
  if(txt.empty() == false){
    if ( txt == "0" ) {
      return false; //ファイルが無効値なのでfalse
    } else {
      return true; //ファイルが有効値なのでtrue
    }
  } else {
    return false; //ファイルがなければfalse
  }
}

int getButtonInt(const char*fn , int defaultNum){ //新fn="../manager/accel_engaged.txt"など、このファイルが無かったらdefaultNum。あとは数字に変換してそのまま返す。
  std::string txt = util::read_file(fn);
  if(txt.empty() == false){
    return std::stoi(txt);
  }
  return defaultNum; //ファイルがない場合はこれを返す。
}

bool fp_error = false;
void setButtonEnabled(const char*fn , bool flag){ //fn="../manager/lockon_disp_disable.txt"など、このファイルが無かったらtrueのニュアンスで。flagは素直にtrueなら有効。
  //util::write_file(fn, (void*)(flag ? "0" : "1"), 1); //flagと書き込む数値文字列の意味が逆なので注意。
  FILE *fp = fopen(fn,"w"); //write_fileだと書き込めないが、こちらは書き込めた。
  if(fp != NULL){
    fp_error = false;
    if(flag == true){
      fwrite("0",1,1,fp);
    } else {
      fwrite("1",1,1,fp);
    }
    fclose(fp);
  } else {
    fp_error = true;
  }
}

void setButtonEnabled0(const char*fn , bool flag){ //旧fn="../manager/accel_engaged.txt"など、このファイルが無かったらfalseのニュアンスで。flagはそのままtrueなら有効。
  FILE *fp = fopen(fn,"w"); //write_fileだと書き込めないが、こちらは書き込めた。
  if(fp != NULL){
    fp_error = false;
    if(flag == true){
      fwrite("1",1,1,fp);
    } else {
      fwrite("0",1,1,fp);
    }
    fclose(fp);
  } else {
    fp_error = true;
  }
}

void setButtonInt(const char*fn , int num){ //新fn="../manager/accel_engaged.txt"など、このファイルが無かったら0。num(0〜3)はそのまま数字で。
  FILE *fp = fopen(fn,"w"); //write_fileだと書き込めないが、こちらは書き込めた。
  if(fp != NULL){
    fp_error = false;
    if(num == 1){
      fwrite("1",1,1,fp);
    } else if(num == 2){
      fwrite("2",1,1,fp);
    } else if(num == 3){
      fwrite("3",1,1,fp);
    } else {
      fwrite("0",1,1,fp);
    }
    fclose(fp);
  } else {
    fp_error = true;
  }
}

// ButtonsWindow
const static char *btn_style = "font-size: 90px; border-radius: 20px; border-color: %1";
ButtonsWindow::ButtonsWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);

  QWidget *btns_wrapper00 = new QWidget;
  QHBoxLayout *btns_layout00  = new QHBoxLayout(btns_wrapper00);
  btns_layout00->setSpacing(0);
  btns_layout00->setContentsMargins(0, 0, 0, 0);
  main_layout->addWidget(btns_wrapper00, 0, 0); //Alignは何も指定しない。

  QWidget *btns_wrapper0L = new QWidget;
  QHBoxLayout *btns_layout0L  = new QHBoxLayout(btns_wrapper0L);
  btns_layout0L->setSpacing(0);
  btns_layout0L->setContentsMargins(0, 0, 0, 0);
  btns_layout00->addWidget(btns_wrapper0L, 0, /*Qt::AlignVCenter |*/ Qt::AlignLeft);

  QWidget *btns_wrapperLL = new QWidget;
  QVBoxLayout *btns_layoutLL  = new QVBoxLayout(btns_wrapperLL);
  btns_layoutLL->setSpacing(0);
  btns_layoutLL->setContentsMargins(30+15, 430-172, 15, 30);

  btns_layout0L->addWidget(btns_wrapperLL,0,Qt::AlignVCenter);
  {
    // turbo boost button
    uiState()->scene.mStartAccelPowerUpButton = mStartAccelPowerUpButton = getButtonEnabled0("../manager/start_accel_power_up_disp_enable.txt");
    startAccelPowerUpButton = new QPushButton("⇧"); //⬆︎
    QObject::connect(startAccelPowerUpButton, &QPushButton::clicked, [=]() {
      uiState()->scene.mStartAccelPowerUpButton = !mStartAccelPowerUpButton;
    });
    startAccelPowerUpButton->setFixedWidth(150);
    startAccelPowerUpButton->setFixedHeight(150*0.9);
    //lockOnButton->setWindowOpacity(all_opac);
    btns_layoutLL->addSpacing(0);
    btns_layoutLL->addWidget(startAccelPowerUpButton);
    startAccelPowerUpButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mStartAccelPowerUpButton)));
  }

  {
    // use lane button
    uiState()->scene.mUseLaneButton = mUseLaneButton = getButtonInt("../manager/lane_sw_mode.txt" , Params().getBool("EndToEndToggle") ? 0 : 1);
    if(mUseLaneButton == 2){
      useLaneButton = new QPushButton("LA"); //レーンレス自動選択
    } else {
      useLaneButton = new QPushButton("/ \\");
    }
    QObject::connect(useLaneButton, &QPushButton::clicked, [=]() {
      // Params().putBool("EndToEndToggle",!Params().getBool("EndToEndToggle"));
      // uiState()->scene.mUseLaneButton = !Params().getBool("EndToEndToggle");
      // uiState()->scene.end_to_end = Params().getBool("EndToEndToggle");
      uiState()->scene.mUseLaneButton = (mUseLaneButton + 1) % 3; //0->1->2->0
    });
    useLaneButton->setFixedWidth(150);
    useLaneButton->setFixedHeight(150*0.9);
    btns_layoutLL->addSpacing(15);
    btns_layoutLL->addWidget(useLaneButton);
    useLaneButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mUseLaneButton > 0)));
  }

  QWidget *btns_wrapper0 = new QWidget;
  QHBoxLayout *btns_layout0  = new QHBoxLayout(btns_wrapper0);
  btns_layout0->setSpacing(0);
  btns_layout0->setContentsMargins(0, 0, 0, 0);
  btns_layout00->addWidget(btns_wrapper0, 0, /*Qt::AlignTop |*/ Qt::AlignRight);

  QWidget *btns_wrapperL = new QWidget;
  QVBoxLayout *btns_layoutL  = new QVBoxLayout(btns_wrapperL);
  btns_layoutL->setSpacing(0);
  btns_layoutL->setContentsMargins(15, 430, 0, 30);

  btns_layout0->addWidget(btns_wrapperL,0,Qt::AlignVCenter);

  //const float all_opac = 0.2;
  {
    // Handle Ctrl button
    uiState()->scene.mHandleCtrlButton = mHandleCtrlButton = getButtonEnabled("../manager/handle_ctrl_disable.txt");
    handleCtrlButton = new QPushButton("↔︎");
    QObject::connect(handleCtrlButton, &QPushButton::clicked, [=]() {
      uiState()->scene.mHandleCtrlButton = !mHandleCtrlButton;
    });
    handleCtrlButton->setFixedWidth(150);
    handleCtrlButton->setFixedHeight(150);
    //handleCtrlButton->setPalette(QColor(255,255,255,all_opac*255));
    //handleCtrlButton->setAutoFillBackground(true);
    //btns_layoutL->addSpacing(70);
    btns_layoutL->addWidget(handleCtrlButton);
    handleCtrlButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mHandleCtrlButton)));
  }

  {
    // LockOn button
    uiState()->scene.mLockOnButton = mLockOnButton = getButtonEnabled("../manager/lockon_disp_disable.txt");
    lockOnButton = new QPushButton("□");
    QObject::connect(lockOnButton, &QPushButton::clicked, [=]() {
      uiState()->scene.mLockOnButton = !mLockOnButton;
    });
    lockOnButton->setFixedWidth(150);
    lockOnButton->setFixedHeight(150);
    //lockOnButton->setWindowOpacity(all_opac);
    btns_layoutL->addSpacing(15);
    btns_layoutL->addWidget(lockOnButton);
    lockOnButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mLockOnButton)));
  }

  QWidget *btns_wrapper = new QWidget;
  QVBoxLayout *btns_layout  = new QVBoxLayout(btns_wrapper);
  btns_layout->setSpacing(0);
  btns_layout->setContentsMargins(15, 430, 30, 30);

  btns_layout0->addWidget(btns_wrapper,0,Qt::AlignVCenter);

  {
    // Accel Ctrl button
    uiState()->scene.mAccelCtrlButton = mAccelCtrlButton = getButtonEnabled("../manager/accel_ctrl_disable.txt");
    accelCtrlButton = new QPushButton("↑");
    QObject::connect(accelCtrlButton, &QPushButton::clicked, [=]() {
      uiState()->scene.mAccelCtrlButton = !mAccelCtrlButton;
    });
    accelCtrlButton->setFixedWidth(150);
    accelCtrlButton->setFixedHeight(150);
    //accelCtrlButton->setWindowOpacity(all_opac);
    //btns_layout->addSpacing(10);
    btns_layout->addWidget(accelCtrlButton);
    accelCtrlButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mAccelCtrlButton)));
  }

  {
    // Decel Ctrl button
    uiState()->scene.mDecelCtrlButton = mDecelCtrlButton = getButtonEnabled("../manager/decel_ctrl_disable.txt");
    decelCtrlButton = new QPushButton("↓");
    QObject::connect(decelCtrlButton, &QPushButton::clicked, [=]() {
      uiState()->scene.mDecelCtrlButton = !mDecelCtrlButton;
    });
    decelCtrlButton->setFixedWidth(150);
    decelCtrlButton->setFixedHeight(150);
    //decelCtrlButton->setWindowOpacity(all_opac);
    btns_layout->addSpacing(15);
    btns_layout->addWidget(decelCtrlButton);
    decelCtrlButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mDecelCtrlButton)));
  }

  {
    // Accel Engage button
    uiState()->scene.mAccelEngagedButton = mAccelEngagedButton = getButtonInt("../manager/accel_engaged.txt" , 0);
    if(mAccelEngagedButton == 3){
      accelEngagedButton = new QPushButton("OP"); //3ならワンペダルモード(O)
    } else if(mAccelEngagedButton == 2){
      accelEngagedButton = new QPushButton("AA"); //2ならAA(ALL ACCEL)
    } else {
      accelEngagedButton = new QPushButton("A");
    }
    QObject::connect(accelEngagedButton, &QPushButton::clicked, [=]() {
      //uiState()->scene.mAccelEngagedButton = !mAccelEngagedButton; //ここを0->1->2・・・にすれば良い
      uiState()->scene.mAccelEngagedButton = (mAccelEngagedButton + 1) % 4; //0->1->2->3->0
    });
    accelEngagedButton->setFixedWidth(150);
    accelEngagedButton->setFixedHeight(150);
    //accelEngagedButton->setWindowOpacity(all_opac);
    btns_layout->addSpacing(15);
    btns_layout->addWidget(accelEngagedButton);
    accelEngagedButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mAccelEngagedButton > 0)));
  }

  // std::string hide_model_long = "true";  // util::read_file("/data/community/params/hide_model_long");
  // if (hide_model_long == "true"){
  //   mlButton->hide();
  // }

  setStyleSheet(R"(
    QPushButton {
      color: rgba(255, 255, 255, 0.7);
      text-align: center;
      padding: 0px;
      border-width: 4px;
      border-style: solid;
      background-color: rgba(75, 75, 75, 0.3);
    }
  )");
}

void ButtonsWindow::updateState(const UIState &s) {
  if (mLockOnButton != s.scene.mLockOnButton) {  // update mLockOnButton
    mLockOnButton = s.scene.mLockOnButton;
    lockOnButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mLockOnButton && fp_error==false)));
    setButtonEnabled("../manager/lockon_disp_disable.txt" , mLockOnButton);
  }

  if (mAccelCtrlButton != s.scene.mAccelCtrlButton) {  // update mAccelCtrlButton
    mAccelCtrlButton = s.scene.mAccelCtrlButton;
    accelCtrlButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mAccelCtrlButton && fp_error==false)));
    setButtonEnabled("../manager/accel_ctrl_disable.txt" , mAccelCtrlButton);
  }

  if (mDecelCtrlButton != s.scene.mDecelCtrlButton) {  // update mDecelCtrlButton
    mDecelCtrlButton = s.scene.mDecelCtrlButton;
    decelCtrlButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mDecelCtrlButton && fp_error==false)));
    setButtonEnabled("../manager/decel_ctrl_disable.txt" , mDecelCtrlButton);
  }

  if (mAccelEngagedButton != s.scene.mAccelEngagedButton) {  // update mAccelEngagedButton
    mAccelEngagedButton = s.scene.mAccelEngagedButton;
    accelEngagedButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mAccelEngagedButton > 0 && fp_error==false)));
    //ここでボタンのラベルを変えられないかな？mAccelEngagedButton == 2でAAとかにしたい。
    if(mAccelEngagedButton == 3){
      accelEngagedButton->setText("OP");
    } else if(mAccelEngagedButton == 2){
      accelEngagedButton->setText("AA");
    } else {
      accelEngagedButton->setText("A");
    }
    setButtonInt("../manager/accel_engaged.txt" , mAccelEngagedButton);
  }

  if (mHandleCtrlButton != s.scene.mHandleCtrlButton) {  // update mHandleCtrlButton
    mHandleCtrlButton = s.scene.mHandleCtrlButton;
    handleCtrlButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mHandleCtrlButton && fp_error==false)));
    setButtonEnabled("../manager/handle_ctrl_disable.txt" , mHandleCtrlButton);
  }
  
  if (mStartAccelPowerUpButton != s.scene.mStartAccelPowerUpButton) {  // update mStartAccelPowerUpButton
    mStartAccelPowerUpButton = s.scene.mStartAccelPowerUpButton;
    startAccelPowerUpButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mStartAccelPowerUpButton && fp_error==false)));
    setButtonEnabled0("../manager/start_accel_power_up_disp_enable.txt" , mStartAccelPowerUpButton);
  }
  
  if (mUseLaneButton != s.scene.mUseLaneButton) {  // update mUseLaneButton
    mUseLaneButton = s.scene.mUseLaneButton;
    useLaneButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mUseLaneButton > 0 && fp_error==false)));
    if(mUseLaneButton == 2){
      useLaneButton->setText("LA");
    } else {
      useLaneButton->setText("/ \\");
    }
    setButtonInt("../manager/lane_sw_mode.txt" , mUseLaneButton);
  }
  
}

// OnroadAlerts
void OnroadAlerts::updateAlert(const Alert &a, const QColor &color) {
  if (!alert.equal(a) || color != bg) {
    alert = a;
    bg = color;
    update();
  }
}

void OnroadAlerts::paintEvent(QPaintEvent *event) {
  if (alert.size == cereal::ControlsState::AlertSize::NONE) {
    return;
  }
  static std::map<cereal::ControlsState::AlertSize, const int> alert_sizes = {
    {cereal::ControlsState::AlertSize::SMALL, 271},
    {cereal::ControlsState::AlertSize::MID, 420},
    {cereal::ControlsState::AlertSize::FULL, height()},
  };
  int h = alert_sizes[alert.size];
  QRect r = QRect(0, height() - h, width(), h);

  QPainter p(this);

  // draw background + gradient
  p.setPen(Qt::NoPen);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  p.setBrush(QBrush(bg));
  p.drawRect(r);

  QLinearGradient g(0, r.y(), 0, r.bottom());
  g.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.05));
  g.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0.35));

  p.setCompositionMode(QPainter::CompositionMode_DestinationOver);
  p.setBrush(QBrush(g));
  p.fillRect(r, g);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  // text
  const QPoint c = r.center();
  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setRenderHint(QPainter::TextAntialiasing);
  if (alert.size == cereal::ControlsState::AlertSize::SMALL) {
    configFont(p, "Open Sans", 74, "SemiBold");
    p.drawText(r, Qt::AlignCenter, alert.text1);
  } else if (alert.size == cereal::ControlsState::AlertSize::MID) {
    configFont(p, "Open Sans", 88, "Bold");
    p.drawText(QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1);
    configFont(p, "Open Sans", 66, "Regular");
    p.drawText(QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2);
  } else if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    configFont(p, "Open Sans", l ? 132 : 177, "Bold");
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    configFont(p, "Open Sans", 88, "Regular");
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}

// NvgWindow

NvgWindow::NvgWindow(VisionStreamType type, QWidget* parent) : fps_filter(UI_FREQ, 3, 1. / UI_FREQ), CameraViewWidget("camerad", type, true, parent) {
  engage_img = loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size});
  dm_img = loadPixmap("../assets/img_driver_face.png", {img_size, img_size});
}

static float vc_speed;
static int tss_type = 0;
void NvgWindow::updateState(const UIState &s) {
  int SET_SPEED_NA = 557; //255;
  const SubMaster &sm = *(s.sm);
  const auto cs = sm["controlsState"].getControlsState();

  float maxspeed = cs.getVCruise();
  vc_speed = sm["carState"].getCarState().getVEgo();
  if(tss_type == 0){
    std::string tss_type_txt = util::read_file("../manager/tss_type_info.txt");
    if(tss_type_txt.empty() == false){
      if ( tss_type_txt == "2" ) {
        //TSS2
        tss_type = 2;
      } else if ( tss_type_txt == "1" ){
        tss_type = 1;
      }
    }
  }
  if(PI0_DEBUG == false && tss_type <= 1){
    //これまでと互換。tss_type_infoがなければTSSP
    maxspeed = maxspeed < (55 - 4) ? (55 - (55 - (maxspeed+4)) * 2 - 4) : maxspeed;
    maxspeed = maxspeed > (110 - 6) ? (110 + ((maxspeed+6) - 110) * 3 - 6) : maxspeed;
  } else if(PI0_DEBUG == true || tss_type == 2){
    SET_SPEED_NA = 255; //TSS2では戻す。
  }

  bool cruise_set = maxspeed > 0 && (int)maxspeed != SET_SPEED_NA;
  if (cruise_set && !s.scene.is_metric) {
    maxspeed *= KM_TO_MILE;
  }
  QString maxspeed_str = cruise_set ? QString::number(std::nearbyint(maxspeed)) : "N/A";
  std::string stdstr_txt = util::read_file("../manager/cruise_info.txt");
  if(cruise_set && stdstr_txt.empty() == false){
    QString qstr = QString::fromStdString(stdstr_txt);
    maxspeed_str = qstr;
  }
  float cur_speed = std::max(0.0, sm["carState"].getCarState().getVEgo() * (s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH));

  setProperty("is_cruise_set", cruise_set);
  setProperty("speed", QString::number(std::nearbyint(cur_speed)));
  setProperty("maxSpeed", maxspeed_str);
  setProperty("speedUnit", s.scene.is_metric ? "km/h" : "mph");
  setProperty("hideDM", cs.getAlertSize() != cereal::ControlsState::AlertSize::NONE);
  setProperty("status", s.status);

  // update engageability and DM icons at 2Hz
  if (sm.frame % (UI_FREQ / 2) == 0) {
    setProperty("engageable", cs.getEngageable() || cs.getEnabled());
    setProperty("dmActive", sm["driverMonitoringState"].getDriverMonitoringState().getIsActiveMode());
  }
}

static bool global_engageable;
static int global_status;
static float curve_value;
static float handle_center = -100;
static int handle_calibct = 0;
static float distance_traveled;
static float global_angle_steer0 = 0;
void NvgWindow::drawHud(QPainter &p) {
  p.save();
  int y_ofs = 150;

  // Header gradient
  QLinearGradient bg(0, header_h - (header_h / 2.5), 0, header_h);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), header_h+y_ofs, bg);

  // max speed
  float max_disp_k = 1.8;
  float max_disp_a = 50;
  const int rect_w = rect().width();
  const int rect_h = rect().height();
  if(false && (float)rect_w / rect_h > 1.4f){
  } else {
    //こちらの大きさを採用。
    max_disp_k = 1.3;
    max_disp_a = 20;
  }

  QRect rc(bdr_s * 2, bdr_s * 1.5+y_ofs, 184*max_disp_k, 202*max_disp_k);
#if 0
  p.setPen(QPen(QColor(0xff, 0xff, 0xff, 100), 10));
#else
  QString ms = QString(maxSpeed);
  if(ms.length() > 1){
    if(maxSpeed.mid(0,1) == ","){ //先頭カンマで加速
      ms = maxSpeed.mid(1,maxSpeed.length()-1);
      p.setPen(QPen(QColor(0, 0xff, 0, 200), 10)); //加速時は緑
    } else if(maxSpeed.mid(maxSpeed.length()-1,1) == "."){ //末尾ピリオドで減速
      ms = maxSpeed.mid(0,maxSpeed.length()-1);
      p.setPen(QPen(QColor(0xff, 0, 0, 200), 10)); //減速時は赤
    } else if(maxSpeed.mid(maxSpeed.length()-1,1) == ";"){ //末尾セミコロンで黄色
      ms = maxSpeed.mid(0,maxSpeed.length()-1);
      p.setPen(QPen(QColor(0xff, 0xff, 0, 200), 10)); //黄色
    } else {
      p.setPen(QPen(QColor(0xff, 0xff, 0xff, 100), 10));
    }
  } else {
    p.setPen(QPen(QColor(0xff, 0xff, 0xff, 100), 10));
  }
#endif
  p.setBrush(QColor(0, 0, 0, 100));
  p.drawRoundedRect(rc, 20, 20);
  p.setPen(Qt::NoPen);

  configFont(p, "Open Sans", 48*max_disp_k, "Regular");
  //const char *max_str = (tss_type == 0 ? "MA+" : (tss_type <= 1 ? "MAX" : "MAX2"));
  drawText(p, rc.center().x(), 118+y_ofs+max_disp_a, "MAX", is_cruise_set ? 200 : 100);
  if (is_cruise_set) {
#if 0
    float mm = maxSpeed.length() < 4 ? 1.1 : 1.0;
    configFont(p, "Open Sans", 88*max_disp_k*mm, is_cruise_set ? "Bold" : "SemiBold");
    drawText(p, rc.center().x(), 212-(212-118)+(212-118)*max_disp_k+y_ofs+max_disp_a, maxSpeed, 255);
#else
    float mm = ms.length() < 4 ? 1.1 : 1.0; //カンマピリオド以外の状況で4桁になってるケースをケアする。セミコロンとかあり得る
    configFont(p, "Open Sans", 88*max_disp_k*mm, is_cruise_set ? "Bold" : "SemiBold");
    drawText(p, rc.center().x(), 212-(212-118)+(212-118)*max_disp_k+y_ofs+max_disp_a, ms, 255);
#endif
  } else {
    configFont(p, "Open Sans", 80*max_disp_k*1.1, "SemiBold");
    drawText(p, rc.center().x(), 212-(212-118)+(212-118)*max_disp_k+y_ofs+max_disp_a, maxSpeed, 100);
  }

  // current speed
  configFont(p, "Open Sans", 176, "Bold");
  drawText(p, rect().center().x()-7, 210+y_ofs-5, speed,bg_colors[status]);
  drawText(p, rect().center().x()+7, 210+y_ofs-5, speed,bg_colors[status]);
  drawText(p, rect().center().x(), -7+210+y_ofs-5, speed,bg_colors[status]);
  drawText(p, rect().center().x(), +7+210+y_ofs-5, speed,bg_colors[status]);
  drawText(p, rect().center().x(), 210+y_ofs-5, speed);
  configFont(p, "Open Sans", 66, "Regular");
  UIState *s = uiState();
  //bool brakePressed = (*s->sm)["carState"].getCarState().getBrakePressed(); //これは実際のブレーキペダルを踏んだかどうか。車体のブレーキランプでは無い。
  if(true /*brakePressed == false*/){
    drawText(p, rect().center().x(), 290+y_ofs-5, speedUnit, 200);
  } else {
    drawText(p, rect().center().x(), 290+y_ofs-5, speedUnit, QColor(0xC9, 0x22, 0x31, 200));
  }

//以下オリジナル表示要素
  //温度を表示(この画面は更新が飛び飛びになる。ハンドル回したりとか何か変化が必要)
  auto deviceState = (*s->sm)["deviceState"].getDeviceState();
  int temp = (int)deviceState.getAmbientTempC();
#if 0
  QString temp_disp = QString("Temp:") + QString::number(temp) + "°C";
#else
  bool okGps = (*s->sm)["liveLocationKalman"].getLiveLocationKalman().getGpsOK();
  bool okConnect = false;
  auto last_ping = deviceState.getLastAthenaPingTime();
  if (last_ping != 0) {
    okConnect = nanos_since_boot() - last_ping < 80e9 ? true : false;
  }
  //下の方がマシかQString temp_disp = QString(okConnect ? "● " : "○ ") + QString(okGps ? "★ " : "☆ ") + QString::number(temp) + "°C";
  //QString temp_disp = QString(okConnect ? "⚫︎ " : "⚪︎ ") + QString(okGps ? "★ " : "☆ ") + QString::number(temp) + "°C";
  QString temp_disp1 = QString(okConnect ? "⚫︎" : "⚪︎");
  QString temp_disp2 = QString(okGps ? "★ " : "☆ ");
  QString temp_disp3 = QString::number(temp) + "°C";
  //QString temp_disp = QString(okConnect ? "⚫︎ " : "⚪︎ ") + QString(okGps ? "◆ " : "◇ ") + QString::number(temp) + "°C";
#endif
  configFont(p, "Open Sans", 44, "SemiBold");

  int th_tmp1 = 47;
  int th_tmp2 = 55;
  if(Hardware::TICI()){
    th_tmp1 = 58;
    th_tmp2 = 64;
  }

  QRect temp_rc(rect().left()+65-27, rect().top()+110+6, 233+27*2, 54);
  p.setPen(Qt::NoPen);
  if(temp < th_tmp1){ //警告色の変化はサイドバーと違う。もっと早く警告される。
    p.setBrush(bg_colors[status]);
  } else if(temp < th_tmp2){
    p.setBrush(QColor(240, 240, 0, 200));
  } else {
    p.setBrush(QColor(240, 0, 0, 200));
  }
  p.drawRoundedRect(temp_rc, 30, 30);

  if(temp < th_tmp1){ //警告色の変化はサイドバーと違う。もっと早く警告される。
    p.setPen(QColor(0xff, 0xff, 0xff , 200));
  } else if(temp < th_tmp2){
    //p.setPen(QColor(0xff, 0xff, 0 , 255));
    p.setPen(QColor(10, 10, 10 , 255));
  } else {
    p.setPen(QColor(0xff, 0xff, 0 , 255));
  }
  //p.drawText(QRect(rect().left()+65, rect().top()+110, 300, 65), Qt::AlignTop | Qt::AlignLeft, temp_disp);
  p.drawText(QRect(rect().left()+65+150, rect().top()+110, 300, 65), Qt::AlignTop | Qt::AlignLeft, temp_disp3);
  configFont(p, "Open Sans", 54, "SemiBold");
  p.drawText(QRect(rect().left()+65+65, rect().top()+110, 300, 65), Qt::AlignTop | Qt::AlignLeft, temp_disp2);
  configFont(p, "Open Sans", 54, "SemiBold");
  p.drawText(QRect(rect().left()+65, rect().top()+110, 300, 65), Qt::AlignTop | Qt::AlignLeft, temp_disp1);

  if((float)rect_w / rect_h > 1.4f){
    configFont(p, "Open Sans", 44, "SemiBold");
    drawText(p, rect().left()+250, 55, "Powered by COMMA.AI", 150);
    configFont(p, "Open Sans", 55, "SemiBold");
    if(tss_type <= 1){
      drawText(p, rect().right()-260, 60, "for prius PHV 2017", 150);
    } else {
      drawText(p, rect().right()-260, 60, "for prius PHV 2021", 150);
    }
  }
  configFont(p, "Open Sans", 33, "SemiBold");
  drawText(p, rect().right()-275, rect().bottom() - 10 , "modified by PROGRAMAN ICHIRO", 150);
  configFont(p, "Open Sans", 33, "Bold");
  float angle_steer = 0;
  std::string angle_steer0_txt = util::read_file("../manager/steer_ang_info.txt");
  if(angle_steer0_txt.empty() == false){
    global_angle_steer0 = std::stof(angle_steer0_txt);
  }
  // char *steer_ang_info = getenv("steer_ang_info");
  // if(steer_ang_info != nullptr){
  //   std::string angle_steer0_txt = steer_ang_info;
  //   if(angle_steer0_txt.empty() == false){
  //     global_angle_steer0 = std::stof(angle_steer0_txt);
  //   }
  // }
  float a0 = 150,a1 = 150,a2 = 150,a3 = 150;
  curve_value = 0;
  global_status = status;
  global_engageable = engageable;
  if (engageable && status != STATUS_ENGAGED) {
    a0 = 50; a1 = 50; a2 = 50; a3 = 50;
  } else if (engageable && status == STATUS_ENGAGED) {
    a0 = 50; a1 = 50; a2 = 50; a3 = 50;
    if(vc_speed < 1/3.6){
      a3 = 200;
    }
    angle_steer = global_angle_steer0;
    if(vc_speed >= 1/3.6 && (angle_steer > 1.5 || angle_steer < -1.5)){ //低速では1.0だが、一緒くたにする
      if(uiState()->scene.mHandleCtrlButton == true){
        a2 = 200; //↔︎ボタンOFFで濃くしない。
      }
    }
    if ( maxSpeed.contains(".", Qt::CaseInsensitive) == true ) {
      a1 = 200;
    }
    if (is_cruise_set){
      float acc_speed = maxSpeed.toFloat();
      if(acc_speed > 0 && (acc_speed < 31.0 || (acc_speed > 119.0 && tss_type <= 1)) ) {
        a0 = 200;
      }
    }
    std::string limit_vc_txt = util::read_file("../manager/limit_vc_info.txt");
    if(limit_vc_txt.empty() == false && vc_speed >= 1/3.6){
      curve_value = std::stof(limit_vc_txt);
    }
  }
  
  bool brake_light = false; //ブレーキランプは無くなった？(*(uiState()->sm))["carState"].getCarState().getBrakeLightsDEPRECATED();
  drawText(p, rect().center().x(), 50 + 40*0 , "extra cruise speed engagement", a0 , brake_light);
  drawText(p, rect().center().x(), 50 + 40*1 , "slow down corner correctly", a1 , brake_light);
  drawText(p, rect().center().x(), 50 + 40*2 , "make curve inner offset", a2 , brake_light);
  //drawText(p, rect().center().x(), 50 + 40*2 , QString::number(angle_steer), a2 , brake_light);
  drawText(p, rect().center().x(), 50 + 40*3 , "auto brake holding", a3 , brake_light);

  // engage-ability icon
  if (engageable) {
    drawIcon(p, rect().right() - radius / 2 - bdr_s * 2, radius / 2 + int(bdr_s * 1.5)+y_ofs,
             engage_img, bg_colors[status], 1.0 , -global_angle_steer0);
  }

  //キャリブレーション値の表示。dm iconより先にやらないと透明度が連動してしまう。
  p.setPen(QPen(QColor(0xff, 0xff, 0xff, 0), 0));
  //int calib_h = radius;
  int calib_h = -33 -33 - 30; //表示位置を上に
  QRect rc2(rect().right() - radius / 2 - bdr_s * 2 - 100, -20 + radius / 2 + int(bdr_s * 1.5)+y_ofs + calib_h -36, 200, 36);
  if(/*engageable ||*/ handle_center == -100){
    std::string handle_center_txt = util::read_file("../manager/handle_center_info.txt");
    if(handle_center_txt.empty() == false){
        handle_center = std::stof(handle_center_txt);
    }
  }
  if(/*engageable ||*/ handle_center > -99){
    //ハンドルセンター値を表示
    p.setBrush(bg_colors[status]);
    p.drawRoundedRect(rc2, 18, 18);
    p.setPen(Qt::NoPen);

    //float hc = -4.73;
    float hc = handle_center;

    configFont(p, "Open Sans", 33, "Bold");
    drawText(p, rect().right() - radius / 2 - bdr_s * 2 , -20 + radius / 2 + int(bdr_s * 1.5)+y_ofs + calib_h - 8, QString::number(hc,'f',2) + "deg", 200);
  } else {
    p.setBrush(QColor(150, 150, 0, 0xf1));
    p.drawRoundedRect(rc2, 18, 18);
    p.setPen(Qt::NoPen);

    if(handle_calibct == 0){
      configFont(p, "Open Sans", 33, "Regular");
      drawText(p, rect().right() - radius / 2 - bdr_s * 2 , -20 + radius / 2 + int(bdr_s * 1.5)+y_ofs + calib_h - 8, "Calibrating", 200);
    } else {
      configFont(p, "Open Sans", 33, "Bold");
      drawText(p, rect().right() - radius / 2 - bdr_s * 2 , -20 + radius / 2 + int(bdr_s * 1.5)+y_ofs + calib_h - 6, QString::number(handle_calibct) + '%', 200);
    }
  }
  
  // dm icon
  if (!hideDM) {
    drawIcon(p, radius / 2 + (bdr_s * 2), rect().bottom() - footer_h / 2,
             dm_img, QColor(0, 0, 0, 70), dmActive ? 1.0 : 0.2 , 0);
  }
  p.restore();
}

void NvgWindow::drawText(QPainter &p, int x, int y, const QString &text, int alpha , bool brakeLight) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  if(brakeLight == false){
    p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  } else {
    alpha += 50;
    if(alpha > 255){
      alpha = 255;
    }
    p.setPen(QColor(0xff, 0, 0, alpha));
  }
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void NvgWindow::drawText(QPainter &p, int x, int y, const QString &text, const QColor &col) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(col);
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void NvgWindow::drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity , float ang) {
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);
  p.setOpacity(opacity);
  p.resetTransform();
  p.translate(x - img_size / 2,y - img_size / 2);
  if(ang != 0){
    p.translate(img_size/2,img_size/2);
    p.rotate(ang); //degree指定
    p.translate(-img_size/2,-img_size/2);
  }
  QRect r(0,0,img_size,img_size);
  p.drawPixmap(r, img);
  p.resetTransform();
}


void NvgWindow::initializeGL() {
  CameraViewWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);
}

void NvgWindow::updateFrameMat(int w, int h) {
  CameraViewWidget::updateFrameMat(w, h);

  UIState *s = uiState();
  s->fb_w = w;
  s->fb_h = h;
  auto intrinsic_matrix = s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;
  float zoom = ZOOM / intrinsic_matrix.v[0];
  if (s->wide_camera) {
    zoom *= 0.5;
  }
  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(w / 2, h / 2 + y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}

void NvgWindow::drawLaneLines(QPainter &painter, const UIState *s) {
  painter.save();

  uiState()->scene.end_to_end = Params().getBool("EndToEndToggle");
  const UIScene &scene = s->scene;
  // lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    painter.drawPolygon(scene.lane_line_vertices[i].v, scene.lane_line_vertices[i].cnt);
  }

  // road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 0, 0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0)));
    painter.drawPolygon(scene.road_edge_vertices[i].v, scene.road_edge_vertices[i].cnt);
  }

  // paint path
  QLinearGradient bg(0, height(), 0, height() / 4);
  if (scene.end_to_end) {
    const auto &orientation = (*s->sm)["modelV2"].getModelV2().getOrientation();
    float orientation_future = 0;
    if (orientation.getZ().size() > 16) {
      orientation_future = std::abs(orientation.getZ()[16]);  // 2.5 seconds
    }
    // straight: 112, in turns: 70
    float curve_hue = fmax(70, 112 - (orientation_future * 420));
    // FIXME: painter.drawPolygon can be slow if hue is not rounded
    curve_hue = int(curve_hue * 100 + 0.5) / 100;

    bg.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 0.4));
    bg.setColorAt(0.75 / 1.5, QColor::fromHslF(curve_hue / 360., 1.0, 0.68, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(curve_hue / 360., 1.0, 0.68, 0.0));
  } else {
    bg.setColorAt(0, whiteColor());
    bg.setColorAt(1, whiteColor(0));
  }
  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices.v, scene.track_vertices.cnt);

  knightScanner(painter);
  painter.restore();
}

void NvgWindow::knightScanner(QPainter &p) {

  static const int ct_n = 1;
  static float ct;

  int rect_w = rect().width();
  int rect_h = rect().height();

  const int n = 15+1; //タイミングの問題で画面外に一つ増やす
  static float t[n];
  //int dim_n = (sin(ct/5) + 1) * (n-0.01);
  //t[dim_n] = 1.0;
  t[(int)(ct/ct_n)] = 1.0;
  int ww = rect_w / (n-1); //画面外の一つ分を外す。
  int hh = ww;

  static float dir0 = 1.0;
  float dir;
  if(curve_value == 0){
    dir = dir0 * 0.25;
    hh = hh / 3;
  } else if(curve_value < 145){
    dir = dir0 * 1.0;
  } else {
    dir = dir0 * 0.5;
    hh = hh * 2 / 3;
  }

  UIState *s = uiState();
  bool left_blinker = (*s->sm)["carState"].getCarState().getLeftBlinker();
  bool right_blinker = (*s->sm)["carState"].getCarState().getRightBlinker();
  int lane_change_height = 0; //280; //↓の下の尖りがウインカーの底辺になるように調整。
  if(left_blinker || right_blinker){
    if(left_blinker == true){
      dir0 = -fabs(dir0);
    } else if(right_blinker == true){
      dir0 = fabs(dir0);
    }
    dir = dir0 * 1.0;
    hh = ww;
    hh = hh * 2 / 3;
#if 0
    if((*s->sm)["carState"].getCarState().getVEgo() >= 50/3.6){ //
      lane_change_height = 270;
    }
#else
    auto lp = (*s->sm)["lateralPlan"].getLateralPlan();
    if( lp.getLaneChangeState() == cereal::LateralPlan::LaneChangeState::PRE_LANE_CHANGE ||
        lp.getLaneChangeState() == cereal::LateralPlan::LaneChangeState::LANE_CHANGE_STARTING){ //レーンチェンジの表示で判定
      lane_change_height = 270;
    }
#endif
  }
  //bool hazard_flashers = left_blinker && right_blinker; //これはtrueにならない。ハザードではleft_blinkerとright_blinkerがfalseのようだ。

  //int h_pos = 0;
  int h_pos = rect_h - hh;

  //ct ++;
  //ct %= n * ct_n;
  ct += dir;
  if(ct <= 0 || ct >= n*ct_n-1){
    if(left_blinker || right_blinker){
      if(left_blinker == true && ct < 0){
        ct = n*ct_n-1;
      } else if(right_blinker == true && ct > n*ct_n-1){
        ct = 0;
      }
    } else {
      if(ct < 0 && dir < 0)ct = 0;
      if(ct > n*ct_n-1 && dir > 0)ct = n*ct_n-1;
      dir0 = -dir0;
    }
    if(vc_speed >= 1/3.6 && global_engageable && global_status == STATUS_ENGAGED) {
      std::string limit_vc_txt = util::read_file("../manager/limit_vc_info.txt");
      if(limit_vc_txt.empty() == false){
        float cv = std::stof(limit_vc_txt);
        if(cv > 0){
          curve_value = cv;
        }
      }
    }
    std::string handle_center_txt = util::read_file("../manager/handle_center_info.txt");
    if(handle_center_txt.empty() == false){
        handle_center = std::stof(handle_center_txt);
    } else {
      std::string handle_calibct_txt = util::read_file("../manager/handle_calibct_info.txt");
      if(handle_calibct_txt.empty() == false){
        handle_calibct = std::stoi(handle_calibct_txt);
      }
    }
  }
  p.setCompositionMode(QPainter::CompositionMode_Plus);
  for(int i=0; i<(n-1); i++){
    //QRect rc(0, h_pos, ww, hh);
    if(t[i] > 0.01){
      //p.drawRoundedRect(rc, 0, 0);
      if(left_blinker || right_blinker){
        //流れるウインカー
        p.setBrush(QColor(192, 102, 0, 255 * t[i]));
      } else if(handle_center > -99){
        p.setBrush(QColor(200, 0, 0, 255 * t[i]));
      } else {
        p.setBrush(QColor(200, 200, 0, 255 * t[i])); //ハンドルセンターキャリブレーション中は色を緑に。
      }
      if(left_blinker || right_blinker){
        p.drawRect(rect_w * i / (n-1), h_pos - lane_change_height, ww, hh); //drawRectを使う利点は、角を取ったりできそうだ。
      } else {
        //ポリゴンで表示。
        float sx_a = rect_w * i / (n-1) - rect_w / 2;
        sx_a /= (rect_w / 2); // -1〜1
        float sx_b = rect_w * (i+1) / (n-1) - rect_w / 2;
        sx_b /= (rect_w / 2); // -1〜1
        float x0 = rect_w * i / (n-1);
        float x1 = x0 + ww;
        float y0 = h_pos;
        float y1 = y0 + hh;
        y0 -= ww/6; //少し持ち上げる。
        float y0_a = y0 + hh/2 * (1 - sx_a*sx_a); //関数の高さ計算に加減速を反省させればビヨビヨするはず。
        float y0_b = y0 + hh/2 * (1 - sx_b*sx_b);
        QPointF scaner[] = {{x0,y0_a},{x1,y0_b}, {x1,y1}, {x0,y1}};
        p.drawPolygon(scaner, std::size(scaner));
      }
    }
    t[i] *= 0.9;
  }

#if 1 //加速減速表示テスト
  //float vc_speed = (*s->sm)["carState"].getCarState().getVEgo();
  float vc_accel0 = (*s->sm)["carState"].getCarState().getAEgo();
  static float vc_accel;
  vc_accel = vc_accel + (vc_accel0 - vc_accel) / 5;
  //vc_accel = -0.5;
  float hha = 0;
  if(vc_accel > 0){
    hha = 1 - 0.1 / vc_accel;
    p.setBrush(QColor(0, 245, 0, 200));
  }
  if(vc_accel < 0){
    hha = 1 + 0.1 / vc_accel;
    p.setBrush(QColor(245, 0, 0, 200));
  }
  if(hha < 0){
    hha = 0;
  }
  hha = hha * rect_h;
  float wp = 35;
  if(vc_accel > 0){
#if 0
    QRect ra = QRect(rect_w - wp , rect_h/2 - hha/2 , wp , hha/2);
    p.drawRect(ra);
#else //メーターを斜めに切る
    QPointF meter[] = {{rect_w - wp/2 - wp/2 * hha / rect_h , (float)rect_h/2 - hha/2},{(float)rect_w , rect_h/2 - hha/2}, {(float)rect_w , (float)rect_h/2}, {rect_w - wp + wp/2 , (float)rect_h/2}};
    p.drawPolygon(meter, std::size(meter));
#endif
  } else {
#if 0
    QRect ra = QRect(rect_w - wp , rect_h/2         , wp , hha/2);
    p.drawRect(ra);
#else //メーターを斜めに切る
    QPointF meter[] = {{rect_w - wp + wp/2 , (float)rect_h/2},{(float)rect_w , (float)rect_h/2}, {(float)rect_w , rect_h/2 + hha/2}, {rect_w - wp/2 - wp/2 * hha / rect_h, rect_h/2 + hha/2}};
    p.drawPolygon(meter, std::size(meter));
#endif
  }
#endif
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

#if 1 //減速度と舵角を表示
  static float cv = 0;
  float ang = global_angle_steer0;
#if 1
  static unsigned int debug_ct;
  if(debug_ct % 10 == 0){
    std::string limit_vc_txt = util::read_file("../manager/limit_vc_info.txt");
    if(limit_vc_txt.empty() == false){
      cv = std::stof(limit_vc_txt);
    }
  }
  debug_ct ++;
#endif
  configFont(p, "Open Sans", 44, "SemiBold");
  p.setPen(QColor(0xdf, 0xdf, 0x00 , 200));
  {
    //float vegostopping = (*s->sm)["carParams"].getCarParams().getVEgoStopping();
    //QString debug_disp = QString("Stop:") + QString::number(vegostopping,'f',0);
    QString debug_disp = QString("Slow:") + QString::number(cv,'f',0);
    p.drawText(QRect(0+20, rect_h - 46, 200, 46), Qt::AlignBottom | Qt::AlignLeft, debug_disp);
  }
  {
    QString debug_disp = QString(",Ang:") + QString::number(ang,'f',1);
    p.drawText(QRect(0+20 + 200, rect_h - 46, 210, 46), Qt::AlignBottom | Qt::AlignLeft, debug_disp);
  }
  {
    QString debug_disp = QString(",Trip:") + QString::number(distance_traveled / 1000,'f',1) + QString("km");
    p.drawText(QRect(0+20 + 200 + 210, rect_h - 46, 290, 46), Qt::AlignBottom | Qt::AlignLeft, debug_disp);
  }
#endif
}

void NvgWindow::drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd , int num , size_t leads_num) {
  painter.save();
  const float speedBuff = 10.;
  const float leadBuff = 40.;
  const float d_rel = lead_data.getX()[0];
  const float v_rel = lead_data.getV()[0];
//  const float t_rel = lead_data.getT()[0];
//  const float y_rel = lead_data.getY()[0];
//  const float a_rel = lead_data.getA()[0];

  float fillAlpha = 0;
  if (d_rel < leadBuff) {
    fillAlpha = 255 * (1.0 - (d_rel / leadBuff));
    if (v_rel < 0) { //速度？負なら赤三角の濃さを増している。
      fillAlpha += 255 * (-1 * (v_rel / speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  float y = std::fmin(height() - sz * .6, (float)vd.y());

  float g_xo = sz / 5;
  float g_yo = sz / 10;

  //QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};
  float homebase_h = 12;
  QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo + homebase_h},{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo},{x - (sz * 1.35) - g_xo, y + sz + g_yo + homebase_h}, {x, y + sz + homebase_h + g_yo + 10}};
  painter.setBrush(QColor(218, 202, 37, 255));
  painter.drawPolygon(glow, std::size(glow));

  // chevron
  //QPointF chevron[] = {{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz}};
  QPointF chevron[] = {{x + (sz * 1.25), y + sz + homebase_h},{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz},{x - (sz * 1.25), y + sz + homebase_h}, {x, y + sz + homebase_h - 7}};
  painter.setBrush(redColor(fillAlpha));
  painter.drawPolygon(chevron, std::size(chevron));

  if(num == 0){ //0番のリードカーまでの距離を表示
    //float dist = d_rel; //lead_data.getT()[0];
    QString dist = QString::number(d_rel,'f',1) + "m";
    int str_w = 200;
//    dist += "<" + QString::number(rect().height()) + ">"; str_w += 500;画面の高さはc2,c3共に1020だった。
//    dist += "<" + QString::number(leads_num) + ">";
//   int str_w = 600; //200;
//    dist += QString::number(v_rel,'f',1) + "v";
//    dist += QString::number(t_rel,'f',1) + "t";
//    dist += QString::number(y_rel,'f',1) + "y";
//    dist += QString::number(a_rel,'f',1) + "a";
    configFont(painter, "Open Sans", 44, "SemiBold");
    painter.setPen(QColor(0x0, 0x0, 0x0 , 200)); //影
    float lock_indicator_dx = 2; //下向きの十字照準を避ける。
    painter.drawText(QRect(x+2+lock_indicator_dx, y-50+2, str_w, 50), Qt::AlignBottom | Qt::AlignLeft, dist);
    painter.setPen(QColor(0xff, 0xff, 0xff));
    painter.drawText(QRect(x+lock_indicator_dx, y-50, str_w, 50), Qt::AlignBottom | Qt::AlignLeft, dist);
    painter.setPen(Qt::NoPen);
  }

    painter.restore();

}

struct LeadcarLockon {
  float x,y,d,a,lxt,lxf,lockOK;
};
#define LeadcarLockon_MAX 5
LeadcarLockon leadcar_lockon[LeadcarLockon_MAX]; //この配列0番を推論1番枠と呼ぶことにする。

void NvgWindow::drawLockon(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd , int num , size_t leads_num , const cereal::ModelDataV2::LeadDataV3::Reader &lead0 , const cereal::ModelDataV2::LeadDataV3::Reader &lead1) {
  //const float speedBuff = 10.;
  //const float leadBuff = 40.;
  const float d_rel = lead_data.getX()[0];
  //const float v_rel = lead_data.getV()[0];
  //const float t_rel = lead_data.getT()[0];
  //const float y_rel = lead_data.getY()[0];
  float a_rel = lead_data.getA()[0];

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  //float y = std::fmin(height() /*- sz * .6*/, (float)vd.y());
  float y = (float)vd.y();

  //float g_xo = sz / 5;
  //float g_yo = sz / 10;

  //QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};

  painter.setCompositionMode(QPainter::CompositionMode_Plus);
  //p.setPen(QColor(0, 255, 0, 255));

  float prob_alpha = lead_data.getProb();
  if(prob_alpha < 0){
    prob_alpha = 0;
  } else if(prob_alpha > 1.0){
    prob_alpha = 1.0;
  }
  prob_alpha *= 245;

  painter.setPen(QPen(QColor(0, 245, 0, prob_alpha), 2));
  painter.setBrush(QColor(0, 0, 0, 0));
  float ww = 300 , hh = 300;
  if(Hardware::TICI()){
    ww *= 1.25; hh *= 1.25;
  }
  float d = d_rel; //距離をロックターケットの大きさに反映させる。
  if(d < 1){
    d = 1;
  }

  //動きに緩衝処理。
  leadcar_lockon[num].x = leadcar_lockon[num].x + (x - leadcar_lockon[num].x) / 6;
  leadcar_lockon[num].y = leadcar_lockon[num].y + (y - leadcar_lockon[num].y) / 6;
  leadcar_lockon[num].d = leadcar_lockon[num].d + (d - leadcar_lockon[num].d) / 6;
  x = leadcar_lockon[num].x;
  y = leadcar_lockon[num].y;
  d = leadcar_lockon[num].d;
  if(d < 1){
    d = 1;
  }

  leadcar_lockon[num].a = leadcar_lockon[num].a + (a_rel - leadcar_lockon[num].a) / 10;
  a_rel = leadcar_lockon[num].a;

  float dh = 50;
  { //dhに奥行き値を反映させる。25mで現状に見えるように。
    float dd = d;
    dd -= 25; //dd=0〜75
    dd /= (75.0/2); //dd=0〜2
    dd += 1; //dd=1〜3
    dh /= dd;
  }

  ww = ww * 2 * 5 / d;
  hh = hh * 2 * 5 / d;
  y = std::fmin(height() /*- sz * .6*/, y - dh) + dh;
  QRect r = QRect(x - ww/2, y /*- g_yo*/ - hh - dh, ww, hh);

#if 0
  float y0 = lead0.getY()[0];
  float y1 = lead1.getY()[0];
#else
  //y?ってわかりにくいな。横方向なんだが。getYは使えなさそうだし。
  float y0 = leadcar_lockon[0].x * leadcar_lockon[0].d; //こうなったら画面座標から逆算。
  float y1 = leadcar_lockon[1].x * leadcar_lockon[1].d;
#endif

  configFont(painter, "Open Sans", 38, "SemiBold");
  if(num == 0 && uiState()->scene.mLockOnButton){
    //推論1番
    painter.setPen(QPen(QColor(0, 245, 0, prob_alpha), 2));
    painter.drawRect(r);

    //painter.setPen(QPen(QColor(0, 245, 0, prob_alpha), 2));
    if(leadcar_lockon[0].x > leadcar_lockon[1].x - 20){
      leadcar_lockon[num].lxt = leadcar_lockon[num].lxt + (r.right() - leadcar_lockon[num].lxt) / 20;
      leadcar_lockon[num].lxf = leadcar_lockon[num].lxf + (width() - leadcar_lockon[num].lxf) / 20;
      //painter.drawLine(r.right(),r.top() , width() , 0);
    } else {
      leadcar_lockon[num].lxt = leadcar_lockon[num].lxt + (r.left() - leadcar_lockon[num].lxt) / 20;
      leadcar_lockon[num].lxf = leadcar_lockon[num].lxf + (0 - leadcar_lockon[num].lxf) / 20;
      //painter.drawLine(r.left(),r.top() , 0 , 0);
    }
    painter.drawText(r, Qt::AlignTop | Qt::AlignLeft, " " + QString::number(num+1));

    //painter.setPen(QPen(QColor(245, 245, 0, prob_alpha), 2));
    float lxt = leadcar_lockon[num].lxt;
    if(lxt < r.left()){
      lxt = r.left();
    } else if(lxt > r.right()){
      lxt = r.right();
    }
    painter.drawLine(lxt,r.top() , leadcar_lockon[num].lxf , 0);
    if(ww >= 40){
      //painter.drawText(r, Qt::AlignTop | Qt::AlignRight, QString::number((int)(lead_data.getProb()*100)) + "％");

      //num==0のロックオンの右端20ドットくらいをa_rel数値メーターとする。
      painter.setPen(Qt::NoPen);
      float wwa = ww * 0.15;
      if(wwa > 40){
        wwa = 40;
      } else if(wwa < 10){
        wwa = 10;
      }
      if(wwa > ww){
        wwa = ww;
      }

      float hha = 0;
      if(a_rel > 0){
        hha = 1 - 0.1 / a_rel;
        painter.setBrush(QColor(0, 245, 0, prob_alpha*0.9));

        if(hha < 0){
          hha = 0;
        }
        hha = hha * hh;
#if 0
        QRect ra = QRect(x - ww/2 + (ww - wwa), y /*- g_yo*/ - hh - dh + (hh-hha), wwa, hha);
        painter.drawRect(ra);
#else //メーターを斜めに切る
        QPointF meter[] = {{(float)x + ww/2 - wwa/2 - wwa/2 * hha / hh , (float)y /*- g_yo*/ - hh - dh + (hh-hha)},{(float)x + ww/2 , (float)y /*- g_yo*/ - hh - dh + (hh-hha)}, {(float)x + ww/2 , (float)y /*- g_yo*/ - hh - dh + hh}, {(float)x + ww/2 - wwa/2 , (float)y /*- g_yo*/ - hh - dh + hh}};
        painter.drawPolygon(meter, std::size(meter));
#endif
      }
      if(a_rel < 0){
        hha = 1 + 0.1 / a_rel;
        painter.setBrush(QColor(245, 0, 0, prob_alpha));
        //減速は上から下へ変更。
        if(hha < 0){
          hha = 0;
        }
        hha = hha * hh;
#if 0
        QRect ra = QRect(x - ww/2 + (ww - wwa), y /*- g_yo*/ - hh - dh , wwa, hha);
        painter.drawRect(ra);
#else //メーターを斜めに切る
        QPointF meter[] = {{(float)x + ww/2 - wwa/2 , (float)y /*- g_yo*/ - hh - dh},{(float)x + ww/2 , (float)y /*- g_yo*/ - hh - dh}, {(float)x + ww/2 , (float)y /*- g_yo*/ - hh - dh + hha}, {(float)x + ww/2 - wwa/2 - wwa/2 * hha / hh, (float)y /*- g_yo*/ - hh - dh + hha}};
        painter.drawPolygon(meter, std::size(meter));
#endif
      }
    }

    if(//lead0.getX()[0] > lead1.getX()[0] //lead1がlead0より後ろ
        //y0 > y1 //lead1がlead0より左
        std::abs(y0 - y1) <= 300 //大きく横にずれた→逆
        // ||ほかにv_relやa_relで前方の急減速を表示したり（num==0に表示してみた）
        //&& lead1.getX()[0] < 10 //lead1が自分の前10m以内
    ){
      leadcar_lockon[num].lockOK = leadcar_lockon[num].lockOK + (40 - leadcar_lockon[num].lockOK) / 5;
      //float td = 40;
    } else {
      leadcar_lockon[num].lockOK = leadcar_lockon[num].lockOK + (0 - leadcar_lockon[num].lockOK) / 5;
    }
    float td = leadcar_lockon[num].lockOK;
    //d:10〜100->1〜3へ変換
    if(td >= 3){
      float dd = leadcar_lockon[num].d;
      if(dd < 10){
        dd = 10;
      }
      dd -= 10; //dd=0〜90
      dd /= (90.0/2); //dd=0〜2
      dd += 1; //dd=1〜3
      td /= dd;

      float tlw = 8;
      float tlw_2 = tlw / 2;
      painter.setPen(QPen(QColor(0, 245, 0, prob_alpha), tlw));
      painter.drawLine(r.center().x() , r.top()-tlw_2 , r.center().x() , r.top() - td);
      painter.drawLine(r.left()-tlw_2 , r.center().y() , r.left() - td , r.center().y());
      painter.drawLine(r.right()+tlw_2 , r.center().y() , r.right() + td , r.center().y());
      painter.drawLine(r.center().x() , r.bottom()+tlw_2 , r.center().x() , r.bottom() + td);
    }

  } else if(uiState()->scene.mLockOnButton){
    if(num == 1){
      //推論2番
      //邪魔な前右寄りを走るバイクを認識したい。
      if(//lead0.getX()[0] > lead1.getX()[0] //lead1がlead0より後ろ
        //y0 > y1 //lead1がlead0より左
        std::abs(y0 - y1) > 300 //大きく横にずれた
        // ||ほかにv_relやa_relで前方の急減速を表示したり（num==0に表示してみた）
        //&& lead1.getX()[0] < 10 //lead1が自分の前10m以内
      ){
        //painter.setPen(QPen(QColor(245, 0, 0, prob_alpha), 4));
        //painter.drawEllipse(r); //縁を描く
        //painter.setPen(QPen(QColor(0, 245, 0, prob_alpha), 1)); //文字を後で書くために色を再設定。->文字は赤でもいいや

        //円を（意味不明だから）書かないで、枠ごと赤くする。推論1が推論と別のものを捉えてるのを簡単に認識できる。
        painter.setPen(QPen(QColor(245, 0, 0, prob_alpha), 2));
      } else {
        painter.setPen(QPen(QColor(0, 245, 0, prob_alpha), 2));
      }

      if(leadcar_lockon[0].x > leadcar_lockon[1].x - 20){ //多少逆転しても許容する
        leadcar_lockon[num].lxt = leadcar_lockon[num].lxt + (r.left() - leadcar_lockon[num].lxt) / 20;
        leadcar_lockon[num].lxf = leadcar_lockon[num].lxf + (0 - leadcar_lockon[num].lxf) / 20;
        //painter.drawLine(r.left(),r.top() , 0 , 0);
      } else {
        leadcar_lockon[num].lxt = leadcar_lockon[num].lxt + (r.right() - leadcar_lockon[num].lxt) / 20;
        leadcar_lockon[num].lxf = leadcar_lockon[num].lxf + (width() - leadcar_lockon[num].lxf) / 20;
        //painter.drawLine(r.right(),r.top() , width() , 0);
      }
      float lxt = leadcar_lockon[num].lxt;
      if(lxt < r.left()){
        lxt = r.left();
      } else if(lxt > r.right()){
        lxt = r.right();
      }
      painter.drawLine(lxt,r.top() , leadcar_lockon[num].lxf , 0);

      if(ww >= 80){
        //float dy = y0 - y1;
        //painter.drawText(r, Qt::AlignBottom | Qt::AlignLeft, " " + QString::number(dy,'f',1) + "m");
        //painter.drawText(r, Qt::AlignBottom | Qt::AlignLeft, " " + QString::number(dy,'f',1));
      }
    } else if(num == 2){
      //推論3番
      //事実上ない。動かない0,0に居るみたい？
      painter.setPen(QPen(QColor(0, 245, 0, prob_alpha), 2));
      //painter.drawLine(r.right(),r.center().y() , width() , height());
    } else {
      //推論4番以降。
      //存在していない。
      painter.setPen(QPen(QColor(0, 245, 0, prob_alpha), 2));
      //painter.drawLine(r.left(),r.center().y() , 0 , height());
    }

    painter.drawRect(r);

    //painter.setPen(QPen(QColor(0, 245, 0, prob_alpha), 2));

    if(ww >= 80){
      //ここではy0,y1を参照できない。
      painter.drawText(r, Qt::AlignBottom | Qt::AlignLeft, " " + QString::number(num+1));
    }
    if(ww >= 160 /*80*/){
      //painter.drawText(r, Qt::AlignBottom | Qt::AlignRight, QString::number((int)(lead_data.getProb()*100)) + "％");
      //painter.drawText(r, Qt::AlignBottom | Qt::AlignRight, QString::number(a_rel,'f',1) + "a");
    }
  }
  painter.setPen(Qt::NoPen);
  painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
}

void NvgWindow::paintGL() {
  const int _width = width();  // for ButtonsWindow
  if (prev_width != _width) {
    emit resizeSignal(_width);
    prev_width = _width;
  }

  CameraViewWidget::paintGL();

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::NoPen);

  UIState *s = uiState();
  if (s->worldObjectsVisible()) {

    drawLaneLines(painter, s);

    if (s->scene.longitudinal_control) {
      auto leads = (*s->sm)["modelV2"].getModelV2().getLeadsV3();
      size_t leads_num = leads.size();
      for(size_t i=0; i<leads_num && i < LeadcarLockon_MAX; i++){
        if(leads[i].getProb() > .2){ //信用度20%以上で表示。調整中。
          drawLockon(painter, leads[i], s->scene.lead_vertices[i] , i , leads_num , leads[0] , leads[1]);
        }
      }
      if (leads[0].getProb() > .5) {
        drawLead(painter, leads[0], s->scene.lead_vertices[0] , 0 , leads_num);
      }
      if (leads[1].getProb() > .5 && (std::abs(leads[1].getX()[0] - leads[0].getX()[0]) > 3.0)) {
        drawLead(painter, leads[1], s->scene.lead_vertices[1] , 1 , leads_num);
      }
    }
  }

  drawHud(painter);

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  double fps = fps_filter.update(1. / dt * 1000);
  if (fps < 15) {
    LOGW("slow frame rate: %.2f fps", fps);
  }
  prev_draw_t = cur_draw_t;
  distance_traveled += (*s->sm)["carState"].getCarState().getVEgo() * dt / 1000;
}

void NvgWindow::showEvent(QShowEvent *event) {
  CameraViewWidget::showEvent(event);

  ui_update_params(uiState());
  prev_draw_t = millis_since_boot();
}
