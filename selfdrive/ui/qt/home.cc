#include "selfdrive/ui/qt/home.h"

#include <QHBoxLayout>
#include <QMouseEvent>
#include <QStackedWidget>
#include <QVBoxLayout>

#include "selfdrive/ui/qt/offroad/experimental_mode.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/prime.h"

// HomeWindow: the container for the offroad and onroad UIs

HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setMargin(0);
  main_layout->setSpacing(0);

  sidebar = new Sidebar(this);
  main_layout->addWidget(sidebar);
  QObject::connect(sidebar, &Sidebar::openSettings, this, &HomeWindow::openSettings);

  slayout = new QStackedLayout();
  main_layout->addLayout(slayout);

  home = new OffroadHome(this);
  QObject::connect(home, &OffroadHome::openSettings, this, &HomeWindow::openSettings);
  slayout->addWidget(home);

  onroad = new OnroadWindow(this);
  slayout->addWidget(onroad);

  body = new BodyWindow(this);
  slayout->addWidget(body);

  driver_view = new DriverViewWindow(this);
  connect(driver_view, &DriverViewWindow::done, [=] {
    showDriverView(false);
  });
  slayout->addWidget(driver_view);
  setAttribute(Qt::WA_NoSystemBackground);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &HomeWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &HomeWindow::offroadTransition);
  QObject::connect(uiState(), &UIState::offroadTransition, sidebar, &Sidebar::offroadTransition);
}

extern bool ipaddress_update;
void HomeWindow::showSidebar(bool show) {
  sidebar->setVisible(show);
  if(show == true){
    ipaddress_update = true;
  }
}

bool head_gesture_home;
void HomeWindow::updateState(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  // switch to the generic robot UI
  if (onroad->isVisible() && !body->isEnabled() && sm["carParams"].getCarParams().getNotCar()) {
    body->setEnabled(true);
    slayout->setCurrentWidget(body);
  }

  if(head_gesture_home){
    head_gesture_home = false;
    sidebar->setVisible(false);
  }

  static bool blinker_stat = false;
#if 0
  uint16_t lsta = (uint16_t)(sm["modelV2"].getModelV2().getMeta().getLaneChangeState()); //enum LaneChangeState.preLaneChange == 1 , log.capnp
  double vEgo = sm["carState"].getCarState().getVEgo() * 3.6;
  bool left_blinker = sm["carState"].getCarState().getLeftBlinker() && vEgo > 45 && lsta == 1;
  bool right_blinker = sm["carState"].getCarState().getRightBlinker() && vEgo > 45 && lsta == 1;
  bool back_gear = ((uint16_t)(sm["carState"].getCarState().getGearShifter()) == 4);//car.capnp , enum GearShifterにバックギアが定義されている。
  if(left_blinker || right_blinker || back_gear){
    if(blinker_stat == false){
      blinker_stat = true;
      showDriverView(true);
    }
  } else {
    if(blinker_stat == true){
      blinker_stat = false;
      showDriverView(false);
    }
  }
#else
  static bool lsta_can_get = false;
  uint16_t lsta = 0;
  const bool left_blinker = false; //sm["carState"].getCarState().getLeftBlinker();
  const bool right_blinker = false; //sm["carState"].getCarState().getRightBlinker();
  if(left_blinker == false && right_blinker == false){
    lsta_can_get = true;
  }
  if(lsta_can_get == true){
    lsta = 0; //あまり有用で無いので、レーンチェンジ時の室内カメラ切り替えを廃止。
    //lsta = (uint16_t)(sm["modelV2"].getModelV2().getMeta().getLaneChangeState()); //enum LaneChangeState.preLaneChange == 1 , log.capnp
  }
  bool back_gear = ((uint16_t)(sm["carState"].getCarState().getGearShifter()) == 4);//car.capnp , enum GearShifterにバックギアが定義されている。
  if(back_gear){
    //developer control
    std::string branch = Params().get("GitBranch");
    std::string dongleId = Params().get("DongleId");
    if(branch != "release3" && branch != "release2" && branch.find("release3-pi")  == std::string::npos && branch.find("release2-pi")  == std::string::npos && branch.find("rehearsal")  == std::string::npos && dongleId.find("1131d250d405") == std::string::npos && branch.find("debug") == std::string::npos){
      back_gear = false;
      lsta = 0;
      blinker_stat = false;
    }
  }
  if(lsta == 1 /*left_blinker || right_blinker*/ || back_gear){
    if(blinker_stat == false){
      blinker_stat = true;
      showDriverView(true);
    }
  } else {
    if(blinker_stat == true){
      blinker_stat = false;
      showDriverView(false);
      lsta_can_get = false; //一旦ウインカーを戻すまでは発動しない。
    }
  }
  double vEgo = sm["carState"].getCarState().getVEgo();
  FILE *fp = fopen("/tmp/car_vego.txt","w");
  if(fp != NULL){
    fprintf(fp,"%.2f",vEgo);
    fclose(fp);
  }
#endif
}

void HomeWindow::offroadTransition(bool offroad) {
  body->setEnabled(false);
  sidebar->setVisible(offroad);
  if (offroad) {
    slayout->setCurrentWidget(home);
  } else {
    slayout->setCurrentWidget(onroad);
  }
}

void HomeWindow::showDriverView(bool show) {
  static bool sidebar_disp = false;
  if (show) {
    if (uiState()->scene.started) {
      sidebar_disp = sidebar->isVisible();
      sidebar->setVisible(false);
    } else {
      emit closeSettings();
    }
    slayout->setCurrentWidget(driver_view);
  } else {
    if (!uiState()->scene.started) {
      slayout->setCurrentWidget(home);
    } else {
      slayout->setCurrentWidget(onroad);
      if(sidebar_disp == true){
        sidebar_disp = false;
        sidebar->setVisible(true);
        ipaddress_update = true;
      }
    }
  }
  if (!uiState()->scene.started) {
    sidebar->setVisible(show == false);
  }
}

void HomeWindow::mousePressEvent(QMouseEvent* e) {
  // Handle sidebar collapsing
  if ((onroad->isVisible() || body->isVisible()) && (!sidebar->isVisible() || e->x() > sidebar->width())) {
    if(!sidebar->isVisible()){
      ipaddress_update = true;
    }
    sidebar->setVisible(!sidebar->isVisible());
  }
}

void HomeWindow::mouseDoubleClickEvent(QMouseEvent* e) {
  if(uiState()->scene.started){
    showDriverView(true);
    return;
  }
  HomeWindow::mousePressEvent(e);
  const SubMaster &sm = *(uiState()->sm);
  if (sm["carParams"].getCarParams().getNotCar()) {
    if (onroad->isVisible()) {
      slayout->setCurrentWidget(body);
    } else if (body->isVisible()) {
      slayout->setCurrentWidget(onroad);
    }
    showSidebar(false);
  }
}

// OffroadHome: the offroad home page
void OffroadHome::poweroff() {
  if (!uiState()->engaged()) {
      if (!uiState()->engaged()) {
        params.putBool("DoShutdown", true);
      }
  // } else {
  //   ConfirmationDialog::alert(tr("Disengage to Power Off"), this);
  }
}

OffroadHome::OffroadHome(QWidget* parent) : QFrame(parent) {
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(40, 40, 40, 40);

  // top header
  QHBoxLayout* header_layout = new QHBoxLayout();
  header_layout->setContentsMargins(0, 0, 0, 0);
  header_layout->setSpacing(16);

  update_notif = new QPushButton(tr("UPDATE"));
  update_notif->setVisible(false);
  update_notif->setStyleSheet("background-color: #364DEF;");
  QObject::connect(update_notif, &QPushButton::clicked, [=]() { center_layout->setCurrentIndex(1); });
  header_layout->addWidget(update_notif, 0, Qt::AlignHCenter | Qt::AlignLeft);

  alert_notif = new QPushButton();
  alert_notif->setVisible(false);
  alert_notif->setStyleSheet("background-color: #E22C2C;");
  QObject::connect(alert_notif, &QPushButton::clicked, [=] { center_layout->setCurrentIndex(2); });
  header_layout->addWidget(alert_notif, 0, Qt::AlignHCenter | Qt::AlignLeft);

  version = new ElidedLabel();
  header_layout->addWidget(version, 0, Qt::AlignHCenter | Qt::AlignRight);

  main_layout->addLayout(header_layout);

  // main content
  main_layout->addSpacing(25);
  center_layout = new QStackedLayout();

  QWidget *home_widget = new QWidget(this);
  {
    QHBoxLayout *home_layout = new QHBoxLayout(home_widget);
    home_layout->setContentsMargins(0, 0, 0, 0);
    home_layout->setSpacing(30);

    // left: PrimeAdWidget
    QStackedWidget *left_widget = new QStackedWidget(this);
    QVBoxLayout *left_prime_layout = new QVBoxLayout();
    QWidget *prime_user = new PrimeUserWidget();
    prime_user->setStyleSheet(R"(
    border-radius: 10px;
    background-color: #333333;
    )");
    left_prime_layout->addWidget(prime_user);
    left_prime_layout->addStretch();
    left_widget->addWidget(new LayoutWidget(left_prime_layout));
    left_widget->addWidget(new PrimeAdWidget);
    left_widget->setStyleSheet("border-radius: 10px;");

    connect(uiState()->prime_state, &PrimeState::changed, [left_widget]() {
      left_widget->setCurrentIndex(uiState()->prime_state->isSubscribed() ? 0 : 1);
    });

    home_layout->addWidget(left_widget, 1);

    // right: ExperimentalModeButton, SetupWidget
    QWidget* right_widget = new QWidget(this);
    QVBoxLayout* right_column = new QVBoxLayout(right_widget);
    right_column->setContentsMargins(0, 0, 0, 0);
    right_widget->setFixedWidth(750);
    right_column->setSpacing(30);

    ExperimentalModeButton *experimental_mode = new ExperimentalModeButton(this);
    QObject::connect(experimental_mode, &ExperimentalModeButton::openSettings, this, &OffroadHome::openSettings);
    right_column->addWidget(experimental_mode, 1);

    SetupWidget *setup_widget = new SetupWidget;
    QObject::connect(setup_widget, &SetupWidget::openSettings, this, &OffroadHome::openSettings);
    right_column->addWidget(setup_widget, 1);

    QPushButton *poweroff_btn = new QPushButton(tr("Power Off"));
    poweroff_btn->setObjectName("poweroff_btn");
    right_column->addWidget(poweroff_btn , 1);
    QObject::connect(poweroff_btn, &QPushButton::clicked, this, &OffroadHome::poweroff);

    home_layout->addWidget(right_widget, 1);
  }
  center_layout->addWidget(home_widget);

  // add update & alerts widgets
  update_widget = new UpdateAlert();
  QObject::connect(update_widget, &UpdateAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(update_widget);
  alerts_widget = new OffroadAlert();
  QObject::connect(alerts_widget, &OffroadAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(alerts_widget);

  main_layout->addLayout(center_layout, 1);

  // set up refresh timer
  timer = new QTimer(this);
  timer->callOnTimeout(this, &OffroadHome::refresh);

  setStyleSheet(R"(
    * {
      color: white;
    }
    OffroadHome {
      background-color: black;
    }
    OffroadHome > QPushButton {
      padding: 15px 30px;
      border-radius: 5px;
      font-size: 40px;
      font-weight: 500;
    }
    OffroadHome > QLabel {
      font-size: 55px;
    }
    #poweroff_btn {font-size: 60px; font-weight: bold; height: 120px; border-radius: 10px; background-color: #E22C2C; }
    #poweroff_btn:pressed { background-color: #FF2424; }
  )");
}

void OffroadHome::showEvent(QShowEvent *event) {
  refresh();
  timer->start(10 * 1000);
}

void OffroadHome::hideEvent(QHideEvent *event) {
  timer->stop();
}

void OffroadHome::refresh() {
  version->setText(getBrand() + " " +  QString::fromStdString(params.get("UpdaterCurrentDescription")));

  bool updateAvailable = update_widget->refresh();
  if(updateAvailable){
    std::system("echo 1 > /data/force_prebuild");
  }
  int alerts = alerts_widget->refresh();

  // pop-up new notification
  int idx = center_layout->currentIndex();
  if (!updateAvailable && !alerts) {
    idx = 0;
  } else if (updateAvailable && (!update_notif->isVisible() || (!alerts && idx == 2))) {
    idx = 1;
  } else if (alerts && (!alert_notif->isVisible() || (!updateAvailable && idx == 1))) {
    idx = 2;
  }
  center_layout->setCurrentIndex(idx);

  update_notif->setVisible(updateAvailable);
  alert_notif->setVisible(alerts);
  if (alerts) {
    alert_notif->setText(QString::number(alerts) + (alerts > 1 ? tr(" ALERTS") : tr(" ALERT")));
  }
}
