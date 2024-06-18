#include "selfdrive/ui/qt/onroad/onroad_home.h"

#include <QPainter>
#include <QStackedLayout>

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_helpers.h"
#include "selfdrive/ui/qt/maps/map_panel.h"
#endif

#include "selfdrive/ui/qt/util.h"

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(UI_BORDER_SIZE);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  nvg = new AnnotatedCameraWidget(VISION_STREAM_ROAD, this);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addWidget(nvg);

  if (getenv("DUAL_CAMERA_VIEW")) {
    CameraWidget *arCam = new CameraWidget("camerad", VISION_STREAM_ROAD, true, this);
    split->insertWidget(0, arCam);
  }

  if (getenv("MAP_RENDER_VIEW")) {
    CameraWidget *map_render = new CameraWidget("navd", VISION_STREAM_MAP, false, this);
    split->insertWidget(0, map_render);
  }

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);
  QObject::connect(uiState(), &UIState::primeChanged, this, &OnroadWindow::primeChanged);
}

bool mapVisible;
bool head_gesture_onroad_home;
bool head_gesture_onroad_home_map_on;
bool head_gesture_map_north_heading_toggle;
void OnroadWindow::updateState(const UIState &s) {
  if (!s.scene.started) {
    return;
  }

  if (s.scene.map_on_left) {
    split->setDirection(QBoxLayout::LeftToRight);
  } else {
    split->setDirection(QBoxLayout::RightToLeft);
  }

  alerts->updateState(s);
  nvg->updateState(s);
  mapVisible = isMapVisible();

  QColor bgColor = bg_colors[s.status];
  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }

  if(head_gesture_onroad_home){
    head_gesture_onroad_home = false;
    if (map != nullptr) {
      void soundButton(int onOff);
      soundButton(!this->isMapVisible());
      map->setVisible(!this->isMapVisible());
    }
  }

  if(head_gesture_onroad_home_map_on){ //ジェスチャーでノースアップ切り替えした時の強制地図出し。
    head_gesture_onroad_home_map_on = false;
    if (map != nullptr) {
      if(this->isMapVisible() == false){
        //地図を強制的に出す。
        map->setVisible(true);
        void soundButton(int onOff);
        soundButton(true);
      } else {
        //地図が出ていたらノース↔︎ヘディング切り替え信号を送る。
        head_gesture_map_north_heading_toggle = true;
      }
    }
  }

}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
#ifdef ENABLE_MAPS
  if (map != nullptr) {
    bool sidebarVisible = geometry().x() > 0;
    bool show_map = !sidebarVisible;
    map->setVisible(show_map && !map->isVisible());
  }
#endif
  // propagation event to parent(HomeWindow)
  QWidget::mousePressEvent(e);
}

void OnroadWindow::createMapWidget() {
#ifdef ENABLE_MAPS
  auto m = new MapPanel(get_mapbox_settings());
  map = m;
  QObject::connect(m, &MapPanel::mapPanelRequested, this, &OnroadWindow::mapPanelRequested);
  QObject::connect(nvg->map_settings_btn, &MapSettingsButton::clicked, m, &MapPanel::toggleMapSettings);
  nvg->map_settings_btn->setEnabled(true);

  std::string my_mapbox_width = util::read_file("/data/mb_width_rate.txt");
  m->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding));
  if(my_mapbox_width.empty() == false){
    this->mb_width_rate = std::stof(my_mapbox_width);
    m->setFixedWidth((topWidget(this)->width() * this->mb_width_rate - UI_BORDER_SIZE));
  } else {
    m->setFixedWidth(topWidget(this)->width() / 2 - UI_BORDER_SIZE);
  }
  split->insertWidget(0, m);
  // hidden by default, made visible when navRoute is published
  m->setVisible(false);
#endif
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    bool mapbox_extra = false;
    std::string my_mapbox_token = util::read_file("/data/mb_token.txt");
    if(my_mapbox_token.empty() == false){
      mapbox_extra = true;
    }
    if (map == nullptr && (uiState()->hasPrime() || !MAPBOX_TOKEN.isEmpty() || mapbox_extra)) {
      createMapWidget();
    }
  }
#endif
  alerts->clear();
}

void OnroadWindow::primeChanged(bool prime) {
#ifdef ENABLE_MAPS
  if (map && (!prime && MAPBOX_TOKEN.isEmpty())) {
    nvg->map_settings_btn->setEnabled(false);
    nvg->map_settings_btn->setVisible(false);
    map->deleteLater();
    map = nullptr;
  } else if (!map && (prime || !MAPBOX_TOKEN.isEmpty())) {
    createMapWidget();
  }
#endif
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));
}
