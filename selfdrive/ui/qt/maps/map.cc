#include "selfdrive/ui/qt/maps/map.h"

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <QMapLibre/Utils>

#include <QDebug>

#include "common/params.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/ui.h"


const int INTERACTION_TIMEOUT = 100;

const float MAX_ZOOM0 = 17;
float MAX_ZOOM_;
#define MAX_ZOOM calc_max_zoom()
float zoom_offset;
const float MIN_ZOOM = 14;
const float MAX_PITCH = 50;
#define MIN_PITCH calc_pich()
float MIN_PITCH_ = 0;
const float MAP_SCALE = 2;

std::string my_mapbox_triangle;
std::string my_mapbox_style;
std::string my_mapbox_style_night;
int night_mode = -1;
//int north_up = 0; //1で北上モード
#define north_up chk_north_up()
bool chg_pitch;
extern void setButtonInt(const char*fn , int num);
extern int getButtonInt(const char*fn , int defaultNum);
float calc_pich(){
  if(MIN_PITCH_ < 0){
    return 0; //north_up用。方位磁石タップにノースアップも混ぜる0->10->20->30->40->ノースアップ
  }
  return MIN_PITCH_;
}
void max_zoom_pitch_effect(){
  MAX_ZOOM_ = MAX_ZOOM0 + sin(MIN_PITCH * M_PI / 180) * 1.7; //30度でMAX_ZOOM=18くらいになる。
  if(MAX_ZOOM_ > 22){
    MAX_ZOOM_ = 22;
  }
}
float calc_max_zoom(){
  const float m_o = MAX_ZOOM_+zoom_offset;
  if(m_o > 22){
    zoom_offset = 22 - MAX_ZOOM_;
    return 22;
  }
  if(m_o < MIN_ZOOM){
    zoom_offset = MIN_ZOOM - MAX_ZOOM_;
    return MIN_ZOOM;
  }
  return m_o; //もしくはMIN_ZOOMを、MAX_ZOOMより大きくならないように小さくする制御も考えられる。
}
int chk_north_up(){
  if(MIN_PITCH_ < 0){
    return 1; //ピッチ角がマイナスならノースアップ。
  }
  return 0;
}

//MapWindow::MapWindow(const QMapLibre::Settings &settings) : m_settings(settings), velocity_filter(0, 10, 0.05, false) {
MapWindow::MapWindow(const QMapLibre::Settings &settings, QFrame *panel) : m_settings(settings), m_panel(panel), velocity_filter(0, 10, 0.05, false) {
  QObject::connect(uiState(), &UIState::uiUpdate, this, &MapWindow::updateState);

  map_overlay = new QWidget (this);
  map_overlay->setAttribute(Qt::WA_TranslucentBackground, true);
  QVBoxLayout *overlay_layout = new QVBoxLayout(map_overlay);
  overlay_layout->setContentsMargins(0, 0, 0, 0);

  std::string my_mapbox_offline = util::read_file("/data/mb_offline.txt");
  int map_offline_mode = 0;
  if(my_mapbox_offline.empty() == false){
    map_offline_mode = std::stoi(my_mapbox_offline);
  }
  QMapLibre::setNetworkMode(map_offline_mode ? QMapLibre::NetworkMode::Offline : QMapLibre::NetworkMode::Online);

  my_mapbox_triangle = util::read_file("/data/mb_triangle.svg");
  MIN_PITCH_ = getButtonInt("/data/mb_pitch.txt",0);
  max_zoom_pitch_effect();

  // Instructions
  map_instructions = new MapInstructions(this);
  map_instructions->setVisible(false);

  map_eta = new MapETA(this);
  map_eta->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  map_eta->setFixedHeight(120);

  error = new QLabel(this);
  error->setStyleSheet(R"(color:white;padding:50px 11px;font-size: 90px; background-color:rgba(0, 0, 0, 150);)");
  error->setAlignment(Qt::AlignCenter);

  overlay_layout->addWidget(error);
  overlay_layout->addWidget(map_instructions);
  overlay_layout->addStretch(1);
  overlay_layout->addWidget(map_eta);

  last_position = coordinate_from_param("LastGPSPosition");
  grabGesture(Qt::GestureType::PinchGesture);
  qDebug() << "MapWindow initialized";

  map_limitspeed = new MapLimitspeed(this);
  QObject::connect(this, &MapWindow::LimitspeedChanged, map_limitspeed, &MapLimitspeed::updateLimitspeed);
#define LS_SIZE 180
  map_limitspeed->setFixedHeight(LS_SIZE);
  map_limitspeed->setFixedWidth(LS_SIZE);
  map_limitspeed->setVisible(false);

#define BS_SIZE_H 180
#define BS_SIZE_W 150
  map_bearing_scale = new MapBearingScale(this);
  QObject::connect(this, &MapWindow::BearingScaleChanged, map_bearing_scale, &MapBearingScale::updateBearingScale);
  map_bearing_scale->setFixedHeight(BS_SIZE_H);
  map_bearing_scale->setFixedWidth(BS_SIZE_W);
  map_bearing_scale->setVisible(false);

}

MapWindow::~MapWindow() {
  makeCurrent();
}

bool check_night_mode(){
#if 0
  auto q_time = QDateTime::currentDateTime(); //.addSecs(9*3600); //UTCなので9時間足す
  QString g_hour = q_time.toString("HH:mm");
  //bool night = (QString::compare(g_hour,"17:00") >= 0 || QString::compare(g_hour,"06:00") < 0);
  bool night = (strcmp(g_hour.toUtf8().data(),"17:00") >= 0 || strcmp(g_hour.toUtf8().data(),"06:00") < 0);
#else
  //時間ではなく、カメラ輝度で判定する
  bool night = false;
  UIState *s = uiState();
  if (s->scene.started) {
    float clipped_brightness = s->scene.light_sensor;

    // CIE 1931 - https://www.photonstophotos.net/GeneralTopics/Exposure/Psychometric_Lightness_and_Gamma.htm
    if (clipped_brightness <= 8) {
      clipped_brightness = (clipped_brightness / 903.3);
    } else {
      clipped_brightness = std::pow((clipped_brightness + 16.0) / 116.0, 3.0);
    }

    // Scale back to 10% to 100%
    clipped_brightness = std::clamp(100.0f * clipped_brightness, 10.0f, 100.0f);

    //night = clipped_brightness < 50; //どのくらいが妥当？
    night = clipped_brightness < (night_mode == -1 ? 80 : (night_mode == 1 ? 85 : 75)); //ばたつかないようにする。80程度でかなり夕方。
#if 0 //昼間で97以上。多少影に入っても関係ない。トンネルで一気に10まで下がった。上の制御で十分だが、夕方の切り替わるタイミングはlight_sensorの挙動依存となる。
    if(1 || (night == true && night_mode != 1) || (night == false && night_mode != 0)){
      //切り替わるなら書き出し。
      FILE *fp = fopen("/tmp/brightness_info.txt","w"); //write_fileだと書き込めないが、こちらは書き込めた。
      if(fp != NULL){
        char buf[32];
        sprintf(buf,"brightness:%.1f",clipped_brightness);
        fwrite(buf,strlen(buf),1,fp);
        fclose(fp);
      }
    }
#endif
  }
#endif
  return night;
}

void MapWindow::initLayers() {
  // This doesn't work from initializeGL
  if (!m_map->layerExists("modelPathLayer")) {
    qDebug() << "Initializing modelPathLayer";
    QVariantMap modelPath;
    //modelPath["id"] = "modelPathLayer";
    modelPath["type"] = "line";
    modelPath["source"] = "modelPathSource";
    m_map->addLayer("modelPathLayer", modelPath);
    m_map->setPaintProperty("modelPathLayer", "line-color", QColor("red"));
    m_map->setPaintProperty("modelPathLayer", "line-width", 5.0);
    m_map->setLayoutProperty("modelPathLayer", "line-cap", "round");
  }
  if (!m_map->layerExists("navLayer")) {
    qDebug() << "Initializing navLayer";
    QVariantMap nav;
    nav["type"] = "line";
    nav["source"] = "navSource";
    m_map->addLayer("navLayer", nav, "road-intersection");

    QVariantMap transition;
    transition["duration"] = 400;  // ms
    m_map->setPaintProperty("navLayer", "line-color", QColor("#31a1ee"));
    m_map->setPaintProperty("navLayer", "line-color-transition", transition);
    m_map->setPaintProperty("navLayer", "line-width", 7.5);
    m_map->setLayoutProperty("navLayer", "line-cap", "round");
  }
  if (!m_map->layerExists("pinLayer")) {
    qDebug() << "Initializing pinLayer";
    m_map->addImage("default_marker", QImage("../assets/navigation/default_marker.svg"));
    QVariantMap pin;
    pin["type"] = "symbol";
    pin["source"] = "pinSource";
    m_map->addLayer("pinLayer", pin);
    m_map->setLayoutProperty("pinLayer", "icon-pitch-alignment", "viewport");
    m_map->setLayoutProperty("pinLayer", "icon-image", "default_marker");
    m_map->setLayoutProperty("pinLayer", "icon-ignore-placement", true);
    m_map->setLayoutProperty("pinLayer", "icon-allow-overlap", true);
    m_map->setLayoutProperty("pinLayer", "symbol-sort-key", 0);
    m_map->setLayoutProperty("pinLayer", "icon-anchor", "bottom");
  }
  if (!m_map->layerExists("carPosLayer")) {
    qDebug() << "Initializing carPosLayer";
    if(my_mapbox_triangle.empty() == false){
      m_map->addImage("label-arrow", QImage("/data/mb_triangle.svg"));
    } else {
      m_map->addImage("label-arrow", QImage("../assets/images/triangle.svg"));
    }

    QVariantMap carPos;
    carPos["type"] = "symbol";
    carPos["source"] = "carPosSource";
    m_map->addLayer("carPosLayer", carPos);
    m_map->setLayoutProperty("carPosLayer", "icon-pitch-alignment", "map");
    m_map->setLayoutProperty("carPosLayer", "icon-image", "label-arrow");
    m_map->setLayoutProperty("carPosLayer", "icon-size", 0.5);
    m_map->setLayoutProperty("carPosLayer", "icon-ignore-placement", true);
    m_map->setLayoutProperty("carPosLayer", "icon-allow-overlap", true);
    // TODO: remove, symbol-sort-key does not seem to matter outside of each layer
    m_map->setLayoutProperty("carPosLayer", "symbol-sort-key", 0);
  }

  if(night_mode >= 0 && my_mapbox_style_night.empty() == false){
#if 1
    if(check_night_mode()){
      if(night_mode != 1 /*&& my_mapbox_style_night.empty() == false*/){
        night_mode = 1;
        m_map->setStyleUrl(my_mapbox_style_night.c_str());
      }
    } else {
      if(night_mode != 0 && my_mapbox_style.empty() == false){
        night_mode = 0;
        m_map->setStyleUrl(my_mapbox_style.c_str());
      }
    }
#else
    //切り替えテスト
    static long long int test_counter = 0;
    test_counter ++;
    if(test_counter % 10 == 5){
      MIN_PITCH_ = 60;
      m_map->setStyleUrl(my_mapbox_style_night.c_str());
    }
    if(test_counter % 10 == 0){
      MIN_PITCH_ = 0;
      m_map->setStyleUrl(my_mapbox_style.c_str());
    }
#endif
  }
}

bool now_navigation = false;
int style_reload = 0;
float g_latitude;
bool head_gesture_map_north_heading_toggle;
bool map_pitch_up,map_pitch_down;
void MapWindow::updateState(const UIState &s) {
  if (!uiState()->scene.started) {
    return;
  }
  const SubMaster &sm = *(s.sm);
  update();

  static bool already_vego_over_8 = false;
  if(already_vego_over_8 == false && sm["carState"].getCarState().getVEgo() > 1/3.6){ //8->4->1km/h
    already_vego_over_8 = true; //一旦時速8km/h以上になった。
  }
  if (sm.updated("liveLocationKalman")) {
    auto locationd_location = sm["liveLocationKalman"].getLiveLocationKalman();
    auto locationd_pos = locationd_location.getPositionGeodetic();
    auto locationd_orientation = locationd_location.getCalibratedOrientationNED();
    auto locationd_velocity = locationd_location.getVelocityCalibrated();
    auto locationd_ecef = locationd_location.getPositionECEF();

    locationd_valid = (locationd_pos.getValid() && locationd_orientation.getValid() && locationd_velocity.getValid() && locationd_ecef.getValid());
    if (locationd_valid) {
      // Check std norm
      auto pos_ecef_std = locationd_ecef.getStd();
      bool pos_accurate_enough = sqrt(pow(pos_ecef_std[0], 2) + pow(pos_ecef_std[1], 2) + pow(pos_ecef_std[2], 2)) < 100;
      locationd_valid = pos_accurate_enough;
    }

    if (locationd_valid) {
      if (already_vego_over_8 == true) {
        last_position = QMapLibre::Coordinate(locationd_pos.getValue()[0], locationd_pos.getValue()[1]);
        last_bearing = RAD2DEG(locationd_orientation.getValue()[2]);
      }
      velocity_filter.update(std::max(10/3.6, locationd_velocity.getValue()[0]));

      if (loaded_once || (m_map && !m_map.isNull() && m_map->isFullyLoaded())) {
        if(false && map_pitch_up){
          map_pitch_up = false;
          void soundButton(int onOff);
          soundButton(true);
          if(MIN_PITCH_ < 0){
            MIN_PITCH_ = -10; //ジェスチャー切り替えでノースアップが-10以外になっている可能性を考慮。
          }

          MIN_PITCH_ /= 10;
          MIN_PITCH_ += 1;
          if(MIN_PITCH_ > 4){
            MIN_PITCH_ = -1;
          }
          MIN_PITCH_ *= 10;
          max_zoom_pitch_effect();
          setButtonInt("/data/mb_pitch.txt",MIN_PITCH_); //MIN_PITCH_ = 0,10,20,30,40度,ノースアップから選択
          chg_pitch = true;
        }

        if(map_pitch_down){
          map_pitch_down = false;
          if(MIN_PITCH_ < 0){
            MIN_PITCH_ = -10; //ジェスチャー切り替えでノースアップが-10以外になっている可能性を考慮。
          }

          MIN_PITCH_ /= 10;
          MIN_PITCH_ -= 1;
          if(MIN_PITCH_ < -1){
            MIN_PITCH_ = 4;
          }
          MIN_PITCH_ *= 10;
          max_zoom_pitch_effect();
          setButtonInt("/data/mb_pitch.txt",MIN_PITCH_); //MIN_PITCH_ = 0,10,20,30,40度,ノースアップから選択
          chg_pitch = true;

          if(MIN_PITCH_ == 0){
            void soundPikiri();
            soundPikiri();
          } else {
            extern void soundPipo();
            soundPipo();
          }
        }

        if(head_gesture_map_north_heading_toggle){
          head_gesture_map_north_heading_toggle = false;
          void soundButton(int onOff);
          soundButton(true);
          if(MIN_PITCH_ == 0){ // 0<->-1
            MIN_PITCH_ = -1;
          } else if(MIN_PITCH_ == -1){
            MIN_PITCH_ = 0;
          } else {
            MIN_PITCH_ = -MIN_PITCH_;
          }
          max_zoom_pitch_effect();
          setButtonInt("/data/mb_pitch.txt",MIN_PITCH_); //MIN_PITCH_ = 0,10,20,30,40度,ノースアップから選択
          chg_pitch = true;
        }

        if(north_up == 0){
          if(m_map->margins().top() == 0){
            m_map->setMargins({0, 350, 0, 50});
            chg_pitch = true;
            max_zoom_pitch_effect();
          }
          if(chg_pitch){
            chg_pitch = false;
            if (sm.valid("navInstruction")) {
              m_map->setPitch(MAX_PITCH); //ナビ中
            } else {
              m_map->setPitch(MIN_PITCH);
            }
            m_map->setZoom(util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM));
          }
        } else {
          if(m_map->margins().top() != 0){
            m_map->setMargins({0, 0, 0, 0});
            m_map->setPitch(0);
            m_map->setBearing(0);
            MAX_ZOOM_ = MAX_ZOOM0;
            //max_zoom_pitch_effect(); //これだとノースアップでも方位磁石タップでスケールが変わってしまう。
          }
        }
      }
      g_latitude = locationd_pos.getValue()[0];

      static unsigned int LimitspeedChanged_ct;
      if ((LimitspeedChanged_ct++ % 10) == 0 && last_bearing && last_position) { //0.5秒ごとに速度標識を更新
        map_limitspeed->setVisible(true);
        emit LimitspeedChanged(rect().width());

        map_bearing_scale->setVisible(true);
        emit BearingScaleChanged(rect().width(),*last_bearing,util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM) , g_latitude);
      }
    }
  }
  // static bool emit_LimitspeedChanged_first_set = false; //最初非表示にしているので要らないかも
  // if(emit_LimitspeedChanged_first_set == false){
  //   emit_LimitspeedChanged_first_set = true;
  //   //このタイミングではrect().width()の値がおかしい。
  //   emit LimitspeedChanged(rect().width()); //最初に右に寄せるために必要。
  //   emit BearingScaleChanged(rect().width(),*last_bearing,util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM) , g_latitude);
  // }

  if (sm.updated("navRoute") && sm["navRoute"].getNavRoute().getCoordinates().size()) {
    auto nav_dest = coordinate_from_param("NavDestination");
    bool allow_open = std::exchange(last_valid_nav_dest, nav_dest) != nav_dest &&
                      nav_dest && !isVisible();
    qWarning() << "Got new navRoute from navd. Opening map:" << allow_open;

    // Show map on destination set/change
    if (allow_open) {
      emit requestSettings(false);
      emit requestVisible(true);
    }
  }

  loaded_once = loaded_once || (m_map && m_map->isFullyLoaded());
  if (!loaded_once) {
    setError(tr("Map Loading"));
    return;
  }
  initLayers();

  if (!locationd_valid) {
    setError(tr("Waiting for GPS"));
  } else if (routing_problem) {
    setError(tr("Waiting for route"));
  } else {
    setError("");
  }

  if (locationd_valid) {
    // Update current location marker
    auto point = coordinate_to_collection(*last_position);
    QMapLibre::Feature feature1(QMapLibre::Feature::PointType, point, {}, {});
    QVariantMap carPosSource;
    carPosSource["type"] = "geojson";
    carPosSource["data"] = QVariant::fromValue<QMapLibre::Feature>(feature1);
    m_map->updateSource("carPosSource", carPosSource);

    // Map bearing isn't updated when interacting, keep location marker up to date
    if (last_bearing) {
      m_map->setLayoutProperty("carPosLayer", "icon-rotate", *last_bearing - m_map->bearing());
    }
  }

  if (interaction_counter == 0) {
    if (last_position) m_map->setCoordinate(*last_position);
    if(north_up == 0){
      if (last_bearing) m_map->setBearing(*last_bearing);
    } else {
      if (last_bearing) m_map->setBearing(0);
    }
    m_map->setZoom(util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM));
  } else {
    interaction_counter--;
  }

  if (sm.updated("navInstruction")) {
    // an invalid navInstruction packet with a nav destination is only possible if:
    // - API exception/no internet
    // - route response is empty
    // - any time navd is waiting for recompute_countdown
    routing_problem = !sm.valid("navInstruction") && coordinate_from_param("NavDestination").has_value();

    if (sm.valid("navInstruction")) {
      auto i = sm["navInstruction"].getNavInstruction();
      map_eta->updateETA(i.getTimeRemaining(), i.getTimeRemainingTypical(), i.getDistanceRemaining());

      if (locationd_valid) {
        if(north_up == 0){
          m_map->setMargins({0, 350, 0, 50});
          m_map->setPitch(MAX_PITCH); // TODO: smooth pitching based on maneuver distance
        } else {
          m_map->setMargins({0, 0, 0, 0});
          m_map->setPitch(0);
        }
        map_instructions->updateInstructions(i);
      }
      if(now_navigation == false && night_mode >= 0){
        night_mode = -1; //ナビ中の昼夜切り替えを無効にする。昼夜切り替えでルートが消えるから、この処理は必須。
        m_map->setStyleUrl("mapbox://styles/commaai/clkqztk0f00ou01qyhsa5bzpj"); //ナビ中はスタイルを公式に戻す。
        style_reload = 10;
      }
      now_navigation = true;
    } else {
      if(now_navigation == true){
        if(my_mapbox_style_night.empty() == false && check_night_mode()){ //夜だったら
          night_mode = 1;
          m_map->setStyleUrl(my_mapbox_style_night.c_str());
        } else if(my_mapbox_style.empty() == false){ //昼だったら
          night_mode = 0;
          m_map->setStyleUrl(my_mapbox_style.c_str());
        }
      }
      now_navigation = false;
      clearRoute();
    }
  }

  if(style_reload == 1 && locationd_valid == true && routing_problem == false){
    style_reload --;
    FILE *fp = fopen("/tmp/route_style_reload.txt","w");
    if(fp != NULL){
      fprintf(fp,"%d",1);
      fclose(fp);
    }
  } else if(style_reload > 1){
    style_reload --;
  }

  if (sm.rcv_frame("navRoute") != route_rcv_frame) {
    qWarning() << "Updating navLayer with new route";
    auto route = sm["navRoute"].getNavRoute();
    auto route_points = capnp_coordinate_list_to_collection(route.getCoordinates());
    QMapLibre::Feature feature(QMapLibre::Feature::LineStringType, route_points, {}, {});
    QVariantMap navSource;
    navSource["type"] = "geojson";
    navSource["data"] = QVariant::fromValue<QMapLibre::Feature>(feature);
    m_map->updateSource("navSource", navSource);
    m_map->setLayoutProperty("navLayer", "visibility", "visible");

    route_rcv_frame = sm.rcv_frame("navRoute");
    updateDestinationMarker();
  }
}

void MapWindow::setError(const QString &err_str) {
  if (err_str != error->text()) {
    error->setText(err_str);
    error->setVisible(!err_str.isEmpty());
    if (!err_str.isEmpty()) map_instructions->setVisible(false);
  }
}

void MapWindow::resizeGL(int w, int h) {
  m_map->resize(size() / MAP_SCALE);
  map_overlay->setFixedSize(width(), height());
}

void MapWindow::initializeGL() {
  m_map.reset(new QMapLibre::Map(this, m_settings, size(), 1));

  zoom_offset = (float)((double)getButtonInt("/data/mb_zoom_offset.txt",0) / 1000);
  //不要north_up = getButtonInt("/data/mb_north_up.txt",0);

  if (last_position) {
    m_map->setCoordinateZoom(*last_position, MAX_ZOOM);
    last_bearing = (float)((double)getButtonInt("/data/mb_last_bearing_info.txt",0) / 1000);
    if (last_bearing) m_map->setBearing(*last_bearing);
  } else {
    m_map->setCoordinateZoom(QMapLibre::Coordinate(64.31990695292795, -149.79038934046247), MIN_ZOOM);
  }

  if(north_up == 0){
    m_map->setMargins({0, 350, 0, 50});
    m_map->setPitch(MIN_PITCH);
  } else {
    m_map->setMargins({0, 0, 0, 0});
    m_map->setPitch(0);
    m_map->setBearing(0);
  }

  my_mapbox_style = util::read_file("/data/mb_style.txt");
  if(my_mapbox_style.empty() == false){
    while(my_mapbox_style.c_str()[my_mapbox_style.length()-1] == 0x0a){
      my_mapbox_style = my_mapbox_style.substr(0,my_mapbox_style.length()-1);
    }
  }
  my_mapbox_style_night = util::read_file("/data/mb_style_night.txt");
  if(my_mapbox_style_night.empty() == false){
    while(my_mapbox_style_night.c_str()[my_mapbox_style_night.length()-1] == 0x0a){
      my_mapbox_style_night = my_mapbox_style_night.substr(0,my_mapbox_style_night.length()-1);
    }
  }

  if(my_mapbox_style_night.empty() == false && check_night_mode()){ //夜だったら
    night_mode = 1;
    m_map->setStyleUrl(my_mapbox_style_night.c_str());
  } else if(my_mapbox_style.empty() == false){ //昼だったら
    night_mode = 0;
    m_map->setStyleUrl(my_mapbox_style.c_str());
  } else {
    // m_map->setStyleUrl("mapbox://styles/commaai/clj7g5vrp007b01qzb5ro0i4j"); 公式旧スタイル
    m_map->setStyleUrl("mapbox://styles/commaai/clkqztk0f00ou01qyhsa5bzpj");
  }

  QObject::connect(m_map.data(), &QMapLibre::Map::mapChanged, [=](QMapLibre::Map::MapChange change) {
    // set global animation duration to 0 ms so visibility changes are instant
    if (change == QMapLibre::Map::MapChange::MapChangeDidFinishLoadingStyle) {
      m_map->setTransitionOptions(0, 0);
    }
    if (change == QMapLibre::Map::MapChange::MapChangeDidFinishLoadingMap) {
      loaded_once = true;
    }
  });
}

void MapWindow::paintGL() {
  if (!isVisible() || m_map.isNull()) return;
  m_map->render();
}

void MapWindow::clearRoute() {
  if (!m_map.isNull()) {
    m_map->setLayoutProperty("navLayer", "visibility", "none");
    if(north_up == 0){
      m_map->setMargins({0, 350, 0, 50});
      m_map->setPitch(MIN_PITCH);
    } else {
      m_map->setMargins({0, 0, 0, 0});
      m_map->setPitch(0);
    }
    updateDestinationMarker();
  }

  map_instructions->setVisible(false);
  map_eta->setVisible(false);
  last_valid_nav_dest = std::nullopt;
}

bool map_dynamic_edit_x; //横スワイプでウインドウリサイズ
bool map_dynamic_edit_y; //縦スワイプで高度変更
void MapWindow::mousePressEvent(QMouseEvent *ev) {
  m_lastPos = ev->localPos();
  ev->accept();

  map_dynamic_edit_x = false;
  map_dynamic_edit_y = false;
  //方位磁石の上の端っこを上下でMAX_ZOOM調整。
  if(m_lastPos.y() < 1080 - 200){ //ボタンの位置は避ける。
    if(uiState()->scene.map_on_left){
      if(m_lastPos.x() > this->width() - 150){
        map_dynamic_edit_y = true;
        m_lastGlbPos = ev->globalPos();
      }
      if(m_lastPos.x() > this->width() - 100){
        map_dynamic_edit_x = true;
        m_lastGlbPos = ev->globalPos();
      }
    } else {
      if(m_lastPos.x() < 150){ //ちょっと広めに取らないと感度悪い。右ハンドルだからタッチの見た目ズレ？
        map_dynamic_edit_y = true;
        m_lastGlbPos = ev->globalPos();
      }
      if(m_lastPos.x() < 100){ //ちょっと広めに取らないと感度悪い。右ハンドルだからタッチの見た目ズレ？
        map_dynamic_edit_x = true;
        m_lastGlbPos = ev->globalPos();
      }
    }
  }
}

static float width_rate = -1;
void MapWindow::mouseDoubleClickEvent(QMouseEvent *ev) {
  if(m_lastPos.y() < 1080 - 200){ //ボタンの位置は避ける。
    bool clear_width_rate = false;
    if(uiState()->scene.map_on_left){
      if(m_lastPos.x() > this->width() - 150){
        clear_width_rate = true;
      }
    } else {
      if(m_lastPos.x() < 150){ //ちょっと広めに取らないと感度悪い。右ハンドルだからタッチの見た目ズレ？
        clear_width_rate = true;
      }
    }
    if(clear_width_rate == true){
      width_rate = 0.5;
      m_panel->setFixedWidth((DEVICE_SCREEN_SIZE.width() * width_rate - UI_BORDER_SIZE));
      if(uiState()->scene.map_on_left){
        emit BearingScaleChanged(rect().width(),*last_bearing,util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM) , g_latitude);
      } else {
        emit LimitspeedChanged(rect().width());
      }
      void soundButton(int onOff);
      soundButton(false);
      return;
    }
  }

  if (last_position) m_map->setCoordinate(*last_position);
  if(north_up == 0){
    if (last_bearing) m_map->setBearing(*last_bearing);
  } else {
    if (last_bearing) m_map->setBearing(0);
  }
  m_map->setZoom(util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM));
  update();

  interaction_counter = 0;
}

void MapWindow::mouseMoveEvent(QMouseEvent *ev) {
  QPointF g_delta;
  bool window_resize = false;
  bool zoom_change = false;
  if(map_dynamic_edit_x || map_dynamic_edit_y){
    g_delta = ev->globalPos() - m_lastGlbPos;
    if(fabs(g_delta.x()) < fabs(g_delta.y())){
      //縦スワイプ
      if(map_dynamic_edit_y){
        zoom_change = true;
      }
    } else if(map_dynamic_edit_x){
      //横スワイプ
      window_resize = true;
    }
    m_lastGlbPos = ev->globalPos();
  }
  if(window_resize){ //端から左右スワイプで地図サイズの調整。負荷が高くて落ちる？
    if(width_rate < 0){
      std::string my_mapbox_width = util::read_file("/data/mb_width_rate.txt");
      if(my_mapbox_width.empty() == false){
        width_rate = std::stof(my_mapbox_width);
      } else {
        width_rate = 0.5;
      }
    }
    if(uiState()->scene.map_on_left){
      width_rate += g_delta.x() / DEVICE_SCREEN_SIZE.width();
    } else {
      width_rate -= g_delta.x() / DEVICE_SCREEN_SIZE.width();
    }
    width_rate = std::clamp(width_rate, 0.265f, 0.535f);
    static int old_width;
    int width_px = DEVICE_SCREEN_SIZE.width() * width_rate;
    if(old_width != width_px){
      old_width = width_px;
      m_panel->setFixedWidth((width_px - UI_BORDER_SIZE));
      if(uiState()->scene.map_on_left){
        emit BearingScaleChanged(rect().width(),*last_bearing,util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM) , g_latitude);
      } else {
        emit LimitspeedChanged(rect().width());
      }
    }
    return;
  }
  if(zoom_change){ //上下で高度調整。
    zoom_offset += g_delta.y() / 100;
    float zoom = util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM);
    m_map->setZoom(zoom);
    emit BearingScaleChanged(rect().width(),*last_bearing,zoom , g_latitude);
    return; //地図は動かさない。
  }
  map_dynamic_edit_x = false;
  map_dynamic_edit_y = false;

  QPointF delta = ev->localPos() - m_lastPos;

  if (!delta.isNull()) {
    interaction_counter = INTERACTION_TIMEOUT;
    m_map->moveBy(delta / MAP_SCALE);
    update();
  }

  m_lastPos = ev->localPos();
  ev->accept();
}

void MapWindow::wheelEvent(QWheelEvent *ev) {
  if (ev->orientation() == Qt::Horizontal) {
      return;
  }

  float factor = ev->delta() / 1200.;
  if (ev->delta() < 0) {
      factor = factor > -1 ? factor : 1 / factor;
  }

  m_map->scaleBy(1 + factor, ev->pos() / MAP_SCALE);
  update();

  interaction_counter = INTERACTION_TIMEOUT;
  ev->accept();
}

bool MapWindow::event(QEvent *event) {
  if (event->type() == QEvent::Gesture) {
    return gestureEvent(static_cast<QGestureEvent*>(event));
  }

  return QWidget::event(event);
}

bool MapWindow::gestureEvent(QGestureEvent *event) {
  if (QGesture *pinch = event->gesture(Qt::PinchGesture)) {
    pinchTriggered(static_cast<QPinchGesture *>(pinch));
  }
  return true;
}

void MapWindow::pinchTriggered(QPinchGesture *gesture) {
  QPinchGesture::ChangeFlags changeFlags = gesture->changeFlags();
  if (changeFlags & QPinchGesture::ScaleFactorChanged) {
    // TODO: figure out why gesture centerPoint doesn't work
    m_map->scaleBy(gesture->scaleFactor(), {width() / 2.0 / MAP_SCALE, height() / 2.0 / MAP_SCALE});
    update();
    interaction_counter = INTERACTION_TIMEOUT;
  }
}

void MapWindow::offroadTransition(bool offroad) {
  if (offroad) {
    clearRoute();
    routing_problem = false;

    setButtonInt("/data/mb_last_bearing_info.txt",(int)(*last_bearing * 1000)); //"%.2f"の代わり。
    //不要setButtonInt("/data/mb_north_up.txt",north_up);
    setButtonInt("/data/mb_zoom_offset.txt",(int)(zoom_offset * 1000));
    if(width_rate >= 0){
      FILE *fp = fopen("/data/mb_width_rate.txt","w"); //write_fileだと書き込めないが、こちらは書き込めた。
      if(fp != NULL){
        fprintf(fp,"%.3f",width_rate);
        fclose(fp);
      }
    }
  } else {
    auto dest = coordinate_from_param("NavDestination");
    emit requestVisible(dest.has_value());
  }
  //last_bearing = {}; //これがあると最終状態保持がキャンセルされる？
}

void MapWindow::updateDestinationMarker() {
  auto nav_dest = coordinate_from_param("NavDestination");
  if (nav_dest.has_value()) {
    auto point = coordinate_to_collection(*nav_dest);
    QMapLibre::Feature feature(QMapLibre::Feature::PointType, point, {}, {});
    QVariantMap pinSource;
    pinSource["type"] = "geojson";
    pinSource["data"] = QVariant::fromValue<QMapLibre::Feature>(feature);
    m_map->updateSource("pinSource", pinSource);
    m_map->setPaintProperty("pinLayer", "visibility", "visible");
  } else {
    m_map->setPaintProperty("pinLayer", "visibility", "none");
  }
}

MapLimitspeed::MapLimitspeed(QWidget * parent) : QWidget(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setContentsMargins(0, 0, 0, 0);

  {
    const static char *btn_styleb_trs = "font-weight:600; font-size: 75px; border-width: 0px; background-color: rgba(0, 0, 0, 0); border-radius: 20px; border-color: %1"; //透明ボタン用
    //const static char *btn_styleb_trs = "font-weight:600; font-size: 75px; border-width: 0px; color: #2457A1; background-color: rgba(0, 0, 0, 0);"; //透明ボタン用
    QHBoxLayout *layout = new QHBoxLayout;
    speed = new QPushButton;
    //speed->setAlignment(Qt::AlignCenter);
    speed->setStyleSheet(QString(btn_styleb_trs).arg("#909090"));
    //this->updateLimitspeed(0);
    speed->setText("━");

    layout->addWidget(speed);
    main_layout->addLayout(layout);

    QObject::connect(speed, &QPushButton::pressed, [=]() {
      std::string last_navi_dest = util::read_file("/data/last_navi_dest.json");
      if(last_navi_dest.empty() == false){
        extern void soundPipo();
        soundPipo();
        Params().put("NavDestination", last_navi_dest);
      }
    });
  }
  setStyleSheet(R"(
    QPushButton {
      color: #2457A1;
      text-align: center;
      padding: 0px;
      border-width: 4px;
      border-style: solid;
      background-color: rgba(75, 75, 75, 0.3);
    }
    * {
      color: #2457A1;
      font-family: "Inter";
      font-size: 75px;
    }
  )");

/*
  QPalette pal = palette();
  pal.setColor(QPalette::Background, QColor(255, 255, 255, 200));
  setAutoFillBackground(true);
  setPalette(pal);
*/
}

//static bool g_stand_still;
extern int limit_speed_auto_detect;
int limit_speed_num;

void MapLimitspeed::updateLimitspeed(int map_width) {

  int old_limit_speed_num = limit_speed_num;

  std::string limitspeed_data_txt = util::read_file("/tmp/limitspeed_data.txt");
  if(limitspeed_data_txt.empty() == false){
    float output[3]; // float型の配列
    int i = 0; // インデックス

    std::stringstream ss(limitspeed_data_txt); // 入力文字列をstringstreamに変換
    std::string token; // 一時的にトークンを格納する変数
    while (std::getline(ss, token, ',') && i < 3) { // カンマで分割し、一つずつ処理する
      output[i] = std::stof(token); // 分割された文字列をfloat型に変換して配列に格納
      i++; // インデックスを1つ進める
    }
    limit_speed_num = (int)output[0];
    if((int)output[2] == 111){
      limit_speed_num = 0;
      speed->setText("━");
      limit_speed_auto_detect = 0;
    } else {
      speed->setText(QString::number(limit_speed_num));
      limit_speed_auto_detect = 1;
    }
  }
#if 0
  std::string stand_still_txt = util::read_file("/tmp/stand_still.txt");
  g_stand_still = false;
  if(stand_still_txt.empty() == false){
    g_stand_still = std::stoi(stand_still_txt) ? true : false;
  }
#endif
  float r = LS_SIZE / 2;
  int stand_still_height = 0;
#if 0 //持ち上げはひとまず取りやめ。
  if(g_stand_still){
    stand_still_height = 270;
  }
#endif
  if(now_navigation == true){
    stand_still_height = 115; //見た目ハードコーディング
  }
  if (map_width == 0 || uiState()->scene.map_on_left) {
    this->move(UI_BORDER_SIZE, 1080 - UI_BORDER_SIZE*2 - r*2 - UI_BORDER_SIZE - stand_still_height); //地図にナビ用ボタンが追加されたので、こちらは使わない。->復活？
  } else {
    this->move(map_width - r*2 - UI_BORDER_SIZE, 1080 - UI_BORDER_SIZE*2 - r*2 - UI_BORDER_SIZE - stand_still_height);
  }
  if(old_limit_speed_num != limit_speed_num){
    this->update(0,0,this->width(),this->height()); //これを呼ばないとpaintEventがすぐに呼ばれない。
  }
}

void MapLimitspeed::paintEvent(QPaintEvent *event) {

  float r = LS_SIZE / 2;
  QPainter p(this);
  p.setPen(Qt::NoPen);
  if(night_mode == 1){
    p.setBrush(QColor::fromRgbF(0.8, 0.8, 0.9, 1.0));
  } else {
    p.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, 1.0));
  }
  p.drawEllipse(0,0,r*2,r*2);

  int arc_w = -LS_SIZE * 25 / 200; //内側に描画
  if(limit_speed_num >= 100){
    arc_w = (int)(arc_w * 0.7); ///枠と数字が被らないように枠を細くする。
  }
  QPen pen = QPen(QColor(205, 44, 38, 255), abs(arc_w));
  pen.setCapStyle(Qt::FlatCap); //端をフラットに
  p.setPen(pen);

  p.drawArc(0-arc_w/2+5, 0-arc_w/2+5, r*2+arc_w-10,r*2+arc_w-10, 0*16, 360*16);
}

int bs_color_revert = 0;
MapBearingScale::MapBearingScale(QWidget * parent) : QWidget(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setContentsMargins(0, 0, 0, 0);

  {
    const static char *btn_styleb_trs = "font-weight:600; font-size: 100px; border-width: 0px; background-color: rgba(0, 0, 0, 0); border-radius: 20px; border-color: %1"; //透明ボタン用
    QHBoxLayout *layout = new QHBoxLayout;
    bearing_scale = new QPushButton("");
    //bearing_scale->setAlignment(Qt::AlignCenter);
    bearing_scale->setStyleSheet(QString(btn_styleb_trs).arg("#909090"));
    //bearing_scale->setText("   "); //ラベルを使わず直に距離描画する

    layout->addWidget(bearing_scale);
    main_layout->addLayout(layout);

    m_pressedTime = 0;
    QObject::connect(bearing_scale, &QPushButton::pressed, [=]() {
      m_pressedTime = QDateTime::currentMSecsSinceEpoch();
      //ボタンを押した時に何かしたいならここで。
    });

    QObject::connect(bearing_scale, &QPushButton::released, [=]() {
      quint64 now = QDateTime::currentMSecsSinceEpoch();
      //ボタンを押した時に何かしたいならここで。
      if(now - m_pressedTime > 2000){
        //qDebug() << "long clicked"; //これでは放さないと長押しが取れない。
        //2秒以上長押しでzoom_offsetクリア。
        zoom_offset = 0;
        void soundButton(int onOff);
        soundButton(false);
      } else if(now - m_pressedTime > 500){
        bs_color_revert  ^= 1; //0.5秒以上の長押しでダークモード反転。
      } else /*if(north_up == 0)*/{
        //qDebug() << "clicked";
        //bs_color_revert ^= 1;
        if(MIN_PITCH_ < 0){
          MIN_PITCH_ = -10; //ジェスチャー切り替えでノースアップが-10以外になっている可能性を考慮。
        }

        MIN_PITCH_ /= 10;
        MIN_PITCH_ += 1;
        if(MIN_PITCH_ > 4){
          MIN_PITCH_ = -1;
        }
        MIN_PITCH_ *= 10;
        max_zoom_pitch_effect();
        setButtonInt("/data/mb_pitch.txt",MIN_PITCH_); //MIN_PITCH_ = 0,10,20,30,40度,ノースアップから選択
        chg_pitch = true;
        if(MIN_PITCH_ == 40){
          void soundPikiri();
          soundPikiri();
        } else {
          extern void soundPipo();
          soundPipo();
        }
      }
      m_pressedTime = 0;
      this->update(0,0,this->width(),this->height()); //これを呼ばないとpaintEventがすぐに呼ばれない。
    });
  }
  setStyleSheet(R"(
    QPushButton {
      color: #201010;
      text-align: center;
      padding: 0px;
      border-width: 4px;
      border-style: solid;
      background-color: rgba(75, 75, 75, 0.3);
    }
    * {
      color: #201010;
      font-family: "Inter";
      font-size: 50px;
    }
  )");

/*
  QPalette pal = palette();
  pal.setColor(QPalette::Background, QColor(255, 255, 255, 200));
  setAutoFillBackground(true);
  setPalette(pal);
*/
}

static int map_bearing_num;
static double map_scale_num;
void MapBearingScale::updateBearingScale(int map_width, int angle, double scale , double latitude) {

  map_bearing_num = angle;
  // map_scale_num = scale; //18〜14scale->25m〜400m,25*(2^4=16)
  // bearing_scale->setText(QString::number(map_scale_num, 'f', 2));
  //1:2:4:8:16:32:64:128 , mapboxのズームレベルは＋1で映る範囲が２倍だそうです。参考→https://docs.mapbox.com/jp/help/glossary/zoom-level/
  //1,2,3,4,5, 6, 7, 8
  //n^2
  //pow(2,18-scale) * 25
  map_scale_num = pow(2,(18-scale)) * 23 * cos(DEG2RAD(latitude));
  // if(map_scale_num < 1000){
  //   bearing_scale->setText(QString::number(map_scale_num, 'f', 0) + "m");
  // } else {
  //   bearing_scale->setText(QString::number(map_scale_num / 1000, 'f', 1) + "K");
  // }
  float r_w = BS_SIZE_W / 2;
  float r_h = BS_SIZE_H / 2;
  int stand_still_height = 0;
  if(now_navigation == true){
    stand_still_height = 115; //見た目ハードコーディング
  }
  if (map_width == 0 || uiState()->scene.map_on_left) {
    this->move(map_width - r_w*2 - UI_BORDER_SIZE, 1080 - UI_BORDER_SIZE*2 - r_h*2 - UI_BORDER_SIZE - stand_still_height);
  } else {
    this->move(UI_BORDER_SIZE, 1080 - UI_BORDER_SIZE*2 - r_h*2 - UI_BORDER_SIZE - stand_still_height); //地図にナビ用ボタンが追加されたので、こちらは使わない。->復活？
  }
  this->update(0,0,this->width(),this->height()); //これを呼ばないとpaintEventがすぐに呼ばれない。
}

void MapBearingScale::paintEvent(QPaintEvent *event) {

  int tmp_map_bearing_num = map_bearing_num;
  if(north_up != 0){
    tmp_map_bearing_num = 0;
  }


  bool tmp_bs_color_revert = bs_color_revert;
  if(night_mode == 1){
    tmp_bs_color_revert ^= 1;
  }

  float d_h = BS_SIZE_H - BS_SIZE_W;
  float r_w = BS_SIZE_W / 2;
  //float r_h = BS_SIZE_H / 2;
  float btm_r = 20;

  QPainter p(this);
#if 0
  p.setPen(Qt::NoPen);
  if(tmp_bs_color_revert == 0){
    //p.setPen(QPen(QColor(150, 150, 150, 255),5));
    p.setBrush(QColor(240, 240, 240, 240));
  } else {
    //p.setPen(QPen(QColor(240, 240, 240, 255),5));
    p.setBrush(QColor(40, 40, 40, 240));
  }
  //p.drawEllipse(0,0,r*2,r*2);
  drawRoundedRect(p,QRectF(0,0,BS_SIZE_W,BS_SIZE_H),r_w,r_w,btm_r,btm_r);
#else
  const float border = 3;
  if(tmp_bs_color_revert == 0){
    p.setPen(QPen(QColor(150, 150, 150, 255),border));
    p.setBrush(QColor(240, 240, 240, 240));
  } else {
    p.setPen(QPen(QColor(240, 240, 240, 255),border));
    p.setBrush(QColor(40, 40, 60, 200));
  }
  drawRoundedRect(p,QRectF(border/2,border/2,BS_SIZE_W-border,BS_SIZE_H-border),r_w-border/2,r_w-border/2,btm_r-border/2,btm_r-border/2);
#endif

  p.setPen(Qt::NoPen);

  //方位メモリ的な演出。
  const static QPointF memo_b[] = {{-4, -BS_SIZE_W/2+border+2}, {-4, -BS_SIZE_W/2+16} , {4, -BS_SIZE_W/2+16} , {4, -BS_SIZE_W/2+border+2}};
  const static QPointF memo[] = {{-3, -BS_SIZE_W/2+border+2}, {-3, -BS_SIZE_W/2+11} , {3, -BS_SIZE_W/2+11} , {3, -BS_SIZE_W/2+border+2}};
  p.resetTransform();
  p.translate(r_w,r_w);
  for(int ang=0; ang < 360; ang += 15){
    if(ang % 45 == 0){
      if(ang % 90 == 0){
        p.setBrush(QColor(220, 20, 20, 255));
      }
      p.drawPolygon(memo_b, std::size(memo_b));
      if(ang % 90 == 0){
        if(tmp_bs_color_revert == 0){
          p.setBrush(QColor(120, 120, 120, 255));
        } else {
          p.setBrush(QColor(200, 200, 200, 255));
        }
      }
    } else {
      p.drawPolygon(memo, std::size(memo));
    }
    p.rotate(15); //degree指定
  }

  //方位磁石風
  const static QPointF needle[] = {{-3, -BS_SIZE_W/2+20}, {-BS_SIZE_W/4+10, 0} , {BS_SIZE_W/4-10, 0} , {3, -BS_SIZE_W/2+20}};
  p.resetTransform();
  p.translate(r_w,r_w);
  p.rotate(-tmp_map_bearing_num); //degree指定
  p.setBrush(QColor(201, 34, 49, 220));
  p.drawPolygon(needle, std::size(needle));
  p.rotate(180); //南側をグレーで描画
  p.setBrush(QColor(150, 150, 150, 220));
  p.drawPolygon(needle, std::size(needle));

  p.resetTransform();

  //ラベルを使わず直に距離描画する
  QString scl;
#if 1
  if(map_scale_num < 1000){
    scl = QString::number(map_scale_num, 'f', 0) + "m";
  } else {
    scl = QString::number(map_scale_num / 1000, 'f', 1) + "K";
  }
#elif 0
    scl = QString::number(tmp_map_bearing_num) + "°"; //本当に-180〜180か？->確かにその通りだった。
#endif
  p.setFont(InterFont(40, QFont::ExtraBold));
  const int SCL_H = d_h;
  const int h_ctl = BS_SIZE_W-6;
  if(tmp_bs_color_revert == 0){
    p.setPen(QColor(240, 240, 240, 255));
    p.drawText(QRect(0,0+h_ctl-3,this->width(),SCL_H), Qt::AlignCenter, scl); //円形メモリと数字の衝突を避けるために白影を上オフセットで表示する。
    p.setPen(QColor(0x24, 0x57, 0xA1, 255));
  } else {
    p.setPen(QColor(220, 220, 220, 255));
  }
  p.drawText(QRect(0,0+h_ctl,this->width(),SCL_H), Qt::AlignCenter, scl);
}
