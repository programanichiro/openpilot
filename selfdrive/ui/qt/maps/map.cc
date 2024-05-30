#include "selfdrive/ui/qt/maps/map.h"

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <QMapLibre/Utils>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QRegularExpression>
#include <QRegularExpressionValidator>

#include <QDebug>

#include "common/params.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/widgets/input.h"


const int INTERACTION_TIMEOUT = 100;
#if 0
const float MAP_SCALE = 3;
const float log2_MAP_SCALE = 0.58496250072; //log2(MAP_SCALE/2)
#elif 1
const float MAP_SCALE = 2;
const float log2_MAP_SCALE = 0; //log2(MAP_SCALE/2)
#else
const float MAP_SCALE = 2.5;
const float log2_MAP_SCALE = 0.32192809488; //log2(MAP_SCALE/2)
#endif
const float MAX_ZOOM0 = (17-log2_MAP_SCALE);
float MAX_ZOOM_;
#define MAX_ZOOM calc_max_zoom()
float zoom_offset;
//const float MIN_ZOOM = 14;
const float MIN_ZOOM0 = (14-log2_MAP_SCALE);
#define MIN_ZOOM calc_min_zoom()
const float MAX_PITCH = 50;
#define MIN_PITCH calc_pich()
float MIN_PITCH_ = 0;

std::string my_mapbox_triangle;
std::string my_mapbox_style;
std::string my_mapbox_style_night;
int night_mode = -1;
//int north_up = 0; //1で北上モード
#define north_up chk_north_up()
bool chg_pitch;
bool reset_zoom;
bool chg_coordinate;
bool north_head , head_north;
extern void setButtonInt(const char*fn , int num);
extern int getButtonInt(const char*fn , int defaultNum);
float _1_vc_accel; //-1〜0〜+1で加速度っぽい値。
float _2_vc_accel;
float calc_pich(){
  if(MIN_PITCH_ < 0){
    return 0; //north_up用。方位磁石タップにノースアップも混ぜる0->10->20->30->40->ノースアップ
  }
  _2_vc_accel = _2_vc_accel + (_1_vc_accel - _2_vc_accel) / 5; //若干緩衝処理。
  return MIN_PITCH_ + (_2_vc_accel*5); //*10だと思ったより動いてしまう。
}
void max_zoom_pitch_effect(){
  float tmp_1_vc_accel = _1_vc_accel;
  float tmp_2_vc_accel = _2_vc_accel;
  _1_vc_accel = 0; //影響を与えない。
  MAX_ZOOM_ = MAX_ZOOM0 + sin(MIN_PITCH * M_PI / 180) * 1.7; //30度でMAX_ZOOM=18くらいになる。
  _1_vc_accel = tmp_1_vc_accel;
  _2_vc_accel = tmp_2_vc_accel;
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
  // if(m_o < MIN_ZOOM){ //calc_min_zoomを使うならここで無限再起になるので注意。
  //   zoom_offset = MIN_ZOOM - MAX_ZOOM_;
  //   return MIN_ZOOM;
  // }
  const float tmp_MIN_ZOOM = (6-log2_MAP_SCALE); //大きな川から島が入る(関東で70〜80km四方)。1で地球レベル
  if(m_o < tmp_MIN_ZOOM){
    zoom_offset = tmp_MIN_ZOOM - MAX_ZOOM_;
    return tmp_MIN_ZOOM;
  }
  return m_o; //もしくはMIN_ZOOMを、MAX_ZOOMより大きくならないように小さくする制御も考えられる。
}
float calc_min_zoom(){
  if(MAX_ZOOM < MIN_ZOOM0){ //14（関東で300m程度）より高度を上げると、速度による高度制御がなくなる
    return MAX_ZOOM;
  }
  return MIN_ZOOM0;
}
int chk_north_up(){
  if(MIN_PITCH_ < 0){
    return 1; //ピッチ角がマイナスならノースアップ。
  }
  return 0;
}
static float width_rate = -1;

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
  error->setStyleSheet(R"(color:white;padding:20px 11px;font-size: 60px; background-color:rgba(0, 0, 0, 150);)");
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

//マップウインドウリサイズポイント
#define WRP_SIZE_H 100
#define WRP_SIZE_W 80
  map_WindowResizePoint = new QLabel(this);
  map_WindowResizePoint->setStyleSheet(R"(color:rgba(128, 128, 128, 200); font-size: 65px; background-color:rgba(0, 0, 0, 0);)");
  map_WindowResizePoint->setVisible(false);
  map_WindowResizePoint->setFixedHeight(WRP_SIZE_H);
  map_WindowResizePoint->setFixedWidth(WRP_SIZE_W);
  map_WindowResizePoint->setAlignment(Qt::AlignCenter);
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
    night = clipped_brightness < (night_mode == 1 ? 90 : 75); //ばたつかないようにする。80程度でかなり夕方。
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
    m_map->setLayoutProperty("carPosLayer", "icon-size", 0.5*2/MAP_SCALE);
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

extern bool g_rightHandDM;
float bearing_ofs(float v_ego){
  float k=0;
  if(v_ego > 30/3.6){
    k=1;
  } else if(v_ego < 0){
    ;
  } else {
    k = v_ego / (30/3.6); //0〜1.0
  }
  if(g_rightHandDM){
    //右ハンドル
    if(uiState()->scene.map_on_left){
      return -8 * k;
    } else {
      return -6 * k;
    }
  } else {
    //左ハンドル
    if(uiState()->scene.map_on_left){
      return 6 * k;
    } else {
      return 8 * k;
    }
  }
}

bool now_navigation = false;
int style_reload = 0;
float g_latitude,g_longitude;
bool head_gesture_map_north_heading_toggle;
bool map_pitch_up,map_pitch_down;
qreal before_pinch_angle,last_pinch_angle;
void MapWindow::updateState(const UIState &s) {
  if (!uiState()->scene.started) {
    return;
  }
  const SubMaster &sm = *(s.sm);
  update();

  static bool already_vego_over_8 = false;
  float sm_vego = sm["carState"].getCarState().getVEgo();
  if(already_vego_over_8 == false && sm_vego > 1/3.6){ //8->4->1km/h
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
          if(MIN_PITCH_ == -1){
            //head->north
            head_north = true; //地図の角度をリセットする。
          } else if(MIN_PITCH_ == 0){
            //north->head
            north_head = true; //地図の角度をリセットする。
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
          if(MIN_PITCH_ == -1){
            //head->north
            head_north = true; //地図の角度をリセットする。
          } else if(MIN_PITCH_ == 4){
            //north->head
            north_head = true; //地図の角度をリセットする。
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
            head_north = true; //地図の角度をリセットする。
          } else if(MIN_PITCH_ == -1){
            MIN_PITCH_ = 0;
            north_head = true; //地図の角度をリセットする。
          } else {
            MIN_PITCH_ = -MIN_PITCH_;
            if(MIN_PITCH_ >= 0){
              north_head = true; //地図の角度をリセットする。
            } else {
              head_north = true; //地図の角度をリセットする。
            }
          }
          max_zoom_pitch_effect();
          setButtonInt("/data/mb_pitch.txt",MIN_PITCH_); //MIN_PITCH_ = 0,10,20,30,40度,ノースアップから選択
          chg_pitch = true;
        }
      }
    }
  }

  if (loaded_once || (m_map && !m_map.isNull() && m_map->isFullyLoaded())) {
    if(north_up == 0){
      if(m_map->margins().top() == 0){
        m_map->setMargins({0, (int)(350*2/MAP_SCALE), 0, (int)(50*2/MAP_SCALE)}); //(1080-350*2-50*2) / 2 = 140 , 700+140=840=yの位置に出ている。
        chg_pitch = true;
        max_zoom_pitch_effect();
      }
      if(reset_zoom){
        reset_zoom = false;
        m_map->setZoom(util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM));
      } else if(chg_pitch){
        chg_pitch = false;
        if(interaction_counter == 0 || north_head || head_north){
          if(north_head || head_north){
            // setBearingに - last_pinch_angle - before_pinch_angleを加える方法もあるが、一旦リセットする意図で。
            before_pinch_angle = 0;
            last_pinch_angle = 0;
          }
          north_head = false;
          head_north = false;
          if (last_bearing) m_map->setBearing(*last_bearing+bearing_ofs(velocity_filter.x()));
        }
        if(interaction_counter == 0){
          m_map->setZoom(util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM));
        }
      }
      //ウイリー演出のために常にpitchセットする。
      if (sm.valid("navInstruction")) {
        m_map->setPitch(MAX_PITCH); //ナビ中
      } else {
        m_map->setPitch(MIN_PITCH);
      }
    } else {
      if(m_map->margins().top() != 0){
        m_map->setMargins({0, 0, 0, 0});
        m_map->setPitch(0);
        if(interaction_counter == 0 || north_head || head_north){
          if(north_head || head_north){
            // setBearingに - last_pinch_angle - before_pinch_angleを加える方法もあるが、一旦リセットする意図で。
            before_pinch_angle = 0;
            last_pinch_angle = 0;
          }
          north_head = false;
          head_north = false;
          m_map->setBearing(0);
        }
        MAX_ZOOM_ = MAX_ZOOM0; //max_zoom_pitch_effect(); //これだとノースアップでも方位磁石タップでスケールが変わってしまう。
        if(interaction_counter == 0){
          m_map->setZoom(util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM));
        }
      } else if(reset_zoom){
        reset_zoom = false;
        m_map->setZoom(util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM));
      } else if(chg_pitch){
        chg_pitch = false;
        m_map->setPitch(0);
        if(interaction_counter == 0){
          m_map->setZoom(util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM));
        }
      }
    }
    if(chg_coordinate){
      chg_coordinate = false;
      m_map->setCoordinate(QMapLibre::Coordinate(g_latitude, g_longitude));
      interaction_counter = INTERACTION_TIMEOUT;
    }
    g_latitude = m_map->coordinate().first;
    g_longitude = m_map->coordinate().second;
  }

  static unsigned int LimitspeedChanged_ct;
  if ((LimitspeedChanged_ct++ % 10) == 0 && LimitspeedChanged_ct >= 10) { //0.5秒ごとに速度標識を更新
    map_limitspeed->setVisible(true);
    emit LimitspeedChanged(rect().width());

    map_bearing_scale->setVisible(true);
    emit BearingScaleChanged(rect().width(),(loaded_once ? m_map->bearing() : *last_bearing),util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM) , g_latitude);

    bool map_w_isVisible = isVisible();
    if(map_WindowResizePoint->isVisible() == false && map_w_isVisible){
      map_WindowResizePoint->setVisible(true); //最初位置が安定するまで毎回セット。
      if (uiState()->scene.map_on_left) {
        map_WindowResizePoint->move(rect().width() - WRP_SIZE_W,  rect().height()/2 - WRP_SIZE_H/2);
        map_WindowResizePoint->setText("◀︎"); //⬅︎
      } else {
        map_WindowResizePoint->move(0, rect().height()/2 - WRP_SIZE_H/2); //地図にナビ用ボタンが追加されたので、こちらは使わない。->復活？
        map_WindowResizePoint->setText("▶︎"); //➡︎
      }
      map_WindowResizePoint->update(0,0,map_WindowResizePoint->width(),map_WindowResizePoint->height()); //これを呼ばないとpaintEventがすぐに呼ばれない。
    } else if(map_w_isVisible == false){
      map_WindowResizePoint->setVisible(false); //左右が入れ替わった時再配置するため。
    }
  }

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

  if(sm_vego < 1/3.6){
    ; //速度が出ていない時は座標をリセットしない。
    if (interaction_counter > 1){
      interaction_counter--; //カウントダウンだけはやっておく。わざと1まで。m_map->setZoomさせないため。
    }
  }
  if (interaction_counter == 0) {
    if (last_position) m_map->setCoordinate(*last_position);
    if(north_up == 0){
      if (last_bearing) m_map->setBearing(*last_bearing+bearing_ofs(velocity_filter.x()));
    } else {
      if (last_bearing) m_map->setBearing(0);
    }
    m_map->setZoom(util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM));
  } else if(sm_vego >= 1/3.6){
    interaction_counter--;
    if(interaction_counter == 0){
      before_pinch_angle = 0;
      last_pinch_angle = 0;
    }
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
          m_map->setMargins({0, (int)(350*2/MAP_SCALE), 0, (int)(50*2/MAP_SCALE)});
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
      if(routing_problem == false){ //Waiting for route中はいちいちスタイルを戻さない。
        if(now_navigation == true){
          now_navigation = false;
          if(my_mapbox_style_night.empty() == false && check_night_mode()){ //夜だったら
            night_mode = 1;
            m_map->setStyleUrl(my_mapbox_style_night.c_str());
          } else if(my_mapbox_style.empty() == false){ //昼だったら
            night_mode = 0;
            m_map->setStyleUrl(my_mapbox_style.c_str());
          }
        }
      }
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

  static unsigned int save_period_counter;
  unsigned int save_period_counter_check = (save_period_counter ++) % (20*60); //60秒周期
  if(save_period_counter_check == 20*30){ //60秒に一回セーブ
    if (last_bearing) setButtonInt("/data/mb_last_bearing_info.txt",(int)(*last_bearing * 1000)); //"%.2f"の代わり。
    //不要setButtonInt("/data/mb_north_up.txt",north_up);
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
    if (last_bearing) m_map->setBearing(*last_bearing+bearing_ofs(velocity_filter.x()));
  } else {
    m_map->setCoordinateZoom(QMapLibre::Coordinate(64.31990695292795, -149.79038934046247), MIN_ZOOM);
  }

  if(north_up == 0){
    m_map->setMargins({0, (int)(350*2/MAP_SCALE), 0, (int)(50*2/MAP_SCALE)});
    m_map->setPitch(MIN_PITCH);
  } else {
    m_map->setMargins({0, 0, 0, 0});
    m_map->setPitch(0);
    m_map->setBearing(0);
  }

  my_mapbox_style = util::read_file("/data/mb_style.txt");
  my_mapbox_style_night = util::read_file("/data/mb_style_night.txt");

  if(my_mapbox_style.empty() == false && my_mapbox_style.c_str()[0] == 'm'){
    while(my_mapbox_style.c_str()[my_mapbox_style.length()-1] == 0x0a){
      my_mapbox_style = my_mapbox_style.substr(0,my_mapbox_style.length()-1);
    }
    if(my_mapbox_style_night.empty() == true){
      my_mapbox_style_night = "x"; //昼が独自設定で夜が空なら、夜の変更はしない（独自設定の昼のまま）
    }
  } else if(my_mapbox_style.empty() == false){ //mb_style.txtに x と記述。
    my_mapbox_style = std::string(); //公式スタイルを使う
    my_mapbox_style_night = "x"; //夜も公式スタイルを使う。下の制御でstd::stringを代入する。
  } else {
    my_mapbox_style = "mapbox://styles/kawombop0/clw7ei7g0029q01rja2dy341n"; //イチロウパイロット昼用公開スタイル
  }

  if(my_mapbox_style_night.empty() == false && my_mapbox_style_night.c_str()[0] == 'm'){
    while(my_mapbox_style_night.c_str()[my_mapbox_style_night.length()-1] == 0x0a){
      my_mapbox_style_night = my_mapbox_style_night.substr(0,my_mapbox_style_night.length()-1);
    }
  } else if(my_mapbox_style_night.empty() == false){
    my_mapbox_style_night = std::string(); //公式スタイルを使う
  } else {
    my_mapbox_style_night = "mapbox://styles/kawombop0/clw7ekw91004c01rd7rtx1g3r"; //イチロウパイロット夜用公開スタイル
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
      m_map->setMargins({0, (int)(350*2/MAP_SCALE), 0, (int)(50*2/MAP_SCALE)});
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
qint64 mouse_pressedTime; //quint64だとマイナス計算ができない。
bool start_window_resize;
QPointF firstPos;
void MapWindow::mousePressEvent(QMouseEvent *ev) {
  m_lastPos = ev->localPos();
  ev->accept();

  firstPos = m_lastPos;
  map_dynamic_edit_x = false;
  map_dynamic_edit_y = false;
  start_window_resize = false;
  mouse_pressedTime = QDateTime::currentMSecsSinceEpoch();
  //方位磁石の上の端っこを上下でMAX_ZOOM調整。
  if(m_lastPos.y() < 1080 - 200){ //ボタンの位置は避ける。
    if(uiState()->scene.map_on_left){
      if(m_lastPos.x() > this->width() - 150){
        map_dynamic_edit_x = true;
        map_dynamic_edit_y = true;
        m_lastGlbPos = ev->globalPos();
        if(m_lastPos.y() > this->height()/2 - 100 && m_lastPos.y() < this->height()/2 + 100){
          start_window_resize = true;
        }
      }
    } else {
      if(m_lastPos.x() < 150){ //ちょっと広めに取らないと感度悪い。右ハンドルだからタッチの見た目ズレ？
        map_dynamic_edit_x = true;
        map_dynamic_edit_y = true;
        m_lastGlbPos = ev->globalPos();
        if(m_lastPos.y() > this->height()/2 - 100 && m_lastPos.y() < this->height()/2 + 100){
          start_window_resize = true;
        }
      }
    }
  }
}

void MapWindow::mouseReleaseEvent(QMouseEvent *ev) {
  QPointF p = ev->localPos();
  ev->accept();

  { //以下はタッチが放された瞬間に記録する。
    setButtonInt("/data/mb_zoom_offset.txt",(int)(zoom_offset * 1000)); //コンパス長押しのクリアはここでは保存されない。
    if(width_rate >= 0){
      //ダブつタップでクリアしてもここで保存される。
      FILE *fp = fopen("/data/mb_width_rate.txt","w"); //write_fileだと書き込めないが、こちらは書き込めた。
      if(fp != NULL){
        fprintf(fp,"%.3f",width_rate);
        fclose(fp);
      }
    }
  }

  //ここで長押し判定できる？
  void soundButton(int onOff);
  qint64 now = QDateTime::currentMSecsSinceEpoch();
  //ボタンを押した時に何かしたいならここで。
  if(now - mouse_pressedTime > 2500 && !m_map.isNull()){
    soundButton(true);

    FILE *latlon = fopen("/data/last_navi_dest.json","w");
    if(latlon){
#if 1
      //MAP_SCALEを加味したらうまく行った。
      QPointF p2(p.x()/MAP_SCALE,p.y()/MAP_SCALE);
      QMapLibre::Coordinate coord = m_map->coordinateForPixel(p2);
      fprintf(latlon,R"({"latitude": %.6f, "longitude": %.6f})",coord.first,coord.second);
      const SubMaster &sm = *(uiState()->sm);
#else
      double len = QMapLibre::metersPerPixelAtLatitude(g_latitude, m_map->zoom()) / MAP_SCALE;
      QMapLibre::ProjectedMeters mm = QMapLibre::projectedMetersForCoordinate(m_map->coordinate());
      const SubMaster &sm = *(uiState()->sm);

      if(north_up){
        //中央からpまでのピクセル差分。
        int dx = p.x() - (width() / 2.0);
        int dy = p.y() - (height() / 2.0);

        //pinchローテーションしている場合を考慮
        double rad = DEG2RAD(m_map->bearing());

        double tmp_dx = cos(rad) * dx - sin(rad) * dy;
        dy = sin(rad) * dx + cos(rad) * dy;
        dx = tmp_dx;

        mm = QMapLibre::ProjectedMeters(mm.first - dy*len, mm.second + dx*len);

        QMapLibre::Coordinate point = QMapLibre::coordinateForProjectedMeters(mm);

        fprintf(latlon,R"({"latitude": %.6f, "longitude": %.6f})",point.first,point.second); //タッチしてる緯度経度。ノースアップじゃないとうまくいかないはず。
      } else {
        int dx = p.x() - (width() / 2.0);
        //int dy = p.y() - 840; //(height() / 2.0);ヘッドアップはy=840が地図座標の位置にくる。
        int dy = p.y() - (height() - (m_map->margins().top()*MAP_SCALE - m_map->margins().bottom()*MAP_SCALE) / 2 + (m_map->margins().top()*MAP_SCALE)); //840を引くはず。

        double rad = DEG2RAD(m_map->bearing());

        double tmp_dx = cos(rad) * dx - sin(rad) * dy;
        dy = sin(rad) * dx + cos(rad) * dy;
        dx = tmp_dx;

        // //傾きによる補正はなんちゃって式。機体と違い随分遠くを指してしまう。ない方がマシ？
        // if (sm.valid("navInstruction")) {
        //   dx /= cos(DEG2RAD(MAX_PITCH));
        //   dy /= cos(DEG2RAD(MAX_PITCH));
        // } else {
        //   dx /= cos(DEG2RAD(MIN_PITCH));
        //   dy /= cos(DEG2RAD(MIN_PITCH));
        // }
        mm = QMapLibre::ProjectedMeters(mm.first - dy*len, mm.second + dx*len);

        QMapLibre::Coordinate point = QMapLibre::coordinateForProjectedMeters(mm);

        fprintf(latlon,R"({"latitude": %.6f, "longitude": %.6f})",point.first,point.second); //ヘッドアップにも対応。
        //fprintf(latlon,R"({"latitude": %.6f, "longitude": %.6f})",m_map->coordinate().first,m_map->coordinate().second); //ノースアップじゃない時はタッチ位置の計算が困難になるから、地図座標を使う。
      }
#endif
      fclose(latlon);

      if (sm.valid("navInstruction")) {
        FILE *fp = fopen("/tmp/route_style_reload.txt","w");
        if(fp != NULL){
          fprintf(fp,"%d",1);
          fclose(fp);
        }
      }
      std::string last_navi_dest = util::read_file("/data/last_navi_dest.json");
      if(last_navi_dest.empty() == false){
        extern void soundPipo();
        soundPipo();
        Params().put("NavDestination", last_navi_dest);
      }
    }

  } else if(now - mouse_pressedTime > 500){
    //soundButton(false);
  }
}

void MapWindow::mouseDoubleClickEvent(QMouseEvent *ev) {
  if(interaction_counter > INTERACTION_TIMEOUT * 0.9){
    //移動直後の0.5秒以内のダブルクリックはリセット動作をしない。
    return;
  }

  if(m_lastPos.y() < 1080 - 200 && start_window_resize == true){ //ボタンの位置は避ける。
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
        emit BearingScaleChanged(rect().width(),m_map->bearing(),util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM) , g_latitude);
        map_WindowResizePoint->move(rect().width() - WRP_SIZE_W,  rect().height()/2 - WRP_SIZE_H/2);
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
    if (last_bearing) m_map->setBearing(*last_bearing+bearing_ofs(velocity_filter.x()));
  } else {
    if (last_bearing) m_map->setBearing(0);
  }
  m_map->setZoom(util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM));
  update();

  before_pinch_angle = 0;
  last_pinch_angle = 0;
  interaction_counter = 0;
}

void MapWindow::mouseMoveEvent(QMouseEvent *ev) {
  QPointF f_delta = ev->localPos() - firstPos;
  if(f_delta.x()*f_delta.x() + f_delta.y()*f_delta.y() > 10*10){ //最初の位置から10ドット以上動いたらmouse_pressedTimeを長押し無判定ロジックに。
    mouse_pressedTime = QDateTime::currentMSecsSinceEpoch() + 1000*1000; //動かしたら長押し判定されないように1000秒後を設定する。
  }
  QPointF g_delta;
  bool window_resize = false;
  bool zoom_change = false;
  if(map_dynamic_edit_x || map_dynamic_edit_y){
    g_delta = ev->globalPos() - m_lastGlbPos;
    if((map_dynamic_edit_y && map_dynamic_edit_x == false) || (map_dynamic_edit_y && map_dynamic_edit_x && fabs(g_delta.x()) < fabs(g_delta.y()))){
      //縦スワイプ
      if(map_dynamic_edit_y){
        map_dynamic_edit_x = false;
        zoom_change = true;
        m_lastGlbPos = ev->globalPos();
      }
    } else if(map_dynamic_edit_x){
      //横スワイプ
      map_dynamic_edit_y = false;
      window_resize = true;
      m_lastGlbPos = ev->globalPos();
      if(start_window_resize == false){
        window_resize = false;
        map_dynamic_edit_x = false;
      }
    }
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
        emit BearingScaleChanged(rect().width(),m_map->bearing(),util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM) , g_latitude);
        map_WindowResizePoint->move(rect().width() - WRP_SIZE_W,  rect().height()/2 - WRP_SIZE_H/2);
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
    emit BearingScaleChanged(rect().width(),m_map->bearing(),zoom , g_latitude);
    return; //地図は動かさない。
  }

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
  mouse_pressedTime = QDateTime::currentMSecsSinceEpoch() + 1000*1000; //動かしたら長押し判定されないように1000秒後を設定する。
  if (changeFlags & QPinchGesture::ScaleFactorChanged) {
    // TODO: figure out why gesture centerPoint doesn't work
    m_map->scaleBy(gesture->scaleFactor(), {width() / 2.0 / MAP_SCALE, height() / 2.0 / MAP_SCALE});
    update();
    interaction_counter = INTERACTION_TIMEOUT;
  }
  if (changeFlags & QPinchGesture::RotationAngleChanged) {
    //m_map->rotateBy(gesture->rotationAngle()); //???どうする。あとinteraction_counterでsetBearingもキャンセルしなくては。
    if(gesture->rotationAngle() == 0){
      //これで最初の瞬間を判定？
      before_pinch_angle += last_pinch_angle;
    }
    if(north_up == 0){
      if (last_bearing) m_map->setBearing(*last_bearing+bearing_ofs(velocity_filter.x()) - gesture->rotationAngle() - before_pinch_angle);
    } else {
      if (last_bearing) m_map->setBearing(0 - gesture->rotationAngle() - before_pinch_angle);
    }
    last_pinch_angle = gesture->rotationAngle();
    emit BearingScaleChanged(rect().width(),m_map->bearing(),util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM) , g_latitude);
    update();
    interaction_counter = INTERACTION_TIMEOUT;
  }
}

void MapWindow::offroadTransition(bool offroad) {
  if (offroad) {
    clearRoute();
    routing_problem = false;

    if (last_bearing) setButtonInt("/data/mb_last_bearing_info.txt",(int)(*last_bearing * 1000)); //"%.2f"の代わり。
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
    //const static char *btn_styleb_trs = "font-weight:600; font-size: 75px; border-width: 0px; background-color: rgba(0, 0, 0, 0); border-radius: 20px; border-color: %1"; //透明ボタン用
    //const static char *btn_styleb_trs = "font-weight:600; font-size: 75px; border-width: 0px; color: #2457A1; background-color: rgba(0, 0, 0, 0);"; //透明ボタン用
    const static char *btn_styleb_trs = "font-weight:600; font-size: 75px; border-width: 0px; background-color: rgba(0, 0, 0, 0); border-radius: 20px; color: %1"; //透明ボタン文字色変更用
    QHBoxLayout *layout = new QHBoxLayout;
    speed = new QPushButton;
    //speed->setAlignment(Qt::AlignCenter);
    //speed->setStyleSheet(QString(btn_styleb_trs).arg("#2457A1"));
    int on_vavi_highway = getButtonInt("/data/navi_highway.txt",0); //1:高速有料を除外する。（exclude=toll,motorway）
    if(on_vavi_highway){
      speed->setStyleSheet(QString(btn_styleb_trs).arg("#10A010"));
    } else {
      speed->setStyleSheet(QString(btn_styleb_trs).arg("#2457A1"));
    }
    //this->updateLimitspeed(0);
    speed->setText("━");

    layout->addWidget(speed);
    main_layout->addLayout(layout);

    m_pressedTime = 0;
    QObject::connect(speed, &QPushButton::pressed, [=]() {
      m_pressedTime = QDateTime::currentMSecsSinceEpoch();
      //ボタンを押した時に何かしたいならここで。
    });

    QObject::connect(speed, &QPushButton::released, [=]() {
      quint64 now = QDateTime::currentMSecsSinceEpoch();
      if(now - m_pressedTime > 1500){
        extern void soundPikiri();
        soundPikiri();

        std::string my_google_key = util::read_file("/data/google_key.txt");
        my_google_key.erase(std::remove(my_google_key.begin(), my_google_key.end(), '\n'), my_google_key.end());
        my_google_key.erase(std::remove(my_google_key.begin(), my_google_key.end(), '\r'), my_google_key.end());
        QString gg_key;
        if(my_google_key.empty() == false){
          gg_key = QString::fromStdString(my_google_key);
        }
        if(my_google_key.empty() == true){
          //先にGoogle API Keyを入れる
          gg_key = InputDialog::getText(tr("Google API key"), this, tr("Enter Google API key. If using only Lat/Lon, input x."), false, -1, gg_key).trimmed();
          if (gg_key.isEmpty() == false) {
            FILE *fp = fopen("/data/google_key.txt","w");
            if(fp != NULL){
              fprintf(fp,"%s",gg_key.toUtf8().constData());
              fclose(fp);
            }
          }
        }

        if (gg_key.isEmpty() == false) {
          //gg_keyがある場合は、poi検索を行う。
          static QString poi_name;
          QString poi_name_ = InputDialog::getText(tr("POI name or keyword"), this, tr("Enter a POI name or keyword or Lat,Lon"), false, -1, poi_name).trimmed(); //起動中は最後に入れた文字を覚えておくのもいいか？
          if(poi_name_.isEmpty() == false) {
            poi_name = poi_name_;
          }

          bool latlon_mode = false; //緯度経度座標モード
          if(gg_key == "x"){
            latlon_mode = true; //APIキーなしではplacesAPIを呼び出せない。
          }
          if(poi_name_.isEmpty() == false){
            //緯度経度として解釈できるか
            static QRegularExpression re(R"(^[\s]*([-+]?\d+\.\d+)[ ,]+([-+]?\d+\.\d+)[\s]*$)"); //緯度,軽度の正規表現。カンマの前後のスペースは許容しない。
            QRegularExpressionMatch match = re.match(poi_name_);
            if (match.hasMatch()) {
              g_latitude = match.captured(1).toDouble(); // 1つ目の浮動小数を取得
              g_longitude = match.captured(2).toDouble(); // 2つ目の浮動小数を取得
              chg_coordinate = true;
              return;
            }
            //APIキーなし(x)で緯度経度のダイレクト数値以外の場合は、Places APIを呼び出さない（latlon_mode == true）
          }

          if (poi_name_.isEmpty() == false /*&& poi_name.isEmpty() == false*/ && latlon_mode == false){
            //Places API呼び出し。
            // g_latitude = m_map->coordinate().first;
            // g_longitude = m_map->coordinate().second;

            QJsonObject jsonObject;
            jsonObject["textQuery"] = poi_name;
            jsonObject["pageSize"] = 1;
            QJsonObject center;
            center["latitude"] = g_latitude;
            center["longitude"] = g_longitude;
            QJsonObject circle;
            circle["center"] = center;
            QJsonObject locationBias;
            locationBias["circle"] = circle;
            jsonObject["locationBias"] = locationBias;

            QNetworkAccessManager *manager = new QNetworkAccessManager();

            // Create the request
            QNetworkRequest request(QUrl("https://places.googleapis.com/v1/places:searchText"));
            request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
            request.setRawHeader("X-Goog-Api-Key", gg_key.toUtf8());
            request.setRawHeader("X-Goog-FieldMask", "places.location");

            // Convert JSON object to QByteArray
            QJsonDocument jsonDoc(jsonObject);
            QByteArray jsonData = jsonDoc.toJson();

            // Send the POST request
            QNetworkReply *reply = manager->post(request, jsonData);

            // Connect to the reply's finished signal to handle the response
            QObject::connect(reply, &QNetworkReply::finished, [reply]() {
              if (reply->error() == QNetworkReply::NoError) {
                // Parse the response
                QByteArray responseData = reply->readAll();
                QJsonDocument all_res = QJsonDocument::fromJson(responseData);
                if(all_res.isObject()){
                  QJsonObject topObj = all_res.object();
                  if(topObj.contains("places")){
                    QJsonValue places = topObj["places"];
                    if(places.isArray()){
                      for (const QJsonValue &value : places.toArray()) {
                        if (value.isObject()) {
                          QJsonObject jsonObj = value.toObject();
                          if (jsonObj.contains("location")) {
                            QJsonObject location = jsonObj["location"].toObject();
                            g_latitude = location["latitude"].toDouble();
                            g_longitude = location["longitude"].toDouble();
                            chg_coordinate = true;
                          }
                        }
                        break; //最初の一個だけで良い。
                      }
                    }
                  }
                } else {
                  //qWarning() << "Response is not a JSON array";
                }
              } else {
                //qWarning() << "Error:" << reply->errorString();
              }
              reply->deleteLater();
            });

            // Optional: Handle network errors
            QObject::connect(manager, &QNetworkAccessManager::finished, manager, &QNetworkAccessManager::deleteLater);
          }
        }
        //Params().put("NavDestination", last_navi_dest);
        return;
      }

      const SubMaster &sm = *(uiState()->sm);
      int on_vavi_highway = getButtonInt("/data/navi_highway.txt",0); //1:高速有料を除外する。（exclude=toll,motorway）
      if (sm.valid("navInstruction")) {
        //ナビ中
        on_vavi_highway ^= 1; //反転
        FILE *fp = fopen("/tmp/route_style_reload.txt","w");
        if(fp != NULL){
          fprintf(fp,"%d",1);
          fclose(fp);
        }
        setButtonInt("/data/navi_highway.txt",on_vavi_highway);
      }
      if(on_vavi_highway){
        speed->setStyleSheet(QString(btn_styleb_trs).arg("#10A010"));
      } else {
        speed->setStyleSheet(QString(btn_styleb_trs).arg("#2457A1"));
      }
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

extern int g_night_mode; //onroadの制御を使う。
void MapLimitspeed::paintEvent(QPaintEvent *event) {

  float r = LS_SIZE / 2;
  QPainter p(this);
  p.setPen(Qt::NoPen);
  if((g_night_mode == 1 && night_mode == -1) || night_mode == 1){
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
      if(now - m_pressedTime > 1500){
        //qDebug() << "long clicked"; //これでは放さないと長押しが取れない。
        //2秒以上長押しでzoom_offsetクリア。
        zoom_offset = 0;
        reset_zoom = true; //ズームリセット専用。
        void soundButton(int onOff);
        soundButton(false);
        setButtonInt("/data/mb_zoom_offset.txt",(int)(zoom_offset * 1000));
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
        if(MIN_PITCH_ == -1){
          //head->north
          head_north = true; //地図の角度をリセットする。
        } else if(MIN_PITCH_ == 0){
          //north->head
          north_head = true; //地図の角度をリセットする。
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
  //map_scale_num = pow(2,(18-scale)) * 23 * cos(DEG2RAD(latitude)); //metersPerPixelAtLatitudeと比べてほぼ合っている結果になった。
  map_scale_num = QMapLibre::metersPerPixelAtLatitude(latitude, scale) * (BS_SIZE_W / MAP_SCALE); //地図のサービス関数でやる方法。独自300メートルが292メートルになってしまう（緯度によって変わる）MIN_ZOOM=14固定をやめることにする。
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

  bool tmp_bs_color_revert = bs_color_revert;
  if((g_night_mode == 1 && night_mode == -1) || night_mode == 1){
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
