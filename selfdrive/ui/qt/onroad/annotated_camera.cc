
#include "selfdrive/ui/qt/onroad/annotated_camera.h"

#include <QPainter>
#include <algorithm>
#include <cmath>
#include <sstream>

#include "common/swaglog.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#include "selfdrive/ui/qt/onroad/buttons.h"
#include "selfdrive/ui/qt/util.h"

#define PI0_DEBUG false

// Window that shows camera view and variety of info drawn on top
AnnotatedCameraWidget::AnnotatedCameraWidget(VisionStreamType type, QWidget* parent) : fps_filter(UI_FREQ, 3, 1. / UI_FREQ), CameraWidget("camerad", type, true, parent) {
  pm = std::make_unique<PubMaster, const std::initializer_list<const char *>>({"uiDebug"});

  main_layout = new QVBoxLayout(this);
/*
  main_layout->setMargin(UI_BORDER_SIZE);
  main_layout->setSpacing(0);

  experimental_btn = new ExperimentalButton(this);
  main_layout->addWidget(experimental_btn, 0, Qt::AlignTop | Qt::AlignRight);

  map_settings_btn = new MapSettingsButton(this);
  main_layout->addWidget(map_settings_btn, 0, Qt::AlignBottom | Qt::AlignRight);
*/
  map_settings_btn = new MapSettingsButton(this);

  buttons = new ButtonsWindow(this , map_settings_btn); //ここならばexperimental_btnとイベントの両立ができ、マップの右画面のスクロール操作ができる。->ExperimentalButtonをLayoutで囲むとイベントが先に登録勝ちになってしまう。
  QObject::connect(uiState(), &UIState::uiUpdate, buttons, &ButtonsWindow::updateState);
  main_layout->addWidget(buttons);

  engage_img = loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size});
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size - 5, img_size - 5});

  dm_img = loadPixmap("../assets/img_driver_face.png", {img_size + 5, img_size + 5});
}

bool global_engageable;
float vc_speed;
static int tss_type = 0;
static float maxspeed_org;
std::string road_info_txt;
bool g_rightHandDM;
int ACC_speed;
extern void setButtonInt(const char*fn , int num);
extern int getButtonInt(const char*fn , int defaultNum);
void AnnotatedCameraWidget::updateState(const UIState &s) {
  int SET_SPEED_NA = 409; //406; //557; //255; ,
  const SubMaster &sm = *(s.sm);

  const bool cs_alive = sm.alive("controlsState");
  const bool nav_alive = sm.alive("navInstruction") && sm["navInstruction"].getValid();
  const auto cs = sm["controlsState"].getControlsState();
  const auto car_state = sm["carState"].getCarState();
  const auto nav_instruction = sm["navInstruction"].getNavInstruction();

  // Handle older routes where vCruiseCluster is not set
  float v_cruise = cs.getVCruiseCluster() == 0.0 ? cs.getVCruise() : cs.getVCruiseCluster();
  ACC_speed = std::nearbyint(v_cruise); //45〜
  v_cruise = cs.getVCruise(); //41〜,間違いない、表示して確認した。改めてこちらを使う。
  maxspeed_org = cs.getVCruise(); //これで元の41〜 , v_cruise; //レバー値の元の値。黄色点滅警告にはマッチしてる気がする。
  //maxspeed_org = v_cruise; //getVCruiseを使うと点滅しすぎる？
  if(tss_type == 0){
    std::string tss_type_txt = util::read_file("/data/tss_type_info.txt");
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
    //これまでと互換。tss_type_infoがなければTSSP。ここの計算はcruise_info.txtの内容で上書きされる。できればここでの計算は無しにしたい。
    v_cruise = v_cruise < (55 - 4) ? (55 - (55 - (v_cruise+4)) * 2 - 4) : v_cruise;
  //v_cruise = v_cruise > (110 - 6) ? (110 + ((v_cruise+6) - 110) * 3 - 6) : v_cruise; //最大119
    v_cruise = v_cruise > (107 - 6) ? (107 + ((v_cruise+6) - 107) * 2 - 6) : v_cruise; //最大119 -> 114 -> 117に。
  } else if(PI0_DEBUG == true || tss_type == 2){
    SET_SPEED_NA = 255; //TSS2では戻す。
  }

  setSpeed = cs_alive ? v_cruise : SET_SPEED_NA;
  is_cruise_set = setSpeed > 0 && (int)setSpeed != SET_SPEED_NA;
  if (is_cruise_set && !s.scene.is_metric) {
    setSpeed *= KM_TO_MILE;
  }

  // Handle older routes where vEgoCluster is not set
  v_ego_cluster_seen = v_ego_cluster_seen || car_state.getVEgoCluster() != 0.0;
  float v_ego = v_ego_cluster_seen ? car_state.getVEgoCluster() : car_state.getVEgo();
  speed = cs_alive ? std::max<float>(0.0, v_ego) : 0.0;
  vc_speed = v_ego;
  QString maxspeed_str = is_cruise_set ? QString::number(std::nearbyint(setSpeed)) : "N/A";
  std::string stdstr_txt = util::read_file("/tmp/cruise_info.txt");
  static std::string stdstr_txt_save;
  if(is_cruise_set && stdstr_txt.empty() == false){
    QString qstr = QString::fromStdString(stdstr_txt);
    maxspeed_str = qstr;
    stdstr_txt_save = stdstr_txt;
  } else if(is_cruise_set && stdstr_txt_save.empty() == false){
    QString qstr = QString::fromStdString(stdstr_txt_save);
    maxspeed_str = qstr;
    stdstr_txt_save.clear(); //過去数字の使用は一度限定。
  }
  speed *= s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH;

  auto speed_limit_sign = nav_instruction.getSpeedLimitSign();
  float old_speedLimit = speedLimit;
  speedLimit = nav_alive ? nav_instruction.getSpeedLimit() : 0.0;
  speedLimit *= (s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH);
  if(speedLimit != old_speedLimit){ //km/h限定
    FILE *fp = fopen("/tmp/limitspeed_navi.txt","w");
    if(fp != NULL){
      fprintf(fp,"%d",(int)std::nearbyint(speedLimit)); //29とか出るので、その対策？
      fclose(fp);
    }
  }
  has_us_speed_limit = false && (nav_alive && speed_limit_sign == cereal::NavInstruction::SpeedLimitSign::MUTCD);
  has_eu_speed_limit = false && (nav_alive && speed_limit_sign == cereal::NavInstruction::SpeedLimitSign::VIENNA);
  // レイアウトは崩れるが、速度は取れる模様。OSMの速度情報の補完には使えるか？
  is_metric = s.scene.is_metric;
  speedUnit =  s.scene.is_metric ? tr("km/h") : tr("mph");
  hideBottomIcons = (cs.getAlertSize() != cereal::ControlsState::AlertSize::NONE);
  status = s.status;

  maxSpeed = maxspeed_str; //ichiro pilot

  // update engageability/experimental mode button
//  experimental_btn->updateState(s);
  buttons->psn_update();
  global_engageable = (cs.getEngageable() || cs.getEnabled());

  // update DM icon
  auto dm_state = sm["driverMonitoringState"].getDriverMonitoringState();
  dmActive = dm_state.getIsActiveMode();
  rightHandDM = dm_state.getIsRHD();
  g_rightHandDM = rightHandDM;
  // DM icon transition
  dm_fade_state = std::clamp(dm_fade_state+0.2*(0.5-dmActive), 0.0, 1.0);

  // hide map settings button for alerts and flip for right hand DM
  // if (map_settings_btn->isEnabled()) {
  //   map_settings_btn->setVisible(!hideBottomIcons);
  //   main_layout->setAlignment(map_settings_btn, (rightHandDM ? Qt::AlignLeft : Qt::AlignRight) | Qt::AlignBottom);
  // }
  map_settings_btn->setVisible(true); //他のボタンの位置へ影響するので、出しっぱなしにする。
}

static bool all_brake_light = false;
int global_status;
float curve_value;
static float handle_center = -100;
static int handle_calibct = 0;
static float distance_traveled;
static float global_angle_steer0 = 0;
static float clipped_brightness0 = 101; //初回ファイルアクセスさせるため、わざと101
static float global_fps;
bool add_v_by_lead;
int limit_speed_auto_detect; //map.ccから参照あり
extern int Limit_speed_mode; //標識
extern bool Long_enable;
extern bool mapVisible;
extern void soundButton2(int onOff);
extern void setButtonEnabled0(const char*fn , bool flag);
int g_night_mode;
void AnnotatedCameraWidget::drawHud(QPainter &p) {
  p.save();
  int y_ofs = 150;

  // Header gradient
  QLinearGradient bg(0, UI_HEADER_HEIGHT - (UI_HEADER_HEIGHT / 2.5), 0, UI_HEADER_HEIGHT);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), UI_HEADER_HEIGHT+y_ofs, bg);

  QString speedLimitStr = (speedLimit > 1) ? QString::number(std::nearbyint(speedLimit)) : "–";
  QString speedStr = QString::number(std::nearbyint(speed));
  //QString setSpeedStr = is_cruise_set ? QString::number(std::nearbyint(setSpeed)) : "–";

  // max speed
  float max_disp_k = 1.8;
  //float max_disp_a = 50;
  const int rect_w = rect().width();
  const int rect_h = rect().height();
  if(false && (float)rect_w / rect_h > 1.4f){
  } else {
    //こちらの大きさを採用。
    max_disp_k = 1.2; //1.3;
    //max_disp_a = 20;
  }

  // Draw outer box + border to contain set speed and speed limit
  const int sign_margin = 12 * max_disp_k;
  const int us_sign_height = 186 * max_disp_k;
  const int eu_sign_size = 176 * max_disp_k;

  const QSize default_size = {(int)(172 * max_disp_k), (int)(204 * max_disp_k)};
  QSize set_speed_size = default_size;
  if (is_metric || has_eu_speed_limit) set_speed_size.rwidth() = 200 * max_disp_k;
  if (has_us_speed_limit && speedLimitStr.size() >= 3) set_speed_size.rwidth() = 223 * max_disp_k;

  if (has_us_speed_limit) set_speed_size.rheight() += us_sign_height + sign_margin;
  else if (has_eu_speed_limit) set_speed_size.rheight() += eu_sign_size + sign_margin;

  int top_radius = 32 * max_disp_k;
  int bottom_radius = (has_eu_speed_limit ? 100 : 32) * max_disp_k;

  // QRect set_speed_rect(60 + default_rect_width / 2 - rect_width / 2, 45 +y_ofs, rect_width, rect_height);
  QRect set_speed_rect(QPoint(60 + (default_size.width() - set_speed_size.width()) / 2, 45 +y_ofs), set_speed_size);
  QString ms = QString(maxSpeed);
  bool limit_speed_override = false;
  add_v_by_lead = false;
  if(ms.length() > 1){
    if(maxSpeed.mid(0,1) == ";"){ //先頭セミコロンで制限速度適用
      ms = maxSpeed.mid(1,maxSpeed.length()-1);
      //p.setPen(QPen(QColor(205, 44, 38, 255), 12)); //標識の赤枠の色に合わせる
      p.setPen(QPen(QColor(0xff, 0xff, 0xff, 255*0.9), 6)); //枠を白
      limit_speed_override = true;
    } else if(maxSpeed.mid(0,1) == ","){ //先頭カンマで増速
      add_v_by_lead = true;
      ms = maxSpeed.mid(1,maxSpeed.length()-1);
      p.setPen(QPen(QColor(0, 0xff, 0, 200), 6)); //前走車追従時は緑
    } else if(maxSpeed.mid(maxSpeed.length()-1,1) == "."){ //末尾ピリオドで減速
      ms = maxSpeed.mid(0,maxSpeed.length()-1);
      p.setPen(QPen(QColor(0xff, 0, 0, 200), 6)); //減速時は赤
    } else if(maxSpeed.mid(maxSpeed.length()-1,1) == ";"){ //末尾セミコロンで黄色
      ms = maxSpeed.mid(0,maxSpeed.length()-1);
      p.setPen(QPen(QColor(0xff, 0xff, 0, 200), 6)); //黄色
    } else {
      p.setPen(QPen(whiteColor(75), 6));
    }
  } else {
    p.setPen(QPen(whiteColor(75), 6));
  }
  static unsigned int yellow_flash_ct = 0;
  yellow_flash_ct ++;
  bool db_rec_mode = false;
  if(limit_speed_override == false){
    bool yellow_flag = false;
    if(Limit_speed_mode == 2){
      p.setBrush(QColor::fromRgbF(0.4, 0.0, 0, 1.0));
      if(ms.toDouble() >= 30){
        db_rec_mode = true;
      }
      yellow_flag = true;
    } else if((Limit_speed_mode == 1 && limit_speed_auto_detect == 1)){
      if(maxspeed_org+12 <= ms.toDouble() && maxspeed_org+5 < vc_speed * 3.6){
        if(yellow_flash_ct %6 < 3){
          p.setBrush(QColor::fromRgbF(1.0, 1.0, 0, 1.0)); //速度がレバーより10km/h以上高いとギクシャクする警告、点滅させる。
          yellow_flag = true;
        }
      }
    }
    if(yellow_flag == false){
      p.setBrush(blackColor(166));
    }
  } else {
    if(maxspeed_org+12 > ms.toDouble() || maxspeed_org+5 >= vc_speed * 3.6){
  AUTO_back_color:
      if(g_night_mode == 0){
        p.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, 0.9)); //速度標識の地の色に合わせる。
      } else {
        p.setBrush(QColor::fromRgbF(0.8, 0.8, 0.9, 0.9)); //標識バックを薄暗く。
      }
    } else {
      if(yellow_flash_ct %6 < 3){
        p.setBrush(QColor::fromRgbF(1.0, 1.0, 0, 1.0)); //速度がレバーより10km/h以上高いとギクシャクする警告、点滅させる。
      } else {
        goto AUTO_back_color;
      }
    }
  }
  drawRoundedRect(p, set_speed_rect, top_radius, top_radius, bottom_radius, bottom_radius);
  if(limit_speed_override == true || (Limit_speed_mode == 1 && limit_speed_auto_detect == 1)){
    //太い赤枠を内側に描画する。
    const int ls_w2 = 27;
    // QRect set_speed_rect2(60 + default_rect_width / 2 - rect_width / 2 +ls_w2/2, 45 +y_ofs +ls_w2/2, rect_width - ls_w2, rect_height -ls_w2);
    QRect set_speed_rect2(60 + (default_size.width() - set_speed_size.width()) / 2 +ls_w2/2, 45 +y_ofs +ls_w2/2, set_speed_size.width() - ls_w2 , set_speed_size.height() - ls_w2);
    p.setPen(QPen(QColor(205, 44, 38, (limit_speed_override ? 255 : 180)), ls_w2)); //標識の赤枠の色に合わせる
    p.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, 0)); //２０描画はしない
    drawRoundedRect(p, set_speed_rect2, top_radius-ls_w2/2, top_radius-ls_w2/2, bottom_radius-ls_w2/2, bottom_radius-ls_w2/2);
  }

  QString setSpeedStr = is_cruise_set ? ms : "–";

  // Draw MAX
  QColor max_color = QColor(0x80, 0xd8, 0xa6, 0xff);
  QColor set_speed_color = whiteColor();
  if(limit_speed_override == true){
    max_color = QColor(0x24, 0x57, 0xa1 , 255); //速度標識の数字に合わせる。
  } else if (is_cruise_set) {
    if (status == STATUS_DISENGAGED) {
      max_color = whiteColor();
    } else if (status == STATUS_OVERRIDE) {
      max_color = QColor(0x91, 0x9b, 0x95, 0xff);
    } else if (speedLimit > 0 && Limit_speed_mode != 1) { //ACC自動設定時は警告カラー設定をしない
      auto interp_color = [=](QColor c1, QColor c2, QColor c3) {
        return speedLimit > 0 ? interpColor(setSpeed, {speedLimit + 5, speedLimit + 15, speedLimit + 25}, {c1, c2, c3}) : c1;
      };
      max_color = interp_color(max_color, QColor(0xff, 0xe4, 0xbf), QColor(0xff, 0xbf, 0xbf));
      set_speed_color = interp_color(set_speed_color, QColor(0xff, 0x95, 0x00), QColor(0xff, 0x00, 0x00));
    }
  } else {
    max_color = QColor(0xa6, 0xa6, 0xa6, 0xff);
    set_speed_color = QColor(0x72, 0x72, 0x72, 0xff);
  }
  p.setFont(InterFont(40*max_disp_k, QFont::DemiBold));
  QString MAX_AUTO = db_rec_mode == true ? tr("REC") : (limit_speed_override == false ? tr("MAX") : tr("AUTO"));
  p.setPen(max_color);
  p.drawText(set_speed_rect.adjusted(0, 27*max_disp_k, 0, 0), Qt::AlignTop | Qt::AlignHCenter, MAX_AUTO);
  p.setFont(InterFont(90*max_disp_k, QFont::Bold));

  static int red_signal_scan_flag = 0;
  static unsigned int red_signal_scan_flag_txt_ct = 0;
  if(red_signal_scan_flag_txt_ct % 7 == 0){
    std::string red_signal_scan_flag_txt = util::read_file("/tmp/red_signal_scan_flag.txt");
    if(red_signal_scan_flag_txt.empty() == false){
      if(uiState()->scene.mAccelEngagedButton >= 3){
        red_signal_scan_flag = std::stoi(red_signal_scan_flag_txt);
      } else {
        red_signal_scan_flag = 0;
      }
    }
  }
  red_signal_scan_flag_txt_ct ++;

  static long long int night_mode_ct;
  if((red_signal_scan_flag >= 2 && (night_mode_ct ++) % 11 == 0) || red_signal_scan_flag_txt_ct % (20*5) == 1 /*5秒に一回は更新*/){
    if (uiState()->scene.started) {
      float clipped_brightness = uiState()->scene.light_sensor;

      // CIE 1931 - https://www.photonstophotos.net/GeneralTopics/Exposure/Psychometric_Lightness_and_Gamma.htm
      if (clipped_brightness <= 8) {
        clipped_brightness = (clipped_brightness / 903.3);
      } else {
        clipped_brightness = std::pow((clipped_brightness + 16.0) / 116.0, 3.0);
      }

      // Scale back to 0% to 100%
      clipped_brightness = std::clamp(100.0f * clipped_brightness, 0.0f, 100.0f);

      if(clipped_brightness0 != clipped_brightness){
        clipped_brightness0 = clipped_brightness;
        setButtonInt("/tmp/night_time_info.txt" , (int)clipped_brightness);

        g_night_mode = clipped_brightness0 < (g_night_mode == 1 ? 90 : 75); //ばたつかないようにする。80程度でかなり夕方。
      }
    }
  }

  if(red_signal_scan_flag >= 3){
    set_speed_color = QColor(0xff, 0, 0 , 255);
  } else if(limit_speed_override == true){
    set_speed_color = QColor(0x24, 0x57, 0xa1 , 255); //速度標識の数字に合わせる。
  }
  p.setPen(set_speed_color);
  if(setSpeedStr == "1" && uiState()->scene.mAccelEngagedButton == 4){ //MAXが1の時
    std::string red_signal_eP_iP_set_txt = util::read_file("/tmp/red_signal_eP_iP_set.txt");
    bool red_signal_eP_iP_set = false;
    if(red_signal_eP_iP_set_txt.empty() == false){
      if(std::stoi(red_signal_eP_iP_set_txt) == 1){
        red_signal_eP_iP_set = true;
      }
    }
    p.drawText(set_speed_rect.adjusted(0, 77*max_disp_k, 0, 0), Qt::AlignTop | Qt::AlignHCenter, red_signal_eP_iP_set == 0 ? "8" : setSpeedStr);
  } else {
    p.drawText(set_speed_rect.adjusted(0, 77*max_disp_k, 0, 0), Qt::AlignTop | Qt::AlignHCenter, setSpeedStr);
  }

  const QRect sign_rect = set_speed_rect.adjusted(sign_margin, default_size.height(), -sign_margin, -sign_margin);
  // US/Canada (MUTCD style) sign
  if (has_us_speed_limit) {
    //ケアしていない
    p.setPen(Qt::NoPen);
    p.setBrush(whiteColor());
    p.drawRoundedRect(sign_rect, 24*max_disp_k, 24*max_disp_k);
    p.setPen(QPen(blackColor(), 6));
    p.drawRoundedRect(sign_rect.adjusted(9*max_disp_k, 9*max_disp_k, -9*max_disp_k, -9*max_disp_k), 16*max_disp_k, 16*max_disp_k);

    p.setFont(InterFont(28*max_disp_k, QFont::DemiBold));
    p.drawText(sign_rect.adjusted(0, 22*max_disp_k, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("SPEED"));
    p.drawText(sign_rect.adjusted(0, 51*max_disp_k, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("LIMIT"));
    p.setFont(InterFont(70*max_disp_k, QFont::Bold));
    p.drawText(sign_rect.adjusted(0, 85*max_disp_k, 0, 0), Qt::AlignTop | Qt::AlignHCenter, speedLimitStr);
  }

  // EU (Vienna style) sign
  if (has_eu_speed_limit) {
    //ケアしていない
    p.setPen(Qt::NoPen);
    p.setBrush(whiteColor());
    p.drawEllipse(sign_rect);
    p.setPen(QPen(Qt::red, 20*max_disp_k));
    p.drawEllipse(sign_rect.adjusted(16*max_disp_k, 16*max_disp_k, -16*max_disp_k, -16*max_disp_k));

    p.setFont(InterFont((speedLimitStr.size() >= 3) ? 60*max_disp_k : 70*max_disp_k, QFont::Bold));
    p.setPen(blackColor());
    p.drawText(sign_rect, Qt::AlignCenter, speedLimitStr);
  }

  QColor speed_waku;
  if(red_signal_scan_flag >= 1/*== 1*/){
    if(speed > 45 && clipped_brightness0 >= 90 //昼は45超を信頼度低いとする。
      || speed > 65 && clipped_brightness0 < 90){ //夜は65超を信頼度低いとする。
      speed_waku = QColor(171, 171, 0 , 255);
    } else {
      speed_waku = QColor(0xff, 0, 0 , 255);
    }
  // } else if(red_signal_scan_flag >= 2){
  //   speed_waku = QColor(0xff, 0xff, 0 , 255);
  } else {
    speed_waku = bg_colors[status];
  }
  // current speed
  p.setFont(InterFont(176, QFont::Bold));
  UIState *s = uiState();
  double velo_for_trans = (*s->sm)["carState"].getCarState().getVEgo() * 3.6; //km/h
  const double velo_for_trans_limit = 0.05; //0.05km/h以下では速度を半透明にする。
  if(velo_for_trans >= velo_for_trans_limit){
    drawText(p, rect().center().x()-7, 210+y_ofs-5, speedStr,speed_waku);
    drawText(p, rect().center().x()+7, 210+y_ofs-5, speedStr,speed_waku);
    drawText(p, rect().center().x(), -7+210+y_ofs-5, speedStr,speed_waku);
    drawText(p, rect().center().x(), +7+210+y_ofs-5, speedStr,speed_waku);
  }
  QColor speed_num;
  static bool red_signal_scan_flag_2 = false;
  if(red_signal_scan_flag >= 2 && red_signal_scan_flag_txt_ct %6 < 3){
    speed_num = QColor(0xff, 100, 100 , 255); //赤信号認識中は点滅。
    if(red_signal_scan_flag_2 == false && setSpeedStr != "1"){
      red_signal_scan_flag_2 = true;
      if(red_signal_scan_flag == 2){
        soundButton2(1); //pikiriオンを信号認識開始に転用。
      }
    }
  } else {
    speed_num = QColor(0xff, 0xff, 0xff , 255);
    if(red_signal_scan_flag < 2 && this->speed > 24){
      red_signal_scan_flag_2 = false; //ある程度スピードが上がらないと、このフラグも戻さない。
    }
    if(velo_for_trans < velo_for_trans_limit){
      speed_num = QColor(0xff, 0xff, 0xff , 70);
    }
  }
  drawText(p, rect().center().x(), 210 + y_ofs-5, speedStr , speed_num);

  p.setFont(InterFont(66));
  if (uiState()->scene.longitudinal_control == false) {
#define COLOR_STATUS_WARNING QColor(0xDA, 0x6F, 0x25, 0xf1)
//  [STATUS_ALERT] = QColor(0xC9, 0x22, 0x31, 0xf1),
    drawText(p, rect().center().x(), 290 + y_ofs-5, speedUnit, COLOR_STATUS_WARNING); //縦制御無効状態でkm/hを警告色に。
  } else {
    int w;
    if(velo_for_trans < velo_for_trans_limit){
      w = drawText(p, rect().center().x(), 290 + y_ofs-5, speedUnit, speed_num);
    } else {
      w = drawText(p, rect().center().x(), 290 + y_ofs-5, speedUnit, 200);
    }
    if(is_cruise_set){
      p.setFont(InterFont(40, QFont::ExtraBold));
      drawTextCenter(p, rect().center().x() + w/2 + 43, 290 + y_ofs-35 , QString::number(ACC_speed) , velo_for_trans < velo_for_trans_limit ? 100 : 235 , false , 0x24, 0x57, 0xa1 , 240, 240, 240, velo_for_trans < velo_for_trans_limit ? 70 : 230 , 9 , 15 , 18 , 2);
    }
  }

//以下オリジナル表示要素
  //温度を表示(この画面は更新が飛び飛びになる。ハンドル回したりとか何か変化が必要)
  auto deviceState = (*s->sm)["deviceState"].getDeviceState();
  //int temp = (int)deviceState.getAmbientTempC();
  int temp = 0; //温度が取れなくなったので目安。
  auto ts = deviceState.getThermalStatus();
  if (ts == cereal::DeviceState::ThermalStatus::GREEN) {
    //tempStatus = {{tr("TEMP"), tr("GOOD")}, good_color};
    temp = 55; //色変化のための参照値
  } else if (ts == cereal::DeviceState::ThermalStatus::YELLOW) {
    //tempStatus = {{tr("TEMP"), tr("OK")}, warning_color};
    temp = 65; //色変化のための参照値
  } else {
    temp = 75; //色変化のための参照値
  }
  int max_temp = (int)deviceState.getMaxTempC(); //表示はこれを使う。

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
  //QString temp_disp1 = QString(okConnect ? "⚫︎" : "⚪︎");
  QString temp_disp1 = QString(okConnect ? "●" : "○");
  QString temp_disp2 = QString(okGps ? "★" : "☆");
  QString temp_disp3 = QString::number(max_temp) + "°C";
  //QString temp_disp = QString(okConnect ? "⚫︎ " : "⚪︎ ") + QString(okGps ? "◆ " : "◇ ") + QString::number(temp) + "°C";

  //制限速度情報をmap.ccからonroadへ移動
  static unsigned int limitspeed_update_ct;
  static double car_bearing;
  if ((limitspeed_update_ct ++) % 10 == 0 && (*s->sm).updated("liveLocationKalman")) {
    auto locationd_location = (*s->sm)["liveLocationKalman"].getLiveLocationKalman();
    auto locationd_pos = locationd_location.getPositionGeodetic();
    auto locationd_orientation = locationd_location.getCalibratedOrientationNED();
    auto locationd_velocity = locationd_location.getVelocityCalibrated();

    bool locationd_valid = (locationd_location.getStatus() == cereal::LiveLocationKalman::Status::VALID) &&
      locationd_pos.getValid() && locationd_orientation.getValid() && locationd_velocity.getValid();

    if (locationd_valid) {
      FILE *fp = fopen("/tmp/limitspeed_info.txt","w");
      if(fp != NULL){
        //この辺で30mか1秒？ごとに、以下を/tmp/limitspeed_info.txtに書き込む。
        double latitude = locationd_pos.getValue()[0]; // 緯度を取得
        double longitude = locationd_pos.getValue()[1]; // 経度を取得
        double bearing = RAD2DEG(locationd_orientation.getValue()[2]);  //-180〜180
        if(bearing < 0){
          bearing += 360;
          if(bearing >= 360){
            bearing = 0;
          }
        } //0〜360へ変換、クエリの角度差分計算は-180でも大丈夫だったみたい。
        //double velo = (*s->sm)["carState"].getCarState().getVEgo() * 3.6; //km/h
        car_bearing = bearing;
        double velo = velo_for_trans;
        if(add_v_by_lead == true){
          velo /= 1.15; //前走車追従中は、増速前の推定速度を学習する。
        }
#if 0 //保留。"○"ボタンONでは思い切って記録しないという選択もありか？
        if(Limit_speed_mode == 2 && ms.toDouble() >= 30){
           velo = ms.toDouble(); //刈り取りモードではMAX値を記録する手もある。
        }
#endif
        QDateTime currentTime = QDateTime::currentDateTime(); // 現在時刻を表すQDateTimeオブジェクトを作成
        double now = (double)currentTime.toMSecsSinceEpoch() / 1000;
        fprintf(fp,"%.7f,%.7f,%.7f,%.3f,%.3f",latitude,longitude,bearing,velo,now);
        fclose(fp);
      }
    }
  }
#endif
  p.setFont(InterFont(44, QFont::DemiBold));

  int th_tmp1 = 47;
  int th_tmp2 = 55;
  if(Hardware::TICI()){
    th_tmp1 = 62; //ここから黄色
    th_tmp2 = 71; //ここから赤
  }

  QRect temp_rc(rect().left()+65-27, rect().top()+110+6, 233+27*2-5, 54);
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
  p.drawText(QRect(rect().left()+65+120-5, rect().top()+110+7, 300, 65), Qt::AlignTop | Qt::AlignLeft, temp_disp3);
  p.setFont(InterFont(54, QFont::Bold));
  p.drawText(QRect(rect().left()+65+55+5-5, rect().top()+110-8+9, 300, 65), Qt::AlignTop | Qt::AlignLeft, temp_disp2);
  p.setFont(InterFont(48));
  p.drawText(QRect(rect().left()+65+5+5, rect().top()+110-8+11, 300, 65+5), Qt::AlignTop | Qt::AlignLeft, temp_disp1);

  bool brake_light = false; //ブレーキランプは無くなった？(*(uiState()->sm))["carState"].getCarState().getBrakeLightsDEPRECATED();
  all_brake_light = false;
  int logo_trs = 150;
  std::string brake_light_txt = util::read_file("/tmp/brake_light_state.txt");
  if(brake_light_txt.empty() == false){
    if(std::stoi(brake_light_txt) != 0){
      if(global_engageable){
        brake_light = true;
        logo_trs = 80; //drawText内部で100足される。
      }
      all_brake_light = true; //こちらはエンゲージしていなくてもセットされる。
    }
  }
  const auto cp = (*uiState()->sm)["carParams"].getCarParams();
  bool tssp_47700 = (((cp.getFlags() & 2048) != 0) && brake_light == false);
  int label_red = 255;
  int label_grn = 255;
  int label_blu = 255;
  if(tssp_47700){
    label_blu = 0;
  }
  if((float)rect_w / rect_h > 1.5f){
    p.setFont(InterFont(44, QFont::DemiBold));
    drawText(p, rect().left()+260, 55, "Powered by COMMA.AI", logo_trs, brake_light);
  } else if((float)rect_w / rect_h > 1.02f){
    p.setFont(InterFont(44, QFont::DemiBold));
    drawText(p, rect().left()+140, 55, "COMMA.AI", logo_trs, brake_light);
  } else if((float)rect_w / rect_h >= 0.975f){ //消えるタイミングは合わせたい。
    p.setFont(InterFont(44, QFont::DemiBold));
    drawText(p, rect().left()+102, 55, "COMMA", logo_trs, brake_light);
  }
  if((float)rect_w / rect_h > 1.35f){
    p.setFont(InterFont(55, QFont::DemiBold));
    if(tss_type <= 1){
      int next_x = drawTextRight(p, rect().right()-20, 60 , " TSSP", logo_trs, brake_light, label_red, label_grn, label_blu); //47060車はTSSP部分が黄色くなる。
      drawTextRight(p, next_x, 60 , "for toyota", logo_trs, brake_light);
    } else {
      drawTextRight(p, rect().right()-20, 60 , "for toyota TSS2", logo_trs, brake_light);
    }
  } else if((float)rect_w / rect_h > 1.2f){
    p.setFont(InterFont(55, QFont::DemiBold));
    if(tss_type <= 1){
      int next_x = drawTextRight(p, rect().right()-20, 60 , " TSSP", logo_trs, brake_light, label_red, label_grn, label_blu);
      drawTextRight(p, next_x, 60 , "toyota", logo_trs, brake_light);
    } else {
      drawTextRight(p, rect().right()-20, 60 , "toyota TSS2", logo_trs, brake_light);
    }
  } else if((float)rect_w / rect_h >= 0.975f){ //消えるタイミングは合わせたい。
    p.setFont(InterFont(50, QFont::DemiBold));
    if(tss_type <= 1){
      int next_x = drawTextRight(p, rect().right()-20, 60 , " P", logo_trs, brake_light, label_red, label_grn, label_blu);
      drawTextRight(p, next_x, 57 , "toyota", logo_trs, brake_light);
    } else {
      drawTextRight(p, rect().right()-20, 57 , "toyota 2", logo_trs, brake_light);
    }
  }
  p.setFont(InterFont(33, QFont::DemiBold));
  bool road_info_txt_flag = false;
  static unsigned int road_info_txt_ct = 0;
  if(road_info_txt_ct++ % 20 == 0){
    road_info_txt = util::read_file("/tmp/road_info.txt");
  }
  if(road_info_txt.empty() == false){
    int i = 0; // インデックス
    std::stringstream ss(road_info_txt); // 入力文字列をstringstreamに変換
    std::string kmh; //制限速度
    std::string token; // 一時的にトークンを格納する変数(最終的に道路名)
    static int road_th_ct_ct = 0;
    static int before_road_th_ct = 0;
    while (i < 3 && std::getline(ss, token, ',')) { // カンマで分割し、一つずつ処理する
      if(i == 0){
        int road_th_ct = std::stoi(token);
        if(road_th_ct == before_road_th_ct){
          road_th_ct_ct ++; //road_th_ctが変化しなければカウントアップし続ける
        } else {
          road_th_ct_ct = 0; //30秒以上ゼロに戻らなければ、road_info_txt_flag = falseにして、道路名は出さない。
        }
        before_road_th_ct = road_th_ct;
      }
      if(i == 1){
        kmh = token;
      }
      i++; // インデックスを1つ進める
    }
    int road_th_ct_ct_limit = 30; //30秒無通信チェック。
    if(velo_for_trans < 0.1){
      road_th_ct_ct_limit = 180; //停止時は3分まで伸ばす。
    }
    if(token.empty() == true || (token == "--" && kmh == "0") || road_th_ct_ct > road_th_ct_ct_limit * 20){
      road_info_txt_flag = false;
    } else {
      road_info_txt_flag = true;
      p.setFont(InterFont(33, QFont::Bold));
      int next_x = drawTextRight(p, rect().right()-10, rect().bottom() - 10 , QString::fromStdString(token), 220);
      if(kmh != "0"){
        drawTextRight(p, next_x-4, rect().bottom() - 10 , QString::fromStdString(kmh) , 255 , false , 0x24, 0x57, 0xa1 , 255,255,255,200 , 6 , 5 , 2 , 0);
      }
    }
  }
  if(road_info_txt_flag == false){
    p.setFont(InterFont(33, QFont::DemiBold));
    drawTextRight(p, rect().right()-10, rect().bottom() - 10 , "Rrograman Ichiro modified", 150 /*, 255 , false , 0x24, 0x57, 0xa1 , 255,255,255,200 , 6*/);
  }
  p.setFont(InterFont(33, QFont::Bold));
  float angle_steer = 0;
  std::string angle_steer0_txt = util::read_file("/tmp/steer_ang_info.txt");
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
  //global_engageable = engageable;
  if (global_engageable && !(status == STATUS_ENGAGED || status == STATUS_OVERRIDE)) {
    a0 = 50; a1 = 50; a2 = 50; a3 = 50;
  } else if (global_engageable && (status == STATUS_ENGAGED || status == STATUS_OVERRIDE)) {
    a0 = 50; a1 = 50; a2 = 50; a3 = 50;
    if(vc_speed < 1/3.6){
      a3 = 200;
    }
    angle_steer = global_angle_steer0;
    if(1 /*vc_speed >= 1/3.6 && (angle_steer > 1.5 || angle_steer < -1.5)*/){
      if(/*Limit_speed_mode == 1 &&*/ limit_speed_auto_detect == 1){ //インジケーターはACC自動設定時にするか、速度標識表示時にするか検討中
        a2 = 200;
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
    std::string limit_vc_txt = util::read_file("/tmp/limit_vc_info.txt");
    if(limit_vc_txt.empty() == false && vc_speed >= 1/3.6){
      curve_value = std::stof(limit_vc_txt);
    }
  }

  drawText(p, rect().center().x(), 50 + 40*0 , "extra cruise speed engagement", a0 , brake_light);
  drawText(p, rect().center().x(), 50 + 40*1 , "slow down corner correctly", a1 , brake_light);
  drawText(p, rect().center().x(), 50 + 40*2 , "speed limit auto detect", a2 , brake_light);
  //drawText(p, rect().center().x(), 50 + 40*2 , "curvature reinforcement", a2 , brake_light);
  //drawText(p, rect().center().x(), 50 + 40*2 , QString::number(angle_steer), a2 , brake_light);
  drawText(p, rect().center().x(), 50 + 40*3 , "auto brake holding", a3 , brake_light);

  // engage-ability icon
  if (true) {
    SubMaster &sm = *(uiState()->sm);
    QBrush bg_color = bg_colors[status];
    if(uiState()->scene.mAccelEngagedButton >= 3 && fabs(global_angle_steer0) >= 50 && (*(uiState()->sm))["carState"].getCarState().getVEgo() <= 0.01/3.6){
      //停止時の青信号発進抑制、一時的に緩和、15->50度
      bg_color = COLOR_STATUS_WARNING; //ワンペダル時に信号スタート可能角度でなければ警告色。
    }
    my_drawIcon(p, rect().right() - btn_size / 2 - UI_BORDER_SIZE * 2, btn_size / 2 + int(UI_BORDER_SIZE * 1.5)+y_ofs,
             //engage_img, bg_color, 1.0 , -global_angle_steer0);
             sm["controlsState"].getControlsState().getExperimentalMode() ? experimental_img : engage_img, blackColor(166), global_engageable ? 1.0 : 0.3 , -global_angle_steer0);
  }
  const float x_Long_enable = rect().right() - btn_size / 2 - UI_BORDER_SIZE * 2;
  const float y_Long_enable = btn_size / 2 + int(UI_BORDER_SIZE * 1.5)+y_ofs;
  std::string long_speeddown_disable_txt = util::read_file("/tmp/long_speeddown_disable.txt");
  Long_enable = true;
  if(long_speeddown_disable_txt.empty() == false){
    if(std::stoi(long_speeddown_disable_txt) != 0){
      Long_enable = false;
    }
  }
  int long_base_angle0 = 45; //下中央から左右に何度か指定する。
  if((Long_enable || (*s->sm)["controlsState"].getControlsState().getExperimentalMode()) && global_engageable){
    const int arc_w = -8; //内側に描画
    QPen pen = QPen(QColor(255, 255, ((*s->sm)["controlsState"].getControlsState().getExperimentalMode()) ? 0 : 255, 180), abs(arc_w));
    pen.setCapStyle(Qt::FlatCap); //端をフラットに
    p.setPen(pen);
    const float x = x_Long_enable;
    const float y = y_Long_enable;
    static float desired_path_x_rate = 0 , desired_path_x_rate0 = 0;
    //static unsigned int desired_path_x_rate_ct = 0;
    if(true /*|| desired_path_x_rate_ct ++ % 2 == 0*/){
      std::string desired_path_x_rate_txt = util::read_file("/tmp/desired_path_x_rate.txt");
      if(desired_path_x_rate_txt.empty() == false){
        desired_path_x_rate0 = std::stof(desired_path_x_rate_txt);
        if(desired_path_x_rate0 > 1.0f){
          desired_path_x_rate0 = 1.0f;
        }
      }
    }
    float max_diff = fabs(desired_path_x_rate0-desired_path_x_rate) / 10; // = 0.1; //これ以上一気にメーターが疎かない。可変式
    if(max_diff < 0.02)max_diff = 0.02; else if(max_diff > 0.05)max_diff = 0.05;
    if(desired_path_x_rate0 > desired_path_x_rate){
      if(desired_path_x_rate0 > desired_path_x_rate + max_diff){
        desired_path_x_rate += max_diff;
      } else {
        desired_path_x_rate = desired_path_x_rate0;
      }
    } else if(desired_path_x_rate0 < desired_path_x_rate){
      if(desired_path_x_rate0 < desired_path_x_rate - max_diff){
        desired_path_x_rate -= max_diff;
      } else {
        desired_path_x_rate = desired_path_x_rate0;
      }
    }
    int long_base_angle = long_base_angle0; //下中央から左右に何度か指定する。
    p.drawArc(x - btn_size / 2 -arc_w/2, y - btn_size / 2 -arc_w/2, btn_size+arc_w, btn_size+arc_w, (-90-long_base_angle)*16, -(360-long_base_angle*2)*16*desired_path_x_rate);
  }
  if(Long_enable){ //エンゲージしてなくても表示する。完全になくなると操作の目標を失うため。(OFFで消えたら仕方がないが) , Experimental Modeでは表示しない。
    const float x = x_Long_enable;
    const float y = y_Long_enable;
    const int long_base_angle = long_base_angle0-3; //下中央から左右に何度か指定する。
    //ONOFFの状態をこれで視認できる。
    const int arc_w_base = -14; //内側に描画
    QPen pen = QPen(QColor(255, 255, ((*s->sm)["controlsState"].getControlsState().getExperimentalMode()) ? 0 : 255, 180), abs(arc_w_base));
    pen.setCapStyle(Qt::FlatCap); //端をフラットに
    p.setPen(pen);
    p.drawArc(x - btn_size / 2 -arc_w_base/2, y - btn_size / 2 -arc_w_base/2, btn_size+arc_w_base, btn_size+arc_w_base, (-90-(long_base_angle))*16, ((long_base_angle)*2)*16);
  }

  //カメラ内に速度標識もどきを表示。
  static unsigned int limitspeed_info_read_ct;
  static int limit_speed_num = 0;
  if (mapVisible == false){
    if(limitspeed_info_read_ct++ % 10 == 5){
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
        if((int)output[2] == 111){
          limit_speed_num = 0;
          limit_speed_auto_detect = 0;
        } else {
          limit_speed_num = (int)output[0];
          limit_speed_auto_detect = 1;
        }
      }
    }

    QString traffic_speed;
    if(limit_speed_num == 0){
      traffic_speed = "━";
    } else {
      traffic_speed = QString::number(limit_speed_num);
    }

    const float traffic_speed_r = 120 / 2 , traffic_speed_x = 247 , traffic_speed_y = rect().height() - traffic_speed_r*2 - 50;
    p.setPen(Qt::NoPen);
    if(g_night_mode == 0){
      p.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, 0.85));
    } else {
      p.setBrush(QColor::fromRgbF(0.8, 0.8, 0.9, 0.85)); //標識バックを薄暗く。
    }
    p.drawEllipse(traffic_speed_x,traffic_speed_y,traffic_speed_r*2,traffic_speed_r*2);

    int arc_w = -22; //内側に描画
    if(limit_speed_num >= 100){
      arc_w = -15; //枠と数字が被らないように枠を細くする。
    }
    arc_w = arc_w * traffic_speed_r / (150 / 2);
    QPen pen = QPen(QColor(205, 44, 38, 255), abs(arc_w));
    pen.setCapStyle(Qt::FlatCap); //端をフラットに
    p.setPen(pen);

    p.drawArc(traffic_speed_x-arc_w/2+4, traffic_speed_y-arc_w/2+4, traffic_speed_r*2+arc_w-8,traffic_speed_r*2+arc_w-8, (90-car_bearing+5)*16, (360-5*2)*16);
    int f_size = traffic_speed_r * 67 / (150 / 2);
    p.setFont(InterFont(f_size, QFont::Bold));
    drawTextCircleCenter(p, traffic_speed_x+traffic_speed_r, traffic_speed_y+traffic_speed_r+f_size/2 -7, traffic_speed , QColor(0x24, 0x57, 0xa1 , 255));
  }

  //キャリブレーション値の表示。dm iconより先にやらないと透明度が連動してしまう。
  p.setPen(QPen(QColor(0xff, 0xff, 0xff, 0), 0));
  //int calib_h = radius;
  int calib_h = -33 -33 - 30; //表示位置を上に
  QRect rc2(rect().right() - btn_size / 2 - UI_BORDER_SIZE * 2 - 100, -20 + btn_size / 2 + int(UI_BORDER_SIZE * 1.5)+y_ofs + calib_h -36, 200, 36);
  if(/*engageable ||*/ handle_center == -100){
    std::string handle_center_txt = util::read_file("/tmp/handle_center_info.txt");
    if(handle_center_txt.empty() == false){
        handle_center = std::stof(handle_center_txt);
    }
  }

  if(fabs(global_angle_steer0) > 5 && handle_center > -99){
    //ハンドル角度を表示
    p.setBrush(bg_colors[status]);
    QRect rc3(rc2.x()+20,rc2.y()-30,rc2.width()-40,rc2.height()+30);
    p.drawRoundedRect(rc3, 30, 30);
    p.setPen(Qt::NoPen);

    char h_ang[16];
    int h_ang_i = (int)global_angle_steer0;
    if(h_ang_i > 99)h_ang_i=99; else if(h_ang_i < -99)h_ang_i=-99;
    sprintf(h_ang,"%+d°",h_ang_i); //99カンスト

    p.setFont(InterFont(60, QFont::Bold));
    drawText(p, rc3.x()+rc3.width()/2 , rc3.y() + rc3.height() - 12, h_ang , 200);
  } else if(/*engageable ||*/ handle_center > -99){
    //ハンドルセンター値を表示
    p.setBrush(bg_colors[status]);
    p.drawRoundedRect(rc2, 18, 18);
    p.setPen(Qt::NoPen);

    //float hc = -4.73;
    float hc = handle_center;

    p.setFont(InterFont(33, QFont::Bold));
    drawText(p, rect().right() - btn_size / 2 - UI_BORDER_SIZE * 2 , -20 + btn_size / 2 + int(UI_BORDER_SIZE * 1.5)+y_ofs + calib_h - 8, QString::number(hc,'f',2) + "deg", 200);
  } else {
    p.setBrush(QColor(150, 150, 0, 0xf1));
    p.drawRoundedRect(rc2, 18, 18);
    p.setPen(Qt::NoPen);

    if(handle_calibct == 0){
      p.setFont(InterFont(33));
      drawText(p, rect().right() - btn_size / 2 - UI_BORDER_SIZE * 2 , -20 + btn_size / 2 + int(UI_BORDER_SIZE * 1.5)+y_ofs + calib_h - 8, "Calibrating", 200);
    } else {
      p.setFont(InterFont(33, QFont::Bold));
      drawText(p, rect().right() - btn_size / 2 - UI_BORDER_SIZE * 2 , -20 + btn_size / 2 + int(UI_BORDER_SIZE * 1.5)+y_ofs + calib_h - 6, QString::number(handle_calibct) + '%', 200);
    }
  }

  p.restore();
}

int AnnotatedCameraWidget::drawText(QPainter &p, int x, int y, const QString &text, int alpha , bool brakeLight) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  if(brakeLight == false){
    p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  } else {
    alpha += 100;
    if(alpha > 255){
      alpha = 255;
    }
    p.setPen(QColor(0xff, 0, 0, alpha));
  }
  p.drawText(real_rect.x(), real_rect.bottom(), text);

  return real_rect.width(); //続けて利用できるように幅を返す。（次の表示を左右の隣に出すために使える）
}

int AnnotatedCameraWidget::drawTextCircleCenter(QPainter &p, int x, int y, const QString &text, const QColor &col) { //円の中央とxを共有したいときに使いやすいので残す。
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(col);
  p.drawText(real_rect.x(), real_rect.bottom(), text);

  return real_rect.width(); //続けて利用できるように幅を返す。（次の表示を左右の隣に出すために使える）
}

int AnnotatedCameraWidget::drawText(QPainter &p, int x, int y, const QString &text, const QColor &col) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(col);
  p.drawText(real_rect.x(), real_rect.bottom(), text);

  return real_rect.width(); //続けて利用できるように幅を返す。（次の表示を左右の隣に出すために使える）
}

int AnnotatedCameraWidget::drawTextLeft(QPainter &p, int x, int y, const QString &text, int alpha , bool brakeLight , int red, int blu, int grn , int bk_red, int bk_blu, int bk_grn, int bk_alp, int bk_yofs, int bk_corner_r , int bk_add_w, int bk_xofs, int bk_add_h) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x + real_rect.width() / 2, y - real_rect.height() / 2});

  if(bk_alp > 0){
    //バックを塗る。
    p.setBrush(QColor(bk_red, bk_blu, bk_grn, bk_alp));
    if(bk_corner_r == 0){
      p.drawRect(real_rect.x()+bk_xofs,real_rect.y() + bk_yofs , real_rect.width()+bk_add_w , real_rect.height() + bk_add_h);
    } else {
      QRect rc(real_rect.x()+bk_xofs,real_rect.y() + bk_yofs , real_rect.width()+bk_add_w , real_rect.height() + bk_add_h);
      p.drawRoundedRect(rc, bk_corner_r, bk_corner_r);
    }
  }

  if(brakeLight == false){
    p.setPen(QColor(red, blu, grn, alpha));
  } else {
    alpha += 100;
    if(alpha > 255){
      alpha = 255;
    }
    p.setPen(QColor(0xff, 0, 0, alpha));
  }
  p.drawText(real_rect.x(), real_rect.bottom(), text);

  return x + real_rect.width(); //続けて並べるxposを返す。
}

int AnnotatedCameraWidget::drawTextRight(QPainter &p, int x, int y, const QString &text, int alpha , bool brakeLight , int red, int blu, int grn , int bk_red, int bk_blu, int bk_grn, int bk_alp, int bk_yofs, int bk_corner_r , int bk_add_w, int bk_xofs) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x - real_rect.width() / 2, y - real_rect.height() / 2});

  if(bk_alp > 0){
    //バックを塗る。
    p.setBrush(QColor(bk_red, bk_blu, bk_grn, bk_alp));
    if(bk_corner_r == 0){
      p.drawRect(real_rect.x()+bk_xofs,real_rect.y() + bk_yofs , real_rect.width()+bk_add_w , real_rect.height());
    } else {
      QRect rc(real_rect.x()+bk_xofs,real_rect.y() + bk_yofs , real_rect.width()+bk_add_w , real_rect.height());
      p.drawRoundedRect(rc, bk_corner_r, bk_corner_r);
    }
  }

  if(brakeLight == false){
    p.setPen(QColor(red, blu, grn, alpha));
  } else {
    alpha += 100;
    if(alpha > 255){
      alpha = 255;
    }
    p.setPen(QColor(0xff, 0, 0, alpha));
  }
  p.drawText(real_rect.x(), real_rect.bottom(), text);

  return x - real_rect.width(); //続けて並べるxposを返す。
}

int AnnotatedCameraWidget::drawTextCenter(QPainter &p, int x, int y, const QString &text, int alpha , bool brakeLight , int red, int blu, int grn , int bk_red, int bk_blu, int bk_grn, int bk_alp, int bk_yofs, int bk_corner_r , int bk_add_w, int bk_xofs) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  if(bk_alp > 0){
    //バックを塗る。
    p.setBrush(QColor(bk_red, bk_blu, bk_grn, bk_alp));
    if(bk_corner_r == 0){
      p.drawRect(real_rect.x()-bk_add_w/2+bk_xofs,real_rect.y() + bk_yofs , real_rect.width()+bk_add_w , real_rect.height());
    } else {
      QRect rc(real_rect.x()-bk_add_w/2+bk_xofs,real_rect.y() + bk_yofs , real_rect.width()+bk_add_w , real_rect.height());
      p.drawRoundedRect(rc, bk_corner_r, bk_corner_r);
    }
  }

  if(brakeLight == false){
    p.setPen(QColor(red, blu, grn, alpha));
  } else {
    alpha += 100;
    if(alpha > 255){
      alpha = 255;
    }
    p.setPen(QColor(0xff, 0, 0, alpha));
  }
  p.drawText(real_rect.x(), real_rect.bottom(), text);

  return real_rect.width(); //続けて利用できるように幅を返す。（次の表示を左右の隣に出すために使える）
}

void AnnotatedCameraWidget::my_drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity , float ang) {
  p.setOpacity(1.0);  // bg dictates opacity of ellipse
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(x - btn_size / 2, y - btn_size / 2, btn_size, btn_size);
  p.setOpacity(opacity);
  p.resetTransform();
  p.translate(x - img.size().width() / 2,y - img.size().height() / 2);
  if(ang != 0){
    p.translate(img.size().width()/2,img.size().height()/2);
    p.rotate(ang); //degree指定
    p.translate(-img.size().width()/2,-img.size().height()/2);
  }
  QRect r(0,0,img.size().width(),img.size().height());
  p.drawPixmap(r, img);
  p.resetTransform();
  p.setOpacity(1.0);
}

void AnnotatedCameraWidget::initializeGL() {
  CameraWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);
}

void AnnotatedCameraWidget::updateFrameMat() {
  CameraWidget::updateFrameMat();
  UIState *s = uiState();
  int w = width(), h = height();

  s->fb_w = w;
  s->fb_h = h;

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(w / 2 - x_offset, h / 2 - y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}

void AnnotatedCameraWidget::drawLaneLines(QPainter &painter, const UIState *s) {
  painter.save();

  const UIScene &scene = s->scene;
  SubMaster &sm = *(s->sm);

  // lanelines
  const bool chill_mode = false; //!sm["controlsState"].getControlsState().getExperimentalMode();
  const float v_ego_car = sm["carState"].getCarState().getVEgo();
  const bool lta_mode = (v_ego_car > 16/3.6 || chill_mode) && scene.mLTA_EnableButton;
  int lane_collision = -1;
  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
#if 0 //レーン依存率をカラーで表す。
    if(lta_mode == true){
      if(i == 1/*左レーン*/ || i == 2/*右レーン*/){
        float lane_prob = scene.lane_line_probs[i] - 0.25;
        if(lane_prob < 0){
          lane_prob = 0;
        }
        painter.setBrush(QColor::fromRgbF(1.0, 0.5 + 0.5 * (1.0-lane_prob), 1.0 * (1.0-lane_prob), std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
      } else {
        painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
      }
    } else
#elif 1
    if(lta_mode == true){
      if(lane_collision == -1){
          std::string lane_collision_txt = util::read_file("/tmp/lane_collision.txt");
          if(lane_collision_txt.empty() == false){
            lane_collision = std::stoi(lane_collision_txt);
          } else {
            lane_collision = 0x80; //lane_collision.txtが無い。
          }
      }
      if((i == 1 && (lane_collision & 0x01))/*左レーン*/ || (i == 2 && (lane_collision & 0x02))/*右レーン*/){
        float lane_prob = scene.lane_line_probs[i];
        if(lane_prob > 0.5){
          lane_prob = 1.0;
        } else {
          lane_prob *= 2; //50％以下でも多少の影響を視覚化。
        }
        if(lane_collision & 0x04){
          //ALDP無視状態
          painter.setBrush(QColor::fromRgbF(0.5, 0.5, 0.5, lane_prob));
        } else {
          painter.setBrush(QColor::fromRgbF(1.0, 0.5, 0, lane_prob));
        }
      } else {
        painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
      }
    } else
#endif //下は意図的に{}無しでインデントを一つ落としている。 elseが有効なので注意。
    painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    painter.drawPolygon(scene.lane_line_vertices[i]);
  }

  // road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 0, 0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0)));
    painter.drawPolygon(scene.road_edge_vertices[i]);
  }

  // paint path
  QLinearGradient bg(0, height(), 0, 0);
  if (sm["controlsState"].getControlsState().getExperimentalMode()) {
    // The first half of track_vertices are the points for the right side of the path
    const auto &acceleration = sm["modelV2"].getModelV2().getAcceleration().getX();
    const int max_len = std::min<int>(scene.track_vertices.length() / 2, acceleration.size());

    for (int i = 0; i < max_len; ++i) {
      // Some points are out of frame
      if (scene.track_vertices[i].y() < 0 || scene.track_vertices[i].y() > height()) continue;

      // Flip so 0 is bottom of frame
      float lin_grad_point = (height() - scene.track_vertices[i].y()) / height();

      // speed up: 120, slow down: 0
      float path_hue = fmax(fmin(60 + acceleration[i] * 35, 120), 0);
      // FIXME: painter.drawPolygon can be slow if hue is not rounded
      path_hue = int(path_hue * 100 + 0.5) / 100;

      float saturation = fmin(fabs(acceleration[i] * 1.5), 1);
      float lightness = util::map_val(saturation, 0.0f, 1.0f, 0.95f, 0.62f);  // lighter when grey
      float alpha = util::map_val(lin_grad_point, 0.75f / 2.f, 0.75f, 0.4f, 0.0f);  // matches previous alpha fade
      bg.setColorAt(lin_grad_point, QColor::fromHslF(path_hue / 360., saturation, lightness, alpha));

      // Skip a point, unless next is last
      i += (i + 2) < max_len ? 1 : 0;
    }

  } else {
    bg.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 0.4));
    bg.setColorAt(0.5, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.0));
  }

  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices);

  knightScanner(painter);
  painter.restore();
}

static void set_face_gesture_arc(QPainter &painter,float x, float y, int start_ang , int ang_width ,const QColor &col , int ww=20){
  //顔が向いてる方を上下左右で表示する。
  QPen pen = QPen(col, ww);
  pen.setCapStyle(Qt::FlatCap); //端をフラットに
  painter.setPen(pen);

  painter.drawArc(QRectF(x - btn_size / 2 +ww/2, y - btn_size / 2 +ww/2 , btn_size-ww, btn_size-ww) , start_ang * 16, ang_width * 16);
}

void AnnotatedCameraWidget::drawDriverState(QPainter &painter, const UIState *s) {
  const UIScene &scene = s->scene;

  painter.save();

  // base icon
  int offset = UI_BORDER_SIZE + (30-UI_BORDER_SIZE) + btn_size / 2;
  int x = false /*rightHandDM*/ ? width() - offset : offset;
  int y = height() - offset;
  float opacity = dmActive ? 0.65 : 0.2; y -= 18 + (30-UI_BORDER_SIZE)*2;
  drawIcon(painter, QPoint(x, y), dm_img, blackColor(70), opacity); //公式drawIconを使う。
  if(rightHandDM){ //ボタンを移動できないので、アイコンはそのまま、左肩に"R"を表示。
    painter.setFont(InterFont(70, QFont::Bold));
    drawText(painter, x - btn_size / 2, y - btn_size / 4, "R" , dmActive ? 200 : 100);
  }

  // face
  QPointF face_kpts_draw[std::size(default_face_kpts_3d)];
  float kp;
  for (int i = 0; i < std::size(default_face_kpts_3d); ++i) {
    kp = (scene.face_kpts_draw[i].v[2] - 8) / 120 + 1.0;
    face_kpts_draw[i] = QPointF(scene.face_kpts_draw[i].v[0] * kp + x, scene.face_kpts_draw[i].v[1] * kp + y);
  }

  painter.setPen(QPen(QColor::fromRgbF(1.0, 1.0, 1.0, opacity), 5.2, Qt::SolidLine, Qt::RoundCap));
  painter.drawPolyline(face_kpts_draw, std::size(default_face_kpts_3d));

  // tracking arcs
  const int arc_l = 133;
  const float arc_t_default = 6.7;
  const float arc_t_extend = 12.0;
  QColor arc_color = QColor::fromRgbF(0.545 - 0.445 * s->engaged(),
                                      0.545 + 0.4 * s->engaged(),
                                      0.545 - 0.285 * s->engaged(),
                                      0.4 * (1.0 - dm_fade_state));
  float delta_x = -scene.driver_pose_sins[1] * arc_l / 2;
  float delta_y = -scene.driver_pose_sins[0] * arc_l / 2;
  painter.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[1] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(std::fmin(x + delta_x, x), y - arc_l / 2, fabs(delta_x), arc_l), (scene.driver_pose_sins[1]>0 ? 90 : -90) * 16, 180 * 16);
  painter.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[0] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(x - arc_l / 2, std::fmin(y + delta_y, y), arc_l, fabs(delta_y)), (scene.driver_pose_sins[0]>0 ? 0 : 180) * 16, 180 * 16);

  //顔が向いてる方を上下左右かしげで表示する。
  float delta_r = scene.driver_pose_sins[2]; //首のかしげ角度のsin
  static unsigned int key_n_ct = 0; //キーが入った順番
  static unsigned int left_face_key_n;
  static unsigned int right_face_key_n;
  static unsigned int up_face_key_n;
  static unsigned int down_face_key_n;
  static unsigned int lr_face_key_n;
  static unsigned int rr_face_key_n;

  const int long_press = 20;
  const float thr_face = 0.9;
  float left_face_x;
  float right_face_x;
  float r_face_r;
  float l_face_r;
  if(rightHandDM == false){
    //左ハンドル？未検証
    left_face_x = -17*thr_face;
    right_face_x = 14*thr_face;
    // r_face_r = -0.19*thr_face; //同じならいらない
    // l_face_r = 0.19*thr_face;
  } else {
    left_face_x = -14*thr_face;
    right_face_x = 18*thr_face;
    r_face_r = -0.19*thr_face;
    l_face_r = 0.19*thr_face;
  }
  float up_face_y = -25*thr_face;
  float down_face_y = 22*thr_face;

  static int face_center_timer = 0;
  const int face_center_timer0 = 10;
  static int face_right_timer = 0;
  const int face_right_timer0 = 10;
  static int face_left_timer = 0;
  const int face_left_timer0 = 10;
  static int face_up_timer = 0;
  const int face_up_timer0 = 10;
  static int face_down_timer = 0;
  const int face_down_timer0 = 10;

  //中央視線検出
  static int face_center_ct = 0;
  const float center_eye_rate = 0.3;
  bool center_detect = (dmActive && delta_x > left_face_x*center_eye_rate && delta_x < right_face_x*center_eye_rate
    && delta_y > up_face_y*center_eye_rate && delta_y < down_face_y*center_eye_rate
    && delta_r > r_face_r*center_eye_rate && delta_r < l_face_r*center_eye_rate);
  if(center_detect || face_center_timer > 0){
    if(center_detect && face_center_ct >= 0){
      face_center_timer = face_center_timer0;
    } else if(face_center_timer > 0){
      face_center_timer --;
    }
    set_face_gesture_arc(painter,x,y , 0 , 360 ,QColor(200,face_center_ct < long_press ? 200 : 100,0,250) , 10);
    if(face_center_ct < 30)
      face_center_ct ++;
  } else {
    if(face_center_ct > 0)
      face_center_ct --;
  }

  bool all_centering = true;
  //右向き検出
  static int face_right_ct = 0;
  if(delta_x > right_face_x || face_right_timer > 0){
    if(delta_x > right_face_x && face_right_ct >= 0){
      face_right_timer = face_right_timer0;
      right_face_key_n = key_n_ct ++;
    } else if(face_right_timer > 0){
      face_right_timer --;
    }
    set_face_gesture_arc(painter,x,y , -45 , 90 ,QColor(200,face_right_ct < long_press ? 200 : 100,0,250));
    if(face_right_ct < 30)
      face_right_ct ++;
    all_centering = false;
  } else {
    //face_right_ct = 0;
  }
  //左向き検出
  static int face_left_ct = 0;
  if(delta_x < left_face_x || face_left_timer > 0){
    if(delta_x < left_face_x && face_left_ct >= 0){
      face_left_timer = face_left_timer0;
      left_face_key_n = key_n_ct ++;
    } else if(face_left_timer > 0){
      face_left_timer --;
    }
    set_face_gesture_arc(painter,x,y , 135 , 90 ,QColor(200,face_left_ct < long_press ? 200 : 100,0,250));
    if(face_left_ct < 30)
      face_left_ct ++;
    all_centering = false;
  } else {
    //face_left_ct = 0;
  }
  //上向き検出
  static int face_up_ct = 0;
  if(delta_y < up_face_y || face_up_timer > 0){
    if(delta_y < up_face_y && face_up_ct >= 0){
      face_up_timer = face_up_timer0;
      up_face_key_n = key_n_ct ++;
    } else if(face_up_timer > 0){
      face_up_timer --;
    }
    set_face_gesture_arc(painter,x,y , 45 , 90, QColor(200,face_up_ct < long_press ? 200 : 100,0,250));
    if(face_up_ct < 30)
      face_up_ct ++;
    all_centering = false;
  } else {
    //face_up_ct = 0;
  }
  //下向き検出
  static int face_down_ct = 0;
  if(delta_y > down_face_y || face_down_timer > 0){
    if(delta_y > down_face_y && face_down_ct >= 0){
      face_down_timer = face_down_timer0;
      down_face_key_n = key_n_ct ++;
    } else if(face_down_timer > 0){
      face_down_timer --;
    }
    set_face_gesture_arc(painter,x,y , -45 , -90, QColor(200,face_down_ct < long_press ? 200 : 100,0,250));
    if(face_down_ct < 30)
      face_down_ct ++;
    all_centering = false;
  } else {
    //face_down_ct = 0;
  }

  //首を傾けるジェスチャー
  static int face_rr_timer = 0; //右に傾げる
  const int face_rr_timer0 = 10;
  static int face_rr_ct = 0;
  if(delta_r < r_face_r || face_rr_timer > 0){
    if(delta_r < r_face_r && face_rr_ct >= 0){
      face_rr_timer = face_rr_timer0;
      rr_face_key_n = key_n_ct ++;
    } else if(face_rr_timer > 0){
      face_rr_timer --;
    }
    set_face_gesture_arc(painter,x,y , -90-20 , 180, QColor(200,face_rr_ct < long_press ? 200 : 100,0,250));
    if(face_rr_ct < 30)
      face_rr_ct ++;
    all_centering = false;
  } else {
    //face_rr_ct = 0;
  }
  static int face_lr_timer = 0;
  const int face_lr_timer0 = 10;
  static int face_lr_ct = 0;
  if(delta_r > l_face_r || face_lr_timer > 0){
    if(delta_r > l_face_r && face_lr_ct >= 0){
      face_lr_timer = face_lr_timer0;
      lr_face_key_n = key_n_ct ++;
    } else if(face_lr_timer > 0){
      face_lr_timer --;
    }
    set_face_gesture_arc(painter,x,y , 90+20 , 180, QColor(200,face_lr_ct < long_press ? 200 : 100,0,250));
    if(face_lr_ct < 30)
      face_lr_ct ++;
    all_centering = false;
  } else {
    //face_lr_ct = 0;
  }

  if(all_centering == true){
    if(face_right_ct > 0)
      face_right_ct --;
    if(face_left_ct > 0)
      face_left_ct --;
    if(face_up_ct > 0)
      face_up_ct --;
    if(face_down_ct > 0)
      face_down_ct --;
    if(face_rr_ct > 0)
      face_rr_ct --;
    if(face_lr_ct > 0)
      face_lr_ct --;

    if(face_right_ct == 0
      && face_left_ct == 0
      && face_up_ct == 0
      && face_down_ct == 0
      && face_rr_ct == 0
      && face_lr_ct == 0){
      key_n_ct = 0;
      left_face_key_n = 0;
      right_face_key_n = 0;
      up_face_key_n = 0;
      down_face_key_n = 0;
      lr_face_key_n = 0;
      rr_face_key_n = 0;
    }
  }
#if 0 //顔一回転は流石に無理があるか。
  if(face_up_ct > 1 && face_down_ct > 1 && face_left_ct > 1 && face_right_ct > 1){
    face_left_ct = 0;
    face_right_ct = 0;
    face_up_ct = 0;
    face_down_ct = 0;
    left_face_key_n = 0;
    right_face_key_n = 0;
    up_face_key_n = 0;
    down_face_key_n = 0;
    void soundPikiri();
    soundPikiri();
  }
#endif
  if(face_down_ct > 0 && face_up_ct > long_press && down_face_key_n < up_face_key_n  //↓↑ジェスチャー
      && face_right_ct == 0 && face_left_ct == 0 //検知以外の向き防止
    ){
    //ACC速度制御モード変更
    face_up_ct = 0; //多キーコマンドは-20にしなくても連続動作しない。
    face_up_timer = 0;
    face_down_ct = 0; //多キーコマンドは-20にしなくても連続動作しない。
    face_down_timer = 0;
    up_face_key_n = 0;
    down_face_key_n = 0;
    if(Limit_speed_mode == 0){
      Limit_speed_mode = 1;
    } else {
      Limit_speed_mode = 0; //2にはならない。
    }
  }

  if(mapVisible && face_left_ct > 1 && face_up_ct > long_press && left_face_key_n < up_face_key_n //←↑ジェスチャー
      && face_down_ct == 0 && face_right_ct == 0 //検知以外の向き防止
    ){
    //地図ピッチアップ
    face_up_ct = 0; //多キーコマンドは-20にしなくても連続動作しない。
    face_up_timer = 0;
    face_left_ct = 0; //多キーコマンドは-20にしなくても連続動作しない。
    face_left_timer = 0;
    up_face_key_n = 0;
    left_face_key_n = 0;
    // extern bool map_pitch_up;
    // map_pitch_up = true;
    extern bool map_pitch_down;
    map_pitch_down = true;
  }

  if(false && mapVisible && face_left_ct > 1 && face_down_ct > long_press && left_face_key_n < down_face_key_n //←↓ジェスチャー
      && face_up_ct == 0 && face_right_ct == 0 //検知以外の向き防止
    ){
    //地図ピッチダウン
    face_down_ct = 0; //多キーコマンドは-20にしなくても連続動作しない。
    face_down_timer = 0;
    face_left_ct = 0; //多キーコマンドは-20にしなくても連続動作しない。
    face_left_timer = 0;
    down_face_key_n = 0;
    left_face_key_n = 0;
    extern bool map_pitch_down;
    map_pitch_down = true;
  }

  if(face_rr_ct > long_press && face_left_ct < 3 && face_up_ct < 3){ //↘︎ジェスチャー
    face_rr_ct = -long_press; //連続動作しないように工夫。
    face_rr_timer = 0;
    rr_face_key_n = 0;
    {
      //カメラ↔︎地図の画面切り替え
      extern bool head_gesture_home; //サイドバーを消す
      extern bool head_gesture_onroad_home;
      head_gesture_home = true;
      head_gesture_onroad_home = true;
    }
  }

  if(face_lr_ct > long_press && face_right_ct < 3 && face_up_ct < 3){ //↙︎ジェスチャー
    face_lr_ct = -long_press; //連続動作しないように工夫。
    face_lr_timer = 0;
    lr_face_key_n = 0;
    {
      //ノースアップ↔︎ヘディングアップの画面切り替え
      extern bool head_gesture_home; //サイドバーを消す
      extern bool head_gesture_onroad_home_map_on;
      head_gesture_home = true;
      head_gesture_onroad_home_map_on = true; //地図を強制的に出す。
    }
  }

  painter.restore();
}

void AnnotatedCameraWidget::knightScanner(QPainter &p) {
  extern int Knight_scanner;

  static const int ct_n = 1;
  static float ct;

#if 1
  std::string signal_start_prompt_info_txt = util::read_file("/tmp/signal_start_prompt_info.txt");
  if(signal_start_prompt_info_txt.empty() == false){
    int pr = std::stoi(signal_start_prompt_info_txt);
    if(pr == 1){
      setButtonInt("/tmp/sound_py_request.txt" , 6); //prompt.wav
      setButtonEnabled0("/tmp/signal_start_prompt_info.txt" , false);
    } else if(pr == 2){ //自動発進とワンペダル->オートパイロットはこちら。
      setButtonInt("/tmp/sound_py_request.txt" , 1); //engage.wav
      setButtonEnabled0("/tmp/signal_start_prompt_info.txt" , false);
    } else if(pr == 3){ //デバッグ用。
      // static QSoundEffect effect;
      // static bool once = false;
      // if(once == false){
      //   once = true;
      //   effect.setSource(QUrl::fromLocalFile("../assets/sounds/po.wav"));
      //   //effect.setLoopCount(QSoundEffect::Infinite);
      //   effect.setLoopCount(0);
      //   effect.setVolume(1.0);
      // }
      // effect.play();
      setButtonInt("/tmp/sound_py_request.txt" , 101); //po.wav
      setButtonEnabled0("/tmp/signal_start_prompt_info.txt" , false);
    }
  }
#endif

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
#elif 0 //メッセージUIの表示手法変更で、下の隙間から見えるので、lane_change_height持ち上げはひとまず取りやめ。
    auto lp = (*s->sm)["lateralPlan"].getLateralPlan();
    if( lp.getLaneChangeState() == cereal::LateralPlan::LaneChangeState::PRE_LANE_CHANGE ||
        lp.getLaneChangeState() == cereal::LateralPlan::LaneChangeState::LANE_CHANGE_STARTING){ //レーンチェンジの表示で判定
      lane_change_height = 270;
    } else { //stand_stillでもウインカーを上げる。
      std::string stand_still_txt = util::read_file("/tmp/stand_still.txt");
      bool stand_still = false;
      if(stand_still_txt.empty() == false){
        stand_still = std::stoi(stand_still_txt) ? true : false;
      }
      if(stand_still){
        lane_change_height = 270;
      }
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
      std::string limit_vc_txt = util::read_file("/tmp/limit_vc_info.txt");
      if(limit_vc_txt.empty() == false){
        float cv = std::stof(limit_vc_txt);
        if(cv > 0){
          curve_value = cv;
        }
      }
    }
    std::string handle_center_txt = util::read_file("/tmp/handle_center_info.txt");
    if(handle_center_txt.empty() == false){
        handle_center = std::stof(handle_center_txt);
    } else {
      std::string handle_calibct_txt = util::read_file("/data/handle_calibct_info.txt");
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
        if(Knight_scanner == 0){
          continue;
        }
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
  extern float _1_vc_accel;
  vc_accel = vc_accel + (vc_accel0 - vc_accel) / 5;
  //vc_accel = -0.5;
  float hha = 0;
  if(vc_accel > 0){
    hha = 1 - 0.1 / vc_accel;
    p.setBrush(QColor(0.09*255, 0.945*255, 0.26*255, 200));
  }
  if(vc_accel < 0){
    hha = 1 + 0.1 / vc_accel;
    p.setBrush(QColor(245, 0, 0, 200));
  }
  if(hha < 0){
    hha = 0;
  }
  _1_vc_accel = hha;
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
    _1_vc_accel = -_1_vc_accel;
#if 0
    QRect ra = QRect(rect_w - wp , rect_h/2         , wp , hha/2);
    p.drawRect(ra);
#else //メーターを斜めに切る
    QPointF meter[] = {{rect_w - wp + wp/2 , (float)rect_h/2},{(float)rect_w , (float)rect_h/2}, {(float)rect_w , rect_h/2 + hha/2}, {rect_w - wp/2 - wp/2 * hha / rect_h, rect_h/2 + hha/2}};
    p.drawPolygon(meter, std::size(meter));
#endif
  }
#endif


#if 1 //曲率、k_v表示テスト
  static float curvature = 0;
  static float k_v = 1.0;
  std::string curvature_info = util::read_file("/tmp/curvature_info.txt");
  if(curvature_info.empty() == false && global_engageable && (status == STATUS_ENGAGED || status == STATUS_OVERRIDE)) {
    auto separator = std::string("/");         // 区切り文字
    //auto separator_length = separator.length(); // 区切り文字の長さ
    auto pos = curvature_info.find(separator, 0);
    if(pos != std::string::npos){
      std::string curvature_str = curvature_info.substr(0,pos);
      std::string k_v_str = curvature_info.substr(pos+1,curvature_info.length() - pos - 1);
      curvature = std::stod(curvature_str); //0〜0.02(30度) , 0.05(70度)
      curvature = fabs(curvature);
      k_v = std::stod(k_v_str); //倍率。1未満もあり得る
    }
  }
  if(global_engageable && (status == STATUS_ENGAGED || status == STATUS_OVERRIDE)){
    float h = rect_h * curvature / (/*tss_type*/2 < 2 ? 0.03 : 0.05);
    float wp1 = 25;
    //float wpa = 10;
    p.setBrush(QColor(245, 245, 0, 200));

    float h2 = h * k_v;
    if(h2 > h){
      //増加
      p.drawRect(QRect(0 , rect_h - h , wp1 , h));
      p.setBrush(QColor(0.09*255, 0.945*255, 0.26*255, 200));
      //p.drawRect(QRect(wp1 , rect_h - h2 , wpa , h2-h));
      p.drawRect(QRect(0 , rect_h - h2 , wp1 , h2-h));
    } else if(h2 <= h){ // == も含める
      //減衰
      p.drawRect(QRect(0 , rect_h - h2 , wp1 , h2));
      if(h2 < h){
        p.setBrush(QColor(245, 0, 0, 200));
        //p.drawRect(QRect(wp1 , rect_h - h , wpa , h-h2));
        p.drawRect(QRect(0 , rect_h - h , wp1 , h-h2));
      }
    }

    p.setCompositionMode(QPainter::CompositionMode_SourceOver);
    for(float yy=0.01; yy<0.05; yy+=0.01){
      float hhy = rect_h * yy / (/*tss_type*/2 < 2 ? 0.03 : 0.05);
      p.setBrush(QColor(245, 0, 0, 200));
      p.drawRect(QRect(0 , rect_h - hhy , wp1 , 5));
    }
  }
#endif

  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

#if 1 //減速度と舵角を表示
  static float cv = 0;
  //float ang = global_angle_steer0;
#if 1
  static unsigned int debug_ct;
  if(debug_ct % 10 == 0){
    std::string limit_vc_txt = util::read_file("/tmp/limit_vc_info.txt");
    if(limit_vc_txt.empty() == false){
      cv = std::stof(limit_vc_txt);
    }
  }
  debug_ct ++;
#endif
  p.setFont(InterFont(44, QFont::DemiBold));
  int debug_disp_xpos = 0+20;
  {
    p.setPen(Qt::NoPen);
    debug_disp_xpos = drawTextLeft(p , debug_disp_xpos , rect_h - 10 , QString("↓") , 200 , false , 0xdf, 0xdf, 0x00 , 0, 0, 0, 140 , 13 , 5 , 11 , 0 , -5) + 11;
    QString debug_disp = QString::number(cv,'f',0);
    debug_disp_xpos = drawTextLeft(p , debug_disp_xpos , rect_h - 10 , debug_disp , 200 , false , 0xdf, 0xdf, 0x00);
  }
  {
    //自立運転距離の割合
    static uint64_t manual_ct = 1 , autopilot_ct; //参考に時間での割合も計算する。
    static double manual_dist = 0.001 , autopilot_dist , before_distance_traveled;
    static double h_manual_dist = 0.001 , h_autopilot_dist; //停止時間は1秒を1m換算でカウントする。
    double now_dist = distance_traveled - before_distance_traveled;
    before_distance_traveled = distance_traveled;
    if(status == STATUS_DISENGAGED || status == STATUS_OVERRIDE){
      manual_dist += now_dist; //手動運転中
      h_manual_dist += now_dist; //手動運転中
      if ((all_brake_light && vc_speed < 0.1/3.6)){
        h_manual_dist += 1.0/20; //1秒を1m換算
      }
      if (status != STATUS_DISENGAGED || (all_brake_light && vc_speed < 0.1/3.6)){
        manual_ct ++; //手動運転中 , エンゲージしていれば停車時も含める。特例としてエンゲージしてなくてもブレーキ踏めば含める（人が運転しているから）
      }
    } else {
      autopilot_dist += now_dist; //オートパイロット中
      h_autopilot_dist += now_dist; //オートパイロット中
      if ((vc_speed < 0.1/3.6)){
        h_autopilot_dist += 1.0/20; //1秒を1m換算
      }
      autopilot_ct ++; //オートパイロット中（ハンドル、アクセル操作時は含めない , 停車時は自動運転停車として含める）
    }
    // double atr = ((double)autopilot_ct * 100) / (autopilot_ct + manual_ct); //autopilot time rate
    // double adr = (autopilot_dist * 100) / (autopilot_dist + manual_dist); //autopilot distance rate
    double ahr = (h_autopilot_dist * 100) / (h_autopilot_dist + h_manual_dist); //autopilot hybrid rate
    p.setPen(Qt::NoPen);
    debug_disp_xpos = drawTextLeft(p , debug_disp_xpos+4 , rect_h - 10 , QString("AP") , 200 , false , 0xdf, 0xdf, 0x00 , 0, 0, 0, 140 , 13 , 5 , 4 , -1 , -5) + 4;
    QString debug_disp = QString::number((int)ahr) + "%";
    debug_disp_xpos = drawTextLeft(p , debug_disp_xpos , rect_h - 10 , debug_disp , 200 , false , 0xdf, 0xdf, 0x00);
    //p.drawText(QRect(0+20 + 130, rect_h - 46, 210, 46), Qt::AlignBottom | Qt::AlignLeft, debug_disp);
    // FILE *fp = fopen("/tmp/autopilot_rate.txt","w");
    // if(fp != NULL){
    //   fprintf(fp,"H:%.0f+%.0f=%.0fh %.2f%%\n",h_autopilot_dist,h_manual_dist,h_autopilot_dist + h_manual_dist,ahr);
    //   fprintf(fp,"D:%.0f+%.0f=%.0fm %.2f%%\n",autopilot_dist,manual_dist,distance_traveled,adr);
    //   fprintf(fp,"T:%ld+%ld=%lds %.2f%%",autopilot_ct/20,manual_ct/20,(autopilot_ct+manual_ct)/20,atr);
    //   fclose(fp);
    // }
  }
  {
    p.setPen(Qt::NoPen);
    debug_disp_xpos = drawTextLeft(p , debug_disp_xpos+6 , rect_h - 10 , QString("Trip") , 200 , false , 0xdf, 0xdf, 0x00 , 0, 0, 0, 140 , 13 , 5 , 10 , -5 , -5) + 5;
    QString debug_disp = QString::number(distance_traveled / 1000,'f',1) + QString("km");
    debug_disp_xpos = drawTextLeft(p , debug_disp_xpos , rect_h - 10 , debug_disp , 200 , false , 0xdf, 0xdf, 0x00);
    //p.drawText(QRect(0+20 + 130 + 210, rect_h - 46, 290, 46), Qt::AlignBottom | Qt::AlignLeft, debug_disp);
  }
#if 0
  {
    extern int camera0_id,camera1_id,camera2_id; //搭載カメラの種類を表示。camera_qcom2.ccは別コマンドなのでリンクできない。
    QString debug_disp = QString(",");
    debug_disp += camera0_id == 8 ? QString("A") : (camera0_id == 9 ? QString("O") : QString("X")); //ワイドカメラ
    debug_disp += camera1_id == 8 ? QString("A") : (camera0_id == 9 ? QString("O") : QString("X")); //望遠カメラ
    debug_disp += camera2_id == 8 ? QString("A") : (camera0_id == 9 ? QString("O") : QString("X")); //ドライバーカメラ
    debug_disp_xpos = drawTextLeft(p , debug_disp_xpos , rect_h - 10 , debug_disp , 200 , false , 0xdf, 0xdf, 0x00);
    //p.drawText(QRect(0+20 + 130 + 210, rect_h - 46, 290, 46), Qt::AlignBottom | Qt::AlignLeft, debug_disp);
  }
#endif
#endif
}

static float global_a_rel;
static float global_a_rel_col;
void AnnotatedCameraWidget::drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd , int num /*使っていない, size_t leads_num*/) {
  painter.save();

  const float speedBuff = 10.;
  const float leadBuff = 40.;
  const float d_rel = lead_data.getDRel();
  const float v_rel = lead_data.getVRel(); //相対速度のようだ。
//  const float d_rel = lead_data.getX()[0];
//  const float v_rel = lead_data.getV()[0];
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
  if(mapVisible){
    sz *= 0.7; //地図表示時はchevronを小さく。
  }
  QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo + homebase_h},{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo},{x - (sz * 1.35) - g_xo, y + sz + g_yo + homebase_h}, {x, y + sz + homebase_h + g_yo + 10}};
  painter.setBrush(QColor(218, 202, 37, 210));
  painter.drawPolygon(glow, std::size(glow));

  // chevron
  //QPointF chevron[] = {{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz}};
  QPointF chevron[] = {{x + (sz * 1.25), y + sz + homebase_h},{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz},{x - (sz * 1.25), y + sz + homebase_h}, {x, y + sz + homebase_h - 7}};
  painter.setBrush(redColor(fillAlpha));
  painter.drawPolygon(chevron, std::size(chevron));

  if(num == 0){ //0番のリードカーまでの距離を表示
    //float dist = d_rel; //lead_data.getT()[0];
    QString dist = QString::number(d_rel,'f',0) + "m";
    int str_w = 200;
    QString kmph = QString::number((v_rel + vc_speed)*3.6,'f',0) + "k";
    int str_w2 = 200;
//    dist += "<" + QString::number(rect().height()) + ">"; str_w += 500;画面の高さはc2,c3共に1020だった。
//    dist += "<" + QString::number(leads_num) + ">";
//   int str_w = 600; //200;
//    dist += QString::number(v_rel,'f',1) + "v";
//    dist += QString::number(t_rel,'f',1) + "t";
//    dist += QString::number(y_rel,'f',1) + "y";
//    dist += QString::number(a_rel,'f',1) + "a";
    painter.setFont(InterFont(44, QFont::DemiBold));
    painter.setPen(QColor(0x0, 0x0, 0x0 , 200)); //影
    float lock_indicator_dx = 2; //下向きの十字照準を避ける。
    painter.drawText(QRect(x+2+lock_indicator_dx, y-50+2, str_w, 50), Qt::AlignBottom | Qt::AlignLeft, dist);
    painter.drawText(QRect(x+2-lock_indicator_dx-str_w2-2, y-50+2, str_w2, 50), Qt::AlignBottom | Qt::AlignRight, kmph);
    painter.setPen(QColor(0xff, 0xff, 0xff));
    painter.drawText(QRect(x+lock_indicator_dx, y-50, str_w, 50), Qt::AlignBottom | Qt::AlignLeft, dist);
    if(global_a_rel >= global_a_rel_col){
      global_a_rel_col = -0.1; //散らつきを抑えるバッファ。
      painter.setPen(QColor(0.09*255, 0.945*255, 0.26*255, 255));
    } else {
      global_a_rel_col = 0;
      painter.setPen(QColor(245, 0, 0, 255));
    }
    painter.drawText(QRect(x-lock_indicator_dx-str_w2-2, y-50, str_w2, 50), Qt::AlignBottom | Qt::AlignRight, kmph);
    painter.setPen(Qt::NoPen);
  }

  painter.restore();
}

struct LeadcarLockon {
  float x,y,d,a,lxt,lxf,lockOK;
};
#define LeadcarLockon_MAX 5
LeadcarLockon leadcar_lockon[LeadcarLockon_MAX]; //この配列0番を推論1番枠と呼ぶことにする。

void AnnotatedCameraWidget::drawLockon(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd , int num  /*使っていない , size_t leads_num , const cereal::RadarState::LeadData::Reader &lead0, const cereal::RadarState::LeadData::Reader &lead1 */) {
  //const float speedBuff = 10.;
  //const float leadBuff = 40.;
  const float d_rel = lead_data.getX()[0];
  //const float d_rel = lead_data.getDRel();
  //const float v_rel = lead_data.getV()[0];
  //const float t_rel = lead_data.getT()[0];
  //const float y_rel = lead_data.getY()[0];
  float a_rel = lead_data.getA()[0];
  //float a_rel = lead_data.getARel(); //ある？
  global_a_rel = a_rel;

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  //float y = std::fmin(height() /*- sz * .6*/, (float)vd.y());
  float y = (float)vd.y();

  //float g_xo = sz / 5;
  //float g_yo = sz / 10;

  //QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};

  painter.setCompositionMode(QPainter::CompositionMode_Plus);
  //p.setPen(QColor(0, 255, 0, 255));

  float prob_alpha = lead_data.getProb(); //getModelProb();
  if(prob_alpha < 0){
    prob_alpha = 0;
  } else if(prob_alpha > 1.0){
    prob_alpha = 1.0;
  }
  prob_alpha *= 245;

  painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
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
  if(uiState()->scene.wide_cam == false) { //dhに奥行き値を反映させる。
    float dd = d;
    dd -= 25; //dd=0〜75
    dd /= (75.0/2); //dd=0〜2
    dd += 1; //dd=1〜3
    if(dd < 1)dd = 1;
    dh /= dd;
  } else { //ワイドカメラ使用でロジック変更。リアルタイムで変わる。
    ww *= 0.5; hh *= 0.5;
    dh = 100;
    float dd = d;
    dd -= 5; //dd=0〜95
    dd /= (95.0/10); //dd=0〜10
    dd += 1; //dd=1〜11
    if(dd < 1)dd = 1;
    dh /= dd*dd;
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

  painter.setFont(InterFont(38, QFont::DemiBold));
  if(num == 0 && uiState()->scene.mLockOnButton){
    //推論1番
    painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
    painter.drawRect(r);

    //painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
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
        painter.setBrush(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha*0.9));

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
      painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), tlw));
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
        //painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 1)); //文字を後で書くために色を再設定。->文字は赤でもいいや

        //円を（意味不明だから）書かないで、枠ごと赤くする。推論1が推論と別のものを捉えてるのを簡単に認識できる。
        painter.setPen(QPen(QColor(245, 0, 0, prob_alpha), 2));
      } else {
        painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
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
      painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
      //painter.drawLine(r.right(),r.center().y() , width() , height());
    } else {
      //推論4番以降。
      //存在していない。
      painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
      //painter.drawLine(r.left(),r.center().y() , 0 , height());
    }

    painter.drawRect(r);

    //painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));

    if(ww >= 80){
      //ここではy0,y1を参照できない。
      float d_lim = 12;
      if(wide_cam_requested == false){
        d_lim = 32; //ロングカメラだとちょっと枠が大きい。実測
      }
      if(num == 0 || (num==1 && (d_rel < d_lim || std::abs(y0 - y1) > 300))){ //num==1のとき、'2'の表示と前走車速度表示がかぶるので、こちらを消す。
        painter.drawText(r, Qt::AlignBottom | Qt::AlignLeft, " " + QString::number(num+1));
      }
    }
    if(ww >= 160 /*80*/){
      //painter.drawText(r, Qt::AlignBottom | Qt::AlignRight, QString::number((int)(lead_data.getProb()*100)) + "％");
      //painter.drawText(r, Qt::AlignBottom | Qt::AlignRight, QString::number(a_rel,'f',1) + "a");
    }
  }
  painter.setPen(Qt::NoPen);
  painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
}

void AnnotatedCameraWidget::paintGL() {
  UIState *s = uiState();
  SubMaster &sm = *(s->sm);
  const double start_draw_t = millis_since_boot();
  const cereal::ModelDataV2::Reader &model = sm["modelV2"].getModelV2();

  // draw camera frame
  {
    std::lock_guard lk(frame_lock);

    if (frames.empty()) {
      if (skip_frame_count > 0) {
        skip_frame_count--;
        qDebug() << "skipping frame, not ready";
        return;
      }
    } else {
      // skip drawing up to this many frames if we're
      // missing camera frames. this smooths out the
      // transitions from the narrow and wide cameras
      skip_frame_count = 5;
    }

    // Wide or narrow cam dependent on speed
    bool has_wide_cam = available_streams.count(VISION_STREAM_WIDE_ROAD);
    if (has_wide_cam) {
      float v_ego = sm["carState"].getCarState().getVEgo();
      if ((v_ego < 10) || available_streams.size() == 1) {
        wide_cam_requested = true;
      } else if (v_ego > 15) {
        wide_cam_requested = false;
      }
      wide_cam_requested = wide_cam_requested && sm["controlsState"].getControlsState().getExperimentalMode();
      // for replay of old routes, never go to widecam
      wide_cam_requested = wide_cam_requested && s->scene.calibration_wide_valid;
    }
    CameraWidget::setStreamType(wide_cam_requested ? VISION_STREAM_WIDE_ROAD : VISION_STREAM_ROAD);

    s->scene.wide_cam = CameraWidget::getStreamType() == VISION_STREAM_WIDE_ROAD;
    if (s->scene.calibration_valid) {
      auto calib = s->scene.wide_cam ? s->scene.view_from_wide_calib : s->scene.view_from_calib;
      CameraWidget::updateCalibration(calib);
    } else {
      CameraWidget::updateCalibration(DEFAULT_CALIBRATION);
    }
    CameraWidget::setFrameId(model.getFrameId());
    CameraWidget::paintGL();
  }

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::NoPen);

  if (s->scene.world_objects_visible) {
    update_model(s, model);
    drawLaneLines(painter, s);

    if (s->scene.longitudinal_control && sm.rcv_frame("radarState") > s->scene.started_frame) {
      auto radar_state = sm["radarState"].getRadarState();
      update_leads(s, radar_state, model.getPosition());

      const auto leads = model.getLeadsV3();
      size_t leads_num = leads.size();
      for(size_t i=0; i<leads_num && i < LeadcarLockon_MAX; i++){
        if(leads[i].getProb() > .2){ //信用度20%以上で表示。調整中。
          drawLockon(painter, leads[i], s->scene.lead_vertices[i] , i /*, leads_num , leads[0] , leads[1]*/);
        }
      }

      auto lead_one = radar_state.getLeadOne();
      auto lead_two = radar_state.getLeadTwo();
      if (lead_one.getStatus()) {
        drawLead(painter, lead_one, s->scene.lead_vertices[0] , 0);
      }
      if (lead_two.getStatus() && (std::abs(lead_one.getDRel() - lead_two.getDRel()) > 3.0)) {
        drawLead(painter, lead_two, s->scene.lead_vertices[1] , 1);
      }
    }
  }

  // DMoji
  if (!hideBottomIcons && (sm.rcv_frame("driverStateV2") > s->scene.started_frame)) {
    update_dmonitoring(s, sm["driverStateV2"].getDriverStateV2(), dm_fade_state, rightHandDM);
    drawDriverState(painter, s);
  }

  drawHud(painter);

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  double fps = fps_filter.update(1. / dt * 1000);
  global_fps = (float)fps;
  if (fps < 15) {
    LOGW("slow frame rate: %.2f fps", fps);
  }
  prev_draw_t = cur_draw_t;
  distance_traveled += (*s->sm)["carState"].getCarState().getVEgo() * dt / 1000;

  // publish debug msg
  MessageBuilder msg;
  auto m = msg.initEvent().initUiDebug();
  m.setDrawTimeMillis(cur_draw_t - start_draw_t);
  pm->send("uiDebug", msg);
}

void AnnotatedCameraWidget::showEvent(QShowEvent *event) {
  CameraWidget::showEvent(event);

  ui_update_params(uiState());
  prev_draw_t = millis_since_boot();
}
