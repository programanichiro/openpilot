#include "selfdrive/ui/qt/onroad/hud.h"

#include <cmath>

#include "selfdrive/ui/qt/util.h"

#include "selfdrive/ui/qt/onroad/buttons.h"
//constexpr int SET_SPEED_NA = 255;
#define PI0_DEBUG false
extern bool global_engageable;
extern float vc_speed;
extern int tss_type;
extern float maxspeed_org;
extern std::string road_info_txt;
extern bool g_rightHandDM;
extern int ACC_speed;
extern void setButtonInt(const char*fn , int num);

HudRenderer::HudRenderer() {
  engage_img = loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size});
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size - 5, img_size - 5});
}

void HudRenderer::updateState(const UIState &s) {
  int SET_SPEED_NA = 410; ///409; //406; //557; //255; ,
  const SubMaster &sm = *(s.sm);

  const bool cs_alive = sm.alive("carState");
  const bool nav_alive = sm.alive("navInstruction") && sm["navInstruction"].getValid();
  //const auto cs = sm["controlsState"].getControlsState();
  const auto &car_state = sm["carState"].getCarState();
  const auto &nav_instruction = sm["navInstruction"].getNavInstruction();

  is_metric = s.scene.is_metric;

  // Handle older routes where vCruise was in controlsState
  float v_cruise = car_state.getVCruiseCluster() == 0.0 ? car_state.getVCruise() : car_state.getVCruiseCluster();
  ACC_speed = std::nearbyint(v_cruise); //45〜
  v_cruise = car_state.getVCruise(); //41〜,間違いない、表示して確認した。改めてこちらを使う。
  maxspeed_org = car_state.getVCruise(); //これで元の41〜 , v_cruise; //レバー値の元の値。黄色点滅警告にはマッチしてる気がする。
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
  //v_cruise = v_cruise > (107 - 6) ? (107 + ((v_cruise+6) - 107) * 2 - 6) : v_cruise; //最大119 -> 114 -> 117に。
    v_cruise = v_cruise > (106 - 6) ? (106 + ((v_cruise+6) - 106) * 2 - 6) : v_cruise; //最大118に。
  } else if(PI0_DEBUG == true || tss_type == 2){
    SET_SPEED_NA = 255; //TSS2では戻す。
  }

  status = s.status;

  if (!sm.alive("carState")) {
    is_cruise_set = false;
    set_speed = SET_SPEED_NA;
    speed = 0.0;
    return;
  }

  //const auto &controls_state = sm["controlsState"].getControlsState();

  // Handle older routes where vCruiseCluster is not set
  //set_speed = car_state.getVCruiseCluster() == 0.0 ? controls_state.getVCruiseDEPRECATED() : car_state.getVCruiseCluster();
  set_speed = cs_alive ? v_cruise : SET_SPEED_NA;
  is_cruise_set = set_speed > 0 && set_speed != SET_SPEED_NA;

  if (is_cruise_set && !is_metric) {
    set_speed *= KM_TO_MILE;
  }

  // Handle older routes where vEgoCluster is not set
  // v_ego_cluster_seen = v_ego_cluster_seen || car_state.getVEgoCluster() != 0.0;
  // float v_ego = v_ego_cluster_seen ? car_state.getVEgoCluster() : car_state.getVEgo();
  // speed = std::max<float>(0.0f, v_ego * (is_metric ? MS_TO_KPH : MS_TO_MPH));

  v_ego_cluster_seen = v_ego_cluster_seen || car_state.getVEgoCluster() != 0.0;
  float v_ego = v_ego_cluster_seen ? car_state.getVEgoCluster() : car_state.getVEgo();
  speed = cs_alive ? std::max<float>(0.0, v_ego) : 0.0;
  vc_speed = v_ego;
  QString maxspeed_str = is_cruise_set ? QString::number(std::nearbyint(set_speed)) : "N/A";
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
  speed *= is_metric ? MS_TO_KPH : MS_TO_MPH;

  auto speed_limit_sign = nav_instruction.getSpeedLimitSign();
  float old_speedLimit = speedLimit;
  speedLimit = nav_alive ? nav_instruction.getSpeedLimit() : 0.0;
  speedLimit *= (is_metric ? MS_TO_KPH : MS_TO_MPH);
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
  speedUnit = is_metric ? tr("km/h") : tr("mph");
  status = s.status;

  maxSpeed = maxspeed_str; //ichiro pilot

}

void HudRenderer::draw(QPainter &p, const QRect &surface_rect) {
#if 0
  p.save();

  // Draw header gradient
  QLinearGradient bg(0, UI_HEADER_HEIGHT - (UI_HEADER_HEIGHT / 2.5), 0, UI_HEADER_HEIGHT);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, surface_rect.width(), UI_HEADER_HEIGHT, bg);


  drawSetSpeed(p, surface_rect);
  drawCurrentSpeed(p, surface_rect);

  p.restore();
#else
  drawHud(p,surface_rect);
#endif
}

void HudRenderer::drawSetSpeed(QPainter &p, const QRect &surface_rect) {
  // Draw outer box + border to contain set speed
  const QSize default_size = {172, 204};
  QSize set_speed_size = is_metric ? QSize(200, 204) : default_size;
  QRect set_speed_rect(QPoint(60 + (default_size.width() - set_speed_size.width()) / 2, 45), set_speed_size);

  // Draw set speed box
  p.setPen(QPen(QColor(255, 255, 255, 75), 6));
  p.setBrush(QColor(0, 0, 0, 166));
  p.drawRoundedRect(set_speed_rect, 32, 32);

  // Colors based on status
  QColor max_color = QColor(0xa6, 0xa6, 0xa6, 0xff);
  QColor set_speed_color = QColor(0x72, 0x72, 0x72, 0xff);
  if (is_cruise_set) {
    if (status == STATUS_DISENGAGED) {
      max_color = QColor(255, 255, 255);
    } else if (status == STATUS_OVERRIDE) {
      max_color = QColor(0x91, 0x9b, 0x95, 0xff);
    } else {
      max_color = QColor(0x80, 0xd8, 0xa6, 0xff);
      set_speed_color = QColor(255, 255, 255);
    }
  }

  // Draw "MAX" text
  p.setFont(InterFont(40, QFont::DemiBold));
  p.setPen(max_color);
  p.drawText(set_speed_rect.adjusted(0, 27, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("MAX"));

  // Draw set speed
  QString setSpeedStr = is_cruise_set ? QString::number(std::nearbyint(set_speed)) : "–";
  p.setFont(InterFont(90, QFont::Bold));
  p.setPen(set_speed_color);
  p.drawText(set_speed_rect.adjusted(0, 77, 0, 0), Qt::AlignTop | Qt::AlignHCenter, setSpeedStr);
}

void HudRenderer::drawCurrentSpeed(QPainter &p, const QRect &surface_rect) {
  QString speedStr = QString::number(std::nearbyint(speed));

  p.setFont(InterFont(176, QFont::Bold));
  drawText(p, surface_rect.center().x(), 210, speedStr);

  p.setFont(InterFont(66));
  drawText(p, surface_rect.center().x(), 290, is_metric ? tr("km/h") : tr("mph"), 200);
}
#if 0
void HudRenderer::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}
#endif
extern bool all_brake_light;
int global_status;
float curve_value;
float handle_center = -100;
int handle_calibct = 0;
float distance_traveled;
float global_angle_steer0 = 0;
float clipped_brightness0 = 101; //初回ファイルアクセスさせるため、わざと101
float global_fps;
bool add_v_by_lead;
int limit_speed_auto_detect; //map.ccから参照あり
extern int Limit_speed_mode; //標識
extern bool Long_enable;
extern bool mapVisible;
extern void soundButton2(int onOff);
extern void setButtonEnabled0(const char*fn , bool flag);
int g_night_mode;
void HudRenderer::drawHud(QPainter &p,const QRect &surface_rect) {
  p.save();
  int y_ofs = 150;

  // Header gradient
  QLinearGradient bg(0, UI_HEADER_HEIGHT - (UI_HEADER_HEIGHT / 2.5), 0, UI_HEADER_HEIGHT);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, surface_rect.width(), UI_HEADER_HEIGHT+y_ofs, bg);

  QString speedLimitStr = (speedLimit > 1) ? QString::number(std::nearbyint(speedLimit)) : "–";
  QString speedStr = QString::number(std::nearbyint(speed));
  //QString setSpeedStr = is_cruise_set ? QString::number(std::nearbyint(set_speed)) : "–";

  // max speed
  float max_disp_k = 1.8;
  //float max_disp_a = 50;
  const int rect_w = surface_rect.width();
  const int rect_h = surface_rect.height();
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
        return speedLimit > 0 ? interpColor(set_speed, {speedLimit + 5, speedLimit + 15, speedLimit + 25}, {c1, c2, c3}) : c1;
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
    drawText(p, surface_rect.center().x()-7, 210+y_ofs-5, speedStr,speed_waku);
    drawText(p, surface_rect.center().x()+7, 210+y_ofs-5, speedStr,speed_waku);
    drawText(p, surface_rect.center().x(), -7+210+y_ofs-5, speedStr,speed_waku);
    drawText(p, surface_rect.center().x(), +7+210+y_ofs-5, speedStr,speed_waku);
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
  drawText(p, surface_rect.center().x(), 210 + y_ofs-5, speedStr , speed_num);

  p.setFont(InterFont(66));
  if (uiState()->scene.longitudinal_control == false) {
#define COLOR_STATUS_WARNING QColor(0xDA, 0x6F, 0x25, 0xf1)
//  [STATUS_ALERT] = QColor(0xC9, 0x22, 0x31, 0xf1),
    drawText(p, surface_rect.center().x(), 290 + y_ofs-5, speedUnit, COLOR_STATUS_WARNING); //縦制御無効状態でkm/hを警告色に。
  } else {
    int w;
    if(velo_for_trans < velo_for_trans_limit){
      w = drawText(p, surface_rect.center().x(), 290 + y_ofs-5, speedUnit, speed_num);
    } else {
      w = drawText(p, surface_rect.center().x(), 290 + y_ofs-5, speedUnit, 200);
    }
    if(is_cruise_set){
      p.setFont(InterFont(40, QFont::ExtraBold));
      drawTextCenter(p, surface_rect.center().x() + w/2 + 43, 290 + y_ofs-35 , QString::number(ACC_speed) , velo_for_trans < velo_for_trans_limit ? 100 : 235 , false , 0x24, 0x57, 0xa1 , 240, 240, 240, velo_for_trans < velo_for_trans_limit ? 70 : 230 , 9 , 15 , 18 , 2);
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
  std::string gps_axs_data_txt = util::read_file("/tmp/gps_axs_data.txt");
  int gps_idx_i = 0;
  bool gps_ok = false;
  static double gps_output[6]; // double型の配列
  if(gps_axs_data_txt.empty() == false){
    int i = 0; // インデックス

    std::stringstream ss(gps_axs_data_txt); // 入力文字列をstringstreamに変換
    std::string token; // 一時的にトークンを格納する変数
    while (std::getline(ss, token, ',') && i < 6) { // カンマで分割し、一つずつ処理する
      gps_output[i] = std::stod(token); // 分割された文字列をdouble型に変換して配列に格納
      i++; // インデックスを1つ進める
    }
    gps_idx_i = i;
    gps_ok = true;
  } else {
    gps_ok = false;
  }
  bool okGps = (gps_idx_i == 6 && gps_ok && (int)gps_output[5]);
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
  if ((limitspeed_update_ct ++) % 10 == 0 && okGps) {
    double locationd_pos[2] = {gps_output[0],gps_output[1]}; //lat,lon
    double locationd_orientation = gps_output[2]; //bearing
    //double locationd_velocity = gps_output[3]; //VEgo、信用できない。

    bool locationd_valid = ((int)gps_output[5] == 1);

    if (locationd_valid) {
      FILE *fp = fopen("/tmp/limitspeed_info.txt","w");
      if(fp != NULL){
        //この辺で30mか1秒？ごとに、以下を/tmp/limitspeed_info.txtに書き込む。
        double latitude = locationd_pos[0]; // 緯度を取得
        double longitude = locationd_pos[1]; // 経度を取得
        double bearing = locationd_orientation;  //-180〜180
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

  QRect temp_rc(surface_rect.left()+65-27, surface_rect.top()+110+6, 233+27*2-5, 54);
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
  //p.drawText(QRect(surface_rect.left()+65, surface_rect.top()+110, 300, 65), Qt::AlignTop | Qt::AlignLeft, temp_disp);
  p.drawText(QRect(surface_rect.left()+65+120-5, surface_rect.top()+110+7, 300, 65), Qt::AlignTop | Qt::AlignLeft, temp_disp3);
  p.setFont(InterFont(54, QFont::Bold));
  p.drawText(QRect(surface_rect.left()+65+55+5-5, surface_rect.top()+110-8+9, 300, 65), Qt::AlignTop | Qt::AlignLeft, temp_disp2);
  p.setFont(InterFont(48));
  p.drawText(QRect(surface_rect.left()+65+5+5, surface_rect.top()+110-8+11, 300, 65+5), Qt::AlignTop | Qt::AlignLeft, temp_disp1);

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
    drawText(p, surface_rect.left()+260, 55, "Powered by COMMA.AI", logo_trs, brake_light);
  } else if((float)rect_w / rect_h > 1.02f){
    p.setFont(InterFont(44, QFont::DemiBold));
    drawText(p, surface_rect.left()+140, 55, "COMMA.AI", logo_trs, brake_light);
  } else if((float)rect_w / rect_h >= 0.975f){ //消えるタイミングは合わせたい。
    p.setFont(InterFont(44, QFont::DemiBold));
    drawText(p, surface_rect.left()+102, 55, "COMMA", logo_trs, brake_light);
  }
  if((float)rect_w / rect_h > 1.35f){
    p.setFont(InterFont(55, QFont::DemiBold));
    if(tss_type <= 1){
      int next_x = drawTextRight(p, surface_rect.right()-20, 60 , " TSSP", logo_trs, brake_light, label_red, label_grn, label_blu); //47060車はTSSP部分が黄色くなる。
      drawTextRight(p, next_x, 60 , "for toyota", logo_trs, brake_light);
    } else {
      drawTextRight(p, surface_rect.right()-20, 60 , "for toyota TSS2", logo_trs, brake_light);
    }
  } else if((float)rect_w / rect_h > 1.2f){
    p.setFont(InterFont(55, QFont::DemiBold));
    if(tss_type <= 1){
      int next_x = drawTextRight(p, surface_rect.right()-20, 60 , " TSSP", logo_trs, brake_light, label_red, label_grn, label_blu);
      drawTextRight(p, next_x, 60 , "toyota", logo_trs, brake_light);
    } else {
      drawTextRight(p, surface_rect.right()-20, 60 , "toyota TSS2", logo_trs, brake_light);
    }
  } else if((float)rect_w / rect_h >= 0.975f){ //消えるタイミングは合わせたい。
    p.setFont(InterFont(50, QFont::DemiBold));
    if(tss_type <= 1){
      int next_x = drawTextRight(p, surface_rect.right()-20, 60 , " P", logo_trs, brake_light, label_red, label_grn, label_blu);
      drawTextRight(p, next_x, 57 , "toyota", logo_trs, brake_light);
    } else {
      drawTextRight(p, surface_rect.right()-20, 57 , "toyota 2", logo_trs, brake_light);
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
      int next_x = drawTextRight(p, surface_rect.right()-10, surface_rect.bottom() - 10 , QString::fromStdString(token), 220);
      if(kmh != "0"){
        drawTextRight(p, next_x-4, surface_rect.bottom() - 10 , QString::fromStdString(kmh) , 255 , false , 0x24, 0x57, 0xa1 , 255,255,255,200 , 6 , 5 , 2 , 0);
      }
    }
  }
  if(road_info_txt_flag == false){
    p.setFont(InterFont(33, QFont::DemiBold));
    drawTextRight(p, surface_rect.right()-10, surface_rect.bottom() - 10 , "Rrograman Ichiro modified", 150 /*, 255 , false , 0x24, 0x57, 0xa1 , 255,255,255,200 , 6*/);
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

  { //タコメーター
    double taco_rpm = (*s->sm)["carState"].getCarState().getEngineRpm();
    const double taco_max = 5000;
    if(taco_rpm > taco_max){
      taco_rpm = taco_max; //5000回転表示がMAX。
    }
    //taco_rpm = taco_max/2; //表示テスト
    double upper_2w = 200+80;
    double under_2w = 100+55;
    double lu = (double)surface_rect.center().x()-upper_2w;
    double ur = lu + upper_2w * 2 * taco_rpm / taco_max; //(double)surface_rect.center().x()+upper_2w;
    double ld = (double)surface_rect.center().x()-under_2w;
    double dr = ld + under_2w * 2 * taco_rpm / taco_max; //(double)surface_rect.center().x()+under_2w;
    QPointF taco_meter[] = {{lu,(double)20},{ld,(double)50 + 40*3+10}, {dr,(double)50 + 40*3+10}, {ur,(double)20}};
    p.setPen(Qt::NoPen);
    //p.setBrush(QColor::fromRgbF(0.8, 0.0, 0.0, 0.65)); //赤
    p.setBrush(QColor::fromRgbF(0.96*0.7, 0.51*0.7, 0.12*0.7, 0.65)); //オレンジ
    p.setCompositionMode(QPainter::CompositionMode_Plus);
    p.drawPolygon(taco_meter, std::size(taco_meter));
    p.setCompositionMode(QPainter::CompositionMode_SourceOver);
  }

  drawText(p, surface_rect.center().x(), 50 + 40*0 , "extra cruise speed engagement", a0 , brake_light);
  drawText(p, surface_rect.center().x(), 50 + 40*1 , "slow down corner correctly", a1 , brake_light);
  drawText(p, surface_rect.center().x(), 50 + 40*2 , "speed limit auto detect", a2 , brake_light);
  //drawText(p, surface_rect.center().x(), 50 + 40*2 , "curvature reinforcement", a2 , brake_light);
  //drawText(p, surface_rect.center().x(), 50 + 40*2 , QString::number(angle_steer), a2 , brake_light);
  drawText(p, surface_rect.center().x(), 50 + 40*3 , "auto brake holding", a3 , brake_light);

  // engage-ability icon
  if (true) {
    SubMaster &sm = *(uiState()->sm);
    QBrush bg_color = bg_colors[status];
    if(uiState()->scene.mAccelEngagedButton >= 3 && fabs(global_angle_steer0) >= 50 && (*(uiState()->sm))["carState"].getCarState().getVEgo() <= 0.01/3.6){
      //停止時の青信号発進抑制、一時的に緩和、15->50度
      bg_color = COLOR_STATUS_WARNING; //ワンペダル時に信号スタート可能角度でなければ警告色。
    }
    my_drawIcon(p, surface_rect.right() - btn_size / 2 - UI_BORDER_SIZE * 2, btn_size / 2 + int(UI_BORDER_SIZE * 1.5)+y_ofs,
             //engage_img, bg_color, 1.0 , -global_angle_steer0);
             sm["selfdriveState"].getSelfdriveState().getExperimentalMode() ? experimental_img : engage_img, blackColor(166), global_engageable ? 1.0 : 0.3 , -global_angle_steer0);
  }
  const float x_Long_enable = surface_rect.right() - btn_size / 2 - UI_BORDER_SIZE * 2;
  const float y_Long_enable = btn_size / 2 + int(UI_BORDER_SIZE * 1.5)+y_ofs;
  std::string long_speeddown_disable_txt = util::read_file("/tmp/long_speeddown_disable.txt");
  Long_enable = true;
  if(long_speeddown_disable_txt.empty() == false){
    if(std::stoi(long_speeddown_disable_txt) != 0){
      Long_enable = false;
    }
  }
  int long_base_angle0 = 45; //下中央から左右に何度か指定する。
  if((Long_enable || (*s->sm)["selfdriveState"].getSelfdriveState().getExperimentalMode()) && global_engageable){
    const int arc_w = -8; //内側に描画
    QPen pen = QPen(QColor(255, 255, ((*s->sm)["selfdriveState"].getSelfdriveState().getExperimentalMode()) ? 0 : 255, 180), abs(arc_w));
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
    QPen pen = QPen(QColor(255, 255, ((*s->sm)["selfdriveState"].getSelfdriveState().getExperimentalMode()) ? 0 : 255, 180), abs(arc_w_base));
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

    const float traffic_speed_r = 120 / 2 , traffic_speed_x = 247 , traffic_speed_y = surface_rect.height() - traffic_speed_r*2 - 50;
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
  QRect rc2(surface_rect.right() - btn_size / 2 - UI_BORDER_SIZE * 2 - 100, -20 + btn_size / 2 + int(UI_BORDER_SIZE * 1.5)+y_ofs + calib_h -36, 200, 36);
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
    drawText(p, surface_rect.right() - btn_size / 2 - UI_BORDER_SIZE * 2 , -20 + btn_size / 2 + int(UI_BORDER_SIZE * 1.5)+y_ofs + calib_h - 8, QString::number(hc,'f',2) + "deg", 200);
  } else {
    p.setBrush(QColor(150, 150, 0, 0xf1));
    p.drawRoundedRect(rc2, 18, 18);
    p.setPen(Qt::NoPen);

    if(handle_calibct == 0){
      p.setFont(InterFont(33));
      drawText(p, surface_rect.right() - btn_size / 2 - UI_BORDER_SIZE * 2 , -20 + btn_size / 2 + int(UI_BORDER_SIZE * 1.5)+y_ofs + calib_h - 8, "Calibrating", 200);
    } else {
      p.setFont(InterFont(33, QFont::Bold));
      drawText(p, surface_rect.right() - btn_size / 2 - UI_BORDER_SIZE * 2 , -20 + btn_size / 2 + int(UI_BORDER_SIZE * 1.5)+y_ofs + calib_h - 6, QString::number(handle_calibct) + '%', 200);
    }
  }

  p.restore();
}

int HudRenderer::drawText(QPainter &p, int x, int y, const QString &text, int alpha , bool brakeLight) {
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

int HudRenderer::drawTextCircleCenter(QPainter &p, int x, int y, const QString &text, const QColor &col) { //円の中央とxを共有したいときに使いやすいので残す。
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(col);
  p.drawText(real_rect.x(), real_rect.bottom(), text);

  return real_rect.width(); //続けて利用できるように幅を返す。（次の表示を左右の隣に出すために使える）
}

int HudRenderer::drawText(QPainter &p, int x, int y, const QString &text, const QColor &col) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(col);
  p.drawText(real_rect.x(), real_rect.bottom(), text);

  return real_rect.width(); //続けて利用できるように幅を返す。（次の表示を左右の隣に出すために使える）
}

int HudRenderer::drawTextLeft(QPainter &p, int x, int y, const QString &text, int alpha , bool brakeLight , int red, int blu, int grn , int bk_red, int bk_blu, int bk_grn, int bk_alp, int bk_yofs, int bk_corner_r , int bk_add_w, int bk_xofs, int bk_add_h) {
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

int HudRenderer::drawTextRight(QPainter &p, int x, int y, const QString &text, int alpha , bool brakeLight , int red, int blu, int grn , int bk_red, int bk_blu, int bk_grn, int bk_alp, int bk_yofs, int bk_corner_r , int bk_add_w, int bk_xofs) {
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

int HudRenderer::drawTextCenter(QPainter &p, int x, int y, const QString &text, int alpha , bool brakeLight , int red, int blu, int grn , int bk_red, int bk_blu, int bk_grn, int bk_alp, int bk_yofs, int bk_corner_r , int bk_add_w, int bk_xofs) {
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

void HudRenderer::my_drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity , float ang) {
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

