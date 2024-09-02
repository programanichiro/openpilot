#include "selfdrive/ui/qt/onroad/driver_monitoring.h"
#include <algorithm>
#include <cmath>

#include "selfdrive/ui/qt/onroad/buttons.h"
#include "selfdrive/ui/qt/util.h"

// Default 3D coordinates for face keypoints
static constexpr vec3 DEFAULT_FACE_KPTS_3D[] = {
  {-5.98, -51.20, 8.00}, {-17.64, -49.14, 8.00}, {-23.81, -46.40, 8.00}, {-29.98, -40.91, 8.00}, {-32.04, -37.49, 8.00},
  {-34.10, -32.00, 8.00}, {-36.16, -21.03, 8.00}, {-36.16, 6.40, 8.00}, {-35.47, 10.51, 8.00}, {-32.73, 19.43, 8.00},
  {-29.30, 26.29, 8.00}, {-24.50, 33.83, 8.00}, {-19.01, 41.37, 8.00}, {-14.21, 46.17, 8.00}, {-12.16, 47.54, 8.00},
  {-4.61, 49.60, 8.00}, {4.99, 49.60, 8.00}, {12.53, 47.54, 8.00}, {14.59, 46.17, 8.00}, {19.39, 41.37, 8.00},
  {24.87, 33.83, 8.00}, {29.67, 26.29, 8.00}, {33.10, 19.43, 8.00}, {35.84, 10.51, 8.00}, {36.53, 6.40, 8.00},
  {36.53, -21.03, 8.00}, {34.47, -32.00, 8.00}, {32.42, -37.49, 8.00}, {30.36, -40.91, 8.00}, {24.19, -46.40, 8.00},
  {18.02, -49.14, 8.00}, {6.36, -51.20, 8.00}, {-5.98, -51.20, 8.00},
};

// Colors used for drawing based on monitoring state
static const QColor DMON_ENGAGED_COLOR = QColor::fromRgbF(0.1, 0.945, 0.26);
static const QColor DMON_DISENGAGED_COLOR = QColor::fromRgbF(0.545, 0.545, 0.545);

DriverMonitorRenderer::DriverMonitorRenderer() : face_kpts_draw(std::size(DEFAULT_FACE_KPTS_3D)) {
  dm_img = loadPixmap("../assets/img_driver_face.png", {img_size + 5, img_size + 5});
}

extern bool g_rightHandDM;
void DriverMonitorRenderer::updateState(const UIState &s) {
  auto &sm = *(s.sm);
  is_visible = sm["selfdriveState"].getSelfdriveState().getAlertSize() == cereal::SelfdriveState::AlertSize::NONE &&
               sm.rcv_frame("driverStateV2") > s.scene.started_frame;
  if (!is_visible) return;

  auto dm_state = sm["driverMonitoringState"].getDriverMonitoringState();
  is_active = dm_state.getIsActiveMode();
  is_rhd = dm_state.getIsRHD();
  g_rightHandDM = is_rhd;
  dm_fade_state = std::clamp(dm_fade_state + 0.2f * (0.5f - is_active), 0.0f, 1.0f);

  const auto &driverstate = sm["driverStateV2"].getDriverStateV2();
  const auto driver_orient = is_rhd ? driverstate.getRightDriverData().getFaceOrientation() : driverstate.getLeftDriverData().getFaceOrientation();

  for (int i = 0; i < 3; ++i) {
    float v_this = (i == 0 ? (driver_orient[i] < 0 ? 0.7 : 0.9) : 0.4) * driver_orient[i];
    driver_pose_diff[i] = std::abs(driver_pose_vals[i] - v_this);
    driver_pose_vals[i] = 0.8f * v_this + (1 - 0.8) * driver_pose_vals[i];
    driver_pose_sins[i] = std::sin(driver_pose_vals[i] * (1.0f - dm_fade_state));
    driver_pose_coss[i] = std::cos(driver_pose_vals[i] * (1.0f - dm_fade_state));
  }

  auto [sin_y, sin_x, sin_z] = driver_pose_sins;
  auto [cos_y, cos_x, cos_z] = driver_pose_coss;

  // Rotation matrix for transforming face keypoints based on driver's head orientation
  const mat3 r_xyz = {{
    cos_x * cos_z, cos_x * sin_z, -sin_x,
    -sin_y * sin_x * cos_z - cos_y * sin_z, -sin_y * sin_x * sin_z + cos_y * cos_z, -sin_y * cos_x,
    cos_y * sin_x * cos_z - sin_y * sin_z, cos_y * sin_x * sin_z + sin_y * cos_z, cos_y * cos_x,
  }};

  // Transform vertices
  for (int i = 0; i < face_kpts_draw.size(); ++i) {
    vec3 kpt = matvecmul3(r_xyz, DEFAULT_FACE_KPTS_3D[i]);
    face_kpts_draw[i] = {{kpt.v[0], kpt.v[1], kpt.v[2] * (1.0f - dm_fade_state) + 8 * dm_fade_state}};
  }
}

static void set_face_gesture_arc(QPainter &painter,float x, float y, int start_ang , int ang_width ,const QColor &col , int ww=20){
  //顔が向いてる方を上下左右で表示する。
  QPen pen = QPen(col, ww);
  pen.setCapStyle(Qt::FlatCap); //端をフラットに
  painter.setPen(pen);

  painter.drawArc(QRectF(x - btn_size / 2 +ww/2, y - btn_size / 2 +ww/2 , btn_size-ww, btn_size-ww) , start_ang * 16, ang_width * 16);
}

void DriverMonitorRenderer::draw(QPainter &painter, const QRect &surface_rect) {
  if (!is_visible) return;

  painter.save();

  int offset = UI_BORDER_SIZE + (30-UI_BORDER_SIZE) + btn_size / 2;
  float x = false /*is_rhd*/ ? surface_rect.width() - offset : offset;
  float y = surface_rect.height() - offset;
  float opacity = is_active ? 0.65f : 0.2f; y -= 18 + (30-UI_BORDER_SIZE)*2;

  drawIcon(painter, QPoint(x, y), dm_img, QColor(0, 0, 0, 70), opacity);
  if(is_rhd){ //ボタンを移動できないので、アイコンはそのまま、左肩に"R"を表示。
    painter.setFont(InterFont(70, QFont::Bold));
    drawText(painter, x - btn_size / 2, y - btn_size / 4, "R" , is_active ? 200 : 100);
  }

  QPointF keypoints[std::size(DEFAULT_FACE_KPTS_3D)];
  for (int i = 0; i < std::size(keypoints); ++i) {
    const auto &v = face_kpts_draw[i].v;
    float kp = (v[2] - 8) / 120.0f + 1.0f;
    keypoints[i] = QPointF(v[0] * kp + x, v[1] * kp + y);
  }

  painter.setPen(QPen(QColor::fromRgbF(1.0, 1.0, 1.0, opacity), 5.2, Qt::SolidLine, Qt::RoundCap));
  painter.drawPolyline(keypoints, std::size(keypoints));

  // tracking arcs
  const int arc_l = 133;
  const float arc_t_default = 6.7f;
  const float arc_t_extend = 12.0f;
  QColor arc_color = uiState()->engaged() ? DMON_ENGAGED_COLOR : DMON_DISENGAGED_COLOR;
  arc_color.setAlphaF(0.4 * (1.0f - dm_fade_state));

  float delta_x = -driver_pose_sins[1] * arc_l / 2.0f;
  float delta_y = -driver_pose_sins[0] * arc_l / 2.0f;

  // Draw horizontal tracking arc
  painter.setPen(QPen(arc_color, arc_t_default + arc_t_extend * std::min(1.0, driver_pose_diff[1] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(std::min(x + delta_x, x), y - arc_l / 2, std::abs(delta_x), arc_l), (driver_pose_sins[1] > 0 ? 90 : -90) * 16, 180 * 16);

  // Draw vertical tracking arc
  painter.setPen(QPen(arc_color, arc_t_default + arc_t_extend * std::min(1.0, driver_pose_diff[0] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(x - arc_l / 2, std::min(y + delta_y, y), arc_l, std::abs(delta_y)), (driver_pose_sins[0] > 0 ? 0 : 180) * 16, 180 * 16);

  //顔が向いてる方を上下左右かしげで表示する。
  float delta_r = driver_pose_sins[2]; //首のかしげ角度のsin
  static unsigned int key_n_ct = 0; //キーが入った順番
  static unsigned int left_face_key_n;
  static unsigned int right_face_key_n;
  static unsigned int up_face_key_n;
  static unsigned int down_face_key_n;
  static unsigned int lr_face_key_n;
  static unsigned int rr_face_key_n;

  const int long_press = 20;
  const int face_max_ct = 60; //long_pressを超えて（face_center_ctなどが）最大ここまでカウントする。
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

  //中央視線検出
  static int face_center_ct = 0;
  const float center_eye_rate = 0.3;
  bool center_detect = (dmActive && delta_x > left_face_x*center_eye_rate && delta_x < right_face_x*center_eye_rate
    && delta_y > up_face_y*center_eye_rate && delta_y < down_face_y*center_eye_rate
    && delta_r > r_face_r*center_eye_rate && delta_r < l_face_r*center_eye_rate);
  if(center_detect){
    set_face_gesture_arc(painter,x,y , 0 , 360 ,QColor(200,face_center_ct < long_press ? 200 : 100,0,250) , 10);
    if(face_center_ct < face_max_ct)
      face_center_ct ++;
  } else {
    // if(face_center_ct > long_press) //こうするか迷うところ。現状face_center_ctは表示以外に影響しない。中央インジケーターの⚪︎が消えるのが遅れるだけ。
    //   face_center_ct = long_press;
    if(face_center_ct > 0)
      face_center_ct --;
  }

  bool all_centering = true;
  //右向き検出
  static int face_right_ct = 0;
  if(delta_x > right_face_x){
    if(delta_x > right_face_x && face_right_ct >= 0){
      right_face_key_n = key_n_ct ++;
    }
    set_face_gesture_arc(painter,x,y , -45 , 90 ,QColor(200,face_right_ct < long_press ? 200 : 100,0,250));
    if(face_right_ct < face_max_ct)
      face_right_ct ++;
    if(delta_x > right_face_x)
      all_centering = false;
  } else {
    //face_right_ct = 0;
  }
  //左向き検出
  static int face_left_ct = 0;
  if(delta_x < left_face_x){
    if(delta_x < left_face_x && face_left_ct >= 0){
      left_face_key_n = key_n_ct ++;
    }
    set_face_gesture_arc(painter,x,y , 135 , 90 ,QColor(200,face_left_ct < long_press ? 200 : 100,0,250));
    if(face_left_ct < face_max_ct)
      face_left_ct ++;
    if(delta_x < left_face_x)
      all_centering = false;
  } else {
    //face_left_ct = 0;
  }
  //上向き検出
  static int face_up_ct = 0;
  if(delta_y < up_face_y){
    if(delta_y < up_face_y && face_up_ct >= 0){
      up_face_key_n = key_n_ct ++;
    }
    set_face_gesture_arc(painter,x,y , 45 , 90, QColor(200,face_up_ct < long_press ? 200 : 100,0,250));
    if(face_up_ct < face_max_ct)
      face_up_ct ++;
    if(delta_y < up_face_y)
      all_centering = false;
  } else {
    //face_up_ct = 0;
  }
  //下向き検出
  static int face_down_ct = 0;
  if(delta_y > down_face_y){
    if(delta_y > down_face_y && face_down_ct >= 0){
      down_face_key_n = key_n_ct ++;
    }
    set_face_gesture_arc(painter,x,y , -45 , -90, QColor(200,face_down_ct < long_press ? 200 : 100,0,250));
    if(face_down_ct < face_max_ct)
      face_down_ct ++;
    if(delta_y > down_face_y)
      all_centering = false;
  } else {
    //face_down_ct = 0;
  }

  //首を傾けるジェスチャー
  //右に傾げる
  static int face_rr_ct = 0;
  if(delta_r < r_face_r){
    if(delta_r < r_face_r && face_rr_ct >= 0){
      rr_face_key_n = key_n_ct ++;
    }
    set_face_gesture_arc(painter,x,y , -90-20 , 180, QColor(200,face_rr_ct < long_press ? 200 : 100,0,250));
    if(face_rr_ct < face_max_ct)
      face_rr_ct ++;
    if(delta_r < r_face_r)
      all_centering = false;
  } else {
    //face_rr_ct = 0;
  }
  //左に傾げる
  static int face_lr_ct = 0;
  if(delta_r > l_face_r){
    if(delta_r > l_face_r && face_lr_ct >= 0){
      lr_face_key_n = key_n_ct ++;
    }
    set_face_gesture_arc(painter,x,y , 90+20 , 180, QColor(200,face_lr_ct < long_press ? 200 : 100,0,250));
    if(face_lr_ct < face_max_ct)
      face_lr_ct ++;
    if(delta_r > l_face_r)
      all_centering = false;
  } else {
    //face_lr_ct = 0;
  }

  if(all_centering == true){
    if(face_right_ct > long_press)
      face_right_ct = long_press;
    if(face_left_ct > long_press)
      face_left_ct = long_press;
    if(face_up_ct > long_press)
      face_up_ct = long_press;
    if(face_down_ct > long_press)
      face_down_ct = long_press;
    if(face_rr_ct > long_press)
      face_rr_ct = long_press;
    if(face_lr_ct > long_press)
      face_lr_ct = long_press;

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
  if(face_down_ct > 0 && face_up_ct >= long_press && down_face_key_n < up_face_key_n  //↓↑ジェスチャー
      && face_right_ct == 0 && face_left_ct == 0 //検知以外の向き防止
    ){
    //ACC速度制御モード変更
    face_up_ct = 0; //多キーコマンドは-20にしなくても連続動作しない。
    face_down_ct = 0; //多キーコマンドは-20にしなくても連続動作しない。
    up_face_key_n = 0;
    down_face_key_n = 0;
    if(Limit_speed_mode == 0){
      Limit_speed_mode = 1;
    } else {
      Limit_speed_mode = 0; //2にはならない。
    }
  }

  if(mapVisible && face_left_ct > 1 && face_up_ct >= long_press && left_face_key_n < up_face_key_n //←↑ジェスチャー
      && face_down_ct == 0 && face_right_ct == 0 //検知以外の向き防止
    ){
    //地図ピッチアップ
    face_up_ct = 0; //多キーコマンドは-20にしなくても連続動作しない。
    face_left_ct = 0; //多キーコマンドは-20にしなくても連続動作しない。
    up_face_key_n = 0;
    left_face_key_n = 0;
    // extern bool map_pitch_up;
    // map_pitch_up = true;
    extern bool map_pitch_down;
    map_pitch_down = true;
  }

  if(false && mapVisible && face_left_ct > 1 && face_down_ct >= long_press && left_face_key_n < down_face_key_n //←↓ジェスチャー
      && face_up_ct == 0 && face_right_ct == 0 //検知以外の向き防止
    ){
    //地図ピッチダウン
    face_down_ct = 0; //多キーコマンドは-20にしなくても連続動作しない。
    face_left_ct = 0; //多キーコマンドは-20にしなくても連続動作しない。
    down_face_key_n = 0;
    left_face_key_n = 0;
    extern bool map_pitch_down;
    map_pitch_down = true;
  }

  // FILE *fp = fopen("/tmp/debug_out_rr","w");
  // if(fp != NULL){
  //   fprintf(fp,"rr:%d,%d,%d",face_rr_ct,face_left_ct,face_up_ct);
  //   fclose(fp);
  // }
  if(face_rr_ct >= long_press && face_left_ct < 3*face_rr_ct/long_press && face_up_ct < 3*face_rr_ct/long_press){ //↘︎ジェスチャー
    face_rr_ct = -long_press; //連続動作しないように工夫。
    rr_face_key_n = 0;
    {
      //カメラ↔︎地図の画面切り替え
      extern bool head_gesture_home; //サイドバーを消す
      extern bool head_gesture_onroad_home;
      head_gesture_home = true;
      head_gesture_onroad_home = true;
    }
  }

  const int long_press_tmp = (int)(long_press*0.75); //ミスカウントする分、少し短く。
  if(face_lr_ct > long_press_tmp && face_right_ct < 3*face_lr_ct/long_press_tmp && face_up_ct < 3*face_lr_ct/long_press_tmp){ //↙︎ジェスチャー
    face_lr_ct = -long_press_tmp; //連続動作しないように工夫。
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
