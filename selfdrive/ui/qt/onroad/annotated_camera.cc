
#include "selfdrive/ui/qt/onroad/annotated_camera.h"
#include "common/transformations/coordinates.hpp"

#include <QPainter>
#include <algorithm>
#include <cmath>
#include <sstream>

#include "common/swaglog.h"
#include "selfdrive/ui/qt/util.h"

// Window that shows camera view and variety of info drawn on top
AnnotatedCameraWidget::AnnotatedCameraWidget(VisionStreamType type, QWidget *parent)
    : fps_filter(UI_FREQ, 3, 1. / UI_FREQ), CameraWidget("camerad", type, parent) {
  pm = std::make_unique<PubMaster>(std::vector<const char*>{"uiDebug"});

  main_layout = new QVBoxLayout(this);
/*
  main_layout->setMargin(UI_BORDER_SIZE);
  main_layout->setSpacing(0);

  experimental_btn = new ExperimentalButton(this);
  main_layout->addWidget(experimental_btn, 0, Qt::AlignTop | Qt::AlignRight);
*/

  buttons = new ButtonsWindow(this); //ここならばexperimental_btnとイベントの両立ができ、マップの右画面のスクロール操作ができる。->ExperimentalButtonをLayoutで囲むとイベントが先に登録勝ちになってしまう。
  QObject::connect(uiState(), &UIState::uiUpdate, buttons, &ButtonsWindow::updateState);
  main_layout->addWidget(buttons);
}

extern float handle_center;
extern int handle_calibct;
extern float distance_traveled;
extern float global_angle_steer0;
extern float clipped_brightness0; //初回ファイルアクセスさせるため、わざと101
extern float global_fps;
bool global_engageable;
float vc_speed;
static int tss_type = 0;
static float maxspeed_org;
std::string road_info_txt;
bool g_rightHandDM;
int ACC_speed;
extern void setButtonEnabled0(const char*fn , bool flag);
extern void setButtonInt(const char*fn , int num);
extern int getButtonInt(const char*fn , int defaultNum);
void AnnotatedCameraWidget::updateState(const UIState &s) {
  // update engageability/experimental mode button
//  experimental_btn->updateState(s);
  dmon.updateState(s);

  buttons->psn_update();
  const auto ss = (*s.sm)["selfdriveState"].getSelfdriveState();
  global_engageable = (ss.getEngageable() || ss.getEnabled());
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

mat4 AnnotatedCameraWidget::calcFrameMatrix() {
  // Project point at "infinity" to compute x and y offsets
  // to ensure this ends up in the middle of the screen
  // for narrow come and a little lower for wide cam.
  // TODO: use proper perspective transform?

  // Select intrinsic matrix and calibration based on camera type
  auto *s = uiState();
  bool wide_cam = active_stream_type == VISION_STREAM_WIDE_ROAD;
  const auto &intrinsic_matrix = wide_cam ? ECAM_INTRINSIC_MATRIX : FCAM_INTRINSIC_MATRIX;
  const auto &calibration = wide_cam ? s->scene.view_from_wide_calib : s->scene.view_from_calib;

   // Compute the calibration transformation matrix
  const auto calib_transform = intrinsic_matrix * calibration;

  float zoom = wide_cam ? 2.0 : 1.1;
  Eigen::Vector3f inf(1000., 0., 0.);
  auto Kep = calib_transform * inf;

  int w = width(), h = height();
  float center_x = intrinsic_matrix(0, 2);
  float center_y = intrinsic_matrix(1, 2);

  float max_x_offset = center_x * zoom - w / 2 - 5;
  float max_y_offset = center_y * zoom - h / 2 - 5;
  float x_offset = std::clamp<float>((Kep.x() / Kep.z() - center_x) * zoom, -max_x_offset, max_x_offset);
  float y_offset = std::clamp<float>((Kep.y() / Kep.z() - center_y) * zoom, -max_y_offset, max_y_offset);

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  Eigen::Matrix3f video_transform =(Eigen::Matrix3f() <<
    zoom, 0.0f, (w / 2 - x_offset) - (center_x * zoom),
    0.0f, zoom, (h / 2 - y_offset) - (center_y * zoom),
    0.0f, 0.0f, 1.0f).finished();

  s->car_space_transform = video_transform * calib_transform;
  s->clip_region = rect().adjusted(-500, -500, 500, 500);

  float zx = zoom * 2 * center_x / w;
  float zy = zoom * 2 * center_y / h;
  return mat4{{
    zx, 0.0, 0.0, -x_offset / w * 2,
    0.0, zy, 0.0, y_offset / h * 2,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
  }};
}

void AnnotatedCameraWidget::drawLaneLines(QPainter &painter, const UIState *s) {
  painter.save();

  const UIScene &scene = s->scene;
  SubMaster &sm = *(s->sm);

  // lanelines
  const bool chill_mode = false; //!sm["selfdriveState"].getSelfdriveState().getExperimentalMode();
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
  if (sm["selfdriveState"].getSelfdriveState().getExperimentalMode()) {
    // The first half of track_vertices are the points for the right side of the path
    const auto &acceleration = sm["modelV2"].getModelV2().getAcceleration().getX();
    const int max_len = std::min<int>(scene.track_vertices.length() / 2, acceleration.size());

    for (int i = 0; i < max_len; ++i) {
      // Some points are out of frame
      int track_idx = max_len - i - 1;  // flip idx to start from bottom right
      if (scene.track_vertices[track_idx].y() < 0 || scene.track_vertices[track_idx].y() > height()) continue;

      // Flip so 0 is bottom of frame
      float lin_grad_point = (height() - scene.track_vertices[track_idx].y()) / height();

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

bool all_brake_light = false;
void AnnotatedCameraWidget::knightScanner(QPainter &p) {
  extern int global_status;
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
  extern float curve_value;
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
  static float _1_vc_accel;
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


#if 0 //曲率、k_v表示テスト
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
#else
  //open street mapへのアクセス頻度を左端緑メーターで視覚化。通信が滞れば赤になる。
  // with open('/tmp/osm_access_counter.txt','w') as fp:
  //   fp.write('%d,%d,%d' % (int(per * 100),self.frame_ct2,self.frame_ct))

  static int osm_frame_ct_ct = -1; //-1 or 100以上でosmへの通信が死んでいる。
  static int osm_per = 0; //2Hzに対してosmの応答率。走行中ならだいたい50パーセントくらいになる。
  static std::string osm_access_counter_txt;
  static unsigned int osm_access_counter_ct = 0;
  if(osm_access_counter_ct++ % 20 == 0){
    osm_access_counter_txt = util::read_file("/tmp/osm_access_counter.txt");
  }
  if(osm_access_counter_txt.empty() == false){
    int i = 0; // インデックス
    std::stringstream ss(osm_access_counter_txt); // 入力文字列をstringstreamに変換
    std::string token; // 一時的にトークンを格納する変数
    static int before_osm_frame_ct = 0;
    while (i < 3 && std::getline(ss, token, ',')) { // カンマで分割し、一つずつ処理する
      if(i == 0){
        osm_per = std::stoi(token);
      }
#if 1
      if(i == 1){
        int osm_frame_ct2 = std::stoi(token);
        if(osm_frame_ct2 == before_osm_frame_ct){
          osm_frame_ct_ct ++; //osm_frame_ct2が変化しなければカウントアップし続ける
        } else {
          osm_frame_ct_ct = 0; //ゼロに戻らなければ、osmへの通信が死んでいる。
        }
        before_osm_frame_ct = osm_frame_ct2;
      }
#else
      if(i == 2){
        int osm_frame_ct = std::stoi(token);
        if(osm_frame_ct == before_osm_frame_ct){
          osm_frame_ct_ct ++; //osm_frame_ctが変化しなければカウントアップし続ける
        } else {
          osm_frame_ct_ct = 0; //ゼロに戻らなければ、osmへの通信が死んでいる。
        }
        before_osm_frame_ct = osm_frame_ct;
      }
#endif
      i++; // インデックスを1つ進める
    }
  }

  if(osm_per >= 0){
    float h = rect_h * osm_per / 100;
    float wp1 = 10;
    if(0 <= osm_frame_ct_ct && osm_frame_ct_ct < 100){
      p.setBrush(QColor(0, 245, 0, 200)); //緑
    } else {
      p.setBrush(QColor(245, 0, 0, 200)); //赤、通信断絶。
    }
    p.drawRect(QRect(0 , rect_h - h , wp1 , h));
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
    if(global_status == STATUS_DISENGAGED || global_status == STATUS_OVERRIDE){
      manual_dist += now_dist; //手動運転中
      h_manual_dist += now_dist; //手動運転中
      if ((all_brake_light && vc_speed < 0.1/3.6)){
        h_manual_dist += 1.0/20; //1秒を1m換算
      }
      if (global_status != STATUS_DISENGAGED || (all_brake_light && vc_speed < 0.1/3.6)){
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
  if(fabs(vc_speed) < 0.1/3.6){
    std::string blue_signal_chk_txt = util::read_file("/tmp/blue_signal_chk.txt");
    if(blue_signal_chk_txt.empty() == false){
      p.setPen(Qt::NoPen);
      debug_disp_xpos = drawTextLeft(p , debug_disp_xpos , rect_h - 10 , QString("⚫︎") , 200 , false , 0xdf, 0xdf, 0x00 , 0, 0, 0, 140 , 13 , 5 , 13 , 0 , -5) + 11;
      int blue_signal_chk = std::stoi(blue_signal_chk_txt);
      QString debug_disp = QString::number(blue_signal_chk);
      debug_disp_xpos = drawTextLeft(p , debug_disp_xpos , rect_h - 10 , debug_disp , 200 , false , 0xdf, 0xdf, 0x00);
    }
  }
  if(0){
    p.setPen(Qt::NoPen);
    debug_disp_xpos = drawTextLeft(p , debug_disp_xpos+6 , rect_h - 10 , QString("RPM") , 200 , false , 0xdf, 0xdf, 0x00 , 0, 0, 0, 140 , 13 , 5 , 10 , -5 , -5) + 5;
    double taco_rpm = (*s->sm)["carState"].getCarState().getEngineRpm(); //rpm
    QString debug_disp = QString::number((int)taco_rpm);
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
  bool wide_cam = active_stream_type == VISION_STREAM_WIDE_ROAD;
  if(wide_cam == false) { //dhに奥行き値を反映させる。
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
      wide_cam_requested = wide_cam_requested && sm["selfdriveState"].getSelfdriveState().getExperimentalMode();
    }
    CameraWidget::setStreamType(wide_cam_requested ? VISION_STREAM_WIDE_ROAD : VISION_STREAM_ROAD);
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

  dmon.draw(painter, rect());
  hud.updateState(*s);
  hud.draw(painter, rect());

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
