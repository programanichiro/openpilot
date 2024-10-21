#include "selfdrive/ui/qt/offroad/driverview.h"

#include <algorithm>
#include <cmath>
#include <QPainter>

#include "selfdrive/ui/qt/util.h"

DriverViewWindow::DriverViewWindow(QWidget* parent) : CameraWidget("camerad", VISION_STREAM_DRIVER, parent) {
  QObject::connect(this, &CameraWidget::clicked, this, &DriverViewWindow::done);
  QObject::connect(device(), &Device::interactiveTimeout, this, [this]() {
    if (isVisible()) {
      //emit done(); , ひとまず時間で閉じるのをやめる
    }
  });
}

void DriverViewWindow::showEvent(QShowEvent* event) {
  params.putBool("IsDriverViewEnabled", true);
  device()->resetInteractiveTimeout(60);
  CameraWidget::showEvent(event);
}

void DriverViewWindow::hideEvent(QHideEvent* event) {
  params.putBool("IsDriverViewEnabled", false);
  stopVipcThread();
  CameraWidget::hideEvent(event);
}

void DriverViewWindow::paintGL() {
  CameraWidget::paintGL();

  std::lock_guard lk(frame_lock);
  QPainter p(this);
  // startup msg
  if (frames.empty() && !uiState()->scene.started) {
    p.setPen(Qt::white);
    p.setRenderHint(QPainter::TextAntialiasing);
    p.setFont(InterFont(100, QFont::Bold));
    p.drawText(geometry(), Qt::AlignCenter, tr("camera starting"));
    return;
  }

  const auto &sm = *(uiState()->sm);
  cereal::DriverStateV2::Reader driver_state = sm["driverStateV2"].getDriverStateV2();
  bool is_rhd = driver_state.getWheelOnRightProb() > 0.5;
  auto driver_data = is_rhd ? driver_state.getRightDriverData() : driver_state.getLeftDriverData();

  bool face_detected = driver_data.getFaceProb() > 0.7;
  if (face_detected) {
    auto fxy_list = driver_data.getFacePosition();
    auto std_list = driver_data.getFaceOrientationStd();
    float face_x = fxy_list[0];
    float face_y = fxy_list[1];
    float face_std = std::max(std_list[0], std_list[1]);

    float alpha = 0.7;
    if (face_std > 0.15) {
      alpha = std::max(0.7 - (face_std-0.15)*3.5, 0.0);
    }
    const int box_size = 220;
    // use approx instead of distort_points
    int fbox_x = 1080.0 - 1714.0 * face_x;
    int fbox_y = -135.0 + (504.0 + std::abs(face_x)*112.0) + (1205.0 - std::abs(face_x)*724.0) * face_y;
    p.setPen(QPen(QColor(255, 255, 255, alpha * 255), 10));
    if(uiState()->scene.started){
      p.setBrush(QColor(192, 102, 0, 255)); //顔を隠す
    }
    p.drawRoundedRect(fbox_x - box_size / 2, fbox_y - box_size / 2, box_size, box_size, 35.0, 35.0);
  }

  driver_monitor.updateState(*uiState());
  driver_monitor.draw(p, rect());

  if(uiState()->scene.started){
    mini_knightScanner(p);
  }
}

extern bool global_engageable;
extern int global_status;
extern float vc_speed;
extern float curve_value;
extern void setButtonInt(const char*fn , int num); //新fn="/data/accel_engaged.txt"など、このファイルが無かったら0。num(0〜3)はそのまま数字で。
extern void setButtonEnabled0(const char*fn , bool flag); //旧fn="/data/accel_engaged.txt"など、このファイルが無かったらfalseのニュアンスで。flagはそのままtrueなら有効。
void DriverViewWindow::mini_knightScanner(QPainter &p) {

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
  }
  p.setCompositionMode(QPainter::CompositionMode_Plus);
  p.setPen(QPen(QColor(255, 255, 255, 0), 0)); //枠をとる
  for(int i=0; i<(n-1); i++){
    //QRect rc(0, h_pos, ww, hh);
    if(t[i] > 0.01){
      //p.drawRoundedRect(rc, 0, 0);
      if(left_blinker || right_blinker){
        //流れるウインカー
        p.setBrush(QColor(192, 102, 0, 255 * t[i]));
        p.drawRect(rect_w * i / (n-1), h_pos - lane_change_height, ww, hh); //drawRectを使う利点は、角を取ったりできそうだ。
      } else {
        ;
      }
    }
    t[i] *= 0.9;
  }

}

mat4 DriverViewWindow::calcFrameMatrix() {
  const float driver_view_ratio = 2.0;
  const float yscale = stream_height * driver_view_ratio / stream_width;
  const float xscale = yscale * glHeight() / glWidth() * stream_width / stream_height;
  return mat4{{
    xscale,  0.0, 0.0, 0.0,
    0.0,  yscale, 0.0, 0.0,
    0.0,  0.0, 1.0, 0.0,
    0.0,  0.0, 0.0, 1.0,
  }};
}
