#include "selfdrive/ui/qt/onroad.h"

#include <cmath>

#include <QSoundEffect>
#include <QDebug>

#include "common/timing.h"
#include "selfdrive/ui/qt/util.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#endif

#define FONT_OPEN_SANS "Inter" //"Open Sans"
#define PI0_DEBUG false

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(bdr_s);
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

  if (s.scene.map_on_left) {
    split->setDirection(QBoxLayout::LeftToRight);
  } else {
    split->setDirection(QBoxLayout::RightToLeft);
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
    bool mapbox_extra = false;
    std::string my_mapbox_token = util::read_file("../../../mb_token.txt");
    if(my_mapbox_token.empty() == false){
      mapbox_extra = true;
    }
    if (map == nullptr && (uiState()->prime_type || !MAPBOX_TOKEN.isEmpty() || mapbox_extra)) {
      MapWindow * m = new MapWindow(get_mapbox_settings());
      map = m;

      QObject::connect(uiState(), &UIState::offroadTransition, m, &MapWindow::offroadTransition);

      std::string my_mapbox_width = util::read_file("../../../mb_width_rate.txt");
      if(my_mapbox_width.empty() == false){
        float w_rate = std::stof(my_mapbox_width);
        m->setFixedWidth((topWidget(this)->width() - 100) * w_rate);
      } else {
        m->setFixedWidth((topWidget(this)->width() - 100) / 2);
      }
      split->insertWidget(0, m);

      // Make map visible after adding to split
      m->offroadTransition(offroad);
    }
  }
#endif

  alerts->updateAlert({}, bg);
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));
}

// ***** onroad widgets *****
const float BUTTON_VOLUME = 0.35; //setVolumeが効いてないかも。
void soundPo(){
  static QSoundEffect effect;
  static bool once = false;
  if(once == false){
    once = true;
    effect.setSource(QUrl::fromLocalFile("../assets/sounds/po.wav"));
    effect.setLoopCount(0);
    effect.setVolume(1.0*BUTTON_VOLUME);
  }
  effect.play();
}

void soundPipo(){
  static QSoundEffect effect;
  static bool once = false;
  if(once == false){
    once = true;
    effect.setSource(QUrl::fromLocalFile("../assets/sounds/pipo.wav"));
    effect.setLoopCount(0);
    effect.setVolume(0.8*BUTTON_VOLUME);
  }
  effect.play();
}

void soundPikiri(){
  static QSoundEffect effect;
  static bool once = false;
  if(once == false){
    once = true;
    effect.setSource(QUrl::fromLocalFile("../assets/sounds/pikiri.wav"));
    effect.setLoopCount(0);
    effect.setVolume(0.6*BUTTON_VOLUME);
  }
  effect.play();
}

void soundButton(int onOff){
  if(onOff == 0){
    soundPo();
  } else {
    soundPipo();
  }
}

void soundButton2(int onOff){
  if(onOff == 0){
    soundPo();
  } else {
    soundPikiri();
  }
}

void copy_manager2tmp(const char*fn_mng , const char*txt_mng , bool first){ //txt_mngはtxt.c_str()を渡す。
  int dir_ofs = 0;
  if(strstr(fn_mng,"../manager/"))dir_ofs = 11;
  else if(strstr(fn_mng,"/data/"))dir_ofs = 6;
  if(dir_ofs > 0){
    char tmpfn[128];
    sprintf(tmpfn,"/tmp/%s",fn_mng + dir_ofs); //11 = strlen("../manager/");
    if(first == true){
      FILE *dst_tmp = fopen(tmpfn,"r");
      if(dst_tmp){
        //初回以外はコピー不要
        fclose(dst_tmp);
        return;
      }
    }
    FILE *dst = fopen(tmpfn,"w");
    if(dst){
      fwrite(txt_mng,strlen(txt_mng),1,dst);
      fclose(dst);
    }
  }
}

bool getButtonEnabled(const char*fn){ //fn="/data/lockon_disp_disable.txt"など、このファイルが無かったらtrueのニュアンスで。
  std::string txt = util::read_file(fn);
  if(txt.empty() == false){
    // ../manager/abc.txtを/tmp/abc.txtにコピーする(pythonでは/tmpから読み込みで高速化を期待する)
    copy_manager2tmp(fn,txt.c_str(),true);
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
    // ../manager/abc.txtを/tmp/abc.txtにコピーする(pythonでは/tmpから読み込みで高速化を期待する)
    copy_manager2tmp(fn,txt.c_str(),true);
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
    // ../manager/abc.txtを/tmp/abc.txtにコピーする(pythonでは/tmpから読み込みで高速化を期待する)
    copy_manager2tmp(fn,txt.c_str(),true);
    return std::stoi(txt);
  }
  return defaultNum; //ファイルがない場合はこれを返す。
}

bool fp_error = false;
void setButtonEnabled(const char*fn , bool flag){ //fn="/data/lockon_disp_disable.txt"など、このファイルが無かったらtrueのニュアンスで。flagは素直にtrueなら有効。
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
    // ../manager/abc.txtを/tmp/abc.txtにコピーする
    copy_manager2tmp(fn,flag ? "0" : "1",false);
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
    // ../manager/abc.txtを/tmp/abc.txtにコピーする
    copy_manager2tmp(fn,flag ? "1" : "0",false);
  } else {
    fp_error = true;
  }
}

void setButtonInt(const char*fn , int num){ //新fn="../manager/accel_engaged.txt"など、このファイルが無かったら0。num(0〜3)はそのまま数字で。
  FILE *fp = fopen(fn,"w"); //write_fileだと書き込めないが、こちらは書き込めた。
  if(fp != NULL){
    fp_error = false;
#if 0
    if(num == 1){
      fwrite("1",1,1,fp);
    } else if(num == 2){
      fwrite("2",1,1,fp);
    } else if(num == 3){
      fwrite("3",1,1,fp);
    } else {
      fwrite("0",1,1,fp);
    }
#else
    char buf[32];
    sprintf(buf,"%d",num);
    fwrite(buf,strlen(buf),1,fp);
#endif
    fclose(fp);
    // ../manager/abc.txt(or /data/abc.txt)を/tmp/abc.txtにコピーする
    copy_manager2tmp(fn,buf,false);
  } else {
    fp_error = true;
  }
}

// ButtonsWindow
const static char *btn_style0 = "font-size: 90px; border-width: 0px; background-color: rgba(0, 0, 0, 0); border-radius: 20px; border-color: %1"; //透明ボタン用
const static char *btn_style = "font-size: 90px; border-radius: 20px; border-color: %1";
bool Long_enable = true;
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
  int forceOnePedalButton_height = 180 * 1.3;
  btns_layoutLL->setContentsMargins(30+15, 430-172 - forceOnePedalButton_height -10, 15, 30);

  btns_layout0L->addWidget(btns_wrapperLL,0,Qt::AlignVCenter);
  {
    //ブレーキで信号停止して、ACCレバーを上げた状態でこのボタンを押すと、
    //OP_ENABLE_v_cruise_kph = v_cruise_kph
    //OP_ENABLE_gas_speed = 1.0 / 3.6
    //強制的にワンペダルモードとなる。
    QPushButton *forceOnePedalButton = new QPushButton(""); //表示文字も無し。
    QObject::connect(forceOnePedalButton, &QPushButton::pressed, [=]() {
      if(getButtonInt("/tmp/accel_engaged.txt" , 0) == 3){ //ワンペダルのみ
        std::string stdstr_txt = util::read_file("/tmp/cruise_info.txt");
        if(stdstr_txt.empty() == false){
          if(stdstr_txt != "1" && stdstr_txt != ",1"){ //MAXが1ではない時
            if((*(uiState()->sm))["carState"].getCarState().getVEgo() < 0.1/3.6){ //スピードが出ていない時
              setButtonEnabled0("/tmp/force_one_pedal.txt" , true); //これがセットされる条件をなるべく絞る。
            } else {
              soundPo(); //操作不能音として鳴らす。
            }
          } else {
            //MAX=1でタッチ(↑ボタン効果で",1"も含む)
            float vego = (*(uiState()->sm))["carState"].getCarState().getVEgo();
            if(vego > 3/3.6 && vego <= 30/3.6){ //スピードが3〜30km/hのとき
              setButtonEnabled0("/tmp/force_low_engage.txt" , true);
            } else {
              soundPo(); //操作不能音として鳴らす。
            }
          }
        }
      }
    });
    int rect_width = 190 * 1.3;
    int rect_height = forceOnePedalButton_height;
    forceOnePedalButton->setFixedWidth(rect_width);
    forceOnePedalButton->setFixedHeight(rect_height);
    //forceOnePedalButton->setWindowOpacity(0.2);
    btns_layoutLL->addSpacing(0);
    btns_layoutLL->addWidget(forceOnePedalButton);
    forceOnePedalButton->setStyleSheet(QString(btn_style0).arg("#909090")); //線の色はダミー。
  }

  {
    // Handle Ctrl button（廃止準備。制限速度標識ボタンに変容予定）
    uiState()->scene.mLimitspeedButton = mLimitspeedButton = getButtonInt("/data/limitspeed_sw.txt",0);
    limitspeedButton = new QPushButton("○"); //この丸がセンターに出る。
    //limitspeedButton = new QPushButton("◯"); //枠内いっぱいに出る大きな丸。
    QObject::connect(limitspeedButton, &QPushButton::pressed, [=]() {
      uiState()->scene.mLimitspeedButton = (mLimitspeedButton + 1) % 2; //0->1->0
    });
    limitspeedButton->setFixedWidth(150);
    limitspeedButton->setFixedHeight(150*0.9);
    //limitspeedButton->setPalette(QColor(255,255,255,all_opac*255));
    //limitspeedButton->setAutoFillBackground(true);
    btns_layoutLL->addSpacing(10);
    btns_layoutLL->addWidget(limitspeedButton);
    limitspeedButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mLimitspeedButton > 0)));
  }

  {
    // use lane button
    uiState()->scene.mUseLaneButton = mUseLaneButton = getButtonInt("/data/lane_sw_mode.txt" , true /*Params().getBool("EndToEndToggle")*/ ? 0 : 1);
    useLaneButton = new QPushButton("dX"); //ダイナミックexperimentalモード
    QObject::connect(useLaneButton, &QPushButton::pressed, [=]() {
      // Params().putBool("EndToEndToggle",!Params().getBool("EndToEndToggle"));
      // uiState()->scene.mUseLaneButton = !Params().getBool("EndToEndToggle");
      // uiState()->scene.end_to_end = Params().getBool("EndToEndToggle");
      //uiState()->scene.mUseLaneButton = (mUseLaneButton + 1) % 4; //0->1->2->3->0
      uiState()->scene.mUseLaneButton = (mUseLaneButton + 1) % 2; //0->1->0
    });
    useLaneButton->setFixedWidth(150);
    useLaneButton->setFixedHeight(150*0.9);
    btns_layoutLL->addSpacing(15);
    btns_layoutLL->addWidget(useLaneButton);
    useLaneButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mUseLaneButton > 0)));
  }

  QWidget *btns_wrapper0U = new QWidget;
  QVBoxLayout *btns_layout0U  = new QVBoxLayout(btns_wrapper0U);
  btns_layout0U->setSpacing(0);
  btns_layout0U->setContentsMargins(0, 430-200-70, 0, 0);
  btns_layout00->addWidget(btns_wrapper0U, 0, Qt::AlignTop);

  {
    QWidget *btns_wrapperUU = new QWidget;
    QHBoxLayout *btns_layoutUU  = new QHBoxLayout(btns_wrapperUU);
    btns_layoutUU->setSpacing(0);
    btns_layoutUU->setContentsMargins(0, 0, 50, 0);
    btns_layout0U->addWidget(btns_wrapperUU, 0, Qt::AlignRight);

    {
      // Long enable 透明button
      QPushButton *LongEnablrButton = new QPushButton(""); //表示文字も無し。
      Long_enable = getButtonEnabled("/data/long_speeddown_disable.txt");
      QObject::connect(LongEnablrButton, &QPushButton::pressed, [=]() {
        Long_enable = !getButtonEnabled("/data/long_speeddown_disable.txt");
        setButtonEnabled("/data/long_speeddown_disable.txt",Long_enable);
        if(Long_enable){
          soundPipo();
        } else {
          soundPo();

          UIState *s = uiState();
          if((*s->sm)["controlsState"].getControlsState().getExperimentalMode()){
            Params().putBool("ExperimentalMode", false);
          } else {
            Params().putBool("ExperimentalMode", true);
          }
        }
      });
      int rect_width = 200;
      int rect_height = 200;
      LongEnablrButton->setFixedWidth(rect_width);
      LongEnablrButton->setFixedHeight(rect_height);
      btns_layoutUU->addSpacing(0);
      btns_layoutUU->addWidget(LongEnablrButton);
      LongEnablrButton->setStyleSheet(QString(btn_style0).arg("#909090")); //線の色はダミー。
      //LongEnablrButton->setStyleSheet(QString(btn_style).arg("#909090")); //線の色はダミー。
    }
  }

  QWidget *btns_wrapper0 = new QWidget;
  QHBoxLayout *btns_layout0  = new QHBoxLayout(btns_wrapper0);
  btns_layout0->setSpacing(0);
  btns_layout0->setContentsMargins(0, 45, 0, 0);
  btns_layout0U->addWidget(btns_wrapper0, 0, Qt::AlignRight);

  QWidget *btns_wrapperL = new QWidget;
  QVBoxLayout *btns_layoutL  = new QVBoxLayout(btns_wrapperL);
  btns_layoutL->setSpacing(0);
  btns_layoutL->setContentsMargins(15, 0, 0, 30);

  btns_layout0->addWidget(btns_wrapperL,0,Qt::AlignVCenter);

  //const float all_opac = 0.2;
  {
    // turbo boost button
    uiState()->scene.mStartAccelPowerUpButton = mStartAccelPowerUpButton = getButtonEnabled0("/data/start_accel_power_up_disp_enable.txt");
    startAccelPowerUpButton = new QPushButton("⇧"); //⬆︎
    QObject::connect(startAccelPowerUpButton, &QPushButton::pressed, [=]() {
      uiState()->scene.mStartAccelPowerUpButton = !mStartAccelPowerUpButton;
    });
    startAccelPowerUpButton->setFixedWidth(150);
    startAccelPowerUpButton->setFixedHeight(150);
    //lockOnButton->setWindowOpacity(all_opac);
    btns_layoutL->addWidget(startAccelPowerUpButton);
    startAccelPowerUpButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mStartAccelPowerUpButton)));
  }

  {
    // LockOn button
    uiState()->scene.mLockOnButton = mLockOnButton = getButtonEnabled("/data/lockon_disp_disable.txt");
    lockOnButton = new QPushButton("□");
    QObject::connect(lockOnButton, &QPushButton::pressed, [=]() {
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
  btns_layout->setContentsMargins(15, 0, 30, 30);

  btns_layout0->addWidget(btns_wrapper,0,Qt::AlignVCenter);

  {
    // Accel Ctrl button
    uiState()->scene.mAccelCtrlButton = mAccelCtrlButton = getButtonEnabled("/data/accel_ctrl_disable.txt");
    accelCtrlButton = new QPushButton("↑");
    QObject::connect(accelCtrlButton, &QPushButton::pressed, [=]() {
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
    uiState()->scene.mDecelCtrlButton = mDecelCtrlButton = getButtonEnabled("/data/decel_ctrl_disable.txt");
    decelCtrlButton = new QPushButton("↓");
    QObject::connect(decelCtrlButton, &QPushButton::pressed, [=]() {
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
    uiState()->scene.mAccelEngagedButton = mAccelEngagedButton = getButtonInt("/data/accel_engaged.txt" , 0);
    if(mAccelEngagedButton == 3){
      accelEngagedButton = new QPushButton("iP"); //3ならイチロウペダル（インテリジェントペダルモード）
    } else if(mAccelEngagedButton == 2){
      accelEngagedButton = new QPushButton("AA"); //2ならAA(ALL ACCEL)
    } else {
      accelEngagedButton = new QPushButton("A");
    }
    QObject::connect(accelEngagedButton, &QPushButton::pressed, [=]() {
      //uiState()->scene.mAccelEngagedButton = !mAccelEngagedButton; //ここを0->1->2・・・にすれば良い
      uiState()->scene.mAccelEngagedButton = (mAccelEngagedButton + 1) % 4; //0->1->2->3->0
      setButtonEnabled0("/tmp/force_one_pedal.txt" , false);
      setButtonEnabled0("/tmp/force_low_engage.txt" , false);
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
    setButtonEnabled("/data/lockon_disp_disable.txt" , mLockOnButton);
    soundButton(mLockOnButton);
  }

  if (mAccelCtrlButton != s.scene.mAccelCtrlButton) {  // update mAccelCtrlButton
    mAccelCtrlButton = s.scene.mAccelCtrlButton;
    accelCtrlButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mAccelCtrlButton && fp_error==false)));
    setButtonEnabled("/data/accel_ctrl_disable.txt" , mAccelCtrlButton);
    soundButton(mAccelCtrlButton);
  }

  if (mDecelCtrlButton != s.scene.mDecelCtrlButton) {  // update mDecelCtrlButton
    mDecelCtrlButton = s.scene.mDecelCtrlButton;
    decelCtrlButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mDecelCtrlButton && fp_error==false)));
    setButtonEnabled("/data/decel_ctrl_disable.txt" , mDecelCtrlButton);
    soundButton(mDecelCtrlButton);
  }

  if (mAccelEngagedButton != s.scene.mAccelEngagedButton) {  // update mAccelEngagedButton
    mAccelEngagedButton = s.scene.mAccelEngagedButton;
    accelEngagedButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mAccelEngagedButton > 0 && fp_error==false)));
    //ここでボタンのラベルを変えられないかな？mAccelEngagedButton == 2でAAとかにしたい。
    if(mAccelEngagedButton == 3){
      accelEngagedButton->setText("iP");
    } else if(mAccelEngagedButton == 2){
      accelEngagedButton->setText("AA");
    } else {
      accelEngagedButton->setText("A");
    }
    setButtonInt("/data/accel_engaged.txt" , mAccelEngagedButton);
    soundButton(mAccelEngagedButton);
  }

  if (mLimitspeedButton != s.scene.mLimitspeedButton) {  // update mLimitspeedButton
    mLimitspeedButton = s.scene.mLimitspeedButton;
    limitspeedButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mLimitspeedButton > 0 && fp_error==false)));
    setButtonInt("/data/limitspeed_sw.txt" , mLimitspeedButton);
    soundButton(mLimitspeedButton);
  }
  
  if (mStartAccelPowerUpButton != s.scene.mStartAccelPowerUpButton) {  // update mStartAccelPowerUpButton
    mStartAccelPowerUpButton = s.scene.mStartAccelPowerUpButton;
    startAccelPowerUpButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mStartAccelPowerUpButton && fp_error==false)));
    setButtonEnabled0("/data/start_accel_power_up_disp_enable.txt" , mStartAccelPowerUpButton);
    soundButton(mStartAccelPowerUpButton);
  }
  
  if (mUseLaneButton != s.scene.mUseLaneButton) {  // update mUseLaneButton
    mUseLaneButton = s.scene.mUseLaneButton;
    useLaneButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mUseLaneButton > 0 && fp_error==false)));
    if(mUseLaneButton >= 1){
      useLaneButton->setText("dX");
    } else {
      useLaneButton->setText("dX"); //どのケースでも"dX"
    }
    setButtonInt("/data/lane_sw_mode.txt" , mUseLaneButton);
    soundButton(mUseLaneButton);
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
    configFont(p, "Inter", 74, "SemiBold");
    p.drawText(r, Qt::AlignCenter, alert.text1);
  } else if (alert.size == cereal::ControlsState::AlertSize::MID) {
    configFont(p, "Inter", 88, "Bold");
    p.drawText(QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1);
    configFont(p, "Inter", 66, "Regular");
    p.drawText(QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2);
  } else if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    configFont(p, "Inter", l ? 132 : 177, "Bold");
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    configFont(p, "Inter", 88, "Regular");
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}


ExperimentalButton::ExperimentalButton(QWidget *parent) : QPushButton(parent) {
  setVisible(false);
  setFixedSize(btn_size, btn_size);
  setCheckable(true);

  params = Params();
  engage_img = loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size});
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size, img_size});

  QObject::connect(this, &QPushButton::toggled, [=](bool checked) {
    params.putBool("ExperimentalMode", checked);
  });
}

void ExperimentalButton::updateState(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  // button is "visible" if engageable or enabled
  const auto cs = sm["controlsState"].getControlsState();
  setVisible(cs.getEngageable() || cs.getEnabled());

  // button is "checked" if experimental mode is enabled
  setChecked(sm["controlsState"].getControlsState().getExperimentalMode());

  // disable button when experimental mode is not available, or has not been confirmed for the first time
  const auto cp = sm["carParams"].getCarParams();
  const bool experimental_mode_available = cp.getExperimentalLongitudinalAvailable() ? params.getBool("ExperimentalLongitudinalEnabled") : cp.getOpenpilotLongitudinalControl();
  setEnabled(params.getBool("ExperimentalModeConfirmed") && experimental_mode_available);
}

void ExperimentalButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  QPoint center(btn_size / 2, btn_size / 2);
  QPixmap img = isChecked() ? experimental_img : engage_img;

  p.setOpacity(1.0);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, 166));
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  p.setOpacity(isDown() ? 0.8 : 1.0);
  p.drawPixmap((btn_size - img_size) / 2, (btn_size - img_size) / 2, img);
}


AnnotatedCameraWidget::AnnotatedCameraWidget(VisionStreamType type, QWidget* parent) : fps_filter(UI_FREQ, 3, 1. / UI_FREQ), CameraWidget("camerad", type, true, parent) {
  pm = std::make_unique<PubMaster, const std::initializer_list<const char *>>({"uiDebug"});

  QVBoxLayout *main_layout  = new QVBoxLayout(this);
/*
  main_layout->setMargin(bdr_s);
  main_layout->setSpacing(0);

  experimental_btn = new ExperimentalButton(this);
  main_layout->addWidget(experimental_btn, 0, Qt::AlignTop | Qt::AlignRight);
*/
  buttons = new ButtonsWindow(this); //ここならばexperimental_btnとイベントの両立ができ、マップの右画面のスクロール操作ができる。->ExperimentalButtonをLayoutで囲むとイベントが先に登録勝ちになってしまう。
  QObject::connect(uiState(), &UIState::uiUpdate, buttons, &ButtonsWindow::updateState);
  main_layout->addWidget(buttons);

  engage_img = loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size});
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size - 5, img_size - 5});
  dm_img = loadPixmap("../assets/img_driver_face.png", {img_size + 5, img_size + 5});
}

static bool global_engageable;
static float vc_speed;
static int tss_type = 0;
static float maxspeed_org;
void AnnotatedCameraWidget::updateState(const UIState &s) {
  int SET_SPEED_NA = 557; //255;
  const SubMaster &sm = *(s.sm);

  const bool cs_alive = sm.alive("controlsState");
  const bool nav_alive = sm.alive("navInstruction") && sm["navInstruction"].getValid();

  const auto cs = sm["controlsState"].getControlsState();

  float maxspeed = cs.getVCruiseCluster() == 0.0 ? cs.getVCruise() : cs.getVCruiseCluster(); //v_cruise->maxspeed
  maxspeed_org = maxspeed; //レバー値の元の値。
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
    //これまでと互換。tss_type_infoがなければTSSP
    maxspeed = maxspeed < (55 - 4) ? (55 - (55 - (maxspeed+4)) * 2 - 4) : maxspeed;
    maxspeed = maxspeed > (110 - 6) ? (110 + ((maxspeed+6) - 110) * 3 - 6) : maxspeed;
  } else if(PI0_DEBUG == true || tss_type == 2){
    SET_SPEED_NA = 255; //TSS2では戻す。
  }

  float set_speed = cs_alive ? maxspeed : SET_SPEED_NA;
  bool cruise_set = set_speed > 0 && (int)set_speed != SET_SPEED_NA;

  if (cruise_set && !s.scene.is_metric) {
    set_speed *= KM_TO_MILE;
  }

  // Handle older routes where vEgoCluster is not set
  float v_ego;
  if (sm["carState"].getCarState().getVEgoCluster() == 0.0 && !v_ego_cluster_seen) {
    v_ego = sm["carState"].getCarState().getVEgo();
  } else {
    v_ego = sm["carState"].getCarState().getVEgoCluster();
    v_ego_cluster_seen = true;
  }
  float cur_speed = cs_alive ? std::max<float>(0.0, v_ego) : 0.0;
  vc_speed = v_ego;
  QString maxspeed_str = cruise_set ? QString::number(std::nearbyint(maxspeed)) : "N/A";
  std::string stdstr_txt = util::read_file("/tmp/cruise_info.txt");
  static std::string stdstr_txt_save;
  if(cruise_set && stdstr_txt.empty() == false){
    QString qstr = QString::fromStdString(stdstr_txt);
    maxspeed_str = qstr;
    stdstr_txt_save = stdstr_txt;
  } else if(cruise_set && stdstr_txt_save.empty() == false){
    QString qstr = QString::fromStdString(stdstr_txt_save);
    maxspeed_str = qstr;
    stdstr_txt_save.clear(); //過去数字の使用は一度限定。
  }
  cur_speed *= s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH;

  auto speed_limit_sign = sm["navInstruction"].getNavInstruction().getSpeedLimitSign();
  float speed_limit = nav_alive ? sm["navInstruction"].getNavInstruction().getSpeedLimit() : 0.0;
  speed_limit *= (s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH);

  setProperty("speedLimit", speed_limit);
  setProperty("has_us_speed_limit", nav_alive && speed_limit_sign == cereal::NavInstruction::SpeedLimitSign::MUTCD);
  setProperty("has_eu_speed_limit", nav_alive && speed_limit_sign == cereal::NavInstruction::SpeedLimitSign::VIENNA);

  setProperty("is_cruise_set", cruise_set);
  setProperty("is_metric", s.scene.is_metric);
  setProperty("speed", cur_speed);
  setProperty("maxSpeed", maxspeed_str);
  setProperty("setSpeed", set_speed);
  setProperty("speedUnit", s.scene.is_metric ? tr("km/h") : tr("mph"));
  setProperty("hideDM", (cs.getAlertSize() != cereal::ControlsState::AlertSize::NONE));
  setProperty("status", s.status);

  // update engageability/experimental mode button
//  experimental_btn->updateState(s);

  // update DM icons at 2Hz
  if (sm.frame % (UI_FREQ / 2) == 0) {
    global_engageable = (cs.getEngageable() || cs.getEnabled());
    setProperty("dmActive", sm["driverMonitoringState"].getDriverMonitoringState().getIsActiveMode());
    setProperty("rightHandDM", sm["driverMonitoringState"].getDriverMonitoringState().getIsRHD());
  }

  // DM icon transition
  dm_fade_state = fmax(0.0, fmin(1.0, dm_fade_state+0.2*(0.5-(float)(dmActive))));
}

static bool all_brake_light = false;
static int global_status;
static float curve_value;
static float handle_center = -100;
static int handle_calibct = 0;
static float distance_traveled;
static float global_angle_steer0 = 0;
static float clipped_brightness0 = 101; //初回ファイルアクセスさせるため、わざと101
static float global_fps;
void AnnotatedCameraWidget::drawHud(QPainter &p) {
  p.save();
  int y_ofs = 150;

  // Header gradient
  QLinearGradient bg(0, header_h - (header_h / 2.5), 0, header_h);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), header_h+y_ofs, bg);

  QString speedLimitStr = (speedLimit > 1) ? QString::number(std::nearbyint(speedLimit)) : "–";
  QString speedStr = QString::number(std::nearbyint(speed));
  //QString setSpeedStr = is_cruise_set ? QString::number(std::nearbyint(setSpeed)) : "–";

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

  // Draw outer box + border to contain set speed and speed limit
  int default_rect_width = 172 * max_disp_k;
  int rect_width = default_rect_width;
  if (is_metric || has_eu_speed_limit) rect_width = 200 * max_disp_k;
  if (has_us_speed_limit && speedLimitStr.size() >= 3) rect_width = 223 * max_disp_k;

  int rect_height = 204 * max_disp_k;
  if (has_us_speed_limit) rect_height = 402 * max_disp_k;
  else if (has_eu_speed_limit) rect_height = 392 * max_disp_k;

  int top_radius = 32 * max_disp_k;
  int bottom_radius = (has_eu_speed_limit ? 100 : 32) * max_disp_k;

  QRect set_speed_rect(60 + default_rect_width / 2 - rect_width / 2, 45 +y_ofs, rect_width, rect_height);
//  p.setPen(QPen(whiteColor(75), 6));
  QString ms = QString(maxSpeed);
  bool lemit_speed_override = false;
  if(ms.length() > 1){
    if(maxSpeed.mid(0,1) == ";"){ //先頭セミコロンで制限速度適用
      ms = maxSpeed.mid(1,maxSpeed.length()-1);
      //p.setPen(QPen(QColor(205, 44, 38, 255), 12)); //標識の赤枠の色に合わせる
      p.setPen(QPen(QColor(0xff, 0xff, 0xff, 255*0.9), 6)); //枠を白
      lemit_speed_override = true;
    } else if(maxSpeed.mid(0,1) == ","){ //先頭カンマで加速
      ms = maxSpeed.mid(1,maxSpeed.length()-1);
      p.setPen(QPen(QColor(0, 0xff, 0, 200), 6)); //加速時は緑
    } else if(maxSpeed.mid(maxSpeed.length()-1,1) == "."){ //末尾ピリオドで減速
      ms = maxSpeed.mid(0,maxSpeed.length()-1);
      p.setPen(QPen(QColor(0xff, 0, 0, 200), 6)); //減速時は赤
    } else if(maxSpeed.mid(maxSpeed.length()-1,1) == ";"){ //末尾セミコロンで黄色
      ms = maxSpeed.mid(0,maxSpeed.length()-1);
      p.setPen(QPen(QColor(0xff, 0xff, 0, 200), 6)); //黄色
    } else {
      p.setPen(QPen(QColor(0xff, 0xff, 0xff, 100), 6));
    }
  } else {
    p.setPen(QPen(QColor(0xff, 0xff, 0xff, 100), 6));
  }
  if(lemit_speed_override == false){
    p.setBrush(blackColor(166));
  } else {
    if(maxspeed_org+10 > ms.toDouble()){
      p.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, 0.9)); //速度標識の地の色に合わせる。
    } else {
      static unsigned int yellow_flash_ct = 0;
      yellow_flash_ct ++;
      if(yellow_flash_ct %6 < 3){
        p.setBrush(QColor::fromRgbF(1.0, 1.0, 0, 1.0)); //速度がレバーより10km/h以上高いとギクシャクする警告、点滅させる。
      } else {
        p.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, 0.9)); //速度標識の地の色に合わせる。
      }
    }
  }
  drawRoundedRect(p, set_speed_rect, top_radius, top_radius, bottom_radius, bottom_radius);
  extern int limit_speed_auto_detect;
  if(lemit_speed_override == true || (uiState()->scene.mLimitspeedButton == 1 && limit_speed_auto_detect == 1)){
    //太い赤枠を内側に描画する。
    const int ls_w2 = 30;
    QRect set_speed_rect2(60 + default_rect_width / 2 - rect_width / 2 +ls_w2/2, 45 +y_ofs +ls_w2/2, rect_width - ls_w2, rect_height -ls_w2);
    p.setPen(QPen(QColor(205, 44, 38, (lemit_speed_override ? 255 : 180)), ls_w2)); //標識の赤枠の色に合わせる
    p.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, 0)); //２０描画はしない
    drawRoundedRect(p, set_speed_rect2, top_radius-ls_w2/2, top_radius-ls_w2/2, bottom_radius-ls_w2/2, bottom_radius-ls_w2/2);
  }

  QString setSpeedStr = is_cruise_set ? ms : "–";

  // Draw MAX
  if(lemit_speed_override == true){
    p.setPen(QColor(0x24, 0x57, 0xa1 , 255)); //速度標識の数字に合わせる。
  } else if (is_cruise_set) {
    if (status == STATUS_DISENGAGED) {
      p.setPen(whiteColor());
    } else if (status == STATUS_OVERRIDE) {
      p.setPen(QColor(0x91, 0x9b, 0x95, 0xff));
    } else if (speedLimit > 0) {
      p.setPen(interpColor(
        setSpeed,
        {speedLimit + 5, speedLimit + 15, speedLimit + 25},
        {QColor(0x80, 0xd8, 0xa6, 0xff), QColor(0xff, 0xe4, 0xbf, 0xff), QColor(0xff, 0xbf, 0xbf, 0xff)}
      ));
    } else {
      p.setPen(QColor(0x80, 0xd8, 0xa6, 0xff));
    }
  } else {
    p.setPen(QColor(0xa6, 0xa6, 0xa6, 0xff));
  }
  configFont(p, "Inter", 40*max_disp_k, "SemiBold");
  QString MAX_AUTO = lemit_speed_override == false ? tr("MAX") : tr("AUTO");
  QRect max_rect = getTextRect(p, Qt::AlignCenter, MAX_AUTO);
  max_rect.moveCenter({set_speed_rect.center().x(), 0});
  max_rect.moveTop(set_speed_rect.top() + 27*max_disp_k);
  p.drawText(max_rect, Qt::AlignCenter, MAX_AUTO);
  // Draw set speed
  if (is_cruise_set) {
    if (speedLimit > 0 && status != STATUS_DISENGAGED && status != STATUS_OVERRIDE) {
      p.setPen(interpColor(
        setSpeed,
        {speedLimit + 5, speedLimit + 15, speedLimit + 25},
        {whiteColor(), QColor(0xff, 0x95, 0x00, 0xff), QColor(0xff, 0x00, 0x00, 0xff)}
      ));
    } else {
      p.setPen(whiteColor());
    }
  } else {
    p.setPen(QColor(0x72, 0x72, 0x72, 0xff));
  }
  configFont(p, "Inter", 90*max_disp_k, "Bold");
  QRect speed_rect = getTextRect(p, Qt::AlignCenter, setSpeedStr);
  speed_rect.moveCenter({set_speed_rect.center().x(), 0});
  speed_rect.moveTop(set_speed_rect.top() + 77*max_disp_k);

  static int red_signal_scan_flag = 0;
  static unsigned int red_signal_scan_flag_txt_ct = 0;
  if(red_signal_scan_flag_txt_ct % 7 == 0){
    std::string red_signal_scan_flag_txt = util::read_file("/tmp/red_signal_scan_flag.txt");
    if(red_signal_scan_flag_txt.empty() == false){
      if(uiState()->scene.mAccelEngagedButton == 3){
        red_signal_scan_flag = std::stoi(red_signal_scan_flag_txt);
      } else {
        red_signal_scan_flag = 0;
      }
    }
  }
  red_signal_scan_flag_txt_ct ++;

  static long long int night_mode_ct;
  if((red_signal_scan_flag >= 2 && (night_mode_ct ++) % 11 == 0) || red_signal_scan_flag_txt_ct % 200 == 1 /*10秒に一回は更新*/){
    //static int night_mode = -1;
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
        //bool night = clipped_brightness < 50; //どのくらいが妥当？
        //bool night = clipped_brightness < (night_mode == -1 ? 50 : (night_mode == 1 ? 60 : 40)); //ばたつかないようにする。
        //setButtonEnabled0("../manager/night_time_info.txt" , night);
        setButtonInt("/tmp/night_time_info.txt" , (int)clipped_brightness);
      }
    }
  }

  if(red_signal_scan_flag >= 3){
    QColor speed_max;
    speed_max = QColor(0xff, 0, 0 , 255);
    p.setPen(speed_max);
  } else if(lemit_speed_override == true){
    QColor speed_max;
    speed_max = QColor(0x24, 0x57, 0xa1 , 255); //速度標識の数字に合わせる。
    p.setPen(speed_max);
  }
  p.drawText(speed_rect, Qt::AlignCenter, setSpeedStr);

  // US/Canada (MUTCD style) sign
  if (has_us_speed_limit) {
    //ケアしていない
    const int border_width = 6;
    const int sign_width = rect_width - 24;
    const int sign_height = 186;

    // White outer square
    QRect sign_rect_outer(set_speed_rect.left() + 12, set_speed_rect.bottom() - 11 - sign_height, sign_width, sign_height);
    p.setPen(Qt::NoPen);
    p.setBrush(whiteColor());
    p.drawRoundedRect(sign_rect_outer, 24, 24);

    // Smaller white square with black border
    QRect sign_rect(sign_rect_outer.left() + 1.5 * border_width, sign_rect_outer.top() + 1.5 * border_width, sign_width - 3 * border_width, sign_height - 3 * border_width);
    p.setPen(QPen(blackColor(), border_width));
    p.setBrush(whiteColor());
    p.drawRoundedRect(sign_rect, 16, 16);

    // "SPEED"
    configFont(p, "Inter", 28, "SemiBold");
    QRect text_speed_rect = getTextRect(p, Qt::AlignCenter, tr("SPEED"));
    text_speed_rect.moveCenter({sign_rect.center().x(), 0});
    text_speed_rect.moveTop(sign_rect_outer.top() + 22);
    p.drawText(text_speed_rect, Qt::AlignCenter, tr("SPEED"));

    // "LIMIT"
    QRect text_limit_rect = getTextRect(p, Qt::AlignCenter, tr("LIMIT"));
    text_limit_rect.moveCenter({sign_rect.center().x(), 0});
    text_limit_rect.moveTop(sign_rect_outer.top() + 51);
    p.drawText(text_limit_rect, Qt::AlignCenter, tr("LIMIT"));

    // Speed limit value
    configFont(p, "Inter", 70, "Bold");
    QRect speed_limit_rect = getTextRect(p, Qt::AlignCenter, speedLimitStr);
    speed_limit_rect.moveCenter({sign_rect.center().x(), 0});
    speed_limit_rect.moveTop(sign_rect_outer.top() + 85);
    p.drawText(speed_limit_rect, Qt::AlignCenter, speedLimitStr);
  }

  // EU (Vienna style) sign
  if (has_eu_speed_limit) {
    //ケアしていない
    int outer_radius = 176 / 2;
    int inner_radius_1 = outer_radius - 6; // White outer border
    int inner_radius_2 = inner_radius_1 - 20; // Red circle

    // Draw white circle with red border
    QPoint center(set_speed_rect.center().x() + 1, set_speed_rect.top() + 204 + outer_radius);
    p.setPen(Qt::NoPen);
    p.setBrush(whiteColor());
    p.drawEllipse(center, outer_radius, outer_radius);
    p.setBrush(QColor(255, 0, 0, 255));
    p.drawEllipse(center, inner_radius_1, inner_radius_1);
    p.setBrush(whiteColor());
    p.drawEllipse(center, inner_radius_2, inner_radius_2);

    // Speed limit value
    int font_size = (speedLimitStr.size() >= 3) ? 60 : 70;
    configFont(p, "Inter", font_size, "Bold");
    QRect speed_limit_rect = getTextRect(p, Qt::AlignCenter, speedLimitStr);
    speed_limit_rect.moveCenter(center);
    p.setPen(blackColor());
    p.drawText(speed_limit_rect, Qt::AlignCenter, speedLimitStr);
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
  configFont(p, "Inter", 176, "Bold");
  drawText(p, rect().center().x()-7, 210+y_ofs-5, speedStr,speed_waku);
  drawText(p, rect().center().x()+7, 210+y_ofs-5, speedStr,speed_waku);
  drawText(p, rect().center().x(), -7+210+y_ofs-5, speedStr,speed_waku);
  drawText(p, rect().center().x(), +7+210+y_ofs-5, speedStr,speed_waku);
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
  }
  drawText(p, rect().center().x(), 210 + y_ofs-5, speedStr , speed_num);

  configFont(p, "Inter", 66, "Regular");
  if (uiState()->scene.longitudinal_control == false) {
    drawText(p, rect().center().x(), 290 + y_ofs-5, speedUnit, bg_colors[STATUS_WARNING]); //縦制御無効状態でkm/hを警告色に。
  } else {
    drawText(p, rect().center().x(), 290 + y_ofs-5, speedUnit, 200);
  }

//以下オリジナル表示要素
  //温度を表示(この画面は更新が飛び飛びになる。ハンドル回したりとか何か変化が必要)
  UIState *s = uiState();
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
  //QString temp_disp1 = QString(okConnect ? "⚫︎" : "⚪︎");
  QString temp_disp1 = QString(okConnect ? "●" : "○");
  QString temp_disp2 = QString(okGps ? "★" : "☆");
  QString temp_disp3 = QString::number(temp) + "°C";
  //QString temp_disp = QString(okConnect ? "⚫︎ " : "⚪︎ ") + QString(okGps ? "◆ " : "◇ ") + QString::number(temp) + "°C";
#endif
  configFont(p, FONT_OPEN_SANS, 44, "SemiBold");

  int th_tmp1 = 47;
  int th_tmp2 = 55;
  if(Hardware::TICI()){
    th_tmp1 = 58;
    th_tmp2 = 64;
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
  configFont(p, FONT_OPEN_SANS, 54, "Bold");
  p.drawText(QRect(rect().left()+65+55+5-5, rect().top()+110-8+9, 300, 65), Qt::AlignTop | Qt::AlignLeft, temp_disp2);
  configFont(p, FONT_OPEN_SANS, 48, "Regular");
  p.drawText(QRect(rect().left()+65+5+5, rect().top()+110-8+11, 300, 65+5), Qt::AlignTop | Qt::AlignLeft, temp_disp1);

  if((float)rect_w / rect_h > 1.4f){
    configFont(p, FONT_OPEN_SANS, 44, "SemiBold");
    drawText(p, rect().left()+260, 55, "Powered by COMMA.AI", 150);
    configFont(p, FONT_OPEN_SANS, 55, "SemiBold");
    if(tss_type <= 1){
      drawText(p, rect().right()-270, 60, "for prius PHV 2017", 150);
    } else {
      drawText(p, rect().right()-270, 60, "for prius PHV 2021", 150);
    }
  } else   if((float)rect_w / rect_h > 1.1f){
    configFont(p, FONT_OPEN_SANS, 44, "SemiBold");
    drawText(p, rect().left()+140, 55, "COMMA.AI", 150);
    configFont(p, FONT_OPEN_SANS, 55, "SemiBold");
    if(tss_type <= 1){
      drawText(p, rect().right()-150, 60, "PHV 2017", 150);
    } else {
      drawText(p, rect().right()-150, 60, "PHV 2021", 150);
    }
  }
  configFont(p, FONT_OPEN_SANS, 33, "SemiBold");
  drawText(p, rect().right()-275, rect().bottom() - 10 , "modified by PROGRAMAN ICHIRO", 150);
  configFont(p, FONT_OPEN_SANS, 33, "Bold");
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
      extern int limit_speed_auto_detect;
      if(/*uiState()->scene.mLimitspeedButton == 1 &&*/ limit_speed_auto_detect == 1){ //インジケーターはACC自動設定時にするか、速度標識表示時にするか検討中
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
  
  bool brake_light = false; //ブレーキランプは無くなった？(*(uiState()->sm))["carState"].getCarState().getBrakeLightsDEPRECATED();
  all_brake_light = false;
  std::string brake_light_txt = util::read_file("/tmp/brake_light_state.txt");
  if(brake_light_txt.empty() == false){
    if(std::stoi(brake_light_txt) != 0){
      if(global_engageable){
        brake_light = true;
      }
      all_brake_light = true; //こちらはエンゲージしていなくてもセットされる。
    }
  }
  drawText(p, rect().center().x(), 50 + 40*0 , "extra cruise speed engagement", a0 , brake_light);
  drawText(p, rect().center().x(), 50 + 40*1 , "slow down corner correctly", a1 , brake_light);
  drawText(p, rect().center().x(), 50 + 40*2 , "speed limit auto detect", a2 , brake_light);
  //drawText(p, rect().center().x(), 50 + 40*2 , "curvature reinforcement", a2 , brake_light);
  //drawText(p, rect().center().x(), 50 + 40*2 , QString::number(angle_steer), a2 , brake_light);
  drawText(p, rect().center().x(), 50 + 40*3 , "auto brake holding", a3 , brake_light);

  // engage-ability icon
  if (global_engageable) {
    SubMaster &sm = *(uiState()->sm);
    QBrush bg_color = bg_colors[status];
    if(uiState()->scene.mAccelEngagedButton == 3 && fabs(global_angle_steer0) >= 50 && (*(uiState()->sm))["carState"].getCarState().getVEgo() <= 0.01/3.6){
      //停止時の青信号発進抑制、一時的に緩和、15->50度
      bg_color = bg_colors[STATUS_WARNING]; //ワンペダル時に信号スタート可能角度でなければ警告色。
    }
    drawIcon(p, rect().right() - btn_size / 2 - bdr_s * 2, btn_size / 2 + int(bdr_s * 1.5)+y_ofs,
             //engage_img, bg_color, 1.0 , -global_angle_steer0);
             sm["controlsState"].getControlsState().getExperimentalMode() ? experimental_img : engage_img, blackColor(166), 1.0 , -global_angle_steer0);
  }
  const float x_Long_enable = rect().right() - btn_size / 2 - bdr_s * 2;
  const float y_Long_enable = btn_size / 2 + int(bdr_s * 1.5)+y_ofs;
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
    QPen pen = QPen(QColor(255, 255, ((*s->sm)["controlsState"].getControlsState().getExperimentalMode() || s->scene.mUseLaneButton == 3) ? 0 : 255, 180), abs(arc_w));
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
    QPen pen = QPen(QColor(255, 255, ((*s->sm)["controlsState"].getControlsState().getExperimentalMode() || s->scene.mUseLaneButton == 3) ? 0 : 255, 180), abs(arc_w_base));
    pen.setCapStyle(Qt::FlatCap); //端をフラットに
    p.setPen(pen);
    p.drawArc(x - btn_size / 2 -arc_w_base/2, y - btn_size / 2 -arc_w_base/2, btn_size+arc_w_base, btn_size+arc_w_base, (-90-(long_base_angle))*16, ((long_base_angle)*2)*16);
  }

  //キャリブレーション値の表示。dm iconより先にやらないと透明度が連動してしまう。
  p.setPen(QPen(QColor(0xff, 0xff, 0xff, 0), 0));
  //int calib_h = radius;
  int calib_h = -33 -33 - 30; //表示位置を上に
  QRect rc2(rect().right() - btn_size / 2 - bdr_s * 2 - 100, -20 + btn_size / 2 + int(bdr_s * 1.5)+y_ofs + calib_h -36, 200, 36);
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

    configFont(p, FONT_OPEN_SANS, 60, "Bold");
    drawText(p, rc3.x()+rc3.width()/2 , rc3.y() + rc3.height() - 12, h_ang , 200);
  } else if(/*engageable ||*/ handle_center > -99){
    //ハンドルセンター値を表示
    p.setBrush(bg_colors[status]);
    p.drawRoundedRect(rc2, 18, 18);
    p.setPen(Qt::NoPen);

    //float hc = -4.73;
    float hc = handle_center;

    configFont(p, FONT_OPEN_SANS, 33, "Bold");
    drawText(p, rect().right() - btn_size / 2 - bdr_s * 2 , -20 + btn_size / 2 + int(bdr_s * 1.5)+y_ofs + calib_h - 8, QString::number(hc,'f',2) + "deg", 200);
  } else {
    p.setBrush(QColor(150, 150, 0, 0xf1));
    p.drawRoundedRect(rc2, 18, 18);
    p.setPen(Qt::NoPen);

    if(handle_calibct == 0){
      configFont(p, FONT_OPEN_SANS, 33, "Regular");
      drawText(p, rect().right() - btn_size / 2 - bdr_s * 2 , -20 + btn_size / 2 + int(bdr_s * 1.5)+y_ofs + calib_h - 8, "Calibrating", 200);
    } else {
      configFont(p, FONT_OPEN_SANS, 33, "Bold");
      drawText(p, rect().right() - btn_size / 2 - bdr_s * 2 , -20 + btn_size / 2 + int(bdr_s * 1.5)+y_ofs + calib_h - 6, QString::number(handle_calibct) + '%', 200);
    }
  }
  
  p.restore();
}

// Window that shows camera view and variety of
// info drawn on top

void AnnotatedCameraWidget::drawText(QPainter &p, int x, int y, const QString &text, int alpha , bool brakeLight) {
  QRect real_rect = getTextRect(p, 0, text);
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
}

void AnnotatedCameraWidget::drawText(QPainter &p, int x, int y, const QString &text, const QColor &col) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(col);
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void AnnotatedCameraWidget::drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity , float ang) {
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

  //uiState()->scene.end_to_end = Params().getBool("EndToEndToggle");
  const UIScene &scene = s->scene;
  SubMaster &sm = *(s->sm);

  // lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    painter.drawPolygon(scene.lane_line_vertices[i]);
  }

  // road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 0, 0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0)));
    painter.drawPolygon(scene.road_edge_vertices[i]);
  }

  // paint path
  QLinearGradient bg(0, height(), 0, height() / 4);
  float start_hue, end_hue;
  if (sm["controlsState"].getControlsState().getExperimentalMode()) {
    const auto &acceleration = sm["modelV2"].getModelV2().getAcceleration();
    float acceleration_future = 0;
    if (acceleration.getZ().size() > 16) {
      acceleration_future = acceleration.getX()[16];  // 2.5 seconds
    }
    start_hue = 60;
    // speed up: 120, slow down: 0
    end_hue = fmax(fmin(start_hue + acceleration_future * 45, 148), 0);

    // FIXME: painter.drawPolygon can be slow if hue is not rounded
    end_hue = int(end_hue * 100 + 0.5) / 100;

    bg.setColorAt(0.0, QColor::fromHslF(start_hue / 360., 0.97, 0.56, 0.4));
    bg.setColorAt(0.5, QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.0));
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

void AnnotatedCameraWidget::drawDriverState(QPainter &painter, const UIState *s) {
  const UIScene &scene = s->scene;

  painter.save();

  // base icon
  int x = false /*rightHandDM*/ ? rect().right() -  (btn_size - 24) / 2 - (bdr_s * 2) : (btn_size - 24) / 2 + (bdr_s * 2);
  int y = rect().bottom() - footer_h / 2;
  float opacity = dmActive ? 0.65 : 0.2;
  drawIcon(painter, x, y, dm_img, blackColor(0), opacity , 0);
  if(rightHandDM){ //ボタンを移動できないので、アイコンはそのまま、左肩に"R"を表示。
    configFont(painter, FONT_OPEN_SANS, 70, "Bold");
    drawText(painter, x - btn_size / 2, y - btn_size / 4, "R" , dmActive ? 200 : 100);
  }

  // circle background
  painter.setOpacity(1.0);
  painter.setPen(Qt::NoPen);
  painter.setBrush(blackColor(70));
  painter.drawEllipse(x - btn_size / 2, y - btn_size / 2, btn_size, btn_size);

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

  painter.restore();
}

void AnnotatedCameraWidget::knightScanner(QPainter &p) {

  static const int ct_n = 1;
  static float ct;

#if 1
  std::string signal_start_prompt_info_txt = util::read_file("/tmp/signal_start_prompt_info.txt");
  if(signal_start_prompt_info_txt.empty() == false){
    int pr = std::stoi(signal_start_prompt_info_txt);
    if(pr == 1){
      static QSoundEffect effect;
      static bool once = false;
      if(once == false){
        once = true;
        effect.setSource(QUrl::fromLocalFile("../assets/sounds/prompt.wav"));
        //effect.setLoopCount(QSoundEffect::Infinite);
        effect.setLoopCount(0);
        effect.setVolume(0.7);
      }
      effect.play();
      setButtonEnabled0("/tmp/signal_start_prompt_info.txt" , false);
    } else if(pr == 2){ //自動発進とワンペダル->オートパイロットはこちら。
      static QSoundEffect effect;
      static bool once = false;
      if(once == false){
        once = true;
        effect.setSource(QUrl::fromLocalFile("../assets/sounds/engage.wav"));
        //effect.setLoopCount(QSoundEffect::Infinite);
        effect.setLoopCount(0);
        effect.setVolume(0.7);
      }
      effect.play();
      setButtonEnabled0("/tmp/signal_start_prompt_info.txt" , false);
    } else if(pr == 3){ //デバッグ用。
      static QSoundEffect effect;
      static bool once = false;
      if(once == false){
        once = true;
        effect.setSource(QUrl::fromLocalFile("../assets/sounds/po.wav"));
        //effect.setLoopCount(QSoundEffect::Infinite);
        effect.setLoopCount(0);
        effect.setVolume(1.0);
      }
      effect.play();
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
#else
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
    p.setBrush(QColor(0.09*255, 0.945*255, 0.26*255, 200));
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
    float h = rect_h * curvature / (tss_type < 2 ? 0.03 : 0.05);
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
      float hhy = rect_h * yy / (tss_type < 2 ? 0.03 : 0.05);
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
  configFont(p, FONT_OPEN_SANS, 44, "SemiBold");
  p.setPen(QColor(0xdf, 0xdf, 0x00 , 200));
  {
    //float vegostopping = (*s->sm)["carParams"].getCarParams().getVEgoStopping();
    //QString debug_disp = QString("Stop:") + QString::number(vegostopping,'f',0);
    QString debug_disp = QString("↓:") + QString::number(cv,'f',0);
    p.drawText(QRect(0+20, rect_h - 46, 130, 46), Qt::AlignBottom | Qt::AlignLeft, debug_disp);
  }
  if(0){
    QString debug_disp = QString(",Fps:") + QString::number(global_fps,'f',1);
    p.drawText(QRect(0+20 + 130, rect_h - 46, 210, 46), Qt::AlignBottom | Qt::AlignLeft, debug_disp);
  } else if(0){
    //自立運転時間の割合
    static uint64_t manual_ct = 1 , autopilot_ct;
    if(status == STATUS_DISENGAGED || status == STATUS_OVERRIDE || status == STATUS_ALERT){
      if(vc_speed > 0.1/3.6){
        manual_ct ++; //手動運転中 , 停車時は除く。
      }
    } else {
      autopilot_ct ++; //オートパイロット中（ハンドル、アクセル操作時は含めない , 停車時は自動運転停車として含める）
    }
    double mar = (autopilot_ct * 100) / (autopilot_ct + manual_ct); //manual auto rate
    QString debug_disp = QString(",Amr:") + QString::number((int)mar) + "%";
    p.drawText(QRect(0+20 + 130, rect_h - 46, 210, 46), Qt::AlignBottom | Qt::AlignLeft, debug_disp);
  } else {
    //自立運転距離の割合
    static uint64_t manual_ct = 1 , autopilot_ct; //参考に時間での割合も計算する。
    static double manual_dist = 0.001 , autopilot_dist , before_distance_traveled;
    static double h_manual_dist = 0.001 , h_autopilot_dist; //停止時間は1秒を1m換算でカウントする。
    double now_dist = distance_traveled - before_distance_traveled;
    before_distance_traveled = distance_traveled;
    if(status == STATUS_DISENGAGED || status == STATUS_OVERRIDE || status == STATUS_ALERT){
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
    QString debug_disp = QString(",Amr:") + QString::number((int)ahr) + "%";
    p.drawText(QRect(0+20 + 130, rect_h - 46, 210, 46), Qt::AlignBottom | Qt::AlignLeft, debug_disp);
    // FILE *fp = fopen("/tmp/autopilot_rate.txt","w");
    // if(fp != NULL){
    //   fprintf(fp,"H:%.0f+%.0f=%.0fh %.2f%%\n",h_autopilot_dist,h_manual_dist,h_autopilot_dist + h_manual_dist,ahr);
    //   fprintf(fp,"D:%.0f+%.0f=%.0fm %.2f%%\n",autopilot_dist,manual_dist,distance_traveled,adr);
    //   fprintf(fp,"T:%ld+%ld=%lds %.2f%%",autopilot_ct/20,manual_ct/20,(autopilot_ct+manual_ct)/20,atr);
    //   fclose(fp);
    // }
  }
  {
    QString debug_disp = QString(",Trip:") + QString::number(distance_traveled / 1000,'f',1) + QString("km");
    p.drawText(QRect(0+20 + 130 + 210, rect_h - 46, 290, 46), Qt::AlignBottom | Qt::AlignLeft, debug_disp);
  }
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
    configFont(painter, FONT_OPEN_SANS, 44, "SemiBold");
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

  configFont(painter, FONT_OPEN_SANS, 38, "SemiBold");
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
  const cereal::RadarState::Reader &radar_state = sm["radarState"].getRadarState();

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

  if (s->worldObjectsVisible()) {
    if (sm.rcv_frame("modelV2") > s->scene.started_frame) {
      update_model(s, sm["modelV2"].getModelV2(), sm["uiPlan"].getUiPlan());
      if (sm.rcv_frame("radarState") > s->scene.started_frame) {
        update_leads(s, radar_state, sm["modelV2"].getModelV2().getPosition());
      }
    }

    drawLaneLines(painter, s);

    if (s->scene.longitudinal_control) {
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
  if (!hideDM && (sm.rcv_frame("driverStateV2") > s->scene.started_frame)) {
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
