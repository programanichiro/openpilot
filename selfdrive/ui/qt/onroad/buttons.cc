#include "selfdrive/ui/qt/onroad/buttons.h"

#include <QPainter>
#include <QVBoxLayout>

#include "selfdrive/ui/qt/util.h"

void drawIcon(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity) {
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);  // bg dictates opacity of ellipse
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  p.setOpacity(opacity);
  p.drawPixmap(center - QPoint(img.width() / 2, img.height() / 2), img);
  p.setOpacity(1.0);
}

//ButtonsWindow
//const float BUTTON_VOLUME = 0.35; //setVolumeが効いてないかも。
void setButtonInt(const char*fn , int num);
void soundPo(){
  setButtonInt("/tmp/sound_py_request.txt" , 101); //po.wav
}

void soundPipo(){
  setButtonInt("/tmp/sound_py_request.txt" , 102); //pipo.wav
}

void soundPikiri(){
  setButtonInt("/tmp/sound_py_request.txt" , 103); //pikiri.wav
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
  if(strstr(fn_mng,"/data/"))dir_ofs = 6;
  if(dir_ofs > 0){
    char tmpfn[128];
    sprintf(tmpfn,"/tmp/%s",fn_mng + dir_ofs);
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
    // /data/abc.txtを/tmp/abc.txtにコピーする(pythonでは/tmpから読み込みで高速化を期待する)
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

bool getButtonEnabled0(const char*fn){ //旧fn="/data/accel_engaged.txt"など、このファイルが無かったらfalseのニュアンスで。
  std::string txt = util::read_file(fn);
  if(txt.empty() == false){
    // /data/abc.txtを/tmp/abc.txtにコピーする(pythonでは/tmpから読み込みで高速化を期待する)
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

int getButtonInt(const char*fn , int defaultNum){ //新fn="/data/accel_engaged.txt"など、このファイルが無かったらdefaultNum。あとは数字に変換してそのまま返す。
  std::string txt = util::read_file(fn);
  if(txt.empty() == false){
    // /data/abc.txtを/tmp/abc.txtにコピーする(pythonでは/tmpから読み込みで高速化を期待する)
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
    // /data/abc.txtを/tmp/abc.txtにコピーする
    copy_manager2tmp(fn,flag ? "0" : "1",false);
  } else {
    fp_error = true;
  }
}

void setButtonEnabled0(const char*fn , bool flag){ //旧fn="/data/accel_engaged.txt"など、このファイルが無かったらfalseのニュアンスで。flagはそのままtrueなら有効。
  FILE *fp = fopen(fn,"w"); //write_fileだと書き込めないが、こちらは書き込めた。
  if(fp != NULL){
    fp_error = false;
    if(flag == true){
      fwrite("1",1,1,fp);
    } else {
      fwrite("0",1,1,fp);
    }
    fclose(fp);
    // /data/abc.txtを/tmp/abc.txtにコピーする
    copy_manager2tmp(fn,flag ? "1" : "0",false);
  } else {
    fp_error = true;
  }
}

void setButtonInt(const char*fn , int num){ //新fn="/data/accel_engaged.txt"など、このファイルが無かったら0。num(0〜3)はそのまま数字で。
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
    // /data/abc.txt(or /data/abc.txt)を/tmp/abc.txtにコピーする
    copy_manager2tmp(fn,buf,false);
  } else {
    fp_error = true;
  }
}

// ButtonsWindow
const static char *btn_style0 = "font-size: 90px; border-width: 0px; background-color: rgba(0, 0, 0, 0); border-radius: 20px; border-color: %1"; //透明ボタン用
//const static char *btn_style = "font-size: 90px; border-radius: 20px; border-color: %1";
const static char *btn_style = "font-size: 90px; border-radius: 20px; border-color: rgba(75, 75, 75, 0.0); color: %1"; //枠なしのボタン色変更
const static char *btn_styleb = "font-size: 35px; border-width: 0px; color: rgba(255, 255, 255, 128); background-color: rgba(0, 0, 0, 0); border-radius: 20px; border-color: %1"; //透明ボタン用
const static char *btn_styleb1 = "font-size: 30px; border-width: 0px; color: rgba(255, 255, 255, 128); background-color: rgba(0, 0, 0, 0); border-radius: 20px; border-color: %1"; //透明ボタン用
const static char *btn_styleb2 = "font-size: 50px; border-width: 0px; color: rgba(255, 255, 255, 128); background-color: rgba(0, 0, 0, 0); border-radius: 20px; border-color: %1"; //透明ボタン用
const static char *btn_styleb3 = "font-size: 40px; border-width: 0px; color: rgba(255, 255, 255, 128); background-color: rgba(0, 0, 0, 0); border-radius: 20px; border-color: %1"; //透明ボタン用
bool Long_enable = true;
int Knight_scanner = 7;
int DrivingPsn = 0; //運転傾向
int Limit_speed_mode = 0; //標識
ButtonsWindow::ButtonsWindow(QWidget *parent , MapSettingsButton *map_settings_btn) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setContentsMargins(0, 0, 0, 0);

  QWidget *btns_wrapper00 = new QWidget;
  QHBoxLayout *btns_layout00  = new QHBoxLayout(btns_wrapper00);
  btns_layout00->setSpacing(0);
  main_layout->addWidget(btns_wrapper00, 0, 0); //Alignは何も指定しない。

  int bottom_btns = 0;
  if(true){ //最下段ボタン
    bottom_btns = 1;
    QWidget *btns_wrapperBB = new QWidget;
    QHBoxLayout *btns_layoutBB  = new QHBoxLayout(btns_wrapperBB);
    btns_layoutBB->setSpacing(0);
    btns_layoutBB->setContentsMargins(0, 0, 0, 0);
    main_layout->addWidget(btns_wrapperBB, 0, 0); //Alignは何も指定しない。
    {
        QSpacerItem *spacerItem = new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Preferred);
        btns_layoutBB->addSpacerItem(spacerItem);
    }
    { //制限速度標識ボタン
      T1_Button = new QPushButton("⚪︎"); //"⚫︎⚪︎⬇︎"
      Limit_speed_mode = getButtonInt("/data/limitspeed_sw.txt",0);
      if(Limit_speed_mode == 1){
        T1_Button->setText("⚫︎"); //自動設定モード
      } else if(Limit_speed_mode == 2){
        T1_Button->setText("⬇︎"); //RECモード
      }
      btns_layoutBB->addWidget(T1_Button);
      T1_Button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
      T1_Button->setContentsMargins(0, 0, 0, 0);
      T1_Button->setFixedHeight(90);
      T1_Button->setStyleSheet(QString(btn_styleb2).arg(mButtonColors.at(false)));
      QObject::connect(T1_Button, &QPushButton::pressed, [=]() {
        MAX_touch();
      });
    }
    { //ナイトスキャナー非表示
      // auto interp_color = [=](QColor c1, QColor c2, QColor c3) { //関数内関数の記述例
      //   return speedLimit > 0 ? interpColor(setSpeed, {speedLimit + 5, speedLimit + 15, speedLimit + 25}, {c1, c2, c3}) : c1;
      // };
      // max_color = interp_color(max_color, QColor(0xff, 0xe4, 0xbf), QColor(0xff, 0xbf, 0xbf));
      Knight_scanner = getButtonInt("/data/knight_scanner_bit3.txt",7);
      std::string btn_str = "";
      if(Knight_scanner & 0x1){
        btn_str += "⚫︎";
      } else {
        btn_str += "⚪︎";
      }
      if(Knight_scanner & 0x2){
        btn_str += "⚫︎";
      } else {
        btn_str += "⚪︎";
      }
      if(Knight_scanner & 0x4){
        btn_str += "⚫︎";
      } else {
        btn_str += "⚪︎";
      }
      QPushButton *T2_Button = new QPushButton(btn_str.c_str()); //"⚫︎⚫︎⚫︎"
      btns_layoutBB->addWidget(T2_Button);
      T2_Button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
      T2_Button->setContentsMargins(0, 0, 0, 0);
      T2_Button->setFixedHeight(90);
      T2_Button->setStyleSheet(QString(btn_styleb).arg(mButtonColors.at(true)));
      QObject::connect(T2_Button, &QPushButton::pressed, [=]() {
        Knight_scanner = (getButtonInt("/data/knight_scanner_bit3.txt",7) + 1) % 8;
        setButtonInt("/data/knight_scanner_bit3.txt",Knight_scanner);
        std::string btn_str = "";
        if(Knight_scanner & 0x1){
          btn_str += "⚫︎";
        } else {
          btn_str += "⚪︎";
        }
        if(Knight_scanner & 0x2){
          btn_str += "⚫︎";
        } else {
          btn_str += "⚪︎";
        }
        if(Knight_scanner & 0x4){
          btn_str += "⚫︎";
        } else {
          btn_str += "⚪︎";
        }
        T2_Button->setText(btn_str.c_str());
        if(Knight_scanner){
          soundPipo();
        } else {
          soundPo();
        }
      });
    }
    { //運転傾向
      T3_Button = new QPushButton("⬆︎⬆︎⬆︎");
      std::string longPsn_txt = Params().get("LongitudinalPersonality"); // static_cast<int>(sm["controlsState"].getControlsState().getPersonality());

      if(longPsn_txt.empty() == false){
        DrivingPsn = std::stoi(longPsn_txt);
      }
      if(DrivingPsn == 1){
        T3_Button->setText("⬆︎⬆︎");
      } else if(DrivingPsn == 2){
        T3_Button->setText("⬆︎");
      }
      btns_layoutBB->addWidget(T3_Button);
      T3_Button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
      T3_Button->setContentsMargins(0, 0, 0, 0);
      T3_Button->setFixedHeight(90);
      T3_Button->setStyleSheet(QString(btn_styleb1).arg(mButtonColors.at(false)));
      QObject::connect(T3_Button, &QPushButton::pressed, [=]() {
        std::string longPsn_txt = Params().get("LongitudinalPersonality"); // static_cast<int>(sm["controlsState"].getControlsState().getPersonality());
        if(longPsn_txt.empty() == false){
          DrivingPsn = std::stoi(longPsn_txt);
        }
        DrivingPsn = (DrivingPsn - 1 + 3) % 3;
#if 0
        enum LongitudinalPersonality { == DrivingPsn
          aggressive @0;
          standard @1;
          relaxed @2;
        }
#endif
        Params().put("LongitudinalPersonality" , std::to_string(DrivingPsn));
        soundPipo();
        if(DrivingPsn == 0){
          T3_Button->setText("⬆︎⬆︎⬆︎");
        } else if(DrivingPsn == 1){
          T3_Button->setText("⬆︎⬆︎");
        } else if(DrivingPsn == 2){
          T3_Button->setText("⬆︎");
        }
        //onroad中にTogglesで変更したら、表示に反映しないので注意。
      });
    }
    { //ロックオンボタン
      uiState()->scene.mLockOnButton = mLockOnButton = getButtonEnabled("/data/lockon_disp_disable.txt");
      lockOnButton = new QPushButton("□");
      QObject::connect(lockOnButton, &QPushButton::pressed, [=]() {
        uiState()->scene.mLockOnButton = !mLockOnButton;
      });
      if(mLockOnButton == false){
        lockOnButton->setText("□");
      } else if(mLockOnButton == true){
        lockOnButton->setText("■");
      }
      btns_layoutBB->addWidget(lockOnButton);
      lockOnButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
      lockOnButton->setContentsMargins(0, 0, 0, 0);
      lockOnButton->setFixedHeight(90);
      lockOnButton->setStyleSheet(QString(btn_styleb3).arg(mButtonColors.at(false)));
    }
#if 0
    {
        QSpacerItem *spacerItem = new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Preferred);
        btns_layoutBB->addSpacerItem(spacerItem);
    }
#endif
  }
  btns_layout00->setContentsMargins(0, 30, 0, 0);

  QWidget *btns_wrapper0L = new QWidget;
  QHBoxLayout *btns_layout0L  = new QHBoxLayout(btns_wrapper0L);
  btns_layout0L->setSpacing(0);
  btns_layout0L->setContentsMargins(0, 0, 0, 0);
  btns_layout00->addWidget(btns_wrapper0L, 0, /*Qt::AlignVCenter |*/ Qt::AlignLeft);

  QWidget *btns_wrapperLL = new QWidget;
  QVBoxLayout *btns_layoutLL  = new QVBoxLayout(btns_wrapperLL);
  btns_layoutLL->setSpacing(0);
  int forceOnePedalButton_height = 180 * 1.3;
  btns_layoutLL->setContentsMargins(30+15, 430-172 - forceOnePedalButton_height -10, 15, 30 * (1-bottom_btns));

  btns_layout0L->addWidget(btns_wrapperLL,0,Qt::AlignVCenter);
  { //強制ワンペダルステルスボタン
    //ブレーキで信号停止して、ACCレバーを上げた状態でこのボタンを押すと、
    //OP_ENABLE_v_cruise_kph = v_cruise_kph
    //OP_ENABLE_gas_speed = 1.0 / 3.6
    //強制的にワンペダルモードとなる。
    QPushButton *forceOnePedalButton = new QPushButton(""); //表示文字も無し。
    QObject::connect(forceOnePedalButton, &QPushButton::pressed, [=]() {
      const auto cs = (*(uiState()->sm))["controlsState"].getControlsState();
      if(getButtonInt("/tmp/accel_engaged.txt" , 0) >= 3 && cs.getEnabled()){ //ワンペダルのみ
        std::string stdstr_txt = util::read_file("/tmp/cruise_info.txt");
        if(stdstr_txt.empty() == false){
          if(stdstr_txt != "1" && stdstr_txt != ",1"){ //MAXが1ではない時
            if((*(uiState()->sm))["carState"].getCarState().getVEgo() < 0.1/3.6){ //スピードが出ていない時
              setButtonEnabled0("/tmp/force_one_pedal.txt" , true); //これがセットされる条件をなるべく絞る。
            } else {
              //⚫︎ボタンの代わりに動作する
              //soundPo(); //操作不能音として鳴らす。
              MAX_touch();
            }
          } else {
            //MAX=1でタッチ(↑ボタン効果で",1"も含む)
            float vego = (*(uiState()->sm))["carState"].getCarState().getVEgo();
            if(vego > 3/3.6 && vego <= 30/3.6){ //スピードが3〜30km/hのとき
              setButtonEnabled0("/tmp/force_low_engage.txt" , true);
            } else {
              //⚫︎ボタンの代わりに動作する
              //soundPo(); //操作不能音として鳴らす。
              MAX_touch();
            }
          }
        }
      } else {
        //⚫︎ボタンの代わりに動作する
        MAX_touch();
      }
    });
    int rect_width = 190 * 1.3;
    int rect_height = forceOnePedalButton_height;
    forceOnePedalButton->setFixedWidth(rect_width);
    forceOnePedalButton->setFixedHeight(rect_height);
    //forceOnePedalButton->setWindowOpacity(0.2);
    btns_layoutLL->addSpacing(30 * bottom_btns);
    btns_layoutLL->addWidget(forceOnePedalButton);
    forceOnePedalButton->setStyleSheet(QString(btn_style0).arg("#909090")); //線の色はダミー。
  }

  { //LTA有効ボタン
    // Handle Ctrl button（廃止準備。制限速度標識ボタンに変容予定）
    uiState()->scene.mLTA_EnableButton = mLTA_EnableButton = getButtonInt("/data/lta_enable_sw.txt",0);
    if(mLTA_EnableButton <= 1){
      LTA_EnableButton = new QPushButton("/ \\");
    } else {
      LTA_EnableButton = new QPushButton("/ \\");
    }
    QObject::connect(LTA_EnableButton, &QPushButton::pressed, [=]() {
      uiState()->scene.mLTA_EnableButton = (mLTA_EnableButton + 1) % 2; //0->1->0
    });
#define BTN_W_NORMAL 154 //150
    LTA_EnableButton->setFixedWidth(BTN_W_NORMAL);
    LTA_EnableButton->setFixedHeight(BTN_W_NORMAL*0.9);
    //LTA_EnableButton->setPalette(QColor(255,255,255,all_opac*255));
    //LTA_EnableButton->setAutoFillBackground(true);
    btns_layoutLL->addSpacing(10);
    btns_layoutLL->addWidget(LTA_EnableButton);
    LTA_EnableButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mLTA_EnableButton > 0)));
  }

  { //dXボタン
    // use lane button
    uiState()->scene.mUseDynmicExpButton = mUseDynmicExpButton = getButtonInt("/data/dexp_sw_mode.txt" , true /*Params().getBool("EndToEndToggle")*/ ? 0 : 1);
    useDynmicExpButton = new QPushButton("dX"); //ダイナミックexperimentalモード
    QObject::connect(useDynmicExpButton, &QPushButton::pressed, [=]() {
      uiState()->scene.mUseDynmicExpButton = (mUseDynmicExpButton + 1) % 2; //0->1->0
    });
    useDynmicExpButton->setFixedWidth(BTN_W_NORMAL);
    useDynmicExpButton->setFixedHeight(BTN_W_NORMAL*0.9);
    btns_layoutLL->addSpacing(15);
    btns_layoutLL->addWidget(useDynmicExpButton);
    useDynmicExpButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mUseDynmicExpButton > 0)));
  }

  QWidget *btns_wrapper0U = new QWidget;
  QVBoxLayout *btns_layout0U  = new QVBoxLayout(btns_wrapper0U);
  btns_layout0U->setSpacing(0);
  btns_layout0U->setContentsMargins(0, 430-200-70 - 20 * bottom_btns, 0, 0);
  btns_layout00->addWidget(btns_wrapper0U, 0, Qt::AlignTop);

  { //exp,long,ステルスボタン
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
        if(mUseDynmicExpButton){
          mUseDynmicExpButton = (mUseDynmicExpButton + 1) % 2; //0->1->0
          uiState()->scene.mUseDynmicExpButton = mUseDynmicExpButton;
          setButtonInt("/data/dexp_sw_mode.txt" , mUseDynmicExpButton);
          useDynmicExpButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mUseDynmicExpButton > 0 && fp_error==false)));

          UIState *s = uiState();
          if((*s->sm)["controlsState"].getControlsState().getExperimentalMode()){
            setButtonEnabled("/data/long_speeddown_disable.txt",false);
          } else {
            setButtonEnabled("/data/long_speeddown_disable.txt",true);
          }
        }

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
  { // ターボボタン
    // turbo boost button
    uiState()->scene.mStartAccelPowerUpButton = mStartAccelPowerUpButton = getButtonEnabled0("/data/start_accel_power_up_disp_enable.txt");
    startAccelPowerUpButton = new QPushButton("⇧"); //⬆︎
    QObject::connect(startAccelPowerUpButton, &QPushButton::pressed, [=]() {
      uiState()->scene.mStartAccelPowerUpButton = !mStartAccelPowerUpButton;
    });
    startAccelPowerUpButton->setFixedWidth(BTN_W_NORMAL);
    startAccelPowerUpButton->setFixedHeight(BTN_W_NORMAL);
    //lockOnButton->setWindowOpacity(all_opac);
    btns_layoutL->addWidget(startAccelPowerUpButton);
    startAccelPowerUpButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mStartAccelPowerUpButton)));
  }

  if(false){ // ロックオンボタン->下段に移動
    // LockOn button
    uiState()->scene.mLockOnButton = mLockOnButton = getButtonEnabled("/data/lockon_disp_disable.txt");
    lockOnButton = new QPushButton("□");
    QObject::connect(lockOnButton, &QPushButton::pressed, [=]() {
      uiState()->scene.mLockOnButton = !mLockOnButton;
    });
    lockOnButton->setFixedWidth(BTN_W_NORMAL);
    lockOnButton->setFixedHeight(BTN_W_NORMAL);
    //lockOnButton->setWindowOpacity(all_opac);
    btns_layoutL->addSpacing(15);
    btns_layoutL->addWidget(lockOnButton);
    lockOnButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mLockOnButton)));
  }

  { // A,iPボタン
    // Accel Engage button
    uiState()->scene.mAccelEngagedButton = mAccelEngagedButton = getButtonInt("/data/accel_engaged.txt" , 0);
    if(mAccelEngagedButton == 4){
      accelEngagedButton = new QPushButton("eP"); //4なら完全停止しないePadalっぽい動作
    } else if(mAccelEngagedButton == 3){
      accelEngagedButton = new QPushButton("iP"); //3ならイチロウペダル（インテリジェントペダルモード）
    } else if(mAccelEngagedButton == 2){
      accelEngagedButton = new QPushButton("AA"); //2ならAA(ALL ACCEL)
    } else {
      accelEngagedButton = new QPushButton("A");
    }
    QObject::connect(accelEngagedButton, &QPushButton::pressed, [=]() {
      //uiState()->scene.mAccelEngagedButton = (mAccelEngagedButton + 1) % 4; //0->1->2->3->0 , 4:ePはしばらく封印。一応8キロで走行はするが、小道でクリープを使いたいのに狭いとパスが邪魔されて停止してしまう。これではワンペダル(3:iP)と変わらない。
      uiState()->scene.mAccelEngagedButton = (mAccelEngagedButton + 1) % 5; //0->1->2->3->4->0
      setButtonEnabled0("/tmp/force_one_pedal.txt" , false);
      setButtonEnabled0("/tmp/force_low_engage.txt" , false);
    });
    accelEngagedButton->setFixedWidth(BTN_W_NORMAL);
    accelEngagedButton->setFixedHeight(BTN_W_NORMAL);
    //accelEngagedButton->setWindowOpacity(all_opac);
    btns_layoutL->addSpacing(15);
    btns_layoutL->addWidget(accelEngagedButton);
    accelEngagedButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mAccelEngagedButton > 0)));
  }

  QWidget *btns_wrapper = new QWidget;
  QVBoxLayout *btns_layout  = new QVBoxLayout(btns_wrapper);
  btns_layout->setSpacing(0);
  btns_layout->setContentsMargins(15, 0, 30, 30 * (1-bottom_btns));

  btns_layout0->addWidget(btns_wrapper,0,Qt::AlignVCenter);

  { //前走車追従ボタン
    // Accel Ctrl button
    uiState()->scene.mAccelCtrlButton = mAccelCtrlButton = getButtonEnabled("/data/accel_ctrl_disable.txt");
    accelCtrlButton = new QPushButton("↑");
    QObject::connect(accelCtrlButton, &QPushButton::pressed, [=]() {
      uiState()->scene.mAccelCtrlButton = !mAccelCtrlButton;
    });
    accelCtrlButton->setFixedWidth(BTN_W_NORMAL);
    accelCtrlButton->setFixedHeight(BTN_W_NORMAL);
    //accelCtrlButton->setWindowOpacity(all_opac);
    //btns_layout->addSpacing(10);
    btns_layout->addWidget(accelCtrlButton);
    accelCtrlButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mAccelCtrlButton)));
  }

  { //カーブ減速ボタン
    // Decel Ctrl button
    uiState()->scene.mDecelCtrlButton = mDecelCtrlButton = getButtonEnabled("/data/decel_ctrl_disable.txt");
    decelCtrlButton = new QPushButton("↓");
    QObject::connect(decelCtrlButton, &QPushButton::pressed, [=]() {
      uiState()->scene.mDecelCtrlButton = !mDecelCtrlButton;
    });
    decelCtrlButton->setFixedWidth(BTN_W_NORMAL);
    decelCtrlButton->setFixedHeight(BTN_W_NORMAL);
    //decelCtrlButton->setWindowOpacity(all_opac);
    btns_layout->addSpacing(15);
    btns_layout->addWidget(decelCtrlButton);
    decelCtrlButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mDecelCtrlButton)));
  }

  { //ナビボタン
    btns_layout->addSpacing(15);
    btns_layout->addWidget(map_settings_btn);
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
      border-width: 0px;
      border-style: solid;
      background-color: rgba(35, 35, 35, 0.5);
    }
  )");
}

void ButtonsWindow::updateState(const UIState &s) {
  if (mLockOnButton != s.scene.mLockOnButton) {  // update mLockOnButton
    mLockOnButton = s.scene.mLockOnButton;
    //lockOnButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mLockOnButton && fp_error==false)));
    if(mLockOnButton == false){
      lockOnButton->setText("□");
    } else if(mLockOnButton == true){
      lockOnButton->setText("■");
    }
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
    if(mAccelEngagedButton == 4){
      accelEngagedButton->setText("eP");
    } else if(mAccelEngagedButton == 3){
      accelEngagedButton->setText("iP");
    } else if(mAccelEngagedButton == 2){
      accelEngagedButton->setText("AA");
    } else {
      accelEngagedButton->setText("A");
    }
    setButtonInt("/data/accel_engaged.txt" , mAccelEngagedButton);
    soundButton(mAccelEngagedButton);
  }

  if (mLTA_EnableButton != s.scene.mLTA_EnableButton) {  // update mLTA_EnableButton
    mLTA_EnableButton = s.scene.mLTA_EnableButton;
    LTA_EnableButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mLTA_EnableButton > 0 && fp_error==false)));
    if(mLTA_EnableButton <= 1){
      LTA_EnableButton->setText("/ \\");
    } else {
      LTA_EnableButton->setText("⬇/ \\"); //ここには来ない
    }
    setButtonInt("/data/lta_enable_sw.txt" , mLTA_EnableButton);
    soundButton(mLTA_EnableButton);
  }

  if (mStartAccelPowerUpButton != s.scene.mStartAccelPowerUpButton) {  // update mStartAccelPowerUpButton
    mStartAccelPowerUpButton = s.scene.mStartAccelPowerUpButton;
    startAccelPowerUpButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mStartAccelPowerUpButton && fp_error==false)));
    setButtonEnabled0("/data/start_accel_power_up_disp_enable.txt" , mStartAccelPowerUpButton);
    soundButton(mStartAccelPowerUpButton);
  }

  if (mUseDynmicExpButton != s.scene.mUseDynmicExpButton) {  // update mUseDynmicExpButton
    mUseDynmicExpButton = s.scene.mUseDynmicExpButton;
    useDynmicExpButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mUseDynmicExpButton > 0 && fp_error==false)));
    if(mUseDynmicExpButton >= 1){
      useDynmicExpButton->setText("dX");
    } else {
      useDynmicExpButton->setText("dX"); //どのケースでも"dX"
    }
    setButtonInt("/data/dexp_sw_mode.txt" , mUseDynmicExpButton);
    soundButton(mUseDynmicExpButton);
    if(mUseDynmicExpButton == 0){
      //ここで"/tmp/long_speeddown_disable.txt"を"/data/long_speeddown_disable.txt"にコピーしないと、dXを切ったあとのイチロウロング切り替えボタン操作で不整合が起きる。そんなに重要じゃないので放置中。
    }
  }

}

void ButtonsWindow::MAX_touch(){
  Limit_speed_mode = (Limit_speed_mode + 1) % 3;
  setButtonInt("/data/limitspeed_sw.txt" , Limit_speed_mode);
  soundButton(Limit_speed_mode);
  if(Limit_speed_mode == 0){
    T1_Button->setText("⚪︎");
  } else if(Limit_speed_mode == 1){
    T1_Button->setText("⚫︎"); //自動設定モード
  } else if(Limit_speed_mode == 2){
    T1_Button->setText("⬇︎"); //RECモード
  }
}

void ButtonsWindow::psn_update(){
  static unsigned int psn_update_ct = 0;
  psn_update_ct = (psn_update_ct + 1) % 2;
  if(psn_update_ct != 0){
    return;
  }
  //int DrivingPsn = 0; //運転傾向
  int new_DrivingPsn = 0; //運転傾向変更を検出
  std::string longPsn_txt = Params().get("LongitudinalPersonality"); // static_cast<int>(sm["controlsState"].getControlsState().getPersonality());
  if(longPsn_txt.empty() == false){
    new_DrivingPsn = std::stoi(longPsn_txt);
  }
  if(new_DrivingPsn != DrivingPsn){
    DrivingPsn = new_DrivingPsn;
    soundPipo();
    if(DrivingPsn == 0){
      T3_Button->setText("⬆︎⬆︎⬆︎");
    } else if(DrivingPsn == 1){
      T3_Button->setText("⬆︎⬆︎");
    } else if(DrivingPsn == 2){
      T3_Button->setText("⬆︎");
    }
  }

  bool lever_mAccelCtrlButton = getButtonEnabled("/tmp/accel_ctrl_disable.txt");
  if (lever_mAccelCtrlButton != uiState()->scene.mAccelCtrlButton) {  // update mAccelCtrlButton
    mAccelCtrlButton = lever_mAccelCtrlButton;
    uiState()->scene.mAccelCtrlButton = lever_mAccelCtrlButton;
    accelCtrlButton->setStyleSheet(QString(btn_style).arg(mButtonColors.at(mAccelCtrlButton && fp_error==false)));
    setButtonEnabled("/data/accel_ctrl_disable.txt" , mAccelCtrlButton);
    soundButton(mAccelCtrlButton);
  }

  int _Limit_speed_mode = getButtonInt("/tmp/limitspeed_sw.txt",0);
  if(_Limit_speed_mode != Limit_speed_mode){
    //顔ジェスチャーの変更をキャッチ。
    setButtonInt("/data/limitspeed_sw.txt" , Limit_speed_mode);
    soundButton(Limit_speed_mode);
    if(Limit_speed_mode == 0){
      T1_Button->setText("⚪︎");
    } else if(Limit_speed_mode == 1){
      T1_Button->setText("⚫︎"); //自動設定モード
    } else if(Limit_speed_mode == 2){
      T1_Button->setText("⬇︎"); //RECモード
    }
  }
}


// ExperimentalButton
ExperimentalButton::ExperimentalButton(QWidget *parent) : experimental_mode(false), engageable(false), QPushButton(parent) {
  setFixedSize(btn_size, btn_size);

  engage_img = loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size});
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size, img_size});
  QObject::connect(this, &QPushButton::clicked, this, &ExperimentalButton::changeMode);
}

void ExperimentalButton::changeMode() {
  const auto cp = (*uiState()->sm)["carParams"].getCarParams();
  bool can_change = hasLongitudinalControl(cp) && params.getBool("ExperimentalModeConfirmed");
  if (can_change) {
    params.putBool("ExperimentalMode", !experimental_mode);
  }
}

void ExperimentalButton::updateState(const UIState &s) {
  const auto cs = (*s.sm)["controlsState"].getControlsState();
  bool eng = cs.getEngageable() || cs.getEnabled();
  if ((cs.getExperimentalMode() != experimental_mode) || (eng != engageable)) {
    engageable = eng;
    experimental_mode = cs.getExperimentalMode();
    update();
  }
}

void ExperimentalButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  QPixmap img = experimental_mode ? experimental_img : engage_img;
  drawIcon(p, QPoint(btn_size / 2, btn_size / 2), img, QColor(0, 0, 0, 166), (isDown() || !engageable) ? 0.6 : 1.0);
}

// MapSettingsButton
MapSettingsButton::MapSettingsButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(152, 152);
  settings_img = loadPixmap("../assets/navigation/icon_directions_outlined.svg", {img_size-20, img_size-20});

  // hidden by default, made visible if map is created (has prime or mapbox token)
  setVisible(false);
  setEnabled(false);
}

void MapSettingsButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  // drawIcon(p, QPoint(btn_size / 2, btn_size / 2), settings_img, QColor(0, 0, 0, 166), isDown() ? 0.6 : 1.0);
  //ボタンの形を変えたので公式のdrawIconが使えない。
  p.setRenderHint(QPainter::Antialiasing);

  //QPoint center(btn_size / 2, btn_size / 2);

  p.setOpacity(1.0);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, 133));
  //p.drawEllipse(center, btn_size / 2, btn_size / 2);
  QRect temp_rc(0,0,152,152);
  p.drawRoundedRect(temp_rc, 20, 20);
  p.setOpacity(isDown() ? 0.6 : 0.9);
  p.drawPixmap((152 - (img_size-20)) / 2, (152 - (img_size-20)) / 2, settings_img);
}
