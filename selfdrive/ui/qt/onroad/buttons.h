#pragma once

#include <QPushButton>

#include "selfdrive/ui/ui.h"

const int btn_size = 192;
const int img_size = (btn_size / 4) * 3;

class MapSettingsButton;
class ButtonsWindow : public QWidget {
  Q_OBJECT

public:
  ButtonsWindow(QWidget *parent = 0 , MapSettingsButton *map_settings_btn = 0);
  void psn_update();
  void MAX_touch();

private:
  QPushButton *lockOnButton;
  QPushButton *accelCtrlButton;
  QPushButton *decelCtrlButton;
  QPushButton *accelEngagedButton;
  QPushButton *LTA_EnableButton;
  QPushButton *startAccelPowerUpButton;
  QPushButton *useDynmicExpButton;
  QPushButton *T1_Button; //⚫︎⚪︎⬇︎
  QPushButton *T3_Button; //⬆︎⬆︎⬆︎

  // int dfStatus = -1;  // always initialize style sheet and send msg
  // const QStringList dfButtonColors = {"#044389", "#24a8bc", "#fcff4b", "#37b868"};

  // int lsStatus = -1;  // always initialize style sheet and send msg
  // const QStringList lsButtonColors = {"#ff3737", "#37b868", "#044389"};

  const QStringList mButtonColors = {"#d0909090", "#e037b868"};

  // model long button
  bool mLockOnButton = true;  // triggers initialization
  bool mAccelCtrlButton = true;  // triggers initialization
  bool mDecelCtrlButton = true;  // triggers initialization
  int mAccelEngagedButton = 0;  // triggers initialization
  int mLTA_EnableButton = 0;  // triggers initialization
  bool mStartAccelPowerUpButton = false;  // triggers initialization
  int mUseDynmicExpButton = 0;  // triggers initialization

public slots:
  void updateState(const UIState &s);
};

class ExperimentalButton : public QPushButton {
  Q_OBJECT

public:
  explicit ExperimentalButton(QWidget *parent = 0);
  void updateState(const UIState &s);

private:
  void paintEvent(QPaintEvent *event) override;
  void changeMode();

  Params params;
  QPixmap engage_img;
  QPixmap experimental_img;
  bool experimental_mode;
  bool engageable;
};


class MapSettingsButton : public QPushButton {
  Q_OBJECT

public:
  explicit MapSettingsButton(QWidget *parent = 0);

private:
  void paintEvent(QPaintEvent *event) override;

  QPixmap settings_img;
};

void drawIcon(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity);
