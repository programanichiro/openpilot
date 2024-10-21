#pragma once

#include <QPainter>
#include "selfdrive/ui/ui.h"

class HudRenderer : public QObject {
  Q_OBJECT

public:
  HudRenderer();
  void updateState(const UIState &s);
  void draw(QPainter &p, const QRect &surface_rect);

private:
  void drawSetSpeed(QPainter &p, const QRect &surface_rect);
  void drawCurrentSpeed(QPainter &p, const QRect &surface_rect);
  //void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255);
  int drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255 , bool brakeLight = false);
  int drawText(QPainter &p, int x, int y, const QString &text, const QColor &col);
  int drawTextCircleCenter(QPainter &p, int x, int y, const QString &text, const QColor &col); //円の中央とxを共有したいときに使いやすいので残す。
  void my_drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity , float ang);
  int drawTextLeft(QPainter &p, int x, int y, const QString &text, int alpha = 255 , bool brakeLight = false , int red=255, int blu=255, int grn=255 , int bk_red=0, int bk_blu=0, int bk_grn=0, int bk_alp=0, int bk_yofs=0, int bk_corner_r=0 , int bk_add_w=0, int bk_xofs=0 , int bk_add_h=0);
  int drawTextRight(QPainter &p, int x, int y, const QString &text, int alpha = 255 , bool brakeLight = false , int red=255, int blu=255, int grn=255 , int bk_red=0, int bk_blu=0, int bk_grn=0, int bk_alp=0, int bk_yofs=0, int bk_corner_r=0 , int bk_add_w=0, int bk_xofs=0);
  int drawTextCenter(QPainter &p, int x, int y, const QString &text, int alpha = 255 , bool brakeLight = false , int red=255, int blu=255, int grn=255 , int bk_red=0, int bk_blu=0, int bk_grn=0, int bk_alp=0, int bk_yofs=0, int bk_corner_r=0 , int bk_add_w=0, int bk_xofs=0);
  void drawHud(QPainter &p, const QRect &surface_rect);

  float speed = 0;
  float set_speed = 0;
  bool is_cruise_set = false;
  bool is_metric = false;
  bool v_ego_cluster_seen = false;
  int status = STATUS_DISENGAGED;

  QPixmap engage_img;
  QPixmap experimental_img;
  QString speedUnit;
  QString maxSpeed;
  float speedLimit;
  bool has_us_speed_limit = false;
  bool has_eu_speed_limit = false;

  inline QColor whiteColor(int alpha = 255) { return QColor(255, 255, 255, alpha); }
  inline QColor blackColor(int alpha = 255) { return QColor(0, 0, 0, alpha); }
};
