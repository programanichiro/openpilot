#pragma once

#include <optional>

#include <QGeoCoordinate>
#include <QGestureEvent>
#include <QLabel>
#include <QMap>
#include <QMapLibre/Map>
#include <QMapLibre/Settings>
#include <QMouseEvent>
#include <QOpenGLWidget>
#include <QPixmap>
#include <QPushButton>
#include <QScopedPointer>
#include <QString>
#include <QVBoxLayout>
#include <QWheelEvent>

#include "cereal/messaging/messaging.h"
#include "common/params.h"
#include "common/util.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/maps/map_eta.h"
#include "selfdrive/ui/qt/maps/map_instructions.h"

class MapLimitspeed : public QWidget {
  Q_OBJECT

private:
  void paintEvent(QPaintEvent *event) override;
  QPushButton *speed;

public:
  MapLimitspeed(QWidget * parent=nullptr);

public slots:
  void updateLimitspeed(int map_width);
};

class MapBearingScale : public QWidget {
  Q_OBJECT

private:
  void paintEvent(QPaintEvent *event) override;
  QPushButton *bearing_scale;

public:
  quint64 m_pressedTime;
  MapBearingScale(QWidget * parent=nullptr);

public slots:
  void updateBearingScale(int map_width, int angle, double scale , double latitude);
};

class MapWindow : public QOpenGLWidget {
  Q_OBJECT

public:
  MapWindow(const QMapLibre::Settings & , const QWidget *);
  ~MapWindow();

private:
  void initializeGL() final;
  void paintGL() final;
  void resizeGL(int w, int h) override;

  QMapLibre::Settings m_settings;
  QWidget *m_panel;
  QScopedPointer<QMapLibre::Map> m_map;

  void initLayers();

  void mousePressEvent(QMouseEvent *ev) final;
  void mouseDoubleClickEvent(QMouseEvent *ev) final;
  void mouseMoveEvent(QMouseEvent *ev) final;
  void wheelEvent(QWheelEvent *ev) final;
  bool event(QEvent *event) final;
  bool gestureEvent(QGestureEvent *event);
  void pinchTriggered(QPinchGesture *gesture);
  void setError(const QString &err_str);

  bool loaded_once = false;

  // Panning
  QPointF m_lastPos;
  int interaction_counter = 0;

  // Position
  std::optional<QMapLibre::Coordinate> last_valid_nav_dest;
  std::optional<QMapLibre::Coordinate> last_position;
  std::optional<float> last_bearing;
  FirstOrderFilter velocity_filter;
  bool locationd_valid = false;
  bool routing_problem = false;

  QWidget *map_overlay;
  QLabel *error;
  MapInstructions* map_instructions;
  MapETA* map_eta;

  MapLimitspeed* map_limitspeed;
  MapBearingScale* map_bearing_scale;

  void clearRoute();
  void updateDestinationMarker();
  uint64_t route_rcv_frame = 0;

private slots:
  void updateState(const UIState &s);

public slots:
  void offroadTransition(bool offroad);

signals:
  void LimitspeedChanged(int map_width);
  void BearingScaleChanged(int map_width, int angle, double scale, double latitude);
  void requestVisible(bool visible);
  void requestSettings(bool settings);
};
