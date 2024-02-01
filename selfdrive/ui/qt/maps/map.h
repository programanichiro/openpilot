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

class MapWindow : public QOpenGLWidget {
  Q_OBJECT

public:
  MapWindow(const QMapLibre::Settings &);
  ~MapWindow();

private:
  void initializeGL() final;
  void paintGL() final;
  void resizeGL(int w, int h) override;

  QMapLibre::Settings m_settings;
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

  // Blue with normal nav, green when nav is input into the model
  QColor getNavPathColor(bool nav_enabled) {
    return nav_enabled ? QColor("#31ee73") : QColor("#31a1ee");
  }
  MapLimitspeed* map_limitspeed;

  void clearRoute();
  void updateDestinationMarker();
  uint64_t route_rcv_frame = 0;

private slots:
  void updateState(const UIState &s);

public slots:
  void offroadTransition(bool offroad);

signals:
  void LimitspeedChanged(int map_width);
  void requestVisible(bool visible);
  void requestSettings(bool settings);
};
