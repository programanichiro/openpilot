#pragma once

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <QThread>

#ifdef QCOM2
#define EGL_EGLEXT_PROTOTYPES
#define EGL_NO_X11
#define GL_TEXTURE_EXTERNAL_OES 0x8D65
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <drm/drm_fourcc.h>
#endif

#include "msgq/visionipc/visionipc_client.h"
#include "system/camerad/cameras/camera_common.h"
#include "selfdrive/ui/ui.h"

const int FRAME_BUFFER_SIZE = 5;
static_assert(FRAME_BUFFER_SIZE <= YUV_BUFFER_COUNT);

class CameraWidget : public QOpenGLWidget, protected QOpenGLFunctions {
  Q_OBJECT

public:
  using QOpenGLWidget::QOpenGLWidget;
  explicit CameraWidget(std::string stream_name, VisionStreamType stream_type, QWidget* parent = nullptr);
  ~CameraWidget();
  void setBackgroundColor(const QColor &color) { bg = color; }
  void setFrameId(int frame_id) { draw_frame_id = frame_id; }
  void setStreamType(VisionStreamType type) { requested_stream_type = type; }
  VisionStreamType getStreamType() { return active_stream_type; }
  void stopVipcThread();

signals:
  void clicked();
  void vipcThreadConnected(VisionIpcClient *);
  void vipcThreadFrameReceived();
  void vipcAvailableStreamsUpdated(std::set<VisionStreamType>);

protected:
  void paintGL() override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override { emit clicked(); }
  virtual mat4 calcFrameMatrix();
  void vipcThread();
  void clearFrames();

  int glWidth();
  int glHeight();

  GLuint frame_vao, frame_vbo, frame_ibo;
  GLuint textures[2];
  std::unique_ptr<QOpenGLShaderProgram> program;
  QColor bg = QColor("#000000");

#ifdef QCOM2
  std::map<int, EGLImageKHR> egl_images;
#endif

  std::string stream_name;
  int stream_width = 0;
  int stream_height = 0;
  int stream_stride = 0;
  bool ready_to_switch_stream = true;  // stream transition may be delayed by a zoom animation
  float zoom_transition = 0;
  std::atomic<VisionStreamType> active_stream_type;
  std::atomic<VisionStreamType> requested_stream_type;
  std::set<VisionStreamType> available_streams;
  QThread *vipc_thread = nullptr;
  std::recursive_mutex frame_lock;
  std::deque<std::pair<uint32_t, VisionBuf*>> frames;
  uint32_t draw_frame_id = 0;
  uint32_t prev_frame_id = 0;

  const float ecam_zoom = 2.0;
  const float fcam_zoom = 1.1;

protected slots:
  void vipcConnected(VisionIpcClient *vipc_client);
  void vipcFrameReceived();
  void availableStreamsUpdated(std::set<VisionStreamType> streams);
};

Q_DECLARE_METATYPE(std::set<VisionStreamType>);
