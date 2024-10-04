#pragma once

#include <QPainter>
#include <QPolygonF>

#include "selfdrive/ui/ui.h"

class ModelRenderer {
public:
  ModelRenderer() {}
  void setTransform(const Eigen::Matrix3f &transform) { car_space_transform = transform; }
  void draw(QPainter &painter, const QRect &surface_rect);

private:
  bool mapToScreen(float in_x, float in_y, float in_z, QPointF *out);
  void mapLineToPolygon(const cereal::XYZTData::Reader &line, float y_off, float z_off,
                        QPolygonF *pvd, int max_idx, bool allow_invert = true);
  void drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd, const QRect &surface_rect , int num);
  void drawLockon(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd , int num  /*使っていない, size_t leads_num , const cereal::RadarState::LeadData::Reader &lead0, const cereal::RadarState::LeadData::Reader &lead1 */);
  void update_leads(const cereal::RadarState::Reader &radar_state, const cereal::XYZTData::Reader &line);
  void update_model(const cereal::ModelDataV2::Reader &model, const cereal::RadarState::LeadData::Reader &lead);
  void drawLaneLines(QPainter &painter, const UIState *s);
  void drawPath(QPainter &painter, const cereal::ModelDataV2::Reader &model, int height, int width);
  void knightScanner(QPainter &p, int height, int width);
  int drawTextLeft(QPainter &p, int x, int y, const QString &text, int alpha = 255 , bool brakeLight = false , int red=255, int blu=255, int grn=255 , int bk_red=0, int bk_blu=0, int bk_grn=0, int bk_alp=0, int bk_yofs=0, int bk_corner_r=0 , int bk_add_w=0, int bk_xofs=0 , int bk_add_h=0);

  bool longitudinal_control = false;
  bool experimental_model = false;
  float lane_line_probs[4] = {};
  float road_edge_stds[2] = {};
  QPolygonF track_vertices;
  QPolygonF lane_line_vertices[4] = {};
  QPolygonF road_edge_vertices[2] = {};
  QPointF lead_vertices[2] = {};
  Eigen::Matrix3f car_space_transform = Eigen::Matrix3f::Zero();
  QRectF clip_region;
};
