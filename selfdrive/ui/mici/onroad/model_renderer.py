import colorsys
import numpy as np
import pyray as rl
from cereal import messaging, car
from dataclasses import dataclass, field
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.locationd.calibrationd import HEIGHT_INIT
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.mici.onroad import blend_colors
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.lib.text_measure import measure_text_cached
import sys

CLIP_MARGIN = 500
MIN_DRAW_DISTANCE = 10.0
MAX_DRAW_DISTANCE = 100.0

THROTTLE_COLORS = [
  rl.Color(13, 248, 122, 102),   # HSLF(148/360, 0.94, 0.51, 0.4)
  rl.Color(114, 255, 92, 89),    # HSLF(112/360, 1.0, 0.68, 0.35)
  rl.Color(114, 255, 92, 0),     # HSLF(112/360, 1.0, 0.68, 0.0)
]

NO_THROTTLE_COLORS = [
  rl.Color(242, 242, 242, 102), # HSLF(148/360, 0.0, 0.95, 0.4)
  rl.Color(242, 242, 242, 89),  # HSLF(112/360, 0.0, 0.95, 0.35)
  rl.Color(242, 242, 242, 0),   # HSLF(112/360, 0.0, 0.95, 0.0)
]

LANE_LINE_COLORS = {
  UIStatus.DISENGAGED: rl.Color(200, 200, 200, 255),
  UIStatus.OVERRIDE: rl.Color(255, 255, 255, 255),
  UIStatus.ENGAGED: rl.Color(0, 255, 64, 255),
}

@dataclass
class LeadcarLockon:
  x: float = 0.0
  y: float = 0.0
  d: float = 0.0
  a: float = 0.0
  lxt: float = 0.0
  lxf: float = 0.0
  lockOK: float = 0.0

LeadcarLockon_MAX = 2 #5
leadcar_lockon = [LeadcarLockon() for _ in range(LeadcarLockon_MAX)]

@dataclass
class LeadVertices:
  x: float = 0.0
  y: float = 0.0

lead_vertices = [LeadVertices() for _ in range(LeadcarLockon_MAX)]

@dataclass
class ModelPoints:
  raw_points: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float32))
  projected_points: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.float32))


@dataclass
class LeadVehicle:
  glow: list[float] = field(default_factory=list)
  chevron: list[float] = field(default_factory=list)
  fill_alpha: int = 0


class ModelRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._longitudinal_control = False
    self._experimental_mode = False
    self._blend_filter = FirstOrderFilter(1.0, 0.25, 1 / gui_app.target_fps)
    self._prev_allow_throttle = True
    self._lane_line_probs = np.zeros(4, dtype=np.float32)
    self._road_edge_stds = np.zeros(2, dtype=np.float32)
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]
    self._path_offset_z = HEIGHT_INIT[0]
    self._enable_lead_indicator = False

    # Initialize ModelPoints objects
    self._path = ModelPoints()
    self._lane_lines = [ModelPoints() for _ in range(4)]
    self._road_edges = [ModelPoints() for _ in range(2)]
    self._acceleration_x = np.empty((0,), dtype=np.float32)

    self._acceleration_x_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)
    self._acceleration_x_filter2 = FirstOrderFilter(0.0, 1, 1 / gui_app.target_fps)

    self._torque_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    self._ll_color_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

    # Transform matrix (3x3 for car space to screen space)
    self._car_space_transform = np.zeros((3, 3), dtype=np.float32)
    self._transform_dirty = True
    self._clip_region = None

    self._exp_gradient = Gradient(
      start=(0.0, 1.0),  # Bottom of path
      end=(0.0, 0.0),  # Top of path
      colors=[],
      stops=[],
    )

    self.global_a_rel = 0
    self.global_a_rel_col = 0
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._fade_texture = gui_app.texture("icons_mici/onroad/onroad_fade.png")
    self.toggle_lead_indicator()
    self.toggle_lead_indicator() #2回呼ぶと_enable_lead_indicatorにlockon_disp_disable.txtの状態が反映される（真偽がトグルするから）

    # Get longitudinal control setting from car parameters
    if car_params := Params().get("CarParams"):
      cp = messaging.log_from_bytes(car_params, car.CarParams)
      self._longitudinal_control = cp.openpilotLongitudinalControl

  def toggle_lead_indicator(self):
    #self._enable_lead_indicator = not self._enable_lead_indicator
    lockon_disp_disable = 0
    try:
      with open('/dev/shm/lockon_disp_disable.txt','r') as fp: # /dev/shmのまま
        lockon_disp_disable_str = fp.read()
        if lockon_disp_disable_str:
          lockon_disp_disable = int(lockon_disp_disable_str)
    except Exception as e:
      pass

    lockon_disp_disable = int(not lockon_disp_disable) # 0->1, 1->0
    with open('/dev/shm/lockon_disp_disable.txt','w') as fp2:
      fp2.write("%d" % (lockon_disp_disable))

    self._enable_lead_indicator = (lockon_disp_disable == 0)

  def set_transform(self, transform: np.ndarray):
    self._car_space_transform = transform.astype(np.float32)
    self._transform_dirty = True

  def _render(self, rect: rl.Rectangle):
    sm = ui_state.sm

    self._torque_filter.update(-ui_state.sm['carOutput'].actuatorsOutput.torque)

    # Check if data is up-to-date
    if (sm.recv_frame["liveCalibration"] < ui_state.started_frame or
        sm.recv_frame["modelV2"] < ui_state.started_frame):
      return

    # Set up clipping region
    self._clip_region = rl.Rectangle(
      rect.x - CLIP_MARGIN, rect.y - CLIP_MARGIN, rect.width + 2 * CLIP_MARGIN, rect.height + 2 * CLIP_MARGIN
    )

    # Update state
    self._experimental_mode = sm['selfdriveState'].experimentalMode

    live_calib = sm['liveCalibration']
    self._path_offset_z = live_calib.height[0] if live_calib.height else HEIGHT_INIT[0]

    if sm.updated['carParams']:
      self._longitudinal_control = sm['carParams'].openpilotLongitudinalControl

    model = sm['modelV2']
    radar_state = sm['radarState'] if sm.valid['radarState'] else None
    lead_one = radar_state.leadOne if radar_state else None
    render_lead_indicator = self._longitudinal_control and radar_state is not None

    # Update model data when needed
    model_updated = sm.updated['modelV2']
    if model_updated or sm.updated['radarState'] or self._transform_dirty:
      if model_updated:
        self._update_raw_points(model)

      path_x_array = self._path.raw_points[:, 0]
      if path_x_array.size == 0:
        return

      self._update_model(lead_one, path_x_array)
      if render_lead_indicator:
        self._update_leads(radar_state, path_x_array)
      self._transform_dirty = False

    # Draw elements (hide when disengaged)
    steer_always = 0
    cruise_available = 0
    if ui_state.status == UIStatus.DISENGAGED:
      try:
        with open('/dev/shm/steer_always.txt','r') as fp:
          steer_always_str = fp.read()
          if steer_always_str:
            if int(steer_always_str) >= 1:
              steer_always = 2
        with open('/dev/shm/cruise_available.txt','r') as fp:
          cruise_available_str = fp.read()
          if cruise_available_str:
            if int(cruise_available_str) >= 1:
              cruise_available = 1 #ACCボタンがOFFならBARRIERSを有効にしない。
      except Exception as e:
        pass

    if ui_state.status != UIStatus.DISENGAGED or (steer_always != 0 and cruise_available):
      self._draw_lane_lines()
      self._draw_path(sm)

    # Fade out bottom of overlays for looks
    if gui_app.big_ui() == False:
      rl.draw_texture_ex(self._fade_texture, rl.Vector2(rect.x, rect.y), 0.0, 1.0, rl.WHITE)

    # if self._enable_lead_indicator and render_lead_indicator and radar_state:
    #   self._draw_lead_indicator()

    if render_lead_indicator:
      leads = model.leadsV3
      leads_num = len(leads)

      for i in range(leads_num):
        if leads[i].prob > 0.2 and i < 2: # 信用度20%以上で表示。調整中。
          self._drawLockon(leads[i],lead_vertices[i], i, rect) #drawLockon(painter, leads[i], lead_vertices[i] , i , surface_rect /*, leads_num , leads[0] , leads[1]*/);

  def _update_raw_points(self, model):
    """Update raw 3D points from model data"""
    self._path.raw_points = np.array([model.position.x, model.position.y, model.position.z], dtype=np.float32).T

    for i, lane_line in enumerate(model.laneLines):
      self._lane_lines[i].raw_points = np.array([lane_line.x, lane_line.y, lane_line.z], dtype=np.float32).T

    for i, road_edge in enumerate(model.roadEdges):
      self._road_edges[i].raw_points = np.array([road_edge.x, road_edge.y, road_edge.z], dtype=np.float32).T

    self._lane_line_probs = np.array(model.laneLineProbs, dtype=np.float32)
    self._road_edge_stds = np.array(model.roadEdgeStds, dtype=np.float32)
    self._acceleration_x = np.array(model.acceleration.x, dtype=np.float32)

  def _update_leads(self, radar_state, path_x_array):
    """Update positions of lead vehicles"""
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]
    leads = [radar_state.leadOne, radar_state.leadTwo]

    for i, lead_data in enumerate(leads):
      if lead_data and lead_data.status:
        d_rel, y_rel, v_rel = lead_data.dRel, lead_data.yRel, lead_data.vRel
        idx = self._get_path_length_idx(path_x_array, d_rel)

        # Get z-coordinate from path at the lead vehicle position
        z = self._path.raw_points[idx, 2] if idx < len(self._path.raw_points) else 0.0
        point = self._map_to_screen(d_rel, -y_rel, z + self._path_offset_z)
        if point:
          lead_vertices[i].x = point[0]
          lead_vertices[i].y = point[1]
          self._lead_vehicles[i] = self._update_lead_vehicle(d_rel, v_rel, point, self._rect)

  def _update_model(self, lead, path_x_array):
    """Update model visualization data based on model message"""
    max_distance = np.clip(path_x_array[-1], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
    max_idx = self._get_path_length_idx(self._lane_lines[0].raw_points[:, 0], max_distance)

    # Update lane lines using raw points
    line_width_factor = 0.12
    for i, lane_line in enumerate(self._lane_lines):
      if i in (1, 2):
        line_width_factor = 0.16
      lane_line.projected_points = self._map_line_to_polygon(
        lane_line.raw_points, line_width_factor * self._lane_line_probs[i], 0.0, max_idx
      )

    # Update road edges using raw points
    for road_edge in self._road_edges:
      road_edge.projected_points = self._map_line_to_polygon(road_edge.raw_points, line_width_factor, 0.0, max_idx)

    # Update path using raw points
    if lead and lead.status:
      lead_d = lead.dRel * 2.0
      max_distance = np.clip(lead_d - min(lead_d * 0.35, 10.0), 0.0, max_distance)

    soon_acceleration = self._acceleration_x[len(self._acceleration_x) // 4] if len(self._acceleration_x) > 0 else 0
    self._acceleration_x_filter.update(soon_acceleration)
    self._acceleration_x_filter2.update(soon_acceleration)

    # make path width wider/thinner when initially braking/accelerating
    if self._experimental_mode and False:
      high_pass_acceleration = self._acceleration_x_filter.x - self._acceleration_x_filter2.x
      y_off = np.interp(high_pass_acceleration, [-1, 0, 1], [0.9 * 2, 0.9, 0.9 / 2])
    else:
      y_off = 0.9

    max_idx = self._get_path_length_idx(path_x_array, max_distance)
    self._path.projected_points = self._map_line_to_polygon(
      self._path.raw_points, y_off, self._path_offset_z, max_idx, allow_invert=False
    )

    self._update_experimental_gradient()

  def _update_experimental_gradient(self):
    """Pre-calculate experimental mode gradient colors"""
    if not self._experimental_mode:
      return

    max_len = min(len(self._path.projected_points) // 2, len(self._acceleration_x))

    segment_colors = []
    gradient_stops = []

    i = 0
    while i < max_len:
      # Some points (screen space) are out of frame (rect space)
      track_y = self._path.projected_points[i][1]
      if track_y < self._rect.y or track_y > (self._rect.y + self._rect.height):
        i += 1
        continue

      # Calculate color based on acceleration (0 is bottom, 1 is top)
      lin_grad_point = 1 - (track_y - self._rect.y) / self._rect.height

      # speed up: 120, slow down: 0
      path_hue = np.clip(60 + self._acceleration_x[i] * 35, 0, 120)

      saturation = min(abs(self._acceleration_x[i] * 1.5), 1)
      lightness = np.interp(saturation, [0.0, 1.0], [0.95, 0.62])
      alpha = np.interp(lin_grad_point, [0.75 / 2.0, 0.75], [0.4, 0.0])

      # Use HSL to RGB conversion
      color = self._hsla_to_color(path_hue / 360.0, saturation, lightness, alpha)

      gradient_stops.append(lin_grad_point)
      segment_colors.append(color)

      # Skip a point, unless next is last
      i += 1 + (1 if (i + 2) < max_len else 0)

    # Store the gradient in the path object
    self._exp_gradient.colors = segment_colors
    self._exp_gradient.stops = gradient_stops

  def _update_lead_vehicle(self, d_rel, v_rel, point, rect):
    speed_buff, lead_buff = 10.0, 40.0

    # Calculate fill alpha
    fill_alpha = 0
    if d_rel < lead_buff:
      fill_alpha = 255 * (1.0 - (d_rel / lead_buff))
      if v_rel < 0:
        fill_alpha += 255 * (-1 * (v_rel / speed_buff))
      fill_alpha = min(fill_alpha, 255)

    # Calculate size and position
    sz = np.clip((25 * 30) / (d_rel / 3 + 30), 15.0, 30.0) * 1
    x = np.clip(point[0], 0.0, rect.width - sz / 2)
    y = min(point[1], rect.height - sz * 0.6)

    g_xo = sz / 5
    g_yo = sz / 10

    glow = [(x + (sz * 1.35) + g_xo, y + sz + g_yo), (x, y - g_yo), (x - (sz * 1.35) - g_xo, y + sz + g_yo)]
    chevron = [(x + (sz * 1.25), y + sz), (x, y), (x - (sz * 1.25), y + sz)]

    return LeadVehicle(glow=glow, chevron=chevron, fill_alpha=int(fill_alpha))

  def _get_ll_color(self, prob: float, adjacent: bool, left: bool):
    alpha = np.clip(prob, 0.0, 0.7)
    if adjacent:
      _base_color = LANE_LINE_COLORS.get(ui_state.status, LANE_LINE_COLORS[UIStatus.DISENGAGED])
      color = rl.Color(_base_color.r, _base_color.g, _base_color.b, int(alpha * 255))

      # turn adjacent lls orange if torque is high
      torque = self._torque_filter.x
      high_torque = abs(torque) > 0.6
      if high_torque and (left == (torque > 0)):
        color = blend_colors(
          color,
          rl.Color(255, 115, 0, int(alpha * 255)),  # orange
          np.interp(abs(torque), [0.6, 0.8], [0.0, 1.0])
        )
    else:
      color = rl.Color(255, 255, 255, int(alpha * 255))

    # if ui_state.status == UIStatus.DISENGAGED:
    #   color = rl.Color(0, 0, 0, int(alpha * 255))

    return color

  def _draw_lane_lines(self):
    """Draw lane lines and road edges"""
    """Two closest lines should be green (lane line or road edges)"""
    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0:
        continue

      color = self._get_ll_color(float(self._lane_line_probs[i]), i in (1, 2), i in (0, 1))
      draw_polygon(self._rect, lane_line.projected_points, color)

    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue

      # if closest lane lines are not confident, make road edges green
      color = self._get_ll_color(float(1.0 - self._road_edge_stds[i]), float(self._lane_line_probs[i + 1]) < 0.25, i == 0)
      draw_polygon(self._rect, road_edge.projected_points, color)

  def _draw_path(self, sm):
    """Draw path with dynamic coloring based on mode and throttle state."""
    if not self._path.projected_points.size:
      return

    allow_throttle = sm['longitudinalPlan'].allowThrottle or not self._longitudinal_control
    self._blend_filter.update(int(allow_throttle))

    if self._experimental_mode:
      # Draw with acceleration coloring
      if False and ui_state.status == UIStatus.DISENGAGED:
        draw_polygon(self._rect, self._path.projected_points, rl.Color(0, 0, 0, 90))
      elif len(self._exp_gradient.colors) > 1:
        draw_polygon(self._rect, self._path.projected_points, gradient=self._exp_gradient)
      else:
        draw_polygon(self._rect, self._path.projected_points, rl.Color(255, 255, 255, 30))
    else:
      # Blend throttle/no throttle colors based on transition
      blend_factor = round(self._blend_filter.x * 100) / 100
      blended_colors = self._blend_colors(NO_THROTTLE_COLORS, THROTTLE_COLORS, blend_factor)
      gradient = Gradient(
        start=(0.0, 1.0),  # Bottom of path
        end=(0.0, 0.0),  # Top of path
        colors=blended_colors,
        stops=[0.0, 0.5, 1.0],
      )

      if False and ui_state.status == UIStatus.DISENGAGED:
        draw_polygon(self._rect, self._path.projected_points, rl.Color(0, 0, 0, 90))
      else:
        draw_polygon(self._rect, self._path.projected_points, gradient=gradient)

  def _draw_lead_indicator(self):
    # Draw lead vehicles if available
    for lead in self._lead_vehicles:
      if not lead.glow or not lead.chevron:
        continue

      rl.draw_triangle_fan(lead.glow, len(lead.glow), rl.Color(218, 202, 37, 255))
      rl.draw_triangle_fan(lead.chevron, len(lead.chevron), rl.Color(201, 34, 49, lead.fill_alpha))

  @staticmethod
  def _get_path_length_idx(pos_x_array: np.ndarray, path_height: float) -> int:
    """Get the index corresponding to the given path height"""
    if len(pos_x_array) == 0:
      return 0
    indices = np.where(pos_x_array <= path_height)[0]
    return indices[-1] if indices.size > 0 else 0

  def _map_to_screen(self, in_x, in_y, in_z):
    """Project a point in car space to screen space"""
    input_pt = np.array([in_x, in_y, in_z])
    pt = self._car_space_transform @ input_pt

    if abs(pt[2]) < 1e-6:
      return None

    x, y = pt[0] / pt[2], pt[1] / pt[2]

    clip = self._clip_region
    if not (clip.x <= x <= clip.x + clip.width and clip.y <= y <= clip.y + clip.height):
      return None

    return (x, y)

  def _map_line_to_polygon(self, line: np.ndarray, y_off: float, z_off: float, max_idx: int, allow_invert: bool = True) -> np.ndarray:
    """Convert 3D line to 2D polygon for rendering."""
    if line.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    # Slice points and filter non-negative x-coordinates
    points = line[:max_idx + 1]
    points = points[points[:, 0] >= 0]
    if points.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    N = points.shape[0]
    # Generate left and right 3D points in one array using broadcasting
    offsets = np.array([[0, -y_off, z_off], [0, y_off, z_off]], dtype=np.float32)
    points_3d = points[None, :, :] + offsets[:, None, :]  # Shape: 2xNx3
    points_3d = points_3d.reshape(2 * N, 3)  # Shape: (2*N)x3

    # Transform all points to projected space in one operation
    proj = self._car_space_transform @ points_3d.T  # Shape: 3x(2*N)
    proj = proj.reshape(3, 2, N)
    left_proj = proj[:, 0, :]
    right_proj = proj[:, 1, :]

    # Filter points where z is sufficiently large
    valid_proj = (np.abs(left_proj[2]) >= 1e-6) & (np.abs(right_proj[2]) >= 1e-6)
    if not np.any(valid_proj):
      return np.empty((0, 2), dtype=np.float32)

    # Compute screen coordinates
    left_screen = left_proj[:2, valid_proj] / left_proj[2, valid_proj][None, :]
    right_screen = right_proj[:2, valid_proj] / right_proj[2, valid_proj][None, :]

    # Define clip region bounds
    clip = self._clip_region
    x_min, x_max = clip.x, clip.x + clip.width
    y_min, y_max = clip.y, clip.y + clip.height

    # Filter points within clip region
    left_in_clip = (
      (left_screen[0] >= x_min) & (left_screen[0] <= x_max) &
      (left_screen[1] >= y_min) & (left_screen[1] <= y_max)
    )
    right_in_clip = (
      (right_screen[0] >= x_min) & (right_screen[0] <= x_max) &
      (right_screen[1] >= y_min) & (right_screen[1] <= y_max)
    )
    both_in_clip = left_in_clip & right_in_clip

    if not np.any(both_in_clip):
      return np.empty((0, 2), dtype=np.float32)

    # Select valid and clipped points
    left_screen = left_screen[:, both_in_clip]
    right_screen = right_screen[:, both_in_clip]

    # Handle Y-coordinate inversion on hills
    if not allow_invert and left_screen.shape[1] > 1:
      y = left_screen[1, :]  # y-coordinates
      keep = y == np.minimum.accumulate(y)
      if not np.any(keep):
        return np.empty((0, 2), dtype=np.float32)
      left_screen = left_screen[:, keep]
      right_screen = right_screen[:, keep]

    return np.vstack((left_screen.T, right_screen[:, ::-1].T)).astype(np.float32)

  @staticmethod
  def _hsla_to_color(h, s, l, a):
    rgb = colorsys.hls_to_rgb(h, l, s)
    return rl.Color(
      int(rgb[0] * 255),
      int(rgb[1] * 255),
      int(rgb[2] * 255),
      int(a * 255)
    )

  @staticmethod
  def _blend_colors(begin_colors, end_colors, t):
    if t >= 1.0:
      return end_colors
    if t <= 0.0:
      return begin_colors

    inv_t = 1.0 - t
    return [rl.Color(
      int(inv_t * start.r + t * end.r),
      int(inv_t * start.g + t * end.g),
      int(inv_t * start.b + t * end.b),
      int(inv_t * start.a + t * end.a)
    ) for start, end in zip(begin_colors, end_colors, strict=True)]

  def _drawLockon(self,lead_data,vd,num,rect): #vdにはrect.xとyが含まれている。
    d_rel = lead_data.x[0]
    a_rel = lead_data.a[0]
    self.global_a_rel = a_rel

    sz = max(15.0, min((25 * 30) / (d_rel / 3 + 30), 30.0)) * 1 #float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
    #x = max(0, min(vd.x, rect.width - sz / 2)) #float x = std::clamp((float)vd.x(), 0.f, surface_rect.width() - sz / 2);
    x = max(rect.x, min(vd.x, rect.x+rect.width - sz / 2)) #こっち？rect.xを含めた方がいいかな。
    y = vd.y #float y = (float)vd.y();

    rl.begin_blend_mode(rl.BLEND_ADDITIVE) #加算ブレンド#   painter.setCompositionMode(QPainter::CompositionMode_Plus);

    prob_alpha = lead_data.prob #getModelProb();
    if prob_alpha < 0:
      prob_alpha = 0
    elif prob_alpha > 1.0:
      prob_alpha = 1.0
    prob_alpha0 = prob_alpha
    prob_alpha *= 245

    pen_size = 2
    pen_color = rl.Color(int(0.09*255), int(0.945*255), int(0.26*255), int(prob_alpha))

    __scale = 1/4 # gui_app._scale/4
    l_850 = 850 * __scale
    l_500 = 500 * __scale
    l_300 = 300 * __scale
    ww = l_500; hh = l_500
    import openpilot.selfdrive.ui.mici.onroad.augmented_road_view as road_view
    g_wide_cam = road_view.g_wide_cam #extern bool g_wide_cam;
    if g_wide_cam:
       ww *= 1.25
       hh *= 1.25

    if not g_wide_cam and gui_app.big_ui():
      ww = l_850; hh = l_850

    l_80 = 80 * __scale
    l_40 = 40 * __scale
    l_15 = 15 * __scale
    l_10 = 10 * __scale
    l_8 = 8 * __scale
    l_1 = 1 # * __scale

    d = d_rel #距離をロックターケットの大きさに反映させる。
    if d < 1:
      d = 1

    #動きに緩衝処理。
    leadcar_lockon[num].x = leadcar_lockon[num].x + (x - leadcar_lockon[num].x) / 6
    leadcar_lockon[num].y = leadcar_lockon[num].y + (y - leadcar_lockon[num].y) / 6
    leadcar_lockon[num].d = leadcar_lockon[num].d + (d - leadcar_lockon[num].d) / 6
    x = leadcar_lockon[num].x
    y = leadcar_lockon[num].y
    d = leadcar_lockon[num].d
    if d < 1:
      d = 1

    leadcar_lockon[num].a = leadcar_lockon[num].a + (a_rel - leadcar_lockon[num].a) / 10
    a_rel = leadcar_lockon[num].a

    dh = 50
    if g_wide_cam == False: #dhに奥行き値を反映させる。
      dd = d
      dd -= 25 #dd=0〜75
      dd /= (75.0/2) #dd=0〜2
      dd += 1 #dd=1〜3
      if dd < 1:
        dd = 1
      dh /= dd
    else: #ワイドカメラ使用でロジック変更。リアルタイムで変わる。
      ww *= 0.5; hh *= 0.5
      dh = 100
      dd = d
      dd -= 5 #dd=0〜95
      dd /= (95.0/10) #dd=0〜10
      dd += 1 #dd=1〜11
      if dd < 1:
        dd = 1
      dh /= dd*dd
    dh *= __scale

    ww = ww * 2 * 5 / d
    hh = hh * 2 * 5 / d
    #y = min(rect.height, y-dh) + dh #y = std::fmin(surface_rect.height() /*- sz * .6*/, y - dh) + dh;
    y = min(rect.y+rect.height-2, y-dh) + dh
    r = rl.Rectangle(x - ww/2, y - hh - dh, ww, hh) #QRect r = QRect(x - ww/2, y /*- g_yo*/ - hh - dh, ww, hh);

    #//y?ってわかりにくいな。横方向なんだが。getYは使えなさそうだし。
    y0 = leadcar_lockon[0].x * leadcar_lockon[0].d #こうなったら画面座標から逆算。
    y1 = leadcar_lockon[1].x * leadcar_lockon[1].d

    #pen_font_size = int(38 * gui_app._scale/4)
    pen_font_size = int(38 * 2/4)
    if not gui_app.big_ui():
      pen_font_size = int(pen_font_size * 1.5) #c4で小さすぎると表示されないようだ。
    # pen_font = self._font_semi_bold #   painter.setFont(InterFont(38, QFont::DemiBold));
    #import openpilot.selfdrive.ui.onroad.hud_renderer as hud #遅延インポート、重くないらしい。
    #hud_g_lockon_disp_disable = False #仮にFalseにしておく。= hud.g_lockon_disp_disable;
    hud_g_lockon_disp_disable = 0 if self._enable_lead_indicator else 1
    if num == 0 and hud_g_lockon_disp_disable == False:
      #推論1番
      pen_color = rl.Color(int(0.09*255), int(0.945*255), int(0.26*255), int(prob_alpha))#     painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
      c_r = l_15 / (r.width/2)
      rl.draw_rectangle_rounded_lines_ex(r, c_r, 5, pen_size, pen_color)#     painter.drawRect(r);

      if leadcar_lockon[0].x > leadcar_lockon[1].x - 20:
        leadcar_lockon[num].lxt = leadcar_lockon[num].lxt + (r.x+r.width - leadcar_lockon[num].lxt) / 20
        leadcar_lockon[num].lxf = leadcar_lockon[num].lxf + (rect.x+rect.width - leadcar_lockon[num].lxf) / 20
        #painter.drawLine(r.right(),r.top() , width() , 0);
      else:
        leadcar_lockon[num].lxt = leadcar_lockon[num].lxt + (r.x - leadcar_lockon[num].lxt) / 20
        leadcar_lockon[num].lxf = leadcar_lockon[num].lxf + (rect.x - leadcar_lockon[num].lxf) / 20
        #painter.drawLine(r.left(),r.top() , 0 , 0);

      text = " "+str(num+1)
      #上端だから不要 size = measure_text_cached(self._font_semi_bold, text, int(pen_font_size))
      if ww >= 30:
        rl.draw_text_ex(self._font_semi_bold,text,rl.Vector2(r.x,r.y),pen_font_size,0,pen_color)#painter.drawText(r, Qt::AlignTop | Qt::AlignLeft, " " + QString::number(num+1));

      lxt = leadcar_lockon[num].lxt
      if lxt < r.x:
        lxt = r.x
      elif lxt > r.x+r.width:
        lxt = r.x+r.width
      rl.draw_line_ex(rl.Vector2(lxt, r.y), rl.Vector2(leadcar_lockon[num].lxf, rect.y), 2, pen_color)#painter.drawLine(lxt,r.top() , leadcar_lockon[num].lxf , 0);

      if ww >= l_40:
        #painter.drawText(r, Qt::AlignTop | Qt::AlignRight, QString::number((int)(lead_data.getProb()*100)) + "％");

        #num==0のロックオンの右端20ドットくらいをa_rel数値メーターとする。
        wwa = ww * 0.15 * 2
        if wwa > l_40:
          wwa = l_40
        elif wwa < l_10:
          wwa = l_10
        if wwa > ww:
          wwa = ww

        hha = 0
        if a_rel > 0:
          hha = 1 - 0.1 / a_rel
          a_color = rl.Color(int(0.09*255), int(0.945*255), int(0.26*255), int(prob_alpha*0.9))
          if hha < 0:
            hha = 0
          hha = hha * hh
# #if 0
#         QRect ra = QRect(x - ww/2 + (ww - wwa), y /*- g_yo*/ - hh - dh + (hh-hha), wwa, hha);
#         painter.drawRect(ra);
# #else //メーターを斜めに切る
          meter = [(x + ww/2 - wwa/2 , y - hh - dh + hh),
                   (x + ww/2 , y - hh - dh + hh),
                   (x + ww/2 , y - hh - dh + (hh-hha)),
                   (x + ww/2 - wwa/2 - wwa/2 * hha / hh , y - hh - dh + (hh-hha))]
          rl.draw_triangle_fan(meter, len(meter), a_color)
# #endif
          pass
        if a_rel < 0:
          hha = 1 + 0.1 / a_rel
          a_color = rl.Color(245, 0, 0, int(prob_alpha))
          #減速は上から下へ変更。
          if hha < 0:
            hha = 0
          hha = hha * hh
# #if 0
#         QRect ra = QRect(x - ww/2 + (ww - wwa), y /*- g_yo*/ - hh - dh , wwa, hha);
#         painter.drawRect(ra);
# #else //メーターを斜めに切る
          meter = [(x + ww/2 - wwa/2 - wwa/2 * hha / hh, y - hh - dh + hha),
                   (x + ww/2 , y - hh - dh + hha),
                   (x + ww/2 , y - hh - dh),
                   (x + ww/2 - wwa/2 , y - hh - dh)]
          rl.draw_triangle_fan(meter, len(meter), a_color)
# #endif
          pass

      if abs(y0 - y1) <= l_300: #大きく横にずれた→逆
        leadcar_lockon[num].lockOK = leadcar_lockon[num].lockOK + (40 - leadcar_lockon[num].lockOK) / 5
      else:
        leadcar_lockon[num].lockOK = leadcar_lockon[num].lockOK + (0 - leadcar_lockon[num].lockOK) / 5

      td = leadcar_lockon[num].lockOK
      #d:10〜100->1〜3へ変換
      if td >= 3:
        dd = leadcar_lockon[num].d
        if dd < 10:
          dd = 10

        dd -= 10 #dd=0〜90
        dd /= (90.0/2) #dd=0〜2
        dd += 1 #dd=1〜3
        td /= dd

        td *= 2 * __scale

        tlw = l_8
        tlw_2 = tlw / 2
        pen_size = tlw * 2
        pen_color = rl.Color(int(0.09*255), int(0.945*255), int(0.26*255), int(prob_alpha))
        rl.draw_line_ex(rl.Vector2(r.x+r.width/2, r.y-tlw_2), rl.Vector2(r.x+r.width/2, r.y-td), pen_size, pen_color)#painter.drawLine(r.center().x() , r.top()-tlw_2 , r.center().x() , r.top() - td);
        rl.draw_line_ex(rl.Vector2(r.x-tlw_2, r.y+r.height/2), rl.Vector2(r.x-td, r.y+r.height/2), pen_size, pen_color)#painter.drawLine(r.left()-tlw_2 , r.center().y() , r.left() - td , r.center().y());
        rl.draw_line_ex(rl.Vector2(r.x+r.width+tlw_2, r.y+r.height/2), rl.Vector2(r.x+r.width+td, r.y+r.height/2), pen_size, pen_color)#painter.drawLine(r.right()+tlw_2 , r.center().y() , r.right() + td , r.center().y());
        rl.draw_line_ex(rl.Vector2(r.x+r.width/2, r.y+r.height+tlw_2), rl.Vector2(r.x+r.width/2, r.y+r.height+td), pen_size, pen_color)#painter.drawLine(r.center().x() , r.bottom()+tlw_2 , r.center().x() , r.bottom() + td);

      pass

    elif hud_g_lockon_disp_disable == False:
      if num == 1:
        #推論2番
        #邪魔な前右寄りを走るバイクを認識したい。
        if abs(y0 - y1) > l_300: #大きく横にずれた
          #painter.setPen(QPen(QColor(245, 0, 0, prob_alpha), 4));
          #painter.drawEllipse(r); //縁を描く
          #painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 1)); //文字を後で書くために色を再設定。->文字は赤でもいいや
          #円を（意味不明だから）書かないで、枠ごと赤くする。推論1が推論と別のものを捉えてるのを簡単に認識できる。
          pen_color = rl.Color(245, 0, 0, int(prob_alpha))#         painter.setPen(QPen(QColor(245, 0, 0, prob_alpha), 2));
        else:
          pen_color = rl.Color(int(0.09*255), int(0.945*255), int(0.26*255), int(prob_alpha))#         painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));

        if leadcar_lockon[0].x > leadcar_lockon[1].x - 20: #多少逆転しても許容する
          leadcar_lockon[num].lxt = leadcar_lockon[num].lxt + (r.x - leadcar_lockon[num].lxt) / 20
          leadcar_lockon[num].lxf = leadcar_lockon[num].lxf + (rect.x - leadcar_lockon[num].lxf) / 20
          #painter.drawLine(r.left(),r.top() , 0 , 0);
        else:
          leadcar_lockon[num].lxt = leadcar_lockon[num].lxt + (r.x+r.width - leadcar_lockon[num].lxt) / 20
          leadcar_lockon[num].lxf = leadcar_lockon[num].lxf + (rect.x+rect.width - leadcar_lockon[num].lxf) / 20
          #painter.drawLine(r.right(),r.top() , width() , 0);

        lxt = leadcar_lockon[num].lxt
        if lxt < r.x:
          lxt = r.x
        elif lxt > r.x+r.width:
          lxt = r.x+r.width
        rl.draw_line_ex(rl.Vector2(lxt, r.y), rl.Vector2(leadcar_lockon[num].lxf, rect.y), 2, pen_color)#painter.drawLine(lxt,r.top() , leadcar_lockon[num].lxf , 0);

#       if(ww >= 80){
#         //float dy = y0 - y1;
#         //painter.drawText(r, Qt::AlignBottom | Qt::AlignLeft, " " + QString::number(dy,'f',1) + "m");
#         //painter.drawText(r, Qt::AlignBottom | Qt::AlignLeft, " " + QString::number(dy,'f',1));
#       }
        pass #num == 1
      elif num == 2:
#       //推論3番
        pen_size = l_1
        pen_color = rl.Color(int(0.09*255), int(0.9*255), int(0.9*255), int(prob_alpha))#       painter.setPen(QPen(QColor(0.9*255, 0.9*255, 0.9*255, prob_alpha), 1));
        pass #num == 2
      else:
#       //推論4番以降。
#       //存在していない。
        pen_size = l_1
        pen_color = rl.Color(int(0.8*255), int(0.2*255), int(0.2*255), int(prob_alpha))#       painter.setPen(QPen(QColor(0.8*255, 0.2*255, 0.2*255, prob_alpha), 1));
        pass #else

      if num < 2:
        c_r = l_15 / (r.width/2)
        rl.draw_rectangle_rounded_lines_ex(r, c_r, 5, pen_size, pen_color)#     painter.drawRect(r);
      else:
        #3番目のサークル描画は一旦保留
        arc_center = rl.Vector2(r.x+r.width/2,r.y+r.height/2)
        rl.draw_ring(arc_center,float(r.width/2), float(r.width/2-pen_size), float(0), float(360*prob_alpha0), 120, pen_color)# painter.drawArc(r , 0 * 16, (int)(360 * 16 * prob_alpha0));

      if ww >= 50 or (ww >= 30 and abs(y0 - y1) > l_500):
        d_lim = 35 * gui_app._scale
        if not gui_app.big_ui():
          d_lim = 15
        g_wide_cam_requested = g_wide_cam #これで代用可能？#       extern bool g_wide_cam_requested;
        if g_wide_cam_requested == False:
          d_lim *= 1.25 #ロングカメラだとちょっと枠が大きい。実測
        if num == 0 or (num==1 and (d_rel < d_lim or abs(y0 - y1) > l_300)): #num==1のとき、'2'の表示と前走車速度表示がかぶるので、こちらを消す。->c4では2を表示する。
          text = " "+str(num+1)
          size = measure_text_cached(self._font_semi_bold, text, int(pen_font_size))
          rl.draw_text_ex(self._font_semi_bold,text,rl.Vector2(r.x,r.y+r.height - size.y),pen_font_size,0,pen_color)#         painter.drawText(r, Qt::AlignBottom | Qt::AlignLeft, " " + QString::number(num+1));

#     if(ww >= 160 /*80*/){
#       //painter.drawText(r, Qt::AlignBottom | Qt::AlignRight, QString::number((int)(lead_data.getProb()*100)) + "％");
#       //painter.drawText(r, Qt::AlignBottom | Qt::AlignRight, QString::number(a_rel,'f',1) + "a");
#     }

      pass #hud_g_lockon_disp_disable == False
  #   }

    rl.end_blend_mode() #元のブレンドに戻す#   painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
    pass
