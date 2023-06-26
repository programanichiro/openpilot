#!/usr/bin/env python3
from abc import ABC, abstractmethod
import os
import sqlite3
import datetime
import threading
import requests
import math

from common.realtime import DT_TRML
from common.numpy_fast import interp
from system.swaglog import cloudlog
from selfdrive.controls.lib.pid import PIDController

class BaseFanController(ABC):
  @abstractmethod
  def update(self, cur_temp: float, ignition: bool) -> int:
    pass


class TiciFanController(BaseFanController):
  def __init__(self) -> None:
    super().__init__()
    cloudlog.info("Setting up TICI fan handler")

    self.last_ignition = False
    self.controller = PIDController(k_p=0, k_i=4e-3, k_f=1, rate=(1 / DT_TRML))

    #ここが2Hzだから、limitspeed.db操作に利用する。
    self.db_path = "../../../limitspeed.db" #例によって遅くないか？

    # テーブルを作成するSQL
    create_table_sql = """
    CREATE TABLE speeds (
        id INTEGER PRIMARY KEY,
        latitude REAL,
        longitude REAL,
        bearing REAL,
        velocity REAL,
        timestamp REAL
    );
    """

    # データベースに接続してカーソルを取得
    speeds_table_size = 0
    try:
      speeds_table_size = os.path.getsize(self.db_path)
    except Exception as e:
      pass

    self.conn = sqlite3.connect(self.db_path)
    self.cur = self.conn.cursor()
    #self.date = datetime.datetime.now()

    # テーブルが存在しない場合にのみ作成する
    table_exists_query = "SELECT name FROM sqlite_master WHERE type='table' AND name='speeds'"
    result = self.cur.execute(table_exists_query)
    table_exists = bool(result.fetchone())

    if not table_exists:
      # テーブルを作成
      self.cur.execute(create_table_sql)
      # データベースに反映
      self.conn.commit()
    else:
      if speeds_table_size > 3 * 1024 * 1024: #テーブルサイズが3Mを超えたら
        # 奇数番目のレコードを削除
        self.cur.execute("SELECT * FROM speeds")
        records = self.cur.fetchall()
        # 削除条件となる日時を計算
        delete_date = datetime.datetime.now().timestamp() - 30*24*3600 #30日前
        for index, record in enumerate(records):
          if index % 2 != 1:
            row_id , latitude, longitude, bearing, velocity,timestamp = record
            self.cur.execute("DELETE FROM speeds WHERE id = ? and timestamp < ?", (row_id,delete_date)) #60日前の奇数番目のレコードを削除
        self.conn.commit() #一旦コミットしないとVACUUMできない。

      self.conn.execute("VACUUM") #起動のたびにゴミ掃除
      # self.conn.execute("REINDEX") #検索インデックスの振り直し？ こちらは保留。
      self.conn.commit()


    # 削除条件となる日時を計算
    # delete_date = datetime.datetime.now().timestamp() - 30*24*3600 #30日前
    # # テーブルから削除する
    # self.cur.execute("DELETE FROM speeds WHERE timestamp < ?", (delete_date,))
    # # 変更を保存,月単位の削除なら起動時に一回で十分。 -> しばらく削除しないで様子見。随時最適化とDELEモード実装でテーブル増加に対処できるか様子見中。
    # self.conn.commit()

    self.latitude = 0
    self.longitude = 0
    self.bearing = 0
    self.velocity = 0
    self.timestamp = 0
    self.get_limit_avg = 0
    self.get_limitspeed_old = 0
    self.velo_ave_ct_old = 0
    self.db_add = 0
    self.db_none = 0
    self.db_del = 0
    self.min_distance_old = 0
    self.tss_type = 0
    
    # # カーソルと接続を閉じる
    # self.cur.close()
    # self.conn.close()

    self.thread = None
    self.th_id = 0
    self.th_ct = 0
    #江ノ島付近
    #self.latitude = 35.308772
    #self.longitude = 139.483487
    self.distance = 100

  def query_roads_in_bbox(self,lat_min, lon_min, lat_max, lon_max):
    overpass_url = "http://overpass-api.de/api/interpreter"
    query = f"""
    [out:json];
    way({lat_min},{lon_min},{lat_max},{lon_max})["highway"];
    out body;
    """
    response = requests.get(overpass_url, params={'data': query})
    data = response.json()
    return data

  def query_node_location(self,node_id):
    overpass_url = "http://overpass-api.de/api/interpreter"
    query = f"""
    [out:json];
    node({node_id});
    out;
    """
    response = requests.get(overpass_url, params={'data': query})
    data = response.json()
    print("node_id :", node_id)
    print("node_id data:", data)
    return data

  def osm_fetch(self):

    self.th_id += 1
    self.th_ct += 1
    #print("スレッドct:", th_ct)

    # 矩形領域内の道路データをクエリ
    lat_diff = self.distance / 111111  # 緯度1度あたりの距離
    lon_diff = self.distance / (111111 * math.cos(math.radians(self.latitude)))  # 経度1度あたりの距離

    lat_min = self.latitude - lat_diff
    lat_max = self.latitude + lat_diff
    lon_min = self.longitude - lon_diff
    lon_max = self.longitude + lon_diff

    response_data = self.query_roads_in_bbox(lat_min, lon_min, lat_max, lon_max)

    # print("allall:", response_data)

    # 道路の位置情報を抽出
    road_info_list = []
    if "elements" in response_data:
      for element in response_data["elements"]:
        if element["type"] == "way":
            road_coordinates = []
            if "nodes" in element:
                road_coords = []
                for node_id in element["nodes"]:
                    #node_data = self.query_node_location(node_id) #ノードに対する座標は取れるが遅すぎる。maxspeedデータが取れたらで妥協するしかないか。
                    #road_coords.append(node_data)
                    road_coords.append(node_id)
                road_coordinates = road_coords
            else:
                road_coordinates = "NA"
            road_name = element.get("tags", {}).get("name", "---")
            speed_limit = element.get("tags", {}).get("maxspeed", "N/A")
            if speed_limit != "N/A":
              road_info_list.append({"all":element, "road_name": road_name, "speed_limit": speed_limit , "coords": road_coordinates})

    with open('/tmp/debug_out_o','w') as fp:
      fp.write('th_id:%s\n' % (self.th_id))
      for road_info in road_info_list:
        road_name = road_info["road_name"]
        speed_limit = road_info["speed_limit"]
        coords = road_info["coords"]
        #print("all:", road_info["all"])
        fp.write(' road_name:%s\n' % (road_name))
        fp.write(' speed_max:%s\n' % (speed_limit))
        #print("座標インデックス:", coords)
      if len(road_info_list) == 0:
        fp.write(' road_name:%s\n' % ("--"))
        fp.write(' speed_max:%s\n' % (0))

    self.th_ct -= 1
    self.thread = None
    #print("スレッドct x:", th_ct)

  def __del__(self):
    # カーソルと接続を閉じる
    self.cur.close()
    self.conn.close()
    pass
  
  def update(self, cur_temp: float, ignition: bool) -> int:
    self.controller.neg_limit = -(100 if ignition else 30)
    self.controller.pos_limit = -(30 if ignition else 0)

    if ignition != self.last_ignition:
      self.controller.reset()

    error = 70 - cur_temp
    fan_pwr_out = -int(self.controller.update(
                      error=error,
                      feedforward=interp(cur_temp, [60.0, 100.0], [0, -100])
                    ))

    self.last_ignition = ignition

    if self.tss_type == 0:
      try:
        with open('../../../tss_type_info.txt','r') as fp:
          tss_type_str = fp.read()
          if tss_type_str:
            if int(tss_type_str) == 2: #TSS2
              self.tss_type = 2
              dc_get_lag_adjusted_curvature = True
            elif int(tss_type_str) == 1: #TSSP
              self.tss_type = 1
      except Exception as e:
        pass

    rec_mode = False
    rec_speed = 0
    try:
      with open('/tmp/limitspeed_sw.txt','r') as fp:
        limitspeed_sw_str = fp.read()
        if limitspeed_sw_str:
          if int(limitspeed_sw_str) == 2: #RECモード
            rec_mode = True
            with open('/tmp/cruise_info.txt','r') as fp:
              cruise_info_str = fp.read()
              if cruise_info_str:
                rec_speed = int(cruise_info_str) #MAX km/h
    except Exception as e:
      pass

    #"/tmp/limitspeed_info.txt"からlatitude, longitude, bearing, velocity,timestampを読み出して速度30km/h以上ならspeedsに挿入する
    limitspeed_info_ok = False
    limitspeed_min = 30
    try:
      with open('/tmp/limitspeed_info.txt','r') as fp:
        limitspeed_info_str = fp.read()
        if limitspeed_info_str:
          limitspeed_info_ok = True
          #pythonを用い、カンマで区切られた文字列を分離して変数a,b,cに格納するプログラムを書いてください。
          #ただしa,b,cはdouble型とします
          self.latitude, self.longitude, self.bearing, self.velocity,self.timestamp = map(float, limitspeed_info_str.split(","))
          if rec_mode == True and rec_speed >= 30 and self.velocity >= limitspeed_min:
            self.velocity = rec_speed
          if self.tss_type < 2 and self.velocity > 119:
            self.velocity = 119 #TSSPでは最高119(メーター125)km/h
          limit_m = self.velocity/3.6
          if limit_m < 10:
            limit_m = 10 #10m以内の範囲には登録しない。
          if self.velocity >= limitspeed_min and (self.get_limit_avg * 0.7 < self.velocity or self.get_limitspeed_old == 0) and ((int(self.get_limit_avg/5) * 5) != int(self.velocity/5) * 5 or self.get_limitspeed_old == 0 or ((self.min_distance_old**0.5) * 100 / 0.0009 > limit_m and self.velo_ave_ct_old < 10)):
            # データを挿入するSQL , self.velocityが平均速度と同等であれば登録しない。もしくは平均より10km/h遅くても登録しない。
            insert_data_sql = """
            INSERT INTO speeds (latitude, longitude, bearing, velocity,timestamp)
            VALUES (?, ?, ?, ?, ?);
            """
            self.cur.execute(insert_data_sql,(self.latitude, self.longitude, self.bearing, self.velocity,self.timestamp))
            #print("緯度: {}, 経度: {}, 方位: {}, 速度: {}, 日時: {}".format(self.latitude, self.longitude, self.bearing, self.velocity,self.timestamp))
            # 変更を保存
            self.conn.commit()
            self.db_add += 1
          else:
            self.db_none += 1
    except Exception as e:
      pass
    #speedsから距離と方位が近いデータを100個読み、100m以内で速度の上位20パーセントの平均を計算する。int(それ/10)*10を現在道路の制限速度と見做す。
    get_limitspeed = 0
    if limitspeed_info_ok or (self.latitude != 0 or self.longitude != 0):
      #self.latitude, self.longitude, self.bearing, self.velocity,self.timestamp
      query = '''
          SELECT *
          FROM (SELECT * , ABS(bearing - ?) AS abs_bear FROM speeds)
          WHERE
              CASE
                  WHEN abs_bear > 180 THEN 360 - abs_bear
              ELSE
                  abs_bear
              END <= 10
              OR
              CASE
                  WHEN abs_bear > 180 THEN 360 - abs_bear
              ELSE
                  abs_bear
              END >= 170
          ORDER BY ((latitude - ?) * (latitude - ?) + (longitude - ?) * (longitude - ?))
          LIMIT 100
      '''

      # クエリを実行し、結果を取得
      self.cur.execute(query, (self.bearing, self.latitude, self.latitude, self.longitude, self.longitude , ))
      if limitspeed_info_ok == False: #limitspeed_info_ok == False(ファイル読み込み失敗)のフォローは一度きり
        self.latitude = 0
        self.longitude = 0

      # データ内容を検査して走行速度を推定する。
      earth_ang = 0.0009 #大体200m四方
      earth_ang *= interp(self.velocity, [0, 50.0], [0.3, 1.0])#検出範囲に速度を反映する。０〜50km/h -> 0.3〜1倍
      rows = []
      velo_max = 0
      velo_max_ct = 0
      for row in self.cur: #一度ループさせると消える。
        row_id , latitude, longitude, bearing, velocity,timestamp , abs_bear = row #サブクエリ使うとabs_bearがくっついてしまう
        velo_max_ct += 1
        if velo_max < velocity and abs(latitude-self.latitude) < earth_ang and abs(longitude-self.longitude) < earth_ang:
          rows.append(row) #取っておく
          velo_max = velocity

      if velo_max > 0:
        #この時点でrowsは自車近傍に絞られている。
        velo_ave = 0
        velo_ave_ct = 0
        self.min_distance_old = 0

        if self.velocity < 110 or self.tss_type == 2: #TSSPで120km/h高速対応にするため、こちらは110超では通さない。,TSS2なら使って良い。
          velo_70 = (velo_max - limitspeed_min) * 0.7 + limitspeed_min #まず70〜90を検査する。
          velo_95 = (velo_max - limitspeed_min) * 0.95 + limitspeed_min
          for row in rows: #rowsは何度でも使える。
            row_id , latitude, longitude, bearing, velocity,timestamp , abs_bear = row #サブクエリ使うとabs_bearがくっついてしまう
            if velo_70 <= velocity: #rowsが自車近傍のみなので、以降の条件はいらない,and velocity <= velo_95 and abs(latitude-self.latitude) < earth_ang and abs(longitude-self.longitude) < earth_ang:
              velo_ave_ct += 1
              velo_ave += velocity
              distance = (latitude-self.latitude) **2 + (longitude-self.longitude) **2
              if distance != 0 and (distance < self.min_distance_old or self.min_distance_old == 0):
                self.min_distance_old = distance #最も近い距離のポイント

        del_speed_max = 0
        del_speed_min = 10000
        if velo_ave_ct < 4: #70〜95の間のサンプルが少なければ80以上で再取得する。
          velo_ave = 0
          velo_ave_ct = 0
          self.min_distance_old = 0
          velo_80 = (velo_max - limitspeed_min) * 0.8 + limitspeed_min
          for row in rows: #rowsは何度でも使える。
            row_id , latitude, longitude, bearing, velocity,timestamp , abs_bear = row #サブクエリ使うとabs_bearがくっついてしまう
            if velo_80 <= velocity: #rowsが自車近傍のみなので、以降の条件はいらない,and abs(latitude-self.latitude) < earth_ang and abs(longitude-self.longitude) < earth_ang:
              velo_ave_ct += 1
              velo_ave += velocity
              distance = (latitude-self.latitude) **2 + (longitude-self.longitude) **2
              if distance != 0 and (distance < self.min_distance_old or self.min_distance_old == 0):
                self.min_distance_old = distance #最も近い距離のポイント
        elif velo_ave_ct > 20: #サンプルが多い時は最大と最小の速度を削除してみる
          del_speed_max_id = 0
          del_speed_min_id = 0
          for row in rows: #rowsは何度でも使える。
            row_id , latitude, longitude, bearing, velocity,timestamp , abs_bear = row #サブクエリ使うとabs_bearがくっついてしまう
            if del_speed_max <= velocity:
              del_speed_max = velocity
              del_speed_max_id = row_id
            if del_speed_min >= velocity:
              del_speed_min = velocity
              del_speed_min_id = row_id

        #削除処理
        if del_speed_max > 0 and del_speed_min < 10000 and del_speed_max != del_speed_min:
          self.cur.execute("DELETE FROM speeds WHERE id IN (?, ?)", (del_speed_max_id,del_speed_min_id)) #二つまとめて削除
          #self.cur.execute("DELETE FROM speeds WHERE id = ?", (del_speed_max_id,)) #一つ削除ならこう
          self.conn.commit()
          self.db_del += 2

        #削除条件、◯ボタンOFF、cruise_info.txt31以上のとき車速以上の速度のデータを削除する
        try:
          if rec_mode: #RECモード
            pass #try節を続行
          else:
            raise Exception("try節を脱出")

          cri = rec_speed #もっと単純に、MAX値(>=30)より速いデータを全部刈り取ってしまう。完全にお掃除モード
          if cri >= 30:
            pass #try節を続行
          else:
            raise Exception("try節を脱出")
              
          del_db_del = False
          for row in rows: #rowsは何度でも使える。
            row_id , latitude, longitude, bearing, velocity,timestamp , abs_bear = row #サブクエリ使うとabs_bearがくっついてしまう
            if velocity > cri: #MAX値より速いデータを削除
              self.cur.execute("DELETE FROM speeds WHERE id = ?", (row_id,)) #一つずつループして削除、一つでもカンマが必要。
              self.db_del += 1
              del_db_del = True

          if del_db_del == True:
            self.conn.commit()

        except Exception as e:
          pass

        self.velo_ave_ct_old = velo_ave_ct
        if velo_ave_ct > 0:
          velo_ave /= velo_ave_ct
          get_limitspeed = velo_ave
          self.get_limit_avg = get_limitspeed

    #制限速度があれば"/tmp/limitspeed_data.txt"へ数値で書き込む。なければ"/tmp/limitspeed_data.txt"を消す。
    self.get_limitspeed_old = get_limitspeed
    if get_limitspeed > 0:
      with open('/tmp/limitspeed_data.txt','w') as fp:
        fp.write('%d,%.2f,999,%d,%.1fm,+%d,-%d' % (int(get_limitspeed/10) * 10 , get_limitspeed , self.velo_ave_ct_old , (self.min_distance_old**0.5) * 100 / 0.0009 , self.db_add , self.db_del))
    else:
      with open('/tmp/limitspeed_data.txt','w') as fp:
        fp.write('%d,%.2f,111,%d,%.1fm,=%d,-%d' % (int(self.get_limit_avg/10) * 10 , self.get_limit_avg , self.velo_ave_ct_old , (self.min_distance_old**0.5) * 100 / 0.0009 , self.db_none , self.db_del))

    # # もしここで削除するなら、近傍の古いデータだけにするとか、単純な月単位よりも細かく制御したい。
    # # 変更を保存
    # self.conn.commit()

    #osmアクセスで制限速度を取得する試み。
    if self.thread == None and (self.latitude != 0 or self.longitude != 0):
      self.distance = 100 * interp(self.velocity, [0, 50.0], [0.3, 1.0]) #検出範囲に速度を反映する。０〜50km/h -> 0.3〜1倍
      self.thread = threading.Thread(target=self.osm_fetch) #argsにselfは要らない。
      #self.thread.setDaemon(True)
      self.thread.start()
      #同期でテスト。
      #self.osm_fetch()

    return fan_pwr_out

