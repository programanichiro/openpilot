#!/usr/bin/env python3
from abc import ABC, abstractmethod
import sqlite3
import datetime

from common.realtime import DT_TRML
from common.numpy_fast import interp
from system.swaglog import cloudlog
from selfdrive.controls.lib.pid import PIDController

class BaseFanController(ABC):
  @abstractmethod
  def update(self, max_cpu_temp: float, ignition: bool) -> int:
    pass


class TiciFanController(BaseFanController):
  def __init__(self) -> None:
    super().__init__()
    cloudlog.info("Setting up TICI fan handler")

    self.last_ignition = False
    self.controller = PIDController(k_p=0, k_i=4e-3, k_f=1, neg_limit=-80, pos_limit=0, rate=(1 / DT_TRML))

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
      # データを挿入するSQL
      insert_data_sql = """
      INSERT INTO speeds (latitude, longitude, bearing, velocity,timestamp)
      VALUES (?, ?, ?, ?, ?);
      """
      # データを挿入
      data = [
          (35.123, 139.456, 90.0, 10.0, datetime.datetime.now().timestamp()),
      ]
      self.cur.executemany(insert_data_sql, data)
      # データベースに反映
      self.conn.commit()

    # # カーソルと接続を閉じる
    # self.cur.close()
    # self.conn.close()

  def __del__(self):
    # カーソルと接続を閉じる
    self.cur.close()
    self.conn.close()
    pass
  
  def update(self, max_cpu_temp: float, ignition: bool) -> int:
    self.controller.neg_limit = -(80 if ignition else 30)
    self.controller.pos_limit = -(30 if ignition else 0)

    if ignition != self.last_ignition:
      self.controller.reset()

    error = 70 - max_cpu_temp
    fan_pwr_out = -int(self.controller.update(
                      error=error,
                      feedforward=interp(max_cpu_temp, [60.0, 100.0], [0, -80])
                    ))

    self.last_ignition = ignition

    #/tmp/limitspeed_info.txt"からlatitude, longitude, bearing, velocity,timestampを読み出してspeedsに挿入する
    #speedsから距離と方位が近いデータを100個読み、100m以内で30km/h以上＆速度の上位20パーセントの平均を計算する。int(それ/10)*10を現在道路の制限速度と見做す。
    #制限速度を/tmp/limitspeed_data.txt"へ数値で書き込む。

    # # 削除条件となる日時を計算
    # delete_date = datetime.datetime.now().timestamp() - 30*24*3600 #30日前
    # # テーブルから削除する
    # self.cur.execute("DELETE FROM tablename WHERE timestamp < ?", (delete_date,))
    # # 変更を保存
    # self.conn.commit()

    return fan_pwr_out

