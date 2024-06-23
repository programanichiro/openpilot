#include "selfdrive/ui/qt/maps/map_panel.h"

#include <QHBoxLayout>
#include <QWidget>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QRegularExpression>
#include <QRegularExpressionValidator>

#include "selfdrive/ui/qt/maps/map.h"
#include "selfdrive/ui/qt/maps/map_settings.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/ui.h"

MapPanel::MapPanel(const QMapLibre::Settings &mapboxSettings, QWidget *parent) : QFrame(parent) {
  content_stack = new QStackedLayout(this);
  content_stack->setContentsMargins(0, 0, 0, 0);

  auto map = new MapWindow(mapboxSettings , this);
  QObject::connect(uiState(), &UIState::offroadTransition, map, &MapWindow::offroadTransition);
  QObject::connect(device(), &Device::interactiveTimeout, this, [=]() {
    content_stack->setCurrentIndex(0);
  });
  QObject::connect(map, &MapWindow::requestVisible, this, [=](bool visible) {
    // when we show the map for a new route, signal HomeWindow to hide the sidebar
    if (visible) { emit mapPanelRequested(); }
    setVisible(visible);
  });
  QObject::connect(map, &MapWindow::requestSettings, this, [=](bool settings) {
    content_stack->setCurrentIndex(settings ? 1 : 0);
  });
  content_stack->addWidget(map);

  auto settings = new MapSettings(true, parent);
  QObject::connect(settings, &MapSettings::closeSettings, this, [=]() {
    content_stack->setCurrentIndex(0);
  });
  content_stack->addWidget(settings);
}

void MapPanel::toggleMapSettings() {
#if 0
  // show settings if not visible, then toggle between map and settings
  int new_index = isVisible() ? (1 - content_stack->currentIndex()) : 1;
  content_stack->setCurrentIndex(new_index);
  emit mapPanelRequested();
  show();
#else
  if(isVisible() == false){
    //地図を出す
    content_stack->setCurrentIndex(0);
    setVisible(true);
    emit mapPanelRequested(); //これでサイドバーを隠す
  }
  if (Params().get("NavDestination").empty()) {
    extern float g_latitude,g_longitude;
    extern bool chg_coordinate;

    std::string my_google_key = util::read_file("/data/google_key.txt");
    my_google_key.erase(std::remove(my_google_key.begin(), my_google_key.end(), '\n'), my_google_key.end());
    my_google_key.erase(std::remove(my_google_key.begin(), my_google_key.end(), '\r'), my_google_key.end());
    QString gg_key;
    if(my_google_key.empty() == false){
      gg_key = QString::fromStdString(my_google_key);
    }
    if(my_google_key.empty() == true){
      //先にGoogle API Keyを入れる
      gg_key = InputDialog::getText(tr("Google API key"), this, tr("Enter Google API key. If using only Lat/Lon, input x."), false, -1, gg_key).trimmed();
      if (gg_key.isEmpty() == false) {
        FILE *fp = fopen("/data/google_key.txt","w");
        if(fp != NULL){
          fprintf(fp,"%s",gg_key.toUtf8().constData());
          fclose(fp);
        }
      }
    }

    if (gg_key.isEmpty() == false) {
      //gg_keyがある場合は、poi検索を行う。
      static QString poi_name;
      QString poi_name_ = InputDialog::getText(tr("POI name or keyword"), this, tr("Enter a POI name or keyword or Lat,Lon"), false, -1, poi_name).trimmed(); //起動中は最後に入れた文字を覚えておくのもいいか？
      if(poi_name_.isEmpty() == false) {
        poi_name = poi_name_;
      }

      bool latlon_mode = false; //緯度経度座標モード
      if(gg_key == "x"){
        latlon_mode = true; //APIキーなしではplacesAPIを呼び出せない。
      }
      if(poi_name_.isEmpty() == false){
        //緯度経度として解釈できるか
        static QRegularExpression re(R"(^[\s]*([-+]?\d+\.\d+)[ ,]+([-+]?\d+\.\d+)[\s]*$)"); //緯度,軽度の正規表現。カンマの前後のスペースは許容しない。
        QRegularExpressionMatch match = re.match(poi_name_);
        if (match.hasMatch()) {
          g_latitude = match.captured(1).toDouble(); // 1つ目の浮動小数を取得
          g_longitude = match.captured(2).toDouble(); // 2つ目の浮動小数を取得
          chg_coordinate = true;
          return;
        }
        //APIキーなし(x)で緯度経度のダイレクト数値以外の場合は、Places APIを呼び出さない（latlon_mode == true）
      }

      if (poi_name_.isEmpty() == false /*&& poi_name.isEmpty() == false*/ && latlon_mode == false){
        //Places API呼び出し。
        // g_latitude = m_map->coordinate().first;
        // g_longitude = m_map->coordinate().second;

        QJsonObject jsonObject;
        jsonObject["textQuery"] = poi_name;
        jsonObject["pageSize"] = 1;
        QJsonObject center;
        center["latitude"] = g_latitude;
        center["longitude"] = g_longitude;
        QJsonObject circle;
        circle["center"] = center;
        QJsonObject locationBias;
        locationBias["circle"] = circle;
        jsonObject["locationBias"] = locationBias;

        QNetworkAccessManager *manager = new QNetworkAccessManager();

        // Create the request
        QNetworkRequest request(QUrl("https://places.googleapis.com/v1/places:searchText"));
        request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
        request.setRawHeader("X-Goog-Api-Key", gg_key.toUtf8());
        request.setRawHeader("X-Goog-FieldMask", "places.location");

        // Convert JSON object to QByteArray
        QJsonDocument jsonDoc(jsonObject);
        QByteArray jsonData = jsonDoc.toJson();

        // Send the POST request
        QNetworkReply *reply = manager->post(request, jsonData);

        // Connect to the reply's finished signal to handle the response
        QObject::connect(reply, &QNetworkReply::finished, [reply]() {
          if (reply->error() == QNetworkReply::NoError) {
            // Parse the response
            QByteArray responseData = reply->readAll();
            QJsonDocument all_res = QJsonDocument::fromJson(responseData);
            if(all_res.isObject()){
              QJsonObject topObj = all_res.object();
              if(topObj.contains("places")){
                QJsonValue places = topObj["places"];
                if(places.isArray()){
                  for (const QJsonValue &value : places.toArray()) {
                    if (value.isObject()) {
                      QJsonObject jsonObj = value.toObject();
                      if (jsonObj.contains("location")) {
                        QJsonObject location = jsonObj["location"].toObject();
                        g_latitude = location["latitude"].toDouble();
                        g_longitude = location["longitude"].toDouble();
                        chg_coordinate = true;
                      }
                    }
                    break; //最初の一個だけで良い。
                  }
                }
              }
            } else {
              //qWarning() << "Response is not a JSON array";
            }
          } else {
            //qWarning() << "Error:" << reply->errorString();
          }
          reply->deleteLater();
        });

        // Optional: Handle network errors
        QObject::connect(manager, &QNetworkAccessManager::finished, manager, &QNetworkAccessManager::deleteLater);
      }
    }
  } else {
    Params().remove("NavDestination"); //ナビ中止
  }
#endif
}
