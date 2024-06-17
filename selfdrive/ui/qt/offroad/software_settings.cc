#include "selfdrive/ui/qt/offroad/settings.h"

#include <cassert>
#include <cmath>
#include <string>

#include <QDebug>
#include <QLabel>

#include "common/params.h"
#include "common/util.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "system/hardware/hw.h"


void SoftwarePanel::checkForUpdates() {
  std::system("pkill -SIGUSR1 -f system.updated.updated");
}

SoftwarePanel::SoftwarePanel(QWidget* parent) : ListWidget(parent) {
  onroadLbl = new QLabel(tr("Updates are only downloaded while the car is off."));
  onroadLbl->setStyleSheet("font-size: 50px; font-weight: 400; text-align: left; padding-top: 30px; padding-bottom: 30px;");
  addItem(onroadLbl);

  // current version
  versionLbl = new LabelControl(tr("Current Version"), "");
  addItem(versionLbl);

  // Mapbox Token
  ButtonControl *editMapboxTokenButton = new ButtonControl(tr("Mapbox Token"), tr("EDIT"));
  std::string my_mapbox_token = util::read_file("/data/mb_token.txt");
  if(my_mapbox_token.empty() == false){
    QString cur_token = QString::fromStdString(my_mapbox_token);
    editMapboxTokenButton->setValue(cur_token);
  }
  connect(editMapboxTokenButton, &ButtonControl::clicked, [=]() {
    std::string my_mapbox_token = util::read_file("/data/mb_token.txt");
    my_mapbox_token.erase(std::remove(my_mapbox_token.begin(), my_mapbox_token.end(), '\n'), my_mapbox_token.end());
    my_mapbox_token.erase(std::remove(my_mapbox_token.begin(), my_mapbox_token.end(), '\r'), my_mapbox_token.end());
    QString cur_token;
    if(my_mapbox_token.empty() == false){
      cur_token = QString::fromStdString(my_mapbox_token);
    }
    QString mb_token = InputDialog::getText(tr("Enter Mapbox Token"), this, tr("Enter a token obtained from the Mapbox website"), false, -1, cur_token).trimmed();

    if (mb_token.isEmpty() == false) {
      FILE *fp = fopen("/data/mb_token.txt","w");
      if(fp != NULL){
        fprintf(fp,"%s",mb_token.toUtf8().constData());
        fclose(fp);
      }
      editMapboxTokenButton->setValue(mb_token);
    } else {
      //キャンセルと空文字OKの区別がつかない。
      //editMapboxTokenButton->setValue("canceled...");
    }
  });
  addItem(editMapboxTokenButton);

  // Google API key
  ButtonControl *editGoogleApiKeynButton = new ButtonControl(tr("Google API key"), tr("EDIT"));
  std::string my_google_key = util::read_file("/data/google_key.txt");
  if(my_google_key.empty() == false){
    QString cur_key = QString::fromStdString(my_google_key);
    editGoogleApiKeynButton->setValue(cur_key);
  }
  connect(editGoogleApiKeynButton, &ButtonControl::clicked, [=]() {
    std::string my_google_key = util::read_file("/data/google_key.txt");
    my_google_key.erase(std::remove(my_google_key.begin(), my_google_key.end(), '\n'), my_google_key.end());
    my_google_key.erase(std::remove(my_google_key.begin(), my_google_key.end(), '\r'), my_google_key.end());
    QString cur_key;
    if(my_google_key.empty() == false){
      cur_key = QString::fromStdString(my_google_key);
    }
    QString gg_key = InputDialog::getText(tr("Google API key"), this, tr("Enter Google API key. If using only Lat/Lon, input x."), false, -1, cur_key).trimmed();

    if (gg_key.isEmpty() == false) {
      FILE *fp = fopen("/data/google_key.txt","w");
      if(fp != NULL){
        fprintf(fp,"%s",gg_key.toUtf8().constData());
        fclose(fp);
      }
      editGoogleApiKeynButton->setValue(gg_key);
    } else {
      //キャンセルと空文字OKの区別がつかない。
      //editGoogleApiKeynButton->setValue("canceled...");
    }
  });
  addItem(editGoogleApiKeynButton);

  // Map Language
  auto mapTranslateBtn = new ButtonControl(tr("Navigation Language"), tr("CHANGE"), "");
  std::string my_map_language = util::read_file("/data/mb_navi_lang.txt"); //langs[selection]
  QMap<QString, QString> langs = getSupportedLanguages();
  QString cur_language; //langs[selection]
  if(my_map_language.empty() == false){
    cur_language = QString::fromStdString(my_map_language); //langs[selection]
    mapTranslateBtn->setValue(cur_language);
  } else {
    cur_language = QString::fromStdString(params.get("LanguageSetting")); //langs[selection].toStdString());
    mapTranslateBtn->setValue(cur_language);
  }
  connect(mapTranslateBtn, &ButtonControl::clicked, [=]() {
    QString selection = MultiOptionDialog::getSelection(tr("Select a language"), langs.keys(), langs.key(cur_language), this);
    if (!selection.isEmpty()) {
      FILE *fp = fopen("/data/mb_navi_lang.txt","w");
      if(fp != NULL){
        fprintf(fp,"%s",langs[selection].toUtf8().constData()); //langs[selection]で保存する。
        fclose(fp);
      }
      mapTranslateBtn->setValue(langs[selection]);
    }
  });
  addItem(mapTranslateBtn);

  // download update btn
  downloadBtn = new ButtonControl(tr("Download"), tr("CHECK"));
  connect(downloadBtn, &ButtonControl::clicked, [=]() {
    std::system("echo 1 > /data/force_prebuild");
    downloadBtn->setEnabled(false);
    if (downloadBtn->text() == tr("CHECK")) {
      checkForUpdates();
    } else {
      std::system("pkill -SIGHUP -f system.updated.updated");
    }
  });
  addItem(downloadBtn);

  // install update btn
  installBtn = new ButtonControl(tr("Install Update"), tr("INSTALL"));
  connect(installBtn, &ButtonControl::clicked, [=]() {
    installBtn->setEnabled(false);
    params.putBool("DoReboot", true);
  });
  addItem(installBtn);

  // branch selecting
  targetBranchBtn = new ButtonControl(tr("Target Branch"), tr("SELECT"));
  connect(targetBranchBtn, &ButtonControl::clicked, [=]() {
    auto current = params.get("GitBranch");
    QStringList branches = QString::fromStdString(params.get("UpdaterAvailableBranches")).split(",");
    for (QString b : {current.c_str(), "devel-staging", "devel", "nightly", "master-ci", "master"}) {
      auto i = branches.indexOf(b);
      if (i >= 0) {
        branches.removeAt(i);
        branches.insert(0, b);
      }
    }

    QString cur = QString::fromStdString(params.get("UpdaterTargetBranch"));
    QString selection = MultiOptionDialog::getSelection(tr("Select a branch"), branches, cur, this);
    if (!selection.isEmpty()) {
      std::system("echo 1 > /data/force_prebuild");
      params.put("UpdaterTargetBranch", selection.toStdString());
      targetBranchBtn->setValue(QString::fromStdString(params.get("UpdaterTargetBranch")));
      checkForUpdates();
    }
  });
  if (!params.getBool("IsTestedBranch")) {
    addItem(targetBranchBtn);
  }

  // uninstall button
  auto uninstallBtn = new ButtonControl(tr("Uninstall %1").arg(getBrand()), tr("UNINSTALL"));
  connect(uninstallBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to uninstall?"), tr("Uninstall"), this)) {
      params.putBool("DoUninstall", true);
    }
  });
  addItem(uninstallBtn);

  fs_watch = new ParamWatcher(this);
  QObject::connect(fs_watch, &ParamWatcher::paramChanged, [=](const QString &param_name, const QString &param_value) {
    updateLabels();
  });

  connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    is_onroad = !offroad;
    updateLabels();
  });

  updateLabels();
}

void SoftwarePanel::showEvent(QShowEvent *event) {
  // nice for testing on PC
  installBtn->setEnabled(true);

  updateLabels();
}

void SoftwarePanel::updateLabels() {
  // add these back in case the files got removed
  fs_watch->addParam("LastUpdateTime");
  fs_watch->addParam("UpdateFailedCount");
  fs_watch->addParam("UpdaterState");
  fs_watch->addParam("UpdateAvailable");

  if (!isVisible()) {
    return;
  }

  // updater only runs offroad
  onroadLbl->setVisible(is_onroad);
  downloadBtn->setVisible(!is_onroad);

  // download update
  QString updater_state = QString::fromStdString(params.get("UpdaterState"));
  bool failed = std::atoi(params.get("UpdateFailedCount").c_str()) > 0;
  if (updater_state != "idle") {
    downloadBtn->setEnabled(false);
    downloadBtn->setValue(updater_state);
  } else {
    if (failed) {
      downloadBtn->setText(tr("CHECK"));
      downloadBtn->setValue(tr("failed to check for update"));
    } else if (params.getBool("UpdaterFetchAvailable")) {
      downloadBtn->setText(tr("DOWNLOAD"));
      downloadBtn->setValue(tr("update available"));
    } else {
      QString lastUpdate = tr("never");
      auto tm = params.get("LastUpdateTime");
      if (!tm.empty()) {
        lastUpdate = timeAgo(QDateTime::fromString(QString::fromStdString(tm + "Z"), Qt::ISODate));
      }
      downloadBtn->setText(tr("CHECK"));
      downloadBtn->setValue(tr("up to date, last checked %1").arg(lastUpdate));
    }
    downloadBtn->setEnabled(true);
  }
  targetBranchBtn->setValue(QString::fromStdString(params.get("UpdaterTargetBranch")));

  // current + new versions
  versionLbl->setText(QString::fromStdString(params.get("UpdaterCurrentDescription")));
  versionLbl->setDescription(QString::fromStdString(params.get("UpdaterCurrentReleaseNotes")));

  installBtn->setVisible(!is_onroad && params.getBool("UpdateAvailable"));
  installBtn->setValue(QString::fromStdString(params.get("UpdaterNewDescription")));
  installBtn->setDescription(QString::fromStdString(params.get("UpdaterNewReleaseNotes")));

  update();
}
