# modem.py の APN 修正と APN 認証：手元の openpilot に取り込むためのまとめ

対象：openpilot のフォークや手元のブランチを持っているコントリビューター
更新：2026-10-12（前の版は 10/11。認証の実機の結果、設定画面、ビルド済みのテスター版を足した）

## 0. 3 つの版

| 版 | 場所 | 中身 | 向いている使い方 |
|---|---|---|---|
| **試験版（master が元）** | [shunnag/openpilot `modem-auth-test`](https://github.com/shunnag/openpilot/tree/modem-auth-test)（先頭 `329569e`） | upstream master `6f4c0faf1` ＋ A ＋ B ＋ 設定画面 ＋ テスター用の道具 | **自分のフォークへの取り込み**（このまとめの §1〜§5） |
| **テスター向けのビルド済み版** | [`apn-auth`](https://github.com/shunnag/openpilot/tree/apn-auth)（`installer.comma.ai/shunnag/apn-auth`） | comma の v0.11.2（`0ea343233`、ビルド済み）＋ Python だけの変更 | SSH を使わない一般のテスター。認証情報は params ではなくファイルに置く（§6）。取り込みには向かない |
| upstream に出す最小版 | 未公開 | A ＋ 認証の最小版 ＋ comma four の設定画面 | [#39061](https://github.com/commaai/openpilot/pull/39061) のマージのあと commaai に提案する予定 |

## 1. `modem-auth-test` のコミットと確認の状況

| コミット | 内容 | 実機での確認 |
|---|---|---|
| `08f582ea2` | **A. 登録を拒否されているときにも APN の変更を反映する**（#39061 と同じ） | comma four（EG916Q-GL）と SoftBank の SIM で確認済み |
| `81ed502e1` | **B. APN の認証（CHAP/PAP）** | comma four と SoftBank の SIM で、モデムへの書き込み・読み返し・再起動での持続・パスワードが漏れないことを確認。**認証が要る SIM と comma 3X では未確認**（テスター待ち） |
| `60d0d4bb6` | B の直し：`none` で `AT+CGAUTH=1,0,"",""` を書く（`1,0` だけではユーザー名とパスワードが残るため） | 確認済み |
| `8c4adac5e` `d72de6035` `adca21b79` | 設定画面（comma four と comma 3X）、「認証情報を消す」の行、日本語の訳 | comma four は実機で一通り確認。comma 3X は Mac の描画のみ |
| `19a18a8b9` `7bfea868e` `893688583` `a0b679780` `0308250fd` `329569e69` | テスター用の道具とマニュアル（`tools/modem_auth/`） | 道具は comma four で確認。取り込みは任意 |

B は A の上に、画面は B の上に作ってあります。B だけ、画面だけを取り込むことはできません。

## 2. 取り込み方

```bash
cd /path/to/your/openpilot
git fetch https://github.com/shunnag/openpilot modem-auth-test
git cherry-pick 08f582ea2                       # A だけならここまで
git cherry-pick 81ed502e1 60d0d4bb6             # B（2 つで 1 組）
git cherry-pick 8c4adac5e d72de6035 adca21b79   # 設定画面（任意）
```

GitHub のコミットの URL の末尾に `.patch` を付けると、パッチのファイルも取れます。

### 当たる場所と注意
- A と B：`openpilot/common/hardware/comma/modem.py` と `openpilot/common/params_keys.h`。modem.py は、2026-10-12 の時点で upstream master、nightly、release-tizi-staging（v0.11.2）で同じ中身です。
- params_keys.h：`GsmApn` の近くが違うブランチでは衝突することがあります。3 行をアルファベット順の位置に足すだけです（§4）。
- **B は params_keys.h を変えるので、ビルドが要ります。** ビルド済みのブランチ（`prebuilt` があるもの）に取り込むときは、自分の CI などでビルドし直してください。ビルドできない場合は、§6 のファイル方式を参考にしてください。
- **古いブランチ（v0.11.1 の release-tizi／release-mici など）は、modem.py の場所が違います**（`system/hardware/tici/modem.py`）。そのままでは当たりません。
- 設定画面のコミットは、`openpilot/system/ui/widgets/network.py`（comma 3X）、`openpilot/selfdrive/ui/mici/layouts/settings/network/network_layout.py`（comma four）、`openpilot/system/ui/widgets/keyboard.py`、訳のファイル（`app.pot`、`app_ja.po`）を変えます。**master が元**なので、v0.11.2 に当てると comma four 側が落ちます（v0.11.2 の `BigButton` には `description` 引数がない）。その場合は `description=` を外してください。

## 3. A：登録を拒否されているときにも APN の変更を反映する（実機で確認済み）

### 直した不具合
APN を明示しないとつながらない SIM は、CID 1 の APN が空のあいだ登録を拒否されます（CREG/CGREG が 3、`AT+CEER` が ESM cause 32）。
- **P1**：設定で APN（`GsmApn`）を入れても、モデムに書かれない。`_params_changed()` を見ているのが `_do_connected()` だけで、`_do_searching()` が見ていないため。
- **P2**：再起動すると CID 1 には新しい APN が入るが、モデムは電源が入った直後に古い APN で登録しに行っていて、自分でやり直すまで約 12 分（実測 740 秒前後）拒否されたまま。

### 変更点（modem.py に 39 行を足す。消す行はなし）
- `_do_searching()` でも `_params_changed()` を見る。
- `_do_initializing()` で、書く前に CID 1 の APN を読む（`AT+CGDCONT?`）。違う APN が入っていたら、つなぎ直しの回数 `_reattach_tries = 2` を用意する。
- 拒否（`denied`）のときだけ `_reattach()`：`AT+COPS=2`（登録を外す）→ 読み返し → `AT+CGDCONT` を書き直す → `AT+COPS=0` → 読み返し。電波と SIM は切らない。
- 登録できているモデムには何もしない。ふつうの起動で増えるのは読み取り 2 つ（`AT+CGDCONT?`、`AT+COPS?`）だけ。

### 実機の結果（comma four、EG916QGLLGR01A05M04、AGNOS 19.9、SoftBank のデータ SIM）
- 拒否のあいだに APN を入れた：変更から 7 秒で CONNECTED（master は 7 回中 7 回、書かれないまま）。
- 古い APN を覚えたまま再起動：起動から 23〜24 秒で CONNECTED（master は約 740 秒）。
- ふつうの起動：21〜22 秒で CONNECTED（master は 20〜22 秒）。
- 間違った APN：つなぎ直しは 1 回だけで、そのあとくり返さない。

## 4. B：APN の認証（CHAP/PAP）

認証を設定しない人の動きは、今と変わりません（認証の AT コマンドを送らず、pppd の起動の仕方も同じ）。

### 新しい設定（params_keys.h に 3 行）
```cpp
{"GsmAuth", {PERSISTENT, STRING}},
{"GsmPassword", {PERSISTENT | DONT_LOG, STRING}},
{"GsmUsername", {PERSISTENT | DONT_LOG, STRING}},
```

| GsmAuth | 動き |
|---|---|
| 空（既定） | **何もしない。** ユーザー名やパスワードだけが入っていても使わない |
| `chap` / `pap` | ユーザー名とパスワードを CID 1 の認証として書き、pppd にも同じ値を渡す |
| `none` | モデムの CID 1 の認証を消す（`AT+CGAUTH=1,0,"",""`。QICSGP の場合は同じ意味の書き方） |

ユーザー名とパスワードは、64 文字以内、ASCII の `!`〜`~`、ただし `"`、`\`、`,` を除く文字だけを受け付けます。外れたら認証を使わず、値を出さない警告を 1 行出します。comma の SIM（Webbing の ICCID）では使いません。

### モデム側（AT コマンド）
- **コマンドの選び方**：`modem_version` が `EG25` で始まる（comma 3X）なら `AT+QICSGP`、それ以外（comma four の EG916Q-GL）は `AT+CGAUTH`。それぞれ `=?` で使えることを確かめ、だめならもう一方を試す。
- **書く順番**：`AT+CGDCONT=1,"IP","<APN>"` のあとに認証を書く。A の `_reattach()` の中（COPS=2 と COPS=0 のあいだ）でも同じく書く。
- **読み返しと、つなぎ直しの準備**：書く前に CID 1 の認証を読み、方式・ユーザー名・パスワードのどれかが違えばつなぎ直しを用意する。設定が変わったとき（`_params_changed()`）も用意する。
- **「検索中」でのつなぎ直し**：認証の失敗は、拒否（CREG 3）ではなく検索中（CREG 2）に見えることがある（EC25-J と KDDI の報告）。そのため認証で用意したときだけ、検索中か未登録が 60 秒続いたらつなぎ直す（多くて 2 回）。
- **診断のログ**：書いたあとの読み返しを、真偽だけでログに出す（`APN auth via CGAUTH: protocol=chap, readback protocol ok=True, user ok=True, password ok=True`）。

### EG916Q-GL（comma four）で分かったこと（2026-10-12、実機）
- `AT+CGAUTH?` は**パスワードをそのまま読み返す**。返事は決してログに出さない。
- 認証の設定は、`AT+CGDCONT` を書き直しても、**再起動しても残る**。
- **`AT+CGAUTH=1,0` は方式を 0 にするだけで、ユーザー名とパスワードは残る**。`1,0,"",""` で消える。
- ユーザー名の `@` `_` `$` `.` はそのまま保たれる。
- 設定を変えた直後に `PPP fail 1/3, retrying` が 1 回出る（前の pppd を止めたときの `NO CARRIER` を新しい chat が受け取るため。2 回目でつながる。今の modem.py の性質）。
- SoftBank の網は認証の値を確かめない。そのため、認証が失敗したときの形（CREG、CEER）は、まだ確かめられていない。

### pppd 側
- 認証を使わないとき（GsmAuth が空、または `none`）：`PPPD_CMD` は 1 バイトも変えない。
- `chap`／`pap` のとき：`user`／`password` の引数を `file <path>` に置き換える。ファイルは `/dev/shm/modem_pppd_auth_*`（0600、持ち主 comma、sudo の pppd が root で読む）で、pppd を起動するたびに作り直し、止めるときに消す。`refuse-*` は付けない。

### パスワードを漏らさないための決まり（取り込むときに崩さないでください）
- **params に登録してから使う。** 登録していない設定は、マネージャーの起動のたびに bootlog に値ごと記録されてから消されます。`DONT_LOG` が付いていれば値は記録されません。
- **pppd の引数にパスワードを入れない。** AGNOS の sudo は、引数を `COMMAND=` としてジャーナルに残します（実機で確認）。openpilot はジャーナルを bootlog と rlog に入れます。
- `_at()` の失敗のログは、認証の書き込みでは `AT+CGAUTH=1,<redacted>` のように伏せる。`AT+CGAUTH?` と `AT+QICSGP=1` の返事はログに出さない。
- `/dev/shm/modem`（ほかのプロセスからも読める状態ファイル）にも入れない。
- 実機の確認：設定してつないだあと、ジャーナル、modem.py のログ、bootlog、swaglog のどこにもパスワードの文字列は出なかった。

### まだ確かめていないこと
- 認証が要る SIM での動き（そもそもの目的）。失敗したときに CREG が 3 か 2 か。
- comma 3X（EG25-G）での `AT+CGAUTH`／`AT+QICSGP` の動き。
- pppd にも資格情報を渡す必要が本当にあるか（モデムの PPP サーバーは何でも受け付けるため、SoftBank では分からない）。

## 5. 設定画面（`8c4adac5e` ほか、任意）

| 行 | comma four | comma 3X | 動き |
|---|---|---|---|
| APN ユーザー名 | 「set／not set」 | 値を表示 | 1〜64 文字、§4 と同じ決まり |
| APN パスワード | 「set／not set」 | 「••••••」 | 前の値は出さない。3X は入力中も伏せる（目のボタンで表示） |
| APN 認証 | タップで off → chap → pap | Off／PAP／CHAP | GsmAuth を書く（ユーザー名とパスワードを先に入れておく） |
| （off を選んだとき） | 確認 | 確認 | GsmAuth=none を書き、「保存したユーザー名とパスワードも消しますか」と聞く |
| 認証情報を消す | 値があるときだけ | 同じ | 確認のあと、GsmAuth=none → ユーザー名とパスワードを消す |

- 出る条件は APN の行と同じです（comma four は prime の「フル」以外、comma 3X は prime の状態が NONE か LITE のとき）。
- 認証の方式を変えるたびに、モデムは 1 回つなぎ直します。

## 6. テスター向けのビルド済み版 `apn-auth` の違い

`modem-auth-test` と同じ modem.py の処理を、comma のビルド済み v0.11.2 に Python だけで重ねた版です。ビルドなしで入るように、次の点を変えています。
- **認証情報は params ではなく `/data/apn_auth/settings.json`**（0600、フォルダは 0700）に置く。ビルド済みの版では params の種類を増やせないため（キーの一覧がビルドされたライブラリに入っている）。中身の意味（空／none／chap／pap）は params 版と同じ。
- **設定の「モデムの状態」画面**：modem.py が `/dev/shm/modem` に写真に写してよい項目だけの `diag` を出し、画面がそれを表示する（状態、登録、認証と読み返しの結果、PPP、CEER など）。IMEI、ICCID、電話番号、IP、基地局の番号、ユーザー名、パスワードは出さない。
- comma 3X の設定の行は、prime の状態がまだ分からない端末でも出るようにした（フル以外）。ユーザー名は「set（N 文字）」と出す。
- 確認の状況：comma four で実機の一通り（画面の操作、再起動、パスワードが漏れないこと）。comma 3X は Mac の描画のみ。

## 7. 試験
- upstream は modem.py の試験を持っていません（#38213 で提案された試験は、マージの前にメンテナーが外しました）。そのため、ブランチにも試験は入れていません。
- 手元では、モデムを模した試験（`modem-auth-test` 用 140 件、`apn-auth` の元にした作業用ブランチ用 185 件）で確かめています。欲しい方には渡せます。
