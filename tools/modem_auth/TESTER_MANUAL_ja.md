# APN の認証（CHAP/PAP）試験版：テストのしかたとログの取り方

対象：ユーザー名とパスワードが要る SIM を持っている comma four／comma 3X のテスター
対象のソフト：試験版ブランチ `modem-auth-test`（openpilot の master ＋ modem.py の 2 つの修正）

> **認証が要る SIM では、まだ誰も試していません。** 作った側が comma four（認証が要らない SoftBank の SIM）で、設定する・つながる・消すの流れと、パスワードがログに出ないことを確かめました（10/12）。ただし、この SIM の網はユーザー名とパスワードを確かめないので、認証に失敗したときの動きはまだ分かっていません。分かったことはマニュアルに書き足します。うまく動かないことも、ログを取ることも、この試験の目的です。運転中には使わず、駐車中に試してください。

---

## 0. 先に読んでほしい決まり（パスワードを守るため）

openpilot は、設定値（params）やシステムのログを comma のサーバーに送る仕組みを持っています。試験版は、パスワードがそこに入らないように作ってありますが、**次の順番を守らないと、パスワードが送られることがあります。**

1. **試験版を入れて、ビルドが終わってから**、ユーザー名とパスワードを入れる。
   - 試験版でない openpilot は、この設定を知りません。知らない設定は、起動のたびに **値ごとログに書かれてから消されます**。
2. **sudo を使って設定を書かない。** 必ず付属のスクリプト `set_apn_auth.sh` を、ユーザー `comma` のまま使う。
   - root で書いたファイルは、modem.py もマネージャーも読めなくなり、モバイル回線が止まったり、openpilot が起動しなくなったりします。
3. **SIM を差し替える前、内蔵の eSIM に戻す前、試験版から出る前**には、§7 の「片付け」をする。
   - モデムは、認証の設定を SIM ごとではなく、本体に 1 つだけ持っています。消さないと、別の SIM の接続にもそのユーザー名とパスワードが使われます。
4. pppd に `debug` を足さない。ユーザー名がシステムのログに出ます。
5. ログを人に送る前に、§6 の道具で伏せてから、**ファイルとして**添付する（本文に貼らない）。
6. params のファイルを手で消したり書いたりしない。`set_apn_auth.sh` が、安全な順番（認証タイプ GsmAuth を、書くときも消すときも最後）で扱います。

---

## 1. 用意するもの

| もの | メモ |
|---|---|
| comma four または comma 3X | AGNOS は試験版が自動で 19.9 に上げます（時間がかかり、再起動します） |
| ユーザー名とパスワードが要る SIM | 事業者の「APN 設定」のページで、APN・ユーザー名・パスワード・認証タイプを控えておく |
| SSH で入れるパソコン | GitHub に公開鍵を登録したアカウント |
| 端末とパソコンが同じ Wi-Fi に入れる環境 | モバイル回線が落ちても SSH が切れないように、Wi-Fi で入る |

> **この試験版には WPA3 の対応がありません**（openpilot の master そのものに、modem.py の修正だけを足したものです）。WPA3 だけの Wi-Fi にはつながらないので、**WPA2 でもつながる Wi-Fi** を用意してください。WPA3 の配布物を使っている人は、試験が終わったら §7 の手順で戻してください。

### 事業者ごとの設定の例（各社の公開ページより、2026-10-11 時点）
| 事業者 | APN | ユーザー名 | パスワード | 認証タイプ |
|---|---|---|---|---|
| mineo（D/A/S プラン） | mineo-d.jp / mineo.jp / mineo-s.jp | mineo@k-opti.com | mineo | CHAP |
| LINEMO | plus.acs.jp.v6 | lm | lm | CHAP |
| OCN モバイル ONE | ocn.ne.jp など | mobileid@ocn | mobile | CHAP |
| IIJmio | iijmio.jp | mio@iij | iij | PAP または CHAP |
| 日本通信 | dm.jplat.net | jci@jci | jci | PAP または CHAP |
| BIGLOBE | biglobe.jp | user（または user@biglobe.jp） | 0000 | CHAP または PAP |
| NURO モバイル | so-net.jp | nuro | nuro | CHAP または PAP |

- 「PAP または CHAP」の SIM は、まず **CHAP** で試してください（モデムの AT コマンドには「どちらでも」という選択肢がありません）。
- ahamo、povo2.0、楽天モバイル（本家）は、ユーザー名もパスワードも要りません。この試験の対象外です。
- APN の名前が `.v6` で終わるものや「IPv4/IPv6」と書かれたものは、modem.py が IPv4 だけで接続するため、認証とは別の理由でつながらないことがあります。そのときは、そのことも報告してください。

---

## 2. SSH で入れるようにする（初回だけ）

1. 端末の **設定 → 開発者 → SSH を有効にする** をオンにし、**SSH 鍵** に GitHub のユーザー名を入れる。
2. 端末の IP アドレスを、設定 → ネットワーク（Wi-Fi の詳細）で確かめる。
3. パソコンから：`ssh comma@<端末の IP>`

---

## 3. 試験版を入れる

1. 今の状態をメモする（あとで戻すため）。SSH で：
   ```bash
   cat /VERSION
   cd /data/openpilot && git branch --show-current && git log -1 --oneline
   ```
2. **設定 → デバイス → アンインストール** を選ぶ。
3. セットアップの画面で **カスタムソフトウェア** を選び、次の URL を入れる：
   ```
   installer.comma.ai/shunnag/modem-auth-test
   ```
4. 初回は、AGNOS の更新と、端末の上でのビルドがあります。**30 分以上かかることがあります。** 画面が openpilot のホームになるまで待ってください。
5. SSH で入り直し、試験版になったことを確かめる：
   ```bash
   cd /data/openpilot && git branch --show-current     # modem-auth-test と出る
   ls tools/modem_auth/                                 # set_apn_auth.sh などがある
   ```

> comma prime の「フル」を契約していると、設定画面に APN の欄が出ません。そのときも、下の SSH の手順だけで試せます。

---

## 4. テストの手順

以下、コマンドはすべて SSH の中で、ユーザー `comma` のまま打ちます（`sudo` は付けません）。

```bash
cd /data/openpilot/tools/modem_auth
```

### 段階 A：今までどおり（認証なし）でどうなるか ― 基準を取る
**段階 A は飛ばさないでください。** 認証情報（ユーザー名・パスワード・方式）を入れる前の記録が、「認証なしでは失敗し、認証を入れると直る」ことの唯一の証拠になります。設定画面や `set_apn_auth.sh` で認証情報を入れるのは、段階 A のログを取ったあと（段階 B）です。

1. APN だけを設定画面（設定 → ネットワーク → 詳細 → APN 設定）で入れる。画面がない場合は：
   ```bash
   python3 -c 'from openpilot.common.params import Params; Params().put("GsmApn", "<APN>")'
   ```
2. 3 分待つ。
3. 状態を見る：
   ```bash
   python3 -m json.tool /dev/shm/modem | grep -E '"state"|"registration"|"connected"|"network_type"'
   ```
4. ログを取る（§6）。ファイル名に `A` を付けておく。
   - 認証が要る SIM なら、ここでは `"registration": "denied"` か `"searching"` のまま、`"connected": false` になるはずです。もし認証なしでつながったら、それも大事な結果です。

### 段階 B：認証を入れる（CHAP）
1. 認証を設定する。**画面か SSH のどちらか**で入れます（両方は要りません）。
   - **画面で入れる場合**（設定 → ネットワーク。comma 3X は右上の「詳細」→ 下のほう、comma four は下へスクロールして APN の行の下）：
     1. 「APN ユーザー名」→ ユーザー名を入れる。
     2. 「APN パスワード」→ パスワードを入れる（前の値は表示されません。comma 3X は入力中も伏せて表示）。
     3. 「APN 認証」（comma 3X の日本語表示は「APNログイン方法」）で **CHAP** を選ぶ。comma four はタップするごとに off → chap → pap と変わります。
     - **この順番（ユーザー名 → パスワード → 方式）で入れてください。** 方式を選んだときに、モデムが 1 回つなぎ直します。
     - 使えない文字（空白、`"`、`\`、`,`、全角）は保存されません（comma 3X は警告が出る、comma four は決定ボタンが押せない）。
     - comma 3X は、prime の状態がまだ分からない端末（ペアリングしていない、ネットにつながったことがないなど）では、これらの行が出ません。そのときは SSH で入れてください。
   - **SSH で入れる場合**：パスワードは画面に出ません：
     ```bash
     ./set_apn_auth.sh chap
     ./set_apn_auth.sh show        # GsmAuth: 'chap'、ユーザー名とパスワードは「set (N chars)」と出る
     ```
2. **再起動はしない。** modem.py が数秒以内に設定の変化に気づき、モデムに書き込みます。
3. 5 分待ってから、状態とログを見る（§5 の「期待されるログ」）。
4. つながったら、通信を確かめる：
   ```bash
   ping -c 3 -I ppp0 1.1.1.1
   ```
5. ログを取る（§6）。ファイル名に `B` を付ける。

### 段階 C：再起動しても使えるか
1. 端末を再起動する（設定 → デバイス → 再起動）。
2. 起動してから **つながるまでの時間** を測る（画面右上の電波の表示、または `/dev/shm/modem` の `"state": "CONNECTED"`）。
3. ログを取る。ファイル名に `C` を付ける。

### 段階 D（できれば）：わざと違うパスワードにする
> つながっているあいだに値を変えても、モデムは今の接続を使い続けます。新しい値が使われるのは次の接続からなので、**変えたら再起動します。**
1. `./set_apn_auth.sh chap` で、パスワードだけわざと 1 文字変えて入れる。
2. **再起動する。** 起動から 5 分待ち、どうなるかを見る（つながらない／`registration` が denied か searching か／CEER の数字）。ログを取る（`D1`）。
   - もし違うパスワードでもつながったら、その SIM の網は値を確かめていないということです。それも大事な結果です。
3. **再起動せずに**、正しいパスワードに戻す（`./set_apn_auth.sh chap`）。つなぎ直し（`registration denied, re-attaching` か `registration stalled, re-attaching`）が起き、数十秒〜数分でつながるかを見る。ログを取る（`D2`）。

### 段階 E（「PAP または CHAP」の SIM のみ、できれば）：PAP で試す
1. `./set_apn_auth.sh pap` で同じ値を入れ、再起動して 5 分待ち、ログを取る（`E`）。

---

## 5. ログの見方

### modem.py のログを見る
```bash
tmux capture-pane -p -J -S - -t comma | grep ' modem: ' | tail -40
```
（上のコマンドは tmux の中に入らずに読めます。中に入った場合、AGNOS の tmux は前置キーが Ctrl-b ではなくバッククォート `` ` `` のことがあります。抜けるときは前置キーのあとに d）

### 期待されるログ（パスワードもユーザー名も出ません）
| ログ | 意味 |
|---|---|
| `APN auth settings changed` | 設定の変化に気づいた |
| `APN auth requires re-attach` | モデムに入っていた値と違うので、つなぎ直す準備をした（2 行続けて出ることがある） |
| `APN auth via CGAUTH: protocol=chap, readback protocol ok=True, user ok=True, password ok=True` | comma four：書き込みと読み返しが一致した |
| `APN auth via QICSGP: ...` | comma 3X：同じく（3X は別のコマンドを使います） |
| `... password not shown` | モデムがパスワードを読み返しで見せなかった（異常ではない。comma four のモデムは見せるので、`password ok=True` になるはず） |
| `... user ok=False` | **重要。** ユーザー名の `@` などが、モデムの中で別の文字に変わった可能性。必ず報告してください |
| `registration denied, re-attaching ...` / `registration stalled, re-attaching ...` | つなぎ直しをした（多くて 2 回） |
| `last network cause (AT+CEER, may be stale): [...]` | 網から返された最後の理由。古い値のこともある |
| `modem has no APN auth command` | このモデムでは認証を設定できなかった。**必ず報告してください** |
| `GsmUsername rejected (unsupported character)` など | 使えない文字が入っている（`"`、`\`、`,`、空白、全角など）。値は出ません |
| `PPP fail 1/3, retrying`（設定を変えた直後に 1 回） | 前の接続を止めたときの合図を、新しい接続が受け取ったため。2 回目でつながれば異常ではない |
| `GsmUsername/GsmPassword set but GsmAuth is empty` | 認証タイプ（chap／pap）が入っていないので、ユーザー名とパスワードを使っていない。`set_apn_auth.sh` で入れ直す |

### AT+CEER の読み方（数字だけ見る）
| comma four の例 | comma 3X の例 | 意味 |
|---|---|---|
| `"ESM",29,...` | `5,29` | **認証の失敗**（ユーザー名・パスワード・認証タイプのどれか） |
| `"ESM",27,...` | `5,27` | APN が違う |
| `"ESM",33,...` | `5,33` | 契約がない・そのサービスが使えない |
| `"ESM",32,...` | `5,32` | APN が空など |
| — | `6,259` | アタッチそのものの失敗 |

---

## 6. ログを取って、伏せて、まとめる

```bash
cd /data/openpilot/tools/modem_auth
./collect_modem_logs.sh
```
- `~/modem_logs_<日時>.tar.gz` ができます。中身は、modem.py のログ、モデムの状態、pppd のログ、モデムへの読み取り専用の問い合わせ（`AT+CGAUTH?` など）、ネットワークの経路です。
- IMEI・ICCID・IMSI・IP アドレス・位置の番号・ユーザー名・パスワードは、自動で伏せます。ユーザー名とパスワードは `<user:7>` のように **長さだけ** 残します（モデムがパスワードを読み返すかが分かるように）。
- **送る前に、自分の目で中を確かめてください**（`tar -xzf` で開いて `grep` する）。
- パソコンに持ってくる：`scp comma@<端末の IP>:modem_logs_*.tar.gz .`
- 再起動すると pppd のログは消えます。**再起動の前に** 取ってください。

### モデムに個別に問い合わせたいとき（読み取りだけ）
```bash
./modem_at_read.py --preset probe --secrets-from-params
./modem_at_read.py --preset eg25 --secrets-from-params     # comma 3X のみ
```
modem.py と同じ鍵（ロック）を使うので、動いている modem.py とぶつかりません。ただし、起動の直後（1 分以内）は避けてください。modem.py が準備中に答えをもらえず、準備をやり直すことがあります。

---

## 7. 片付け（必ずこの順番で）

**画面で片付ける場合**：「APN 認証」で off を選ぶ → 「保存したユーザー名とパスワードも消しますか」で **はい**。または「APN 認証情報を消す」（comma 3X の日本語表示は「APNログイン情報を削除」）→ **はい**。どちらも、方式を none にしてモデムの値を消してから、ユーザー名とパスワードを消します。そのあと下の 2 の行を確かめてください。

**SSH で片付ける場合**：

1. 認証を消す設定にする：
   ```bash
   ./set_apn_auth.sh none
   ```
2. 1 分待ち、ログに次の行が出たことを確かめる：
   ```
   APN auth via CGAUTH: protocol=none, ...      （comma 3X は QICSGP）
   ```
   これで、モデムの中の認証の設定が消えます。
3. 設定を消す：
   ```bash
   ./set_apn_auth.sh clear
   ./set_apn_auth.sh show      # すべて not set
   ```
4. そのあと、SIM を差し替える／元のソフトに戻す。

### 元のソフトに戻す
- 設定 → デバイス → アンインストール → カスタムソフトウェアで、次のどちらかを入れる。
  - `openpilot-test.comma.ai`（comma four は release-mici-staging、comma 3X は release-tizi-staging に自動で切り替わります。AGNOS 19.9 のまま使えます）
  - `openpilot-nightly.comma.ai`
  - WPA3 の配布物を使っていた人は、その配布物の URL
- **`openpilot.comma.ai`（0.11.1）には、0.11.2 が出るまで戻さないでください。** AGNOS 19.9 の上の 0.11.1 は、起動を繰り返す不具合が報告されています（commaai/openpilot#39063）。

---

## 8. 報告のしかた

次をコピーして埋め、§6 の `.tar.gz` を（段階ごとに）添付してください。

```
機種：comma four / comma 3X
AGNOS：（cat /VERSION）
モデム：（ログの ver= か AT+GMR。例 EG916QGLLGR01A05M04 / EG25GGBR07A08M2G）
SIM の事業者と回線：（例：mineo D プラン＝docomo 回線）
APN：（公開されている APN 名）
認証タイプ：CHAP / PAP
段階 A（認証なし）：つながった / つながらない（registration=… / CEER=…）
段階 B（認証あり）：つながった / つながらない。設定してからつながるまで約 … 秒
   ping：成功 / 失敗
   readback の行：protocol ok=… user ok=… password ok=… / not shown
段階 C（再起動）：起動からつながるまで約 … 秒
段階 D（違うパスワード）：どうなったか、つなぎ直しの回数、CEER
段階 E（PAP）：
気づいたこと：
```

---

## 9. 機種ごとの違い

### comma four（モデム EG916Q-GL）
- 認証は `AT+CGAUTH` で書きます。
- 物理 SIM を挿すと、内蔵の eSIM の代わりに物理 SIM が使われます。

### comma 3X（モデム EG25-G）
- Quectel の資料に `AT+CGAUTH` がないため、`AT+QICSGP` を使います（起動時にどちらが使えるか問い合わせて選びます）。**comma 3X でこの方式を確かめた人はまだいません。** 段階 B のログの `APN auth via ...` の行が特に大事です。
- au 回線の SIM では、モデムに最初から au 用の認証の値が入っていることがあります。試験版は、**自分で認証を設定しないかぎり、その値には触りません。**
- 戻るときの注意（§7）は comma four と同じです。

---

## 10. 困ったとき
| 症状 | まず見るところ |
|---|---|
| `set_apn_auth.sh` が「Install and build the test branch first」と出る | 試験版のビルドが終わっていない。§3 の 5 を確かめる |
| ずっと `searching` / `denied` | §5 の CEER。29 なら認証の値、27 なら APN を見直す |
| `PPP fail 1/3` が繰り返す | pppd のログ（`sudo journalctl -b -t pppd -t chat`）を §6 で取って報告 |
| openpilot が起動しない | SSH で `ls -l /data/params/d/Gsm*` を見て、持ち主が `comma` 以外のものがあれば報告（sudo で書いた可能性） |
